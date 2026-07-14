/*
 * Communication core implementation. See control_service.hpp.
 */

#include "control_service.hpp"

#include <cstring>

#include <metal/log.h>
#include <metal/version.h>
#include <openamp/open_amp.h>
#include <openamp/version.h>

#include "FreeRTOS.h"
#include "task.h"

#include "xil_printf.h"
#include "xparameters.h"

#define LPRINTF(fmt, ...) \
	xil_printf("%s():%u " fmt, __func__, __LINE__, ##__VA_ARGS__)
#define LPERROR(fmt, ...) LPRINTF("ERROR: " fmt, ##__VA_ARGS__)

namespace msap1 {

CoreConfig CoreConfig::current()
{
#if XPAR_CPU_ID == 0
	return CoreConfig{
		/*core_id=*/0u,
		/*service_name=*/"mncos-r5c0-ctrl",
	};
#elif XPAR_CPU_ID == 1
	return CoreConfig{
		/*core_id=*/1u,
		/*service_name=*/"mncos-r5c1-ctrl",
	};
#else
#error "Unsupported RPU CPU ID"
#endif
}

ControlService::ControlService(const CoreConfig &config) : config_(config) {}

std::uint32_t ControlService::uptime_ms() const
{
	return static_cast<std::uint32_t>(
		(xTaskGetTickCount() * 1000u) / configTICK_RATE_HZ);
}

void ControlService::send_response(const msap1_rpu_msg_header *request,
				   std::uint32_t dst, std::uint8_t type,
				   std::uint32_t status, const void *payload,
				   std::uint16_t payload_len)
{
	std::uint8_t tx[MSAP1_RPU_MAX_FRAME_SIZE] = {0};
	msap1_rpu_msg_header header;
	const std::uint32_t frame_len = sizeof(header) + payload_len;

	header.magic = MSAP1_RPU_MAGIC;
	header.version = MSAP1_RPU_VERSION;
	header.type = type;
	header.payload_len = payload_len;
	header.sequence = request ? request->sequence : 0;
	header.status = status;

	if (frame_len > sizeof(tx)) {
		error_count_++;
		return;
	}

	std::memcpy(tx, &header, sizeof(header));
	if (payload_len)
		std::memcpy(tx + sizeof(header), payload, payload_len);

	if (endpoint_.send_to(tx, frame_len, dst) < 0)
		error_count_++;
}

void ControlService::send_status(const msap1_rpu_msg_header *request,
				 std::uint32_t dst)
{
	msap1_rpu_status_payload payload = {};

	/* Communication-owned fields. */
	payload.core_id = config_.core_id;
	payload.uptime_ms = uptime_ms();
	payload.rx_count = rx_count_;
	payload.error_count = error_count_;

	/* Application-owned fields (led_mode/led_on/heartbeat_count). */
	on_fill_status(payload);

	send_response(request, dst, MSAP1_RPU_MSG_STATUS, MSAP1_RPU_STATUS_OK,
		      &payload, sizeof(payload));
}

bool ControlService::handle_custom(const msap1_rpu_msg_header &request,
				   const void *payload,
				   std::uint16_t payload_len,
				   std::uint32_t src)
{
	(void)request;
	(void)payload;
	(void)payload_len;
	(void)src;
	return false;
}

int ControlService::on_message(const void *data, std::size_t len,
			       std::uint32_t src)
{
	msap1_rpu_msg_header request;

	rx_count_++;

	if (len < sizeof(request)) {
		error_count_++;
		return RPMSG_SUCCESS;
	}

	std::memcpy(&request, data, sizeof(request));

	if (request.magic != MSAP1_RPU_MAGIC) {
		error_count_++;
		send_response(&request, src, MSAP1_RPU_MSG_ERROR,
			      MSAP1_RPU_STATUS_BAD_MAGIC, nullptr, 0);
		return RPMSG_SUCCESS;
	}
	if (request.version != MSAP1_RPU_VERSION) {
		error_count_++;
		send_response(&request, src, MSAP1_RPU_MSG_ERROR,
			      MSAP1_RPU_STATUS_BAD_VERSION, nullptr, 0);
		return RPMSG_SUCCESS;
	}
	if (request.payload_len + sizeof(request) != len ||
	    len > MSAP1_RPU_MAX_FRAME_SIZE) {
		error_count_++;
		send_response(&request, src, MSAP1_RPU_MSG_ERROR,
			      MSAP1_RPU_STATUS_BAD_LENGTH, nullptr, 0);
		return RPMSG_SUCCESS;
	}

	const void *payload =
		static_cast<const std::uint8_t *>(data) + sizeof(request);

	switch (request.type) {
	case MSAP1_RPU_MSG_PING:
		send_response(&request, src, MSAP1_RPU_MSG_PONG,
			      MSAP1_RPU_STATUS_OK, nullptr, 0);
		break;
	case MSAP1_RPU_MSG_GET_STATUS:
		send_status(&request, src);
		break;
	case MSAP1_RPU_MSG_SET_LED: {
		msap1_rpu_led_payload led;
		std::uint32_t status;

		if (request.payload_len != sizeof(led)) {
			error_count_++;
			send_response(&request, src, MSAP1_RPU_MSG_ERROR,
				      MSAP1_RPU_STATUS_BAD_PAYLOAD, nullptr, 0);
			break;
		}
		std::memcpy(&led, payload, sizeof(led));
		status = on_set_led(led.mode);
		if (status == MSAP1_RPU_STATUS_OK) {
			send_response(&request, src, MSAP1_RPU_MSG_ACK,
				      MSAP1_RPU_STATUS_OK, nullptr, 0);
		} else {
			error_count_++;
			send_response(&request, src, MSAP1_RPU_MSG_ERROR,
				      status, nullptr, 0);
		}
		break;
	}
	default:
		if (!handle_custom(request, payload, request.payload_len, src)) {
			error_count_++;
			send_response(&request, src, MSAP1_RPU_MSG_ERROR,
				      MSAP1_RPU_STATUS_BAD_TYPE, nullptr, 0);
		}
		break;
	}

	return RPMSG_SUCCESS;
}

void ControlService::on_unbind()
{
	ML_INFO("remote endpoint destroyed\r\n");
}

void ControlService::run()
{
	LPRINTF("OpenAMP %s, libmetal %s\r\n", openamp_version(), metal_ver());
	LPRINTF("Starting %s on R5 core %u\r\n", config_.service_name,
		config_.core_id);

	if (!platform_.init()) {
		LPERROR("Failed to initialize OpenAMP platform.\r\n");
		while (1)
			;
	}

	while (1) {
		rpmsg_device *rpdev = platform_.create_rpmsg_vdev();
		if (!rpdev) {
			ML_ERR("Failed to create RPMsg virtio device.\r\n");
			while (1)
				;
		}

		ML_INFO("Creating RPMsg endpoint %s\r\n", config_.service_name);
		if (endpoint_.create(rpdev, config_.service_name, *this)) {
			platform_.poll_until_reset(rpdev);
			endpoint_.destroy();
		}

		platform_.release_rpmsg_vdev(rpdev);
	}
}

} // namespace msap1
