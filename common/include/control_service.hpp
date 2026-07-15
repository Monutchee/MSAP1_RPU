#ifndef MSAP1_CONTROL_SERVICE_HPP
#define MSAP1_CONTROL_SERVICE_HPP

/*
 * Communication core for the MSAP1 RPU firmware.
 *
 * ControlService is a reusable BASE CLASS that owns only the rpmsg side of the
 * world: the OpenAMP platform lifecycle, an rpmsg endpoint, and the MSAP1
 * control protocol (rpu_control_protocol.h). It speaks the same wire protocol
 * as the APU-side msap1-apu-app utility:
 *   PING        -> PONG
 *   GET_STATUS  -> STATUS payload
 *   SET_LED     -> ACK (or ERROR)
 *
 * It deliberately knows nothing about GPIO, LEDs, or FreeRTOS task creation.
 * Each R5 core derives a small subclass (R5c0Service, R5c1Service) that
 * supplies the core-specific behaviour by overriding the virtual hooks below.
 * The owning main.cpp creates the FreeRTOS task(s) and calls run().
 */

#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"

#include "mnc/openamp_platform.hpp"
#include "mnc/rpmsg_endpoint.hpp"
#include "rpu_control_protocol.h"

namespace msap1 {

/* Per-core communication identity. CoreConfig::current() resolves it from
 * XPAR_CPU_ID. */
struct CoreConfig {
	std::uint32_t core_id;
	const char *service_name;

	static CoreConfig current();
};

class ControlService : public mnc::MessageSink {
public:
	explicit ControlService(const CoreConfig &config);
	~ControlService() override = default;

	ControlService(const ControlService &) = delete;
	ControlService &operator=(const ControlService &) = delete;

	/*
	 * Blocking OpenAMP/rpmsg service loop: initialises the platform, then
	 * repeatedly creates the rpmsg vdev + endpoint and services traffic
	 * until the remote tears it down. Intended to be the body of a FreeRTOS
	 * task created by main.cpp.
	 */
	void run();

	const CoreConfig &config() const { return config_; }

protected:
	/*
	 * SET_LED hook. Apply the requested LED mode and return a
	 * MSAP1_RPU_STATUS_* code. Default: OK no-op, for cores without an LED.
	 */
	virtual std::uint32_t on_set_led(std::uint8_t mode)
	{
		(void)mode;
		return MSAP1_RPU_STATUS_OK;
	}

	/*
	 * GET_STATUS hook. Fill the application-owned status fields
	 * (led_mode, led_on, heartbeat_count). Default: leave them zero.
	 */
	virtual void on_fill_status(msap1_rpu_status_payload &status)
	{
		(void)status;
	}

	/*
	 * Hook for message types beyond the built-in protocol. Return true if
	 * handled (a response should already have been sent via
	 * send_response()). Default returns false -> base replies BAD_TYPE.
	 */
	virtual bool handle_custom(const msap1_rpu_msg_header &request,
				   const void *payload, std::uint16_t payload_len,
				   std::uint32_t src);

	/* Called when Linux removes its endpoint. */
	virtual void on_transport_unbind() {}

	/* Send a framed response/error back to the remote. */
	bool send_response(const msap1_rpu_msg_header *request,
			   std::uint32_t dst, std::uint8_t type,
			   std::uint32_t status, const void *payload,
			   std::uint16_t payload_len);

private:
	/* MessageSink */
	int on_message(const void *data, std::size_t len,
		       std::uint32_t src) override;
	void on_unbind() override;

	void send_status(const msap1_rpu_msg_header *request,
			 std::uint32_t dst);
	std::uint32_t uptime_ms() const;

	CoreConfig config_;
	mnc::OpenAmpPlatform platform_;
	mnc::RpmsgEndpoint endpoint_;
	SemaphoreHandle_t tx_mutex_ = nullptr;

	volatile std::uint32_t rx_count_ = 0;
	volatile std::uint32_t error_count_ = 0;
};

} // namespace msap1

#endif /* MSAP1_CONTROL_SERVICE_HPP */
