/**
 * @file r5c0_service.cpp
 * @brief Wire-level dispatch of the R5 core 0 control service.
 */

#include "r5c0_service.hpp"

#include <cstring>
#include "xparameters.h"

#include "handlers/adc/adc_capture.hpp"
#include "handlers/adc/adc_diagnostic.hpp"
#include "handlers/adc/adc_health.hpp"
#include "handlers/meter/meter_config.hpp"

/* The wire ABI must stay byte-identical with the APU copy. */
static_assert(sizeof(msap1_adc_health_payload) == 238,
	      "ADC health wire layout must match the APU");
static_assert(sizeof(msap1_meter_config_payload) == 176,
	      "meter configuration wire layout must match the APU");
static_assert(sizeof(msap1_adc_diagnostic_payload) == 188,
	      "ADC diagnostic wire layout must match the APU");
static_assert(sizeof(msap1_rpu_msg_header) +
	      sizeof(msap1_adc_health_payload) <= MSAP1_RPU_MAX_FRAME_SIZE,
	      "ADC health response exceeds protocol frame size");
static_assert(sizeof(msap1_rpu_msg_header) +
	      sizeof(msap1_adc_diagnostic_payload) <= MSAP1_RPU_MAX_FRAME_SIZE,
	      "ADC diagnostic response exceeds protocol frame size");

R5c0Service::R5c0Service(const msap1::CoreConfig &config,
			 msap1::adc::AdcController &adc,
			 msap1::meter::MeteringPipeline &metering)
	: msap1::ControlService(config),
	  led_(XPAR_XGPIO_0_BASEADDR, /*led_mask=*/0x01u,
	       /*heartbeat_period_ms=*/500u), // 1 Hz full cycle
	  adc_(adc), metering_(metering)
{
}

bool R5c0Service::init_led()
{
	return led_.init();
}

void R5c0Service::run_heartbeat()
{
	led_.run_heartbeat();
}

std::uint32_t R5c0Service::on_set_led(std::uint8_t mode)
{
	return led_.set_mode(mode);
}

void R5c0Service::on_fill_status(msap1_rpu_status_payload &status)
{
	led_.fill_status(status);
}

bool R5c0Service::handle_custom(const msap1_rpu_msg_header &request,
				const void *payload,
				std::uint16_t payload_len, std::uint32_t src)
{
	switch (request.type) {
	case MSAP1_RPU_MSG_ADC_HEALTH_GET:
		return handle_adc_health(request, payload_len, src);
	case MSAP1_RPU_MSG_METER_CONFIG_SET:
		return handle_meter_config(request, payload, payload_len, src);
	case MSAP1_RPU_MSG_ADC_DIAGNOSTIC_RUN:
		return handle_adc_diagnostic(request, payload, payload_len,
					     src);
	case MSAP1_RPU_MSG_ADC_CAPTURE_START:
		return handle_capture_start(request, payload_len, src);
	case MSAP1_RPU_MSG_ADC_CAPTURE_STOP:
		return handle_capture_stop(request, payload_len, src);
	default:
		return false;
	}
}

void R5c0Service::on_transport_unbind()
{
	// A vanished Linux owner cannot continue draining DMA. Stop the PL
	// stream and reset its FIFO so reconnect starts from a clean boundary.
	adc_.stop_capture();
}

void R5c0Service::send_error(const msap1_rpu_msg_header &request,
			     std::uint32_t src, std::uint32_t status)
{
	send_response(&request, src, MSAP1_RPU_MSG_ERROR, status, nullptr, 0);
}

bool R5c0Service::handle_adc_health(const msap1_rpu_msg_header &request,
				    std::uint16_t payload_len,
				    std::uint32_t src)
{
	if (payload_len != 0u) {
		send_error(request, src, MSAP1_RPU_STATUS_BAD_PAYLOAD);
		return true;
	}
	msap1_adc_health_payload health;
	msap1::r5c0::build_adc_health(adc_, metering_, health);
	send_response(&request, src, MSAP1_RPU_MSG_ADC_HEALTH,
		      MSAP1_RPU_STATUS_OK, &health, sizeof(health));
	return true;
}

bool R5c0Service::handle_meter_config(const msap1_rpu_msg_header &request,
				      const void *payload,
				      std::uint16_t payload_len,
				      std::uint32_t src)
{
	if (payload_len != sizeof(msap1_meter_config_payload)) {
		send_error(request, src, MSAP1_RPU_STATUS_BAD_PAYLOAD);
		return true;
	}
	msap1_meter_config_payload wire = {};
	std::memcpy(&wire, payload, sizeof(wire));
	msap1_meter_config_ack_payload acknowledgement = {};
	const auto status = msap1::r5c0::apply_meter_config(
		adc_, metering_, wire, acknowledgement);
	if (status != MSAP1_RPU_STATUS_OK) {
		send_error(request, src, status);
		return true;
	}
	send_response(&request, src, MSAP1_RPU_MSG_ACK, MSAP1_RPU_STATUS_OK,
		      &acknowledgement, sizeof(acknowledgement));
	return true;
}

bool R5c0Service::handle_adc_diagnostic(const msap1_rpu_msg_header &request,
					const void *payload,
					std::uint16_t payload_len,
					std::uint32_t src)
{
	if (payload_len != sizeof(msap1_adc_diagnostic_request)) {
		send_error(request, src, MSAP1_RPU_STATUS_BAD_PAYLOAD);
		return true;
	}
	msap1_adc_diagnostic_request diagnostic_request{};
	std::memcpy(&diagnostic_request, payload, sizeof(diagnostic_request));
	msap1_adc_diagnostic_payload wire{};
	const auto status = msap1::r5c0::run_adc_diagnostic(
		adc_, diagnostic_request, wire);
	if (status != MSAP1_RPU_STATUS_OK) {
		send_error(request, src, status);
		return true;
	}
	send_response(&request, src, MSAP1_RPU_MSG_ADC_DIAGNOSTIC,
		      MSAP1_RPU_STATUS_OK, &wire, sizeof(wire));
	return true;
}

bool R5c0Service::handle_capture_start(const msap1_rpu_msg_header &request,
				       std::uint16_t payload_len,
				       std::uint32_t src)
{
	if (payload_len != 0u) {
		send_error(request, src, MSAP1_RPU_STATUS_BAD_PAYLOAD);
		return true;
	}
	const auto status = msap1::r5c0::start_capture(adc_);
	if (status != MSAP1_RPU_STATUS_OK) {
		send_error(request, src, status);
		return true;
	}
	send_response(&request, src, MSAP1_RPU_MSG_ACK, MSAP1_RPU_STATUS_OK,
		      nullptr, 0);
	return true;
}

bool R5c0Service::handle_capture_stop(const msap1_rpu_msg_header &request,
				      std::uint16_t payload_len,
				      std::uint32_t src)
{
	if (payload_len != 0u) {
		send_error(request, src, MSAP1_RPU_STATUS_BAD_PAYLOAD);
		return true;
	}
	msap1::r5c0::stop_capture(adc_);
	send_response(&request, src, MSAP1_RPU_MSG_ACK, MSAP1_RPU_STATUS_OK,
		      nullptr, 0);
	return true;
}
