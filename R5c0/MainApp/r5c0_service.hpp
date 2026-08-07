#ifndef MSAP1_R5C0_SERVICE_HPP
#define MSAP1_R5C0_SERVICE_HPP

/**
 * @file r5c0_service.hpp
 * @brief R5 core 0 control service: LED ownership and command dispatch.
 *
 * R5c0 owns the board's single LED ("UF2_LED", bit 0x01 of AXI_GPIO_0) and
 * runs the heartbeat. It extends the comm-only ControlService with LED
 * behaviour and dispatches the ADC/meter commands to the focused modules
 * under handlers/.
 *
 * Division of labour: this class owns the WIRE concerns — payload-length
 * validation, request/response framing, and the frame-size static_asserts
 * (see r5c0_service.cpp) — while the handlers implement the product
 * behaviour and return a wire status plus, on success, the response
 * payload.
 */

#include <cstdint>

#include "adc_controller.hpp"
#include "control_service.hpp"
#include "led_controller.hpp"
#include "metering.hpp"

class R5c0Service : public msap1::ControlService {
public:
	R5c0Service(const msap1::CoreConfig &config,
		     msap1::adc::AdcController &adc,
		     msap1::meter::MeteringPipeline &metering);

	/** Initialise the LED GPIO. Call before starting the scheduler. */
	bool init_led();

	/** Heartbeat task body. */
	void run_heartbeat();

protected:
	/** Apply an LED mode change requested over RPMsg. */
	std::uint32_t on_set_led(std::uint8_t mode) override;

	/** Add the LED state to the GET_STATUS payload. */
	void on_fill_status(msap1_rpu_status_payload &status) override;

	/** Route the ADC/meter commands to their handlers/ implementations. */
	bool handle_custom(const msap1_rpu_msg_header &request,
			   const void *payload, std::uint16_t payload_len,
			   std::uint32_t src) override;

	/** Stop the PL stream when the Linux endpoint owner vanishes. */
	void on_transport_unbind() override;

private:
	/** Reply with an empty error frame carrying @p status. */
	void send_error(const msap1_rpu_msg_header &request, std::uint32_t src,
			std::uint32_t status);

	bool handle_adc_health(const msap1_rpu_msg_header &request,
			       std::uint16_t payload_len, std::uint32_t src);
	bool handle_meter_config(const msap1_rpu_msg_header &request,
				 const void *payload,
				 std::uint16_t payload_len, std::uint32_t src);
	bool handle_adc_diagnostic(const msap1_rpu_msg_header &request,
				   const void *payload,
				   std::uint16_t payload_len,
				   std::uint32_t src);
	bool handle_capture_start(const msap1_rpu_msg_header &request,
				  std::uint16_t payload_len,
				  std::uint32_t src);
	bool handle_capture_stop(const msap1_rpu_msg_header &request,
				 std::uint16_t payload_len, std::uint32_t src);

	msap1::LedController led_;
	msap1::adc::AdcController &adc_;
	msap1::meter::MeteringPipeline &metering_;
};

#endif /* MSAP1_R5C0_SERVICE_HPP */
