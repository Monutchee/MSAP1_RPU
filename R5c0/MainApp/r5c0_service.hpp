#ifndef MSAP1_R5C0_SERVICE_HPP
#define MSAP1_R5C0_SERVICE_HPP

/*
 * R5 core 0 control service.
 *
 * R5c0 owns the board's single LED ("UF2_LED", bit 0x01 of AXI_GPIO_0) and runs
 * the heartbeat. It extends the comm-only ControlService with LED behaviour by
 * composing a LedController and overriding the LED hooks.
 */

#include <cstdint>
#include "xparameters.h"

#include "ad7771.hpp"
#include "control_service.hpp"
#include "led_controller.hpp"

class R5c0Service : public msap1::ControlService {
public:
	R5c0Service(const msap1::CoreConfig &config, msap1::adc::Ad7771 &adc)
		: msap1::ControlService(config),
		  led_(XPAR_XGPIO_0_BASEADDR, /*led_mask=*/0x01u,
		       /*heartbeat_period_ms=*/200u),
		  adc_(adc)
	{
	}

	/* Initialise the LED GPIO. Call before starting the scheduler. */
	bool init_led() { return led_.init(); }

	/* Heartbeat task body. */
	void run_heartbeat() { led_.run_heartbeat(); }

protected:
	std::uint32_t on_set_led(std::uint8_t mode) override
	{
		return led_.set_mode(mode);
	}

	void on_fill_status(msap1_rpu_status_payload &status) override
	{
		led_.fill_status(status);
	}

	bool handle_custom(const msap1_rpu_msg_header &request,
			   const void *payload, std::uint16_t payload_len,
			   std::uint32_t src) override
	{
		(void)payload;
		switch (request.type) {
		case MSAP1_RPU_MSG_ADC_HEALTH_GET: {
			if (payload_len != 0u) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}

			msap1_adc_health_payload health = {};
			const auto capture = adc_.status();
			health.sample_rate_hz = msap1::adc::sample_rate_hz(
				adc_.configuration().sample_rate);
			health.capture_flags = capture.flags;
			health.frame_count = capture.frames;
			health.overflow_count = capture.overflows;
			health.header_error_count = capture.header_errors;
			health.alert_count = capture.alerts;
			health.packet_count = capture.packets;
			if (adc_.initialized())
				health.health_flags |= MSAP1_ADC_HEALTH_INITIALIZED;
			if (adc_.capture_active())
				health.health_flags |= MSAP1_ADC_HEALTH_CAPTURE_ACTIVE;
			if (capture.overflows == 0u)
				health.health_flags |= MSAP1_ADC_HEALTH_NO_OVERFLOW;
			if (capture.frames != 0u && capture.header_errors == 0u)
				health.health_flags |= MSAP1_ADC_HEALTH_HEADERS_VALID;

			msap1::adc::RegisterHealth registers;
			const auto error = adc_.read_register_health(registers);
			switch (error) {
			case msap1::adc::Error::None:
				health.spi_error = MSAP1_ADC_SPI_HEALTH_OK;
				health.health_flags |=
					MSAP1_ADC_HEALTH_SPI_RESPONSIVE;
				break;
			case msap1::adc::Error::SpiInitialization:
				health.spi_error =
					MSAP1_ADC_SPI_HEALTH_NOT_INITIALIZED;
				break;
			case msap1::adc::Error::SpiTransfer:
				health.spi_error =
					MSAP1_ADC_SPI_HEALTH_TRANSFER_FAILED;
				break;
			case msap1::adc::Error::SpiProtocol:
				health.spi_error =
					MSAP1_ADC_SPI_HEALTH_PROTOCOL_FAILED;
				break;
			default:
				health.spi_error =
					MSAP1_ADC_SPI_HEALTH_INTERNAL_ERROR;
				break;
			}
			health.expected_decimation = registers.expected_decimation;
			health.status_3 = registers.status_3;
			health.general_user_config_1 =
				registers.general_user_config_1;
			health.general_user_config_2 =
				registers.general_user_config_2;
			health.general_user_config_3 =
				registers.general_user_config_3;
			health.dout_format = registers.dout_format;
			health.src_n_msb = registers.src_n_msb;
			health.src_n_lsb = registers.src_n_lsb;
			health.src_if_msb = registers.src_if_msb;
			health.src_if_lsb = registers.src_if_lsb;
			health.src_update = registers.src_update;
			if ((registers.status_3 & (1u << 4)) != 0u)
				health.health_flags |= MSAP1_ADC_HEALTH_INIT_COMPLETE;
			if (registers.configuration_matches)
				health.health_flags |= MSAP1_ADC_HEALTH_CONFIG_MATCH;

			send_response(&request, src, MSAP1_RPU_MSG_ADC_HEALTH,
				      MSAP1_RPU_STATUS_OK, &health, sizeof(health));
			return true;
		}
		case MSAP1_RPU_MSG_ADC_CAPTURE_START: {
			if (payload_len != 0u) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}
			const auto error = adc_.start_capture();
			if (error == msap1::adc::Error::CaptureNotInitialized) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_ADC_UNAVAILABLE,
					      nullptr, 0);
				return true;
			}
			if (error == msap1::adc::Error::CaptureAlreadyActive) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_ADC_STATE,
					      nullptr, 0);
				return true;
			}
			if (error != msap1::adc::Error::None) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_INTERNAL_ERROR,
					      nullptr, 0);
				return true;
			}
			send_response(&request, src, MSAP1_RPU_MSG_ACK,
				      MSAP1_RPU_STATUS_OK, nullptr, 0);
			return true;
		}
		case MSAP1_RPU_MSG_ADC_CAPTURE_STOP:
			if (payload_len != 0u) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}
			adc_.stop_capture();
			send_response(&request, src, MSAP1_RPU_MSG_ACK,
				      MSAP1_RPU_STATUS_OK, nullptr, 0);
			return true;
		default:
			return false;
		}
	}

	void on_transport_unbind() override
	{
		// A vanished Linux owner cannot continue draining DMA. Stop the PL
		// stream and reset its FIFO so reconnect starts from a clean boundary.
		adc_.stop_capture();
	}

private:
	static_assert(sizeof(msap1_adc_health_payload) == 48,
		      "ADC health wire layout must match the APU");
	static_assert(sizeof(msap1_rpu_msg_header) +
		      sizeof(msap1_adc_health_payload) <= MSAP1_RPU_MAX_FRAME_SIZE,
		      "ADC health response exceeds protocol frame size");

	msap1::LedController led_;
	msap1::adc::Ad7771 &adc_;
};

#endif /* MSAP1_R5C0_SERVICE_HPP */
