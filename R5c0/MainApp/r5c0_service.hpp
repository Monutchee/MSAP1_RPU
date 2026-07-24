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
#include <cstring>
#include "xparameters.h"

#include "ad7771.hpp"
#include "control_service.hpp"
#include "led_controller.hpp"
#include "metering.hpp"

class R5c0Service : public msap1::ControlService {
public:
	R5c0Service(const msap1::CoreConfig &config, msap1::adc::Ad7771 &adc,
		     msap1::meter::MeteringPipeline &metering)
		: msap1::ControlService(config),
		  led_(XPAR_XGPIO_0_BASEADDR, /*led_mask=*/0x01u,
		       /*heartbeat_period_ms=*/500u), // 1 Hz full cycle
		  adc_(adc), metering_(metering)
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
			health.dclk_frequency_hz = capture.dclk_frequency_hz;
			health.drdy_frequency_hz = capture.drdy_frequency_hz;
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

			const auto meter = metering_.status();
			health.meter_generation = meter.generation;
			health.conversion_status = meter.conversion_status;
			health.processing_status = meter.processing_status;
			if (meter.cores_present)
				health.meter_health_flags |=
					MSAP1_METER_HEALTH_CORES_PRESENT;
			if (meter.configured)
				health.meter_health_flags |=
					MSAP1_METER_HEALTH_CONFIGURED;
			if (meter.generation_matches)
				health.meter_health_flags |=
					MSAP1_METER_HEALTH_GENERATION_MATCH;
			if (meter.enabled)
				health.meter_health_flags |=
					MSAP1_METER_HEALTH_ENABLED;
			if (meter.remove_dc)
				health.meter_health_flags |=
					MSAP1_METER_HEALTH_REMOVE_DC;

			send_response(&request, src, MSAP1_RPU_MSG_ADC_HEALTH,
				      MSAP1_RPU_STATUS_OK, &health, sizeof(health));
			return true;
		}
		case MSAP1_RPU_MSG_METER_CONFIG_SET: {
			if (payload_len != sizeof(msap1_meter_config_payload)) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}

			msap1_meter_config_payload wire = {};
			std::memcpy(&wire, payload, sizeof(wire));
			if ((wire.valid_mask & ~0xffu) != 0u ||
			    (wire.flags & ~(MSAP1_METER_CONFIG_ENABLE |
					    MSAP1_METER_CONFIG_REMOVE_DC)) != 0u ||
			    (wire.frequency_flags &
			     ~MSAP1_FREQUENCY_CONFIG_ENABLE) != 0u ||
			    wire.sample_rate_hz != msap1::adc::sample_rate_hz(
				adc_.configuration().sample_rate)) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}

			std::array<msap1::adc::PgaGain,
				   msap1::adc::channel_count> gains{};
			bool gains_valid = true;
			for (std::size_t channel = 0; channel < gains.size();
			     ++channel) {
				switch (wire.adc_pga_gain[channel]) {
				case 1u:
					gains[channel] = msap1::adc::PgaGain::X1;
					break;
				case 2u:
					gains[channel] = msap1::adc::PgaGain::X2;
					break;
				case 4u:
					gains[channel] = msap1::adc::PgaGain::X4;
					break;
				case 8u:
					gains[channel] = msap1::adc::PgaGain::X8;
					break;
				default:
					gains_valid = false;
					break;
				}
			}
			if (!gains_valid) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}
			if (adc_.capture_active()) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_ADC_STATE,
					      nullptr, 0);
				return true;
			}
			const auto adc_error = adc_.configure_pga(gains);
			if (adc_error != msap1::adc::Error::None) {
				const auto status =
					adc_error ==
						msap1::adc::Error::CaptureNotInitialized ?
					MSAP1_RPU_STATUS_ADC_UNAVAILABLE :
					MSAP1_RPU_STATUS_INTERNAL_ERROR;
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      status, nullptr, 0);
				return true;
			}

			msap1::meter::Configuration configuration;
			configuration.generation = wire.generation;
			configuration.sample_rate_hz = wire.sample_rate_hz;
			configuration.rms_window_samples =
				wire.rms_window_samples;
			configuration.valid_mask =
				static_cast<std::uint8_t>(wire.valid_mask);
			for (std::size_t channel = 0;
			     channel < configuration.scale_micro_units_q16.size();
			     ++channel)
				configuration.scale_micro_units_q16[channel] =
					wire.scale_micro_units_q16[channel];
			configuration.enable =
				(wire.flags & MSAP1_METER_CONFIG_ENABLE) != 0u;
			configuration.remove_dc =
				(wire.flags & MSAP1_METER_CONFIG_REMOVE_DC) != 0u;
			configuration.frequency.enable =
				(wire.frequency_flags &
				 MSAP1_FREQUENCY_CONFIG_ENABLE) != 0u;
			configuration.frequency.mode = wire.frequency_mode;
			configuration.frequency.reference_channel =
				wire.frequency_reference_channel;
			configuration.frequency.averaging_cycles =
				wire.frequency_averaging_cycles;
			configuration.frequency.window_samples =
				wire.frequency_window_samples;
			configuration.frequency.minimum_millihz =
				wire.frequency_minimum_millihz;
			configuration.frequency.maximum_millihz =
				wire.frequency_maximum_millihz;
			configuration.frequency.hysteresis_microvolts =
				wire.frequency_hysteresis_microvolts;

			const auto error = metering_.configure(configuration);
			if (error != msap1::meter::Error::None) {
				const auto status =
					error == msap1::meter::Error::CoreNotFound ?
					MSAP1_RPU_STATUS_METER_UNAVAILABLE :
					MSAP1_RPU_STATUS_METER_CONFIG;
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      status, nullptr, 0);
				return true;
			}

			const auto meter = metering_.status();
			msap1_meter_config_ack_payload acknowledgement = {};
			acknowledgement.generation = configuration.generation;
			acknowledgement.conversion_active_generation =
				meter.conversion_active_generation;
			acknowledgement.processing_active_generation =
				meter.processing_active_generation;
			acknowledgement.conversion_status =
				meter.conversion_status;
			acknowledgement.processing_status =
				meter.processing_status;
			send_response(&request, src, MSAP1_RPU_MSG_ACK,
				      MSAP1_RPU_STATUS_OK, &acknowledgement,
				      sizeof(acknowledgement));
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
	static_assert(sizeof(msap1_adc_health_payload) == 72,
		      "ADC health wire layout must match the APU");
	static_assert(sizeof(msap1_meter_config_payload) == 92,
		      "meter configuration wire layout must match the APU");
	static_assert(sizeof(msap1_rpu_msg_header) +
		      sizeof(msap1_adc_health_payload) <= MSAP1_RPU_MAX_FRAME_SIZE,
		      "ADC health response exceeds protocol frame size");

	msap1::LedController led_;
	msap1::adc::Ad7771 &adc_;
	msap1::meter::MeteringPipeline &metering_;
};

#endif /* MSAP1_R5C0_SERVICE_HPP */
