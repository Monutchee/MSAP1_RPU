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
			const auto rate_difference =
				capture.drdy_frequency_hz > health.sample_rate_hz ?
				capture.drdy_frequency_hz - health.sample_rate_hz :
				health.sample_rate_hz - capture.drdy_frequency_hz;
			if (capture.drdy_frequency_hz != 0u &&
			    rate_difference <= health.sample_rate_hz / 100u + 2u)
				health.health_flags |= MSAP1_ADC_HEALTH_RATE_MATCH;

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
			for (std::size_t channel = 0;
			     channel < registers.channel_config.size(); ++channel) {
				health.channel_config[channel] =
					registers.channel_config[channel];
					health.channel_sync_offset[channel] =
						registers.channel_sync_offset[channel];
					health.channel_error[channel] =
						registers.channel_error[channel];
					for (std::size_t byte = 0;
					     byte < registers.channel_offset[channel].size();
					     ++byte) {
						health.channel_offset[channel][byte] =
							registers.channel_offset[channel][byte];
						health.channel_gain[channel][byte] =
							registers.channel_gain[channel][byte];
					}
				}
			health.channel_disable = registers.channel_disable;
			health.adc_mux_config = registers.adc_mux_config;
			health.global_mux_config = registers.global_mux_config;
			health.gpio_config = registers.gpio_config;
			health.gpio_data = registers.gpio_data;
			health.buffer_config_1 = registers.buffer_config_1;
			health.buffer_config_2 = registers.buffer_config_2;
			for (std::size_t pair = 0;
			     pair < registers.saturation_error.size(); ++pair)
				health.saturation_error[pair] =
					registers.saturation_error[pair];
			health.channel_error_enable =
				registers.channel_error_enable;
			health.general_error_1 = registers.general_error_1;
			health.general_error_1_enable =
				registers.general_error_1_enable;
			health.general_error_2 = registers.general_error_2;
			health.general_error_2_enable =
				registers.general_error_2_enable;
			health.status_1 = registers.status_1;
			health.status_2 = registers.status_2;
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
			     ~MSAP1_FREQUENCY_CONFIG_ENABLE) != 0u) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}

			msap1::adc::SampleRate sample_rate =
				msap1::adc::SampleRate::Sps32000;
			if (!msap1::adc::sample_rate_from_hz(
				    wire.sample_rate_hz, sample_rate)) {
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
			const auto adc_error = adc_.configure_operating_point(
				sample_rate, gains);
			if (adc_error != msap1::adc::Error::None) {
				const auto status = adc_error ==
						msap1::adc::Error::InvalidConfiguration ?
					MSAP1_RPU_STATUS_BAD_PAYLOAD :
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
		case MSAP1_RPU_MSG_ADC_DIAGNOSTIC_RUN: {
			if (payload_len != sizeof(msap1_adc_diagnostic_request)) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}
			msap1_adc_diagnostic_request diagnostic_request{};
			std::memcpy(&diagnostic_request, payload,
				    sizeof(diagnostic_request));
			if (diagnostic_request.flow != 1u) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}

			msap1::adc::DiagnosticResult result;
			const auto error = adc_.run_diagnostic_flow1(result);
			msap1_adc_diagnostic_payload wire{};
			wire.flow = diagnostic_request.flow;
			wire.requested_sample_rate_hz =
				result.requested_sample_rate_hz;
			wire.diagnostic_flags = result.flags;
			wire.diagnostic_error = diagnostic_error(error);
			wire.failure_stage = result.failure_stage;
			wire.reset_hold_ms = result.reset_hold_ms;
			wire.src_update_high_readback =
				result.src_update_high_readback;
			wire.src_update_low_readback =
				result.src_update_low_readback;
			copy_diagnostic_snapshot(wire.before, result.before);
			copy_diagnostic_snapshot(
				wire.reset_asserted, result.reset_asserted);
			copy_diagnostic_snapshot(
				wire.reset_defaults, result.reset_defaults);
			copy_diagnostic_snapshot(wire.after, result.after);
			send_response(&request, src,
				      MSAP1_RPU_MSG_ADC_DIAGNOSTIC,
				      MSAP1_RPU_STATUS_OK, &wire, sizeof(wire));
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
	static std::uint32_t diagnostic_error(msap1::adc::Error error)
	{
		switch (error) {
		case msap1::adc::Error::None:
			return MSAP1_ADC_DIAGNOSTIC_ERROR_NONE;
		case msap1::adc::Error::CaptureNotInitialized:
			return MSAP1_ADC_DIAGNOSTIC_ERROR_NOT_INITIALIZED;
		case msap1::adc::Error::CaptureAlreadyActive:
			return MSAP1_ADC_DIAGNOSTIC_ERROR_CAPTURE_ACTIVE;
		case msap1::adc::Error::SpiInitialization:
		case msap1::adc::Error::SpiTransfer:
		case msap1::adc::Error::SpiProtocol:
			return MSAP1_ADC_DIAGNOSTIC_ERROR_SPI;
		case msap1::adc::Error::AdcNotReady:
			return MSAP1_ADC_DIAGNOSTIC_ERROR_ADC_NOT_READY;
		case msap1::adc::Error::AdcRegisterMismatch:
			return MSAP1_ADC_DIAGNOSTIC_ERROR_REGISTER_MISMATCH;
		default:
			return MSAP1_ADC_DIAGNOSTIC_ERROR_INTERNAL;
		}
	}

	static void copy_diagnostic_snapshot(
		msap1_adc_diagnostic_snapshot &wire,
		const msap1::adc::DiagnosticSnapshot &snapshot)
	{
		wire.snapshot_flags = snapshot.spi_valid ?
			MSAP1_ADC_DIAGNOSTIC_SNAPSHOT_SPI_VALID : 0u;
		wire.capture_flags = snapshot.capture_flags;
		wire.frame_count = snapshot.frame_count;
		wire.packet_count = snapshot.packet_count;
		wire.dclk_frequency_hz = snapshot.dclk_frequency_hz;
		wire.drdy_frequency_hz = snapshot.drdy_frequency_hz;
		wire.status_1 = snapshot.status_1;
		wire.status_2 = snapshot.status_2;
		wire.status_3 = snapshot.status_3;
		wire.general_user_config_1 =
			snapshot.general_user_config_1;
		wire.general_user_config_2 =
			snapshot.general_user_config_2;
		wire.general_user_config_3 =
			snapshot.general_user_config_3;
		wire.dout_format = snapshot.dout_format;
		wire.channel_disable = snapshot.channel_disable;
		wire.buffer_config_1 = snapshot.buffer_config_1;
		wire.buffer_config_2 = snapshot.buffer_config_2;
		wire.src_n_msb = snapshot.src_n_msb;
		wire.src_n_lsb = snapshot.src_n_lsb;
		wire.src_if_msb = snapshot.src_if_msb;
		wire.src_if_lsb = snapshot.src_if_lsb;
		wire.src_update = snapshot.src_update;
	}

	static_assert(sizeof(msap1_adc_health_payload) == 162,
			      "ADC health wire layout must match the APU");
	static_assert(sizeof(msap1_meter_config_payload) == 92,
		      "meter configuration wire layout must match the APU");
	static_assert(sizeof(msap1_adc_diagnostic_payload) == 188,
		      "ADC diagnostic wire layout must match the APU");
	static_assert(sizeof(msap1_rpu_msg_header) +
		      sizeof(msap1_adc_health_payload) <= MSAP1_RPU_MAX_FRAME_SIZE,
		      "ADC health response exceeds protocol frame size");
	static_assert(sizeof(msap1_rpu_msg_header) +
		      sizeof(msap1_adc_diagnostic_payload) <=
			      MSAP1_RPU_MAX_FRAME_SIZE,
		      "ADC diagnostic response exceeds protocol frame size");

	msap1::LedController led_;
	msap1::adc::Ad7771 &adc_;
	msap1::meter::MeteringPipeline &metering_;
};

#endif /* MSAP1_R5C0_SERVICE_HPP */
