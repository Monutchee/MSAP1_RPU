#include "adc_health.hpp"

#include <cstddef>

namespace msap1::r5c0 {

void build_adc_health(msap1::adc::AdcController &adc,
		      msap1::meter::MeteringPipeline &metering,
		      msap1_adc_health_payload &health)
{
	health = {};
	const auto device = adc.status();
	const auto &capture = device.capture;
	health.adc_source = static_cast<std::uint32_t>(device.source);
	health.simulator_status = device.simulator_status;
	health.simulator_active_generation = device.active_generation;
	health.simulator_frame_count = capture.frames;
	health.simulator_saturation_count = device.saturation_count;
	health.simulator_missed_sample_count = device.missed_sample_count;
	health.sample_rate_hz = msap1::adc::sample_rate_hz(
		adc.configuration().sample_rate);
	health.capture_flags = capture.flags;
	health.frame_count = capture.frames;
	health.overflow_count = capture.overflows;
	health.header_error_count = capture.header_errors;
	health.alert_count = capture.alerts;
	health.packet_count = capture.packets;
	health.dclk_frequency_hz = capture.dclk_frequency_hz;
	health.drdy_frequency_hz = capture.drdy_frequency_hz;
	if (device.initialized)
		health.health_flags |= MSAP1_ADC_HEALTH_INITIALIZED;
	if (device.capture_active)
		health.health_flags |= MSAP1_ADC_HEALTH_CAPTURE_ACTIVE;
	if (capture.overflows == 0u)
		health.health_flags |= MSAP1_ADC_HEALTH_NO_OVERFLOW;
	if (capture.frames != 0u && capture.header_errors == 0u)
		health.health_flags |= MSAP1_ADC_HEALTH_HEADERS_VALID;
	/* Health is not valid until the physically measured DRDY rate matches
	 * the configured sample rate within 1% + 2 Hz. */
	const auto rate_difference =
		capture.drdy_frequency_hz > health.sample_rate_hz ?
		capture.drdy_frequency_hz - health.sample_rate_hz :
		health.sample_rate_hz - capture.drdy_frequency_hz;
	if (capture.drdy_frequency_hz != 0u &&
	    rate_difference <= health.sample_rate_hz / 100u + 2u)
		health.health_flags |= MSAP1_ADC_HEALTH_RATE_MATCH;

	if (device.source == msap1::adc::Source::Simulator) {
		/* Physical SPI diagnostics are not applicable while the PL
		 * simulator feeds the raw stream. */
		health.spi_error = MSAP1_ADC_SPI_HEALTH_NOT_APPLICABLE;
		if (device.initialized && device.configuration_matches &&
		    device.saturation_count == 0u &&
		    device.missed_sample_count == 0u)
			health.health_flags |=
				MSAP1_ADC_HEALTH_SIMULATOR_HEALTHY |
				MSAP1_ADC_HEALTH_CONFIG_MATCH;
	} else {
		health.health_flags |= MSAP1_ADC_HEALTH_PHYSICAL_DIAGNOSTICS;
		msap1::adc::RegisterHealth registers;
		const auto error =
			adc.physical().read_register_health(registers);
		switch (error) {
		case msap1::adc::Error::None:
			health.spi_error = MSAP1_ADC_SPI_HEALTH_OK;
			health.health_flags |= MSAP1_ADC_HEALTH_SPI_RESPONSIVE;
			break;
		case msap1::adc::Error::SpiInitialization:
			health.spi_error = MSAP1_ADC_SPI_HEALTH_NOT_INITIALIZED;
			break;
		case msap1::adc::Error::SpiTransfer:
			health.spi_error = MSAP1_ADC_SPI_HEALTH_TRANSFER_FAILED;
			break;
		case msap1::adc::Error::SpiProtocol:
			health.spi_error = MSAP1_ADC_SPI_HEALTH_PROTOCOL_FAILED;
			break;
		default:
			health.spi_error = MSAP1_ADC_SPI_HEALTH_INTERNAL_ERROR;
			break;
		}
		/* Report the register mirror even after a failed sweep: the
		 * partial bytes plus spi_error tell Linux what the probe saw. */
		health.expected_decimation = registers.expected_decimation;
		health.status_3 = registers.status_3;
		health.general_user_config_1 = registers.general_user_config_1;
		health.general_user_config_2 = registers.general_user_config_2;
		health.general_user_config_3 = registers.general_user_config_3;
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
		health.channel_error_enable = registers.channel_error_enable;
		health.general_error_1 = registers.general_error_1;
		health.general_error_1_enable =
			registers.general_error_1_enable;
		health.general_error_2 = registers.general_error_2;
		health.general_error_2_enable =
			registers.general_error_2_enable;
		health.status_1 = registers.status_1;
		health.status_2 = registers.status_2;
		const auto &spi_diagnostics =
			adc.physical().spi_health_diagnostics();
		health.spi_protocol_error_count =
			spi_diagnostics.protocol_error_count;
		health.spi_retry_recovery_count =
			spi_diagnostics.retry_recovery_count;
		health.spi_last_failed_register =
			spi_diagnostics.last_failed_register;
		health.spi_last_received_header =
			spi_diagnostics.last_received_header;
		if ((registers.status_3 & (1u << 4)) != 0u)
			health.health_flags |= MSAP1_ADC_HEALTH_INIT_COMPLETE;
		if (registers.configuration_matches)
			health.health_flags |= MSAP1_ADC_HEALTH_CONFIG_MATCH;
	}

	const auto meter = metering.status();
	health.meter_generation = meter.generation;
	health.conversion_status = meter.conversion_status;
	health.processing_status = meter.processing_status;
	if (meter.cores_present)
		health.meter_health_flags |= MSAP1_METER_HEALTH_CORES_PRESENT;
	if (meter.configured)
		health.meter_health_flags |= MSAP1_METER_HEALTH_CONFIGURED;
	if (meter.generation_matches)
		health.meter_health_flags |=
			MSAP1_METER_HEALTH_GENERATION_MATCH;
	if (meter.enabled)
		health.meter_health_flags |= MSAP1_METER_HEALTH_ENABLED;
	if (meter.remove_dc)
		health.meter_health_flags |= MSAP1_METER_HEALTH_REMOVE_DC;
}

} // namespace msap1::r5c0
