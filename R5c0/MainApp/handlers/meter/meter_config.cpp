#include "meter_config.hpp"

#include <array>
#include <cstddef>

namespace msap1::r5c0 {

std::uint32_t apply_meter_config(
	msap1::adc::AdcController &adc,
	msap1::meter::MeteringPipeline &metering,
	const msap1_meter_config_payload &wire,
	msap1_meter_config_ack_payload &acknowledgement)
{
	if ((wire.valid_mask & ~0xffu) != 0u ||
	    wire.adc_source > MSAP1_ADC_SOURCE_SIMULATOR ||
	    (wire.simulator_valid_mask & ~0x7fu) != 0u ||
	    (wire.flags & ~(MSAP1_METER_CONFIG_ENABLE |
			    MSAP1_METER_CONFIG_REMOVE_DC)) != 0u ||
	    (wire.frequency_flags & ~MSAP1_FREQUENCY_CONFIG_ENABLE) != 0u ||
	    (wire.simulator_flags & ~MSAP1_SIMULATOR_FLAG_PRESERVE_PHASE) != 0u)
		return MSAP1_RPU_STATUS_BAD_PAYLOAD;

	msap1::adc::SampleRate sample_rate = msap1::adc::SampleRate::Sps32000;
	if (!msap1::adc::sample_rate_from_hz(wire.sample_rate_hz, sample_rate))
		return MSAP1_RPU_STATUS_BAD_PAYLOAD;

	std::array<msap1::adc::PgaGain, msap1::adc::channel_count> gains{};
	for (std::size_t channel = 0; channel < gains.size(); ++channel) {
		switch (wire.adc_pga_gain[channel]) {
		case 1u: gains[channel] = msap1::adc::PgaGain::X1; break;
		case 2u: gains[channel] = msap1::adc::PgaGain::X2; break;
		case 4u: gains[channel] = msap1::adc::PgaGain::X4; break;
		case 8u: gains[channel] = msap1::adc::PgaGain::X8; break;
		default: return MSAP1_RPU_STATUS_BAD_PAYLOAD;
		}
	}

	if (adc.capture_active())
		return MSAP1_RPU_STATUS_ADC_STATE;

	msap1::adc::Configuration adc_configuration = adc.configuration();
	adc_configuration.sample_rate = sample_rate;
	adc_configuration.pga_gains = gains;
	msap1::adc::SimulatorConfiguration simulator_configuration;
	simulator_configuration.generation = wire.generation;
	simulator_configuration.frequency_millihz =
		wire.simulator_frequency_millihz;
	simulator_configuration.valid_mask = wire.simulator_valid_mask;
	simulator_configuration.phase_step_q32 = wire.simulator_phase_step_q32;
	simulator_configuration.preserve_phase =
		(wire.simulator_flags & MSAP1_SIMULATOR_FLAG_PRESERVE_PHASE) != 0u;
	for (std::size_t channel = 0; channel < msap1::adc::channel_count;
	     ++channel) {
		simulator_configuration.peak_counts[channel] =
			wire.simulator_peak_counts[channel];
		simulator_configuration.phase_q32[channel] =
			wire.simulator_phase_q32[channel];
		simulator_configuration.dc_offset_counts[channel] =
			wire.simulator_dc_offset_counts[channel];
		simulator_configuration.noise_level_counts[channel] =
			wire.simulator_noise_level_counts[channel];
	}
	for (std::size_t word = 0;
	     word < simulator_configuration.harmonic_words.size(); ++word)
		simulator_configuration.harmonic_words[word] =
			wire.simulator_harmonics[word];
	const auto source = wire.adc_source == MSAP1_ADC_SOURCE_SIMULATOR ?
		msap1::adc::Source::Simulator : msap1::adc::Source::Physical;
	const auto adc_error = adc.configure(source, adc_configuration,
					     simulator_configuration);
	if (adc_error != msap1::adc::Error::None)
		return adc_error == msap1::adc::Error::InvalidConfiguration ?
			MSAP1_RPU_STATUS_BAD_PAYLOAD :
			adc_error == msap1::adc::Error::CaptureNotInitialized ?
			MSAP1_RPU_STATUS_ADC_UNAVAILABLE :
			MSAP1_RPU_STATUS_INTERNAL_ERROR;

	msap1::meter::Configuration configuration;
	configuration.generation = wire.generation;
	configuration.sample_rate_hz = wire.sample_rate_hz;
	configuration.rms_window_samples = wire.rms_window_samples;
	configuration.valid_mask = static_cast<std::uint8_t>(wire.valid_mask);
	for (std::size_t channel = 0;
	     channel < configuration.scale_micro_units_q16.size(); ++channel)
		configuration.scale_micro_units_q16[channel] =
			wire.scale_micro_units_q16[channel];
	configuration.enable = (wire.flags & MSAP1_METER_CONFIG_ENABLE) != 0u;
	configuration.remove_dc =
		(wire.flags & MSAP1_METER_CONFIG_REMOVE_DC) != 0u;
	configuration.frequency.enable =
		(wire.frequency_flags & MSAP1_FREQUENCY_CONFIG_ENABLE) != 0u;
	configuration.frequency.mode = wire.frequency_mode;
	configuration.frequency.reference_channel =
		wire.frequency_reference_channel;
	configuration.frequency.averaging_cycles =
		wire.frequency_averaging_cycles;
	configuration.frequency.window_samples = wire.frequency_window_samples;
	configuration.frequency.minimum_millihz =
		wire.frequency_minimum_millihz;
	configuration.frequency.maximum_millihz =
		wire.frequency_maximum_millihz;
	configuration.frequency.hysteresis_microvolts =
		wire.frequency_hysteresis_microvolts;
	configuration.nominal_frequency_hz = wire.nominal_frequency_hz;
	configuration.power_quality.reference_microvolts =
		wire.pq_reference_microvolts;
	configuration.power_quality.sag_threshold_e4 = wire.pq_sag_threshold_e4;
	configuration.power_quality.swell_threshold_e4 =
		wire.pq_swell_threshold_e4;
	configuration.power_quality.interruption_threshold_e4 =
		wire.pq_interruption_threshold_e4;
	configuration.power_quality.hysteresis_e4 = wire.pq_hysteresis_e4;

	const auto error = metering.configure(configuration);
	if (error != msap1::meter::Error::None)
		return error == msap1::meter::Error::CoreNotFound ?
			MSAP1_RPU_STATUS_METER_UNAVAILABLE :
			MSAP1_RPU_STATUS_METER_CONFIG;

	const auto meter = metering.status();
	acknowledgement = {};
	acknowledgement.generation = configuration.generation;
	acknowledgement.conversion_active_generation =
		meter.conversion_active_generation;
	acknowledgement.processing_active_generation =
		meter.processing_active_generation;
	acknowledgement.conversion_status = meter.conversion_status;
	acknowledgement.processing_status = meter.processing_status;
	acknowledgement.adc_source = static_cast<std::uint32_t>(adc.source());
	acknowledgement.simulator_active_generation =
		adc.simulator().status().active_generation;
	return MSAP1_RPU_STATUS_OK;
}

} // namespace msap1::r5c0
