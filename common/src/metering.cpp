#include "metering.hpp"

#include "power_quality_configuration.hpp"

#include "sleep.h"
#include "xil_io.h"

#include <array>
#include <cstring>

namespace msap1::meter {
namespace {

constexpr std::uint32_t version = 0x00;
constexpr std::uint32_t identifier = 0x04;
constexpr std::uint32_t control = 0x08;
constexpr std::uint32_t status_register = 0x0c;

constexpr std::uint32_t conversion_shadow_generation = 0x10;
constexpr std::uint32_t conversion_shadow_valid_mask = 0x14;
constexpr std::uint32_t conversion_shadow_scale = 0x18;
constexpr std::uint32_t conversion_active_generation = 0x38;
constexpr std::uint32_t conversion_active_valid_mask = 0x3c;

constexpr std::uint32_t processing_shadow_generation = 0x10;
constexpr std::uint32_t processing_shadow_sample_rate = 0x14;
constexpr std::uint32_t processing_shadow_window_samples = 0x18;
constexpr std::uint32_t processing_shadow_valid_mask = 0x1c;
constexpr std::uint32_t processing_active_generation = 0x20;
constexpr std::uint32_t processing_frequency_shadow_control = 0x30;
constexpr std::uint32_t processing_frequency_shadow_window = 0x34;
constexpr std::uint32_t processing_frequency_shadow_minimum = 0x38;
constexpr std::uint32_t processing_frequency_shadow_maximum = 0x3c;
constexpr std::uint32_t processing_frequency_shadow_hysteresis = 0x40;
constexpr std::uint32_t processing_frequency_active_control = 0x44;
constexpr std::uint32_t processing_frequency_active_window = 0x48;
constexpr std::uint32_t processing_frequency_active_minimum = 0x4c;
constexpr std::uint32_t processing_frequency_active_maximum = 0x50;
constexpr std::uint32_t processing_frequency_active_hysteresis = 0x54;
constexpr std::uint32_t processing_frequency_status = 0x58;
constexpr std::uint32_t processing_grid_shadow_config = 0x6c;
constexpr std::uint32_t processing_grid_active_config = 0x70;
constexpr std::uint32_t processing_grid_status = 0x74;
constexpr std::uint32_t processing_agg_status = 0x78;
constexpr std::uint32_t processing_agg_record_count = 0x7c;
constexpr std::uint32_t processing_agg_reset_count = 0x80;
constexpr std::uint32_t processing_agg_ineligible_count = 0x84;
constexpr std::uint32_t processing_agg_continuity_count = 0x88;
constexpr std::uint32_t processing_agg_drop_count = 0x8c;
// Urms(1/2) event detection (metrology M12, PL pq_event_pkg). These
// shadow registers commit on the same CONTROL apply toggle as the RMS,
// frequency, and grid configuration. There is deliberately NO active
// readback: the PL engine holds the committed thresholds internally and
// echoes them in every PQ record (words 32..36), which is a stronger
// check than a register mirror -- the value that reaches the consumer is
// the value the record was evaluated against.
constexpr std::uint32_t processing_pq_shadow_reference = 0xa0;
constexpr std::uint32_t processing_pq_shadow_threshold = 0xa4;
constexpr std::uint32_t processing_pq_shadow_limits = 0xa8;
constexpr std::uint32_t processing_pq_status = 0xac;
constexpr std::uint32_t processing_m18_config_index = 0xf4;
constexpr std::uint32_t processing_m18_config_data = 0xf8;
constexpr std::uint32_t processing_m18_config_status = 0xfc;
constexpr std::size_t m18_config_words = 79u;
static_assert(sizeof(msap1_m18_config_payload) == m18_config_words * 4u);

constexpr std::uint32_t conversion_identifier = 0x41435631u; // "ACV1"
constexpr std::uint32_t processing_identifier = 0x4d505231u; // "MPR1"
constexpr std::uint32_t supported_major_version = 1u;
constexpr std::uint32_t control_apply = 1u << 0;
constexpr std::uint32_t control_enable = 1u << 1;
constexpr std::uint32_t control_remove_dc = 1u << 2;
constexpr unsigned int readback_attempts = 1000;

bool valid_configuration(const Configuration &configuration)
{
	if (configuration.generation == 0u ||
	    configuration.sample_rate_hz < 1000u ||
	    configuration.sample_rate_hz > 128000u ||
	    configuration.rms_window_samples == 0u ||
	    configuration.rms_window_samples >
		configuration.sample_rate_hz * 10u ||
	    (configuration.nominal_frequency_hz != 50u &&
	     configuration.nominal_frequency_hz != 60u) ||
	    configuration.valid_mask == 0u ||
	    configuration.frequency.mode > 2u ||
	    configuration.frequency.reference_channel != 6u ||
	    configuration.frequency.averaging_cycles == 0u ||
	    configuration.frequency.averaging_cycles > 64u ||
	    configuration.frequency.window_samples <
		configuration.sample_rate_hz / 10u ||
	    configuration.frequency.window_samples > configuration.sample_rate_hz ||
	    configuration.frequency.minimum_millihz < 10000u ||
	    configuration.frequency.maximum_millihz > 200000u ||
	    configuration.frequency.minimum_millihz >=
		configuration.frequency.maximum_millihz ||
	    configuration.frequency.hysteresis_microvolts == 0u ||
	    configuration.frequency.hysteresis_microvolts > 100000000u)
		return false;

	// Event detection: a zero reference is the documented DISARMED
	// state, but a configured reference with a nonsensical band would
	// declare events that mean nothing. Every threshold is a 1e-4
	// fraction, so it must fit 16 bits, and the band must be ordered
	// interruption < sag < swell with room for the hysteresis to sit
	// inside the sag threshold.
	{
		const auto &pq = configuration.power_quality;
		if (pq.sag_threshold_e4 > 0xffffu ||
		    pq.swell_threshold_e4 > 0xffffu ||
		    pq.interruption_threshold_e4 > 0xffffu ||
		    pq.hysteresis_e4 > 0xffffu)
			return false;
		if (pq.reference_microvolts != 0u &&
		    (pq.interruption_threshold_e4 >= pq.sag_threshold_e4 ||
		     pq.sag_threshold_e4 >= pq.swell_threshold_e4 ||
		     pq.hysteresis_e4 >= pq.sag_threshold_e4))
			return false;
	}

	for (std::size_t channel = 0; channel < channel_count; ++channel) {
		if ((configuration.valid_mask & (1u << channel)) != 0u &&
		    configuration.scale_micro_units_q16[channel] == 0u)
			return false;
	}
	return true;
}

std::uint32_t frequency_control(const Configuration::Frequency &frequency)
{
	return (frequency.enable ? 1u : 0u) |
		((frequency.mode & 0x7u) << 1) |
		((frequency.reference_channel & 0xfu) << 4) |
		((frequency.averaging_cycles & 0xffu) << 8);
}

// IEC 61000-4-30 basic measurement blocks span 10 cycles at 50 Hz and
// 12 cycles at 60 Hz. The cycle count never travels on the wire; it is
// derived here from the validated nominal frequency.
constexpr std::uint32_t cycles_per_block(std::uint32_t nominal_frequency_hz)
{
	return nominal_frequency_hz == 50u ? 10u : 12u;
}

// The PQ shadow words pack two 1e-4 fractions each, mirroring the PL's
// pq_event_pkg layout.
std::uint32_t pq_threshold(const Configuration::PowerQuality &pq)
{
	return (pq.sag_threshold_e4 & 0xffffu) |
		((pq.swell_threshold_e4 & 0xffffu) << 16);
}

std::uint32_t pq_limits(const Configuration::PowerQuality &pq)
{
	return (pq.interruption_threshold_e4 & 0xffffu) |
		((pq.hysteresis_e4 & 0xffffu) << 16);
}

std::uint32_t grid_config(const Configuration &configuration)
{
	// Cycle timing is always requested; the PL falls back to the
	// sample-count window on its own when the voltage reference is lost.
	return (cycles_per_block(configuration.nominal_frequency_hz) & 0xffu) |
		((configuration.nominal_frequency_hz & 0xffu) << 8) |
		(1u << 16);
}

} // namespace

const char *to_string(Error error)
{
	switch (error) {
	case Error::None: return "none";
	case Error::InvalidConfiguration: return "invalid meter configuration";
	case Error::CoreNotFound: return "PL metering core not found";
	case Error::ReadbackMismatch: return "PL metering readback mismatch";
	}
	return "unknown";
}

MeteringPipeline::MeteringPipeline(Hardware hardware) : hardware_(hardware) {}

Error MeteringPipeline::stage_power_quality_configuration(
	const msap1_m18_config_payload &configuration)
{
	if (!msap1::power_quality::valid_configuration(configuration))
		return Error::InvalidConfiguration;
	/*
	 * R5C0 owns the validated pending image. The PL register write is completed
	 * by the M18 register-bank integration and committed by the existing meter
	 * APPLY toggle, so the engines can never observe a partial profile array.
	 */
	power_quality_configuration_ = configuration;
	m18_staged_ = true;
	return Error::None;
}

std::uint32_t MeteringPipeline::conversion_read(std::uint32_t offset) const
{
	return Xil_In32(hardware_.conversion_base + offset);
}

void MeteringPipeline::conversion_write(std::uint32_t offset,
					std::uint32_t value) const
{
	Xil_Out32(hardware_.conversion_base + offset, value);
}

std::uint32_t MeteringPipeline::processing_read(std::uint32_t offset) const
{
	return Xil_In32(hardware_.processing_base + offset);
}

void MeteringPipeline::processing_write(std::uint32_t offset,
					std::uint32_t value) const
{
	Xil_Out32(hardware_.processing_base + offset, value);
}

bool MeteringPipeline::cores_present() const
{
	return conversion_read(identifier) == conversion_identifier &&
	       (conversion_read(version) >> 16) == supported_major_version &&
	       processing_read(identifier) == processing_identifier &&
	       (processing_read(version) >> 16) == supported_major_version;
}

Error MeteringPipeline::configure(const Configuration &configuration)
{
	if (!valid_configuration(configuration))
		return Error::InvalidConfiguration;
	if (m18_staged_ &&
	    (power_quality_configuration_.generation != configuration.generation ||
	     !msap1::power_quality::valid_configuration(
		power_quality_configuration_, configuration.sample_rate_hz)))
		return Error::InvalidConfiguration;
	if (!cores_present())
		return Error::CoreNotFound;

	configured_ = false;
	conversion_write(conversion_shadow_generation,
			 configuration.generation);
	conversion_write(conversion_shadow_valid_mask,
			 configuration.valid_mask);
	for (std::size_t channel = 0; channel < channel_count; ++channel)
		conversion_write(conversion_shadow_scale + channel * 4u,
				 configuration.scale_micro_units_q16[channel]);

	processing_write(processing_shadow_generation,
			 configuration.generation);
	processing_write(processing_shadow_sample_rate,
			 configuration.sample_rate_hz);
	processing_write(processing_shadow_window_samples,
			 configuration.rms_window_samples);
	processing_write(processing_shadow_valid_mask,
			 configuration.valid_mask);
	const auto expected_frequency_control =
		frequency_control(configuration.frequency);
	processing_write(processing_frequency_shadow_control,
			 expected_frequency_control);
	processing_write(processing_frequency_shadow_window,
			 configuration.frequency.window_samples);
	processing_write(processing_frequency_shadow_minimum,
			 configuration.frequency.minimum_millihz);
	processing_write(processing_frequency_shadow_maximum,
			 configuration.frequency.maximum_millihz);
	processing_write(processing_frequency_shadow_hysteresis,
			 configuration.frequency.hysteresis_microvolts);
	const auto expected_grid_config = grid_config(configuration);
	processing_write(processing_grid_shadow_config, expected_grid_config);
	processing_write(processing_pq_shadow_reference,
			 configuration.power_quality.reference_microvolts);
	processing_write(processing_pq_shadow_threshold,
			 pq_threshold(configuration.power_quality));
	processing_write(processing_pq_shadow_limits,
			 pq_limits(configuration.power_quality));

	std::array<std::uint32_t, m18_config_words> expected_m18{};
	if (m18_staged_) {
		std::memcpy(expected_m18.data(), &power_quality_configuration_,
			sizeof(power_quality_configuration_));
		if ((processing_read(processing_m18_config_status) >> 16u) !=
			m18_config_words)
			return Error::ReadbackMismatch;
		for (std::size_t word = 0; word < expected_m18.size(); ++word) {
			processing_write(processing_m18_config_index,
				static_cast<std::uint32_t>(word));
			processing_write(processing_m18_config_data,
				expected_m18[word]);
		}
		for (std::size_t word = 0; word < expected_m18.size(); ++word) {
			processing_write(processing_m18_config_index,
				static_cast<std::uint32_t>(word));
			if (processing_read(processing_m18_config_data) !=
				expected_m18[word])
				return Error::ReadbackMismatch;
		}
	}

	const auto conversion_control =
		(configuration.enable ? control_enable : 0u) | control_apply;
	const auto processing_control =
		(configuration.enable ? control_enable : 0u) |
		(configuration.remove_dc ? control_remove_dc : 0u) |
		control_apply;
	conversion_write(control, conversion_control);
	processing_write(control, processing_control);

	for (unsigned int attempt = 0; attempt < readback_attempts; ++attempt) {
		if (conversion_read(conversion_active_generation) ==
			configuration.generation &&
		    processing_read(processing_active_generation) ==
			configuration.generation)
			break;
		usleep(1);
	}

	const bool generation_matches =
		conversion_read(conversion_active_generation) ==
			configuration.generation &&
		processing_read(processing_active_generation) ==
			configuration.generation;
	const bool mask_matches =
		(conversion_read(conversion_active_valid_mask) & 0xffu) ==
			configuration.valid_mask;
	const bool conversion_enabled =
		(conversion_read(status_register) & 1u) != 0u;
	const bool processing_enabled =
		(processing_read(status_register) & 1u) != 0u;
	const bool frequency_matches =
		processing_read(processing_frequency_active_control) ==
			expected_frequency_control &&
		processing_read(processing_frequency_active_window) ==
			configuration.frequency.window_samples &&
		processing_read(processing_frequency_active_minimum) ==
			configuration.frequency.minimum_millihz &&
		processing_read(processing_frequency_active_maximum) ==
			configuration.frequency.maximum_millihz &&
		processing_read(processing_frequency_active_hysteresis) ==
			configuration.frequency.hysteresis_microvolts;
	// Verify the grid config latched like the frequency registers above;
	// an unverified register would silently accept an unlatched value.
	const bool grid_matches =
		processing_read(processing_grid_active_config) ==
			expected_grid_config;
	if (!generation_matches || !mask_matches ||
	    conversion_enabled != configuration.enable ||
	    processing_enabled != configuration.enable || !frequency_matches ||
	    !grid_matches)
		return Error::ReadbackMismatch;

	configuration_ = configuration;
	configured_ = true;
	return Error::None;
}

Status MeteringPipeline::status() const
{
	Status result;
	result.cores_present = cores_present();
	if (!result.cores_present)
		return result;

	result.conversion_active_generation =
		conversion_read(conversion_active_generation);
	result.processing_active_generation =
		processing_read(processing_active_generation);
	result.conversion_status = conversion_read(status_register);
	result.processing_status = processing_read(status_register);
	result.frequency_status = processing_read(processing_frequency_status);
	result.grid_status = processing_read(processing_grid_status);
	result.aggregate_status = processing_read(processing_agg_status);
	result.aggregate_records = processing_read(processing_agg_record_count);
	result.aggregate_resets = processing_read(processing_agg_reset_count);
	result.aggregate_ineligible =
		processing_read(processing_agg_ineligible_count);
	result.aggregate_continuity_errors =
		processing_read(processing_agg_continuity_count);
	result.aggregate_drops = processing_read(processing_agg_drop_count);
	result.generation = configuration_.generation;
	result.configured = configured_;
	result.generation_matches = configured_ &&
		result.conversion_active_generation == configuration_.generation &&
		result.processing_active_generation == configuration_.generation;
	result.enabled = (result.conversion_status & 1u) != 0u &&
		(result.processing_status & 1u) != 0u;
	result.remove_dc = configuration_.remove_dc;
	return result;
}

} // namespace msap1::meter
