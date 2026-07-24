#include "metering.hpp"

#include "sleep.h"
#include "xil_io.h"

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
	    configuration.valid_mask == 0u ||
	    configuration.frequency.mode > 2u ||
	    configuration.frequency.reference_channel != 6u ||
	    configuration.frequency.averaging_cycles == 0u ||
	    configuration.frequency.averaging_cycles > 64u ||
	    configuration.frequency.window_samples <
		configuration.sample_rate_hz / 10u ||
	    configuration.frequency.window_samples > configuration.sample_rate_hz ||
	    configuration.frequency.minimum_millihz < 10000u ||
	    configuration.frequency.maximum_millihz > 100000u ||
	    configuration.frequency.minimum_millihz >=
		configuration.frequency.maximum_millihz ||
	    configuration.frequency.hysteresis_microvolts == 0u ||
	    configuration.frequency.hysteresis_microvolts > 100000000u)
		return false;

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
	if (!generation_matches || !mask_matches ||
	    conversion_enabled != configuration.enable ||
	    processing_enabled != configuration.enable || !frequency_matches)
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
