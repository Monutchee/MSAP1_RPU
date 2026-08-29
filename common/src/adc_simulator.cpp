#include "adc_simulator.hpp"

#include "sleep.h"
#include "xil_io.h"

namespace msap1::adc {
namespace {

constexpr std::uint32_t reg_id = 0x00;
constexpr std::uint32_t reg_version = 0x04;
constexpr std::uint32_t reg_shadow_control = 0x08;
constexpr std::uint32_t reg_shadow_sample_rate = 0x0c;
constexpr std::uint32_t reg_shadow_frequency = 0x10;
constexpr std::uint32_t reg_shadow_valid_mask = 0x14;
constexpr std::uint32_t reg_shadow_generation = 0x18;
constexpr std::uint32_t reg_apply = 0x1c;
constexpr std::uint32_t reg_status = 0x20;
constexpr std::uint32_t reg_active_sample_rate = 0x24;
constexpr std::uint32_t reg_active_frequency = 0x28;
constexpr std::uint32_t reg_active_valid_mask = 0x2c;
constexpr std::uint32_t reg_active_generation = 0x30;
constexpr std::uint32_t reg_frame_count = 0x34;
constexpr std::uint32_t reg_saturation_count = 0x38;
constexpr std::uint32_t reg_missed_sample_count = 0x3c;
constexpr std::uint32_t reg_shadow_peak_base = 0x40;
constexpr std::uint32_t reg_shadow_phase_base = 0x60;
constexpr std::uint32_t reg_shadow_phase_step = 0x80;
constexpr std::uint32_t reg_active_control = 0x84;
constexpr std::uint32_t reg_active_phase_step = 0x88;
constexpr std::uint32_t reg_shadow_dc_base = 0x8c;
constexpr std::uint32_t reg_counter_clear = 0xac;
constexpr std::uint32_t reg_active_dc_base = 0xb0;
constexpr std::uint32_t reg_shadow_noise_base = 0xd0;
constexpr std::uint32_t reg_active_noise_base = 0x100;
constexpr std::uint32_t reg_shadow_harmonic_base = 0x200;
constexpr std::uint32_t reg_active_harmonic_base = 0x240;
constexpr std::uint32_t reg_shadow_m18_base = 0x280;
constexpr std::uint32_t reg_active_m18_base = 0x2c0;
constexpr std::uint32_t reg_shadow_event_control = 0x300;
constexpr std::uint32_t reg_shadow_event_scale = 0x304;
constexpr std::uint32_t reg_shadow_event_timing = 0x308;
constexpr std::uint32_t reg_event_trigger = 0x30c;
constexpr std::uint32_t reg_event_status = 0x310;
constexpr std::uint32_t reg_event_remaining = 0x314;
constexpr std::uint32_t reg_active_event_control = 0x318;
constexpr std::uint32_t reg_active_event_scale = 0x31c;
constexpr std::uint32_t reg_active_event_timing = 0x320;

constexpr std::uint32_t simulator_id = 0x53494d31u; // SIM1
constexpr std::uint32_t supported_major_version = 1u;
/* 1.1: DC offset, noise, preserve-phase, counter clears. 1.2: the four
 * global harmonic slots. 1.3: event sequencer. 1.4: Q16.16 slot ratios.
 * 1.5: deterministic AM and absolute carrier/adjacent tones. */
constexpr std::uint32_t required_minor_version = 5u;
constexpr std::uint32_t control_source_simulator = 1u << 0;
constexpr std::uint32_t control_enable = 1u << 1;
constexpr std::uint32_t control_preserve_phase = 1u << 2;
constexpr std::uint32_t apply_command = 1u;
constexpr std::uint32_t counter_clear_all = 0x7u;
/* Event sequencer: the burst's channel mask sits in [7:0] and the repeat
 * bit at 8; the trigger register's strobes are write-1, read-zero. */
constexpr std::uint32_t event_repeat_bit = 1u << 8;
constexpr std::uint32_t event_trigger_arm = 1u << 0;
constexpr std::uint32_t event_trigger_cancel = 1u << 1;
constexpr std::uint32_t event_trigger_clear = 1u << 2;
/* Amplitude scale cap mirrored from the PL (adc_simulator_pkg): 4.0 in
 * Q16. The PL clamps too; rejecting here turns a typo into a rejected
 * request instead of a silently different scenario. */
constexpr std::uint32_t event_scale_max = 0x40000u;
constexpr unsigned int readback_attempts = 1000;

std::uint32_t phase_step_q32(std::uint32_t frequency_millihz,
			     std::uint32_t sample_rate)
{
	if (frequency_millihz == 0u || sample_rate == 0u)
		return 0u;
	const std::uint64_t denominator =
		static_cast<std::uint64_t>(sample_rate) * 1000u;
	const std::uint64_t numerator =
		static_cast<std::uint64_t>(frequency_millihz) << 32;
	return static_cast<std::uint32_t>(
		(numerator + denominator / 2u) / denominator);
}

bool valid(const Configuration &configuration,
	   const SimulatorConfiguration &simulator)
{
	const auto rate = sample_rate_hz(configuration.sample_rate);
	if (rate < 1000u || rate > 128000u || simulator.generation == 0u ||
	    simulator.frequency_millihz == 0u ||
	    (simulator.valid_mask & ~0x7fu) != 0u ||
	    simulator.phase_step_q32 == 0u)
		return false;
	for (const auto peak : simulator.peak_counts) {
		if (peak < -8388608 || peak > 8388607)
			return false;
	}
	for (const auto dc : simulator.dc_offset_counts) {
		if (dc < -8388608 || dc > 8388607)
			return false;
	}
	for (const auto noise : simulator.noise_level_counts) {
		if (noise > 8388607u)
			return false;
	}
	for (std::size_t slot = 0; slot < 4; ++slot) {
		const auto ratio_q16 = simulator.harmonic_words[slot * 3];
		const auto control = simulator.harmonic_words[slot * 3 + 1];
		const auto phase = simulator.harmonic_words[slot * 3 + 2];
		if (ratio_q16 == 0u) {
			if (control != 0u || phase != 0u)
				return false;
			continue;
		}
		/* Ratios at or below 1 duplicate/subdivide the fundamental; M16
		 * slots cover harmonic/interharmonic content above it through 127. */
		if (ratio_q16 <= 0x00010000u || ratio_q16 >= 0x00800000u ||
		    (control & 0x0000ff00u) != 0u || (control & 0x7fu) == 0u ||
		    (control & 0x80u) != 0u)
			return false;
		const std::uint64_t tone_millihz =
			(static_cast<std::uint64_t>(ratio_q16) *
			 simulator.frequency_millihz) >> 16;
		if (tone_millihz * 2u >= static_cast<std::uint64_t>(rate) * 1000u)
			return false;
	}
	if ((simulator.am_channel_mask & ~0x7fu) != 0u ||
	    simulator.am_depth_q16 > 0x10000u)
		return false;
	if (simulator.am_frequency_millihz == 0u) {
		if (simulator.am_depth_q16 != 0u || simulator.am_channel_mask != 0u)
			return false;
	} else if (simulator.am_depth_q16 == 0u ||
		   simulator.am_channel_mask == 0u ||
		   simulator.am_frequency_millihz >= 1000000u ||
		   static_cast<std::uint64_t>(simulator.am_frequency_millihz) * 2u >=
			static_cast<std::uint64_t>(rate) * 1000u) {
		return false;
	}
	if ((simulator.carrier_phase_mask & ~0x70u) != 0u ||
	    simulator.carrier_fraction_q16 >= 0x10000u ||
	    simulator.adjacent_fraction_q16 >= 0x10000u)
		return false;
	if (simulator.carrier_frequency_millihz == 0u) {
		if (simulator.carrier_fraction_q16 != 0u ||
		    simulator.carrier_phase_mask != 0u ||
		    simulator.carrier_phase_q32 != 0u ||
		    simulator.adjacent_frequency_millihz != 0u ||
		    simulator.adjacent_fraction_q16 != 0u ||
		    simulator.adjacent_phase_q32 != 0u)
			return false;
	} else {
		if (simulator.carrier_fraction_q16 == 0u ||
		    simulator.carrier_phase_mask == 0u ||
		    static_cast<std::uint64_t>(
			    simulator.carrier_frequency_millihz) * 2u >=
			    static_cast<std::uint64_t>(rate) * 1000u)
			return false;
		if (simulator.adjacent_frequency_millihz == 0u) {
			if (simulator.adjacent_fraction_q16 != 0u ||
			    simulator.adjacent_phase_q32 != 0u)
				return false;
		} else if (simulator.adjacent_fraction_q16 == 0u ||
			   static_cast<std::uint64_t>(
				   simulator.adjacent_frequency_millihz) * 2u >=
				   static_cast<std::uint64_t>(rate) * 1000u) {
			return false;
		}
	}
	return true;
}

} // namespace

AdcSimulator::AdcSimulator(SimulatorHardware hardware) : hardware_(hardware) {}

std::uint32_t AdcSimulator::read(std::uint32_t offset) const
{
	return Xil_In32(hardware_.base + offset);
}

void AdcSimulator::write(std::uint32_t offset, std::uint32_t value) const
{
	Xil_Out32(hardware_.base + offset, value);
}

bool AdcSimulator::core_present() const
{
	const auto version = read(reg_version);
	return read(reg_id) == simulator_id &&
	       (version >> 16) == supported_major_version &&
	       (version & 0xffffu) >= required_minor_version;
}

Error AdcSimulator::initialize(const Configuration &configuration)
{
	if (!core_present())
		return Error::CaptureCoreNotFound;
	configuration_ = configuration;
	initialized_ = true;
	capture_active_ = false;
	selected_ = false;
	return Error::None;
}

Error AdcSimulator::apply(bool selected, bool enabled)
{
	if (!initialized_)
		return Error::CaptureNotInitialized;

	const auto control = (selected ? control_source_simulator : 0u) |
		(enabled ? control_enable : 0u) |
		(simulator_configuration_.preserve_phase ?
			control_preserve_phase : 0u);
	write(reg_shadow_control, control);
	write(reg_apply, apply_command);
	for (unsigned int attempt = 0; attempt < readback_attempts; ++attempt) {
		if (read(reg_active_control) == control)
			break;
		usleep(1);
	}
	if (read(reg_active_control) != control)
		return Error::AdcRegisterMismatch;
	selected_ = selected;
	capture_active_ = selected && enabled;
	return Error::None;
}

Error AdcSimulator::configure(const Configuration &configuration,
			      const SimulatorConfiguration &simulator,
			      bool select_source)
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	if (capture_active_)
		return Error::CaptureAlreadyActive;
	if (!valid(configuration, simulator))
		return Error::InvalidConfiguration;

	write(reg_shadow_sample_rate, sample_rate_hz(configuration.sample_rate));
	write(reg_shadow_frequency, simulator.frequency_millihz);
	write(reg_shadow_valid_mask, simulator.valid_mask & 0x7fu);
	write(reg_shadow_generation, simulator.generation);
	for (std::size_t channel = 0; channel < channel_count; ++channel) {
		write(reg_shadow_peak_base + channel * 4u,
		      static_cast<std::uint32_t>(simulator.peak_counts[channel]));
		write(reg_shadow_phase_base + channel * 4u,
		      simulator.phase_q32[channel]);
		write(reg_shadow_dc_base + channel * 4u,
		      static_cast<std::uint32_t>(simulator.dc_offset_counts[channel]));
		write(reg_shadow_noise_base + channel * 4u,
		      simulator.noise_level_counts[channel]);
	}
	for (std::size_t word = 0; word < simulator.harmonic_words.size(); ++word)
		write(reg_shadow_harmonic_base + word * 4u,
		      simulator.harmonic_words[word]);
	const std::array<std::uint32_t, 10> m18_words{
		phase_step_q32(simulator.am_frequency_millihz,
			       sample_rate_hz(configuration.sample_rate)),
		simulator.am_depth_q16,
		simulator.am_channel_mask,
		phase_step_q32(simulator.carrier_frequency_millihz,
			       sample_rate_hz(configuration.sample_rate)),
		simulator.carrier_fraction_q16,
		simulator.carrier_phase_mask,
		simulator.carrier_phase_q32,
		phase_step_q32(simulator.adjacent_frequency_millihz,
			       sample_rate_hz(configuration.sample_rate)),
		simulator.adjacent_fraction_q16,
		simulator.adjacent_phase_q32,
	};
	for (std::size_t word = 0; word < m18_words.size(); ++word)
		write(reg_shadow_m18_base + word * 4u, m18_words[word]);
	write(reg_shadow_phase_step, simulator.phase_step_q32);

	/* apply() folds the preserve-phase level into CONTROL. */
	simulator_configuration_.preserve_phase = simulator.preserve_phase;
	const auto error = apply(select_source, false);
	if (error != Error::None)
		return error;
	bool matches =
		read(reg_active_sample_rate) == sample_rate_hz(configuration.sample_rate) &&
		read(reg_active_frequency) == simulator.frequency_millihz &&
		(read(reg_active_valid_mask) & 0xffu) ==
			(simulator.valid_mask & 0x7fu) &&
		read(reg_active_generation) == simulator.generation &&
		read(reg_active_phase_step) == simulator.phase_step_q32;
	for (std::size_t channel = 0; matches && channel < channel_count;
	     ++channel) {
		matches =
			read(reg_active_dc_base + channel * 4u) ==
				static_cast<std::uint32_t>(
					simulator.dc_offset_counts[channel]) &&
			read(reg_active_noise_base + channel * 4u) ==
				simulator.noise_level_counts[channel];
	}
	for (std::size_t word = 0; matches && word < simulator.harmonic_words.size();
	     ++word)
		matches = read(reg_active_harmonic_base + word * 4u) ==
			simulator.harmonic_words[word];
	for (std::size_t word = 0; matches && word < m18_words.size(); ++word)
		matches = read(reg_active_m18_base + word * 4u) == m18_words[word];
	if (!matches)
		return Error::AdcRegisterMismatch;

	/* Each committed scenario starts with clean per-scenario counters
	 * (saturation, missed samples, frames). */
	write(reg_counter_clear, counter_clear_all);

	configuration_ = configuration;
	simulator_configuration_ = simulator;
	return Error::None;
}

Error AdcSimulator::select_source(bool selected)
{
	return apply(selected, false);
}

Error AdcSimulator::arm_event(const SimulatorEvent &event)
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	/* A burst with no channels, no length, or an out-of-range scale is
	 * a malformed scenario, not a quiet no-op: reject it so the
	 * operator sees the mistake instead of a stream that never dips. */
	if ((event.channel_mask & ~0xffu) != 0u || event.channel_mask == 0u ||
	    event.duration_half_cycles == 0u ||
	    event.duration_half_cycles > 0xffffu ||
	    event.period_half_cycles > 0xffffu ||
	    event.scale_q16 > event_scale_max)
		return Error::InvalidConfiguration;

	const auto control = (event.channel_mask & 0xffu) |
		(event.repeat ? event_repeat_bit : 0u);
	const auto timing = (event.duration_half_cycles & 0xffffu) |
		((event.period_half_cycles & 0xffffu) << 16);
	write(reg_shadow_event_control, control);
	write(reg_shadow_event_scale, event.scale_q16);
	write(reg_shadow_event_timing, timing);
	write(reg_event_trigger, event_trigger_arm);
	/* The trigger latches the shadow bank in the same cycle it arms, so
	 * the active readback is the confirmation that the burst the PL
	 * will run is the burst that was asked for. */
	if (read(reg_active_event_control) != control ||
	    read(reg_active_event_scale) != event.scale_q16 ||
	    read(reg_active_event_timing) != timing)
		return Error::AdcRegisterMismatch;
	return Error::None;
}

Error AdcSimulator::cancel_event()
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	write(reg_event_trigger, event_trigger_cancel);
	return Error::None;
}

Error AdcSimulator::clear_event_count()
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	write(reg_event_trigger, event_trigger_clear);
	return Error::None;
}

SimulatorEventStatus AdcSimulator::event_status() const
{
	SimulatorEventStatus result;
	if (!initialized_)
		return result;
	result.status = read(reg_event_status);
	result.remaining = read(reg_event_remaining);
	result.active_control = read(reg_active_event_control);
	result.active_scale = read(reg_active_event_scale);
	result.active_timing = read(reg_active_event_timing);
	return result;
}

Error AdcSimulator::start_capture()
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	if (capture_active_)
		return Error::CaptureAlreadyActive;
	return apply(true, true);
}

void AdcSimulator::stop_capture()
{
	if (initialized_)
		(void)apply(selected_, false);
}

Error AdcSimulator::configure_operating_point(
	SampleRate sample_rate,
	const std::array<PgaGain, channel_count> &channel_gains)
{
	auto configuration = configuration_;
	configuration.sample_rate = sample_rate;
	configuration.pga_gains = channel_gains;
	return configure(configuration, simulator_configuration_, selected_);
}

DeviceStatus AdcSimulator::status() const
{
	DeviceStatus result;
	result.source = Source::Simulator;
	result.initialized = initialized_ && core_present();
	if (!result.initialized)
		return result;
	const auto active_control = read(reg_active_control);
	result.capture_active = (active_control &
		(control_source_simulator | control_enable)) ==
		(control_source_simulator | control_enable);
	result.active_generation = read(reg_active_generation);
	result.simulator_status = read(reg_status);
	result.saturation_count = read(reg_saturation_count);
	result.missed_sample_count = read(reg_missed_sample_count);
	result.configuration_matches =
		result.active_generation == simulator_configuration_.generation &&
		read(reg_active_sample_rate) == sample_rate_hz(configuration_.sample_rate) &&
		read(reg_active_frequency) == simulator_configuration_.frequency_millihz &&
		(read(reg_active_valid_mask) & 0xffu) ==
			(simulator_configuration_.valid_mask & 0x7fu) &&
		read(reg_active_phase_step) == simulator_configuration_.phase_step_q32;
	result.capture.frames = read(reg_frame_count);
	result.capture.packets = result.capture.frames /
		configuration_.frames_per_packet;
	result.capture.drdy_frequency_hz = result.capture_active ?
		sample_rate_hz(configuration_.sample_rate) : 0u;
	return result;
}

} // namespace msap1::adc
