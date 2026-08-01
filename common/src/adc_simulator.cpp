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

constexpr std::uint32_t simulator_id = 0x53494d31u; // SIM1
constexpr std::uint32_t supported_major_version = 1u;
constexpr std::uint32_t control_source_simulator = 1u << 0;
constexpr std::uint32_t control_enable = 1u << 1;
constexpr std::uint32_t apply_command = 1u;
constexpr unsigned int readback_attempts = 1000;

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
	return read(reg_id) == simulator_id &&
	       (read(reg_version) >> 16) == supported_major_version;
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
		(enabled ? control_enable : 0u);
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
	}
	write(reg_shadow_phase_step, simulator.phase_step_q32);

	const auto error = apply(select_source, false);
	if (error != Error::None)
		return error;
	const bool matches =
		read(reg_active_sample_rate) == sample_rate_hz(configuration.sample_rate) &&
		read(reg_active_frequency) == simulator.frequency_millihz &&
		(read(reg_active_valid_mask) & 0xffu) ==
			(simulator.valid_mask & 0x7fu) &&
		read(reg_active_generation) == simulator.generation &&
		read(reg_active_phase_step) == simulator.phase_step_q32;
	if (!matches)
		return Error::AdcRegisterMismatch;

	configuration_ = configuration;
	simulator_configuration_ = simulator;
	return Error::None;
}

Error AdcSimulator::select_source(bool selected)
{
	return apply(selected, false);
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
