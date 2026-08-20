#ifndef MSAP1_ADC_SIMULATOR_HPP
#define MSAP1_ADC_SIMULATOR_HPP

#include "adc_device.hpp"

namespace msap1::adc {

struct SimulatorHardware {
	std::uintptr_t base;
};

/*
 * RPU-side driver for the raw-sample generator inside MeterCore.  The
 * simulator owns no SPI pins: every operation is an AXI-Lite shadow/apply
 * transaction against the PL register bank.
 */
class AdcSimulator final : public AdcDevice {
public:
	explicit AdcSimulator(SimulatorHardware hardware);

	Source source() const override { return Source::Simulator; }
	Error initialize(const Configuration &configuration) override;
	Error start_capture() override;
	void stop_capture() override;
	Error configure_operating_point(
		SampleRate sample_rate,
		const std::array<PgaGain, channel_count> &channel_gains) override;
	DeviceStatus status() const override;
	const Configuration &configuration() const override { return configuration_; }
	bool initialized() const override { return initialized_; }
	bool capture_active() const override { return capture_active_; }

	Error configure(const Configuration &configuration,
			const SimulatorConfiguration &simulator,
			bool select_source);
	Error select_source(bool selected);

	/*
	 * Event sequencer (metrology M12). These deliberately do NOT touch
	 * the waveform shadow bank or APPLY: a burst is armed against a
	 * running configuration and starts on the generator's own
	 * half-cycle boundary, so the only discontinuity in the stream is
	 * the programmed amplitude step.
	 */
	Error arm_event(const SimulatorEvent &event);
	Error cancel_event();
	Error clear_event_count();
	SimulatorEventStatus event_status() const;
	const SimulatorConfiguration &simulator_configuration() const
	{
		return simulator_configuration_;
	}

private:
	std::uint32_t read(std::uint32_t offset) const;
	void write(std::uint32_t offset, std::uint32_t value) const;
	bool core_present() const;
	Error apply(bool selected, bool enabled);

	SimulatorHardware hardware_;
	Configuration configuration_{};
	SimulatorConfiguration simulator_configuration_{};
	bool initialized_ = false;
	bool capture_active_ = false;
	bool selected_ = false;
};

} // namespace msap1::adc

#endif
