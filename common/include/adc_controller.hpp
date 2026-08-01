#ifndef MSAP1_ADC_CONTROLLER_HPP
#define MSAP1_ADC_CONTROLLER_HPP

#include "ad7771.hpp"
#include "adc_simulator.hpp"

namespace msap1::adc {

/*
 * Owns source selection without dynamic allocation or RTTI.  The concrete
 * objects remain available for source-specific diagnostics, while normal
 * lifecycle callers depend only on AdcDevice.
 */
class AdcController {
public:
	AdcController(Ad7771 &physical, AdcSimulator &simulator);

	Error initialize(const Configuration &physical_configuration);
	Error configure(Source source, const Configuration &configuration,
			const SimulatorConfiguration &simulator_configuration);
	Error start_capture();
	void stop_capture();
	DeviceStatus status() const { return active_->status(); }
	Source source() const { return active_->source(); }
	const Configuration &configuration() const
	{
		return active_->configuration();
	}
	bool initialized() const { return active_->initialized(); }
	bool capture_active() const { return active_->capture_active(); }

	Ad7771 &physical() { return physical_; }
	const Ad7771 &physical() const { return physical_; }
	AdcSimulator &simulator() { return simulator_; }
	const AdcSimulator &simulator() const { return simulator_; }

private:
	Ad7771 &physical_;
	AdcSimulator &simulator_;
	AdcDevice *active_;
};

} // namespace msap1::adc

#endif
