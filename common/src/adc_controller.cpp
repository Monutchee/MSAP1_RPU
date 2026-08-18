#include "adc_controller.hpp"

namespace msap1::adc {

AdcController::AdcController(Ad7771 &physical, AdcSimulator &simulator)
	: physical_(physical), simulator_(simulator), active_(&physical)
{
}

Error AdcController::initialize(const Configuration &physical_configuration)
{
	/*
	 * The two devices fail independently: a missing or version-stale
	 * simulator core (a PL bitstream older than this firmware, seen
	 * 2026-08-18) must never take physical metering down with it.
	 * Health reports each device's state; only the selected source's
	 * device has to be healthy for that source to operate.
	 */
	const auto simulator_error = simulator_.initialize(physical_configuration);
	const auto physical_error = physical_.initialize(physical_configuration);
	active_ = &physical_;
	return physical_error != Error::None ? physical_error : simulator_error;
}

Error AdcController::configure(
	Source target_source, const Configuration &configuration,
	const SimulatorConfiguration &simulator_configuration)
{
	const bool was_active = active_->capture_active();
	const auto previous_source = active_->source();
	const auto previous_physical_configuration = physical_.configuration();
	const auto previous_simulator_configuration = simulator_.configuration();
	const auto previous_simulator_signal = simulator_.simulator_configuration();
	active_->stop_capture();

	Error error = Error::InvalidConfiguration;
	if (target_source == Source::Physical) {
		error = physical_.configure_operating_point(
			configuration.sample_rate, configuration.pga_gains);
		/*
		 * Deselecting the simulator is best-effort when its core is
		 * unavailable: the PL reset default is the physical source,
		 * and an unusable simulator must not block physical capture.
		 */
		if (error == Error::None && simulator_.initialized())
			error = simulator_.select_source(false);
		if (error == Error::None)
			active_ = &physical_;
	} else if (target_source == Source::Simulator) {
		error = simulator_.configure(configuration,
					     simulator_configuration, true);
		if (error == Error::None)
			active_ = &simulator_;
	}

	if (error == Error::None && was_active)
		error = active_->start_capture();
	if (error == Error::None)
		return Error::None;

	/* Best-effort rollback restores the last complete operating point. */
	active_->stop_capture();
	if (previous_source == Source::Physical) {
		(void)physical_.configure_operating_point(
			previous_physical_configuration.sample_rate,
			previous_physical_configuration.pga_gains);
		if (simulator_.initialized())
			(void)simulator_.select_source(false);
		active_ = &physical_;
	} else {
		(void)simulator_.configure(previous_simulator_configuration,
					   previous_simulator_signal, true);
		active_ = &simulator_;
	}
	if (was_active)
		(void)active_->start_capture();
	return error;
}

Error AdcController::start_capture()
{
	return active_->start_capture();
}

void AdcController::stop_capture()
{
	active_->stop_capture();
}

} // namespace msap1::adc

