#include "simulator_event.hpp"

namespace msap1::r5c0 {

std::uint32_t apply_simulator_event(
	msap1::adc::AdcController &adc,
	const msap1_simulator_event_payload &wire,
	msap1_simulator_event_ack_payload &acknowledgement)
{
	if (wire.action > MSAP1_SIMULATOR_EVENT_QUERY ||
	    (wire.flags & ~MSAP1_SIMULATOR_EVENT_FLAG_REPEAT) != 0u)
		return MSAP1_RPU_STATUS_BAD_PAYLOAD;
	/* The sequencer lives inside the simulator core; against the
	 * physical ADC there is nothing to drive. */
	if (adc.source() != msap1::adc::Source::Simulator)
		return MSAP1_RPU_STATUS_ADC_STATE;

	auto &simulator = adc.simulator();
	msap1::adc::Error error = msap1::adc::Error::None;
	switch (wire.action) {
	case MSAP1_SIMULATOR_EVENT_ARM: {
		msap1::adc::SimulatorEvent event;
		event.channel_mask = wire.channel_mask;
		event.scale_q16 = wire.scale_q16;
		event.duration_half_cycles = wire.duration_half_cycles;
		event.period_half_cycles = wire.period_half_cycles;
		event.repeat =
			(wire.flags & MSAP1_SIMULATOR_EVENT_FLAG_REPEAT) != 0u;
		error = simulator.arm_event(event);
		break;
	}
	case MSAP1_SIMULATOR_EVENT_CANCEL:
		error = simulator.cancel_event();
		break;
	case MSAP1_SIMULATOR_EVENT_CLEAR_COUNT:
		error = simulator.clear_event_count();
		break;
	default:
		break;  /* QUERY reads the state below without changing it */
	}
	if (error != msap1::adc::Error::None)
		return error == msap1::adc::Error::InvalidConfiguration ?
			MSAP1_RPU_STATUS_BAD_PAYLOAD :
			error == msap1::adc::Error::CaptureNotInitialized ?
			MSAP1_RPU_STATUS_ADC_UNAVAILABLE :
			MSAP1_RPU_STATUS_INTERNAL_ERROR;

	const auto state = simulator.event_status();
	acknowledgement = {};
	acknowledgement.status = state.status;
	acknowledgement.remaining = state.remaining;
	acknowledgement.active_control = state.active_control;
	acknowledgement.active_scale = state.active_scale;
	acknowledgement.active_timing = state.active_timing;
	return MSAP1_RPU_STATUS_OK;
}

} // namespace msap1::r5c0
