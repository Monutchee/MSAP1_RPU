#ifndef MSAP1_R5C0_HANDLERS_SIMULATOR_EVENT_HPP
#define MSAP1_R5C0_HANDLERS_SIMULATOR_EVENT_HPP

/**
 * @file simulator_event.hpp
 * @brief MSAP1_RPU_MSG_SIMULATOR_EVENT_SET: drive the PL event sequencer.
 */

#include <cstdint>

#include "adc_controller.hpp"
#include "rpu_control_protocol.h"

namespace msap1::r5c0 {

/**
 * @brief Arm, cancel, or query one simulator amplitude-envelope burst.
 *
 * Unlike a configuration snapshot this is allowed WHILE CAPTURE RUNS, and
 * that is the point: the sequencer starts and ends the burst on the
 * generator's own half-cycle boundaries, so the sag/swell/interruption
 * under test is the only discontinuity in the stream. Stopping capture to
 * program it would restart the waveform and destroy the evidence.
 *
 * Requires the simulator to be the selected raw source; against the
 * physical ADC the request is rejected rather than silently ignored.
 *
 * @return MSAP1_RPU_STATUS_OK with @p acknowledgement holding the
 *         sequencer state read back from the PL; otherwise the wire
 *         status the caller must send as an error reply.
 */
std::uint32_t apply_simulator_event(
	msap1::adc::AdcController &adc,
	const msap1_simulator_event_payload &wire,
	msap1_simulator_event_ack_payload &acknowledgement);

} // namespace msap1::r5c0

#endif /* MSAP1_R5C0_HANDLERS_SIMULATOR_EVENT_HPP */
