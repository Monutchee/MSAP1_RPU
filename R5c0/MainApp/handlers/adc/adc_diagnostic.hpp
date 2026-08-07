#ifndef MSAP1_R5C0_HANDLERS_ADC_DIAGNOSTIC_HPP
#define MSAP1_R5C0_HANDLERS_ADC_DIAGNOSTIC_HPP

/**
 * @file adc_diagnostic.hpp
 * @brief MSAP1_RPU_MSG_ADC_DIAGNOSTIC_RUN: destructive ADC reset diagnostic.
 */

#include <cstdint>

#include "adc_controller.hpp"
#include "rpu_control_protocol.h"

namespace msap1::r5c0 {

/**
 * @brief Run diagnostic flow 1 and fill the wire report.
 *
 * Flow 1 is a warm hardware-pin diagnostic, not a board power cycle:
 * before -> assert ADC RESET_N -> read reset defaults -> conservative SRC
 * load -> after. It requires the physical ADC source and capture stopped
 * (the flow itself enforces the latter).
 *
 * A flow that runs but finds a fault still returns MSAP1_RPU_STATUS_OK:
 * the failure is DATA, reported inside @p wire (diagnostic_error,
 * failure_stage, and the four capture/register snapshots). Only precondition
 * violations produce an error status.
 *
 * @return MSAP1_RPU_STATUS_OK with @p wire filled; BAD_PAYLOAD for an
 *         unsupported flow; ADC_STATE when the simulator is active.
 */
std::uint32_t run_adc_diagnostic(msap1::adc::AdcController &adc,
				 const msap1_adc_diagnostic_request &request,
				 msap1_adc_diagnostic_payload &wire);

} // namespace msap1::r5c0

#endif /* MSAP1_R5C0_HANDLERS_ADC_DIAGNOSTIC_HPP */
