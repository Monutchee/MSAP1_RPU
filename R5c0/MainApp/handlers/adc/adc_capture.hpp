#ifndef MSAP1_R5C0_HANDLERS_ADC_CAPTURE_HPP
#define MSAP1_R5C0_HANDLERS_ADC_CAPTURE_HPP

/**
 * @file adc_capture.hpp
 * @brief MSAP1_RPU_MSG_ADC_CAPTURE_START/STOP: capture pipeline control.
 */

#include <cstdint>

#include "adc_controller.hpp"
#include "rpu_control_protocol.h"

namespace msap1::r5c0 {

/**
 * @brief Start the raw capture stream of the active ADC source.
 *
 * Linux arms both DMA consumers before sending this command, so the first
 * PL block after start is never lost.
 *
 * @return MSAP1_RPU_STATUS_OK on success; ADC_UNAVAILABLE before
 *         initialization; ADC_STATE when capture is already active;
 *         INTERNAL_ERROR otherwise.
 */
std::uint32_t start_capture(msap1::adc::AdcController &adc);

/**
 * @brief Stop the raw capture stream; idempotent by design so Linux can
 *        use STOP to recover cleanly after a daemon crash.
 */
void stop_capture(msap1::adc::AdcController &adc);

} // namespace msap1::r5c0

#endif /* MSAP1_R5C0_HANDLERS_ADC_CAPTURE_HPP */
