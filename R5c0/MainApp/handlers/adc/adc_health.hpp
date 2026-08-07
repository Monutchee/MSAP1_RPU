#ifndef MSAP1_R5C0_HANDLERS_ADC_HEALTH_HPP
#define MSAP1_R5C0_HANDLERS_ADC_HEALTH_HPP

/**
 * @file adc_health.hpp
 * @brief MSAP1_RPU_MSG_ADC_HEALTH_GET: build the full health snapshot.
 */

#include "adc_controller.hpp"
#include "metering.hpp"
#include "rpu_control_protocol.h"

namespace msap1::r5c0 {

/**
 * @brief Fill @p health with the complete ADC and meter health snapshot.
 *
 * Collects the capture counters and PL-measured DCLK/DRDY rates, derives
 * the health flags (including the DRDY-vs-configured-rate match within a
 * 1% + 2 Hz tolerance), and appends the MeterCore generation/status view.
 *
 * Source-dependent part: with the simulator active, SPI diagnostics are
 * reported not-applicable and simulator health is derived from generation
 * match and saturation/missed-sample counters. With the physical ADC, the
 * AD7771 register file is read over SPI; on a failed read the partial
 * register bytes are still reported together with the SPI error class so
 * Linux can log exactly what the probe saw.
 *
 * Never fails: a broken SPI link is DATA (spi_error), not an error reply.
 */
void build_adc_health(msap1::adc::AdcController &adc,
		      msap1::meter::MeteringPipeline &metering,
		      msap1_adc_health_payload &health);

} // namespace msap1::r5c0

#endif /* MSAP1_R5C0_HANDLERS_ADC_HEALTH_HPP */
