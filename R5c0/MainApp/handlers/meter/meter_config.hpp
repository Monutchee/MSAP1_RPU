#ifndef MSAP1_R5C0_HANDLERS_METER_CONFIG_HPP
#define MSAP1_R5C0_HANDLERS_METER_CONFIG_HPP

/**
 * @file meter_config.hpp
 * @brief MSAP1_RPU_MSG_METER_CONFIG_SET: apply one operating-point snapshot.
 */

#include <cstdint>

#include "adc_controller.hpp"
#include "metering.hpp"
#include "rpu_control_protocol.h"

namespace msap1::r5c0 {

/**
 * @brief Validate and apply a complete meter configuration snapshot.
 *
 * The transaction, in order:
 *  1. Reject out-of-range masks/flags, unsupported sample rates, and PGA
 *     gains other than 1/2/4/8 (MSAP1_RPU_STATUS_BAD_PAYLOAD).
 *  2. Reject the request entirely while capture is active
 *     (MSAP1_RPU_STATUS_ADC_STATE) — PGA/coefficient changes are a
 *     coordinated ADC/PL transaction that Linux performs with capture
 *     stopped.
 *  3. Configure the selected raw source (physical AD7771 or PL simulator)
 *     through AdcController's transactional source switch.
 *  4. Commit the conversion/processing/frequency configuration to the PL
 *     MeterCore stages under the snapshot's generation.
 *  5. Fill @p acknowledgement with the readback generations so Linux can
 *     verify every stage latched this exact snapshot.
 *
 * @return MSAP1_RPU_STATUS_OK when the snapshot applied and the ack is
 *         valid; otherwise the wire status the caller must send as an
 *         error reply.
 */
std::uint32_t apply_meter_config(
	msap1::adc::AdcController &adc,
	msap1::meter::MeteringPipeline &metering,
	const msap1_meter_config_payload &wire,
	msap1_meter_config_ack_payload &acknowledgement);

} // namespace msap1::r5c0

#endif /* MSAP1_R5C0_HANDLERS_METER_CONFIG_HPP */
