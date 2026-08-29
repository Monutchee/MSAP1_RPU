#ifndef MSAP1_R5C0_HANDLERS_POWER_QUALITY_CONFIG_HPP
#define MSAP1_R5C0_HANDLERS_POWER_QUALITY_CONFIG_HPP

#include "metering.hpp"
#include "rpu_control_protocol.h"

#include <cstdint>

namespace msap1::r5c0 {

std::uint32_t stage_power_quality_config(
	msap1::meter::MeteringPipeline &metering,
	const msap1_m18_config_payload &wire,
	msap1_m18_config_ack_payload &acknowledgement);

} // namespace msap1::r5c0

#endif // MSAP1_R5C0_HANDLERS_POWER_QUALITY_CONFIG_HPP
