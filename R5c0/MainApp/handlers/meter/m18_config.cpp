#include "m18_config.hpp"

namespace msap1::r5c0 {

std::uint32_t stage_m18_config(
	msap1::meter::MeteringPipeline &metering,
	const msap1_m18_config_payload &wire,
	msap1_m18_config_ack_payload &acknowledgement)
{
	const auto error = metering.stage_m18_configuration(wire);
	if (error != msap1::meter::Error::None)
		return MSAP1_RPU_STATUS_BAD_PAYLOAD;
	acknowledgement = {};
	acknowledgement.generation = wire.generation;
	/* Transient capability remains intentionally unavailable. */
	acknowledgement.capability_flags = 0u;
	return MSAP1_RPU_STATUS_OK;
}

} // namespace msap1::r5c0
