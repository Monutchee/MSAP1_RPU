#ifndef MSAP1_M18_CONFIGURATION_HPP
#define MSAP1_M18_CONFIGURATION_HPP

#include "rpu_control_protocol.h"

#include <cstddef>
#include <cstdint>

namespace msap1::m18 {

inline bool valid_event_profile(const msap1_m18_event_profile &profile,
	std::size_t index) noexcept
{
	constexpr std::uint32_t allowed_flags =
		MSAP1_M18_EVENT_ENABLED | MSAP1_M18_EVENT_WAVEFORM_ENABLED |
		MSAP1_M18_EVENT_PER_PHASE | MSAP1_M18_EVENT_IEC_CLASSIFICATION;
	if ((profile.flags & ~allowed_flags) != 0u ||
	    profile.threshold_e4 > 0xffffu ||
	    profile.hysteresis_e4 > 0xffffu ||
	    profile.hysteresis_e4 >= profile.threshold_e4 ||
	    profile.phase_mask == 0u || (profile.phase_mask & ~0x7u) != 0u ||
	    profile.waveform_pretrigger_ms > 120000u ||
	    profile.waveform_posttrigger_ms > 120000u)
		return false;
	const auto decimation = profile.waveform_decimation;
	if (decimation != 1u && decimation != 2u && decimation != 4u &&
	    decimation != 8u && decimation != 16u && decimation != 32u)
		return false;
	const bool should_be_iec =
		index <= MSAP1_M18_EVENT_RAPID_VOLTAGE_CHANGE ||
		index == MSAP1_M18_EVENT_TRANSIENT_VOLTAGE;
	if (((profile.flags & MSAP1_M18_EVENT_IEC_CLASSIFICATION) != 0u) !=
	    should_be_iec)
		return false;
	return index != MSAP1_M18_EVENT_TRANSIENT_VOLTAGE ||
		(profile.flags & MSAP1_M18_EVENT_ENABLED) == 0u;
}

inline bool valid_configuration(const msap1_m18_config_payload &value,
	std::uint32_t sample_rate_hz = 0u) noexcept
{
	if (value.generation == 0u ||
	    value.event_profile_count != MSAP1_M18_EVENT_TYPE_COUNT ||
	    (value.flicker_flags & ~MSAP1_M18_ENGINE_ENABLED) != 0u ||
	    (value.mains_flags & ~MSAP1_M18_ENGINE_ENABLED) != 0u ||
	    value.flicker_phase_mask == 0u ||
	    (value.flicker_phase_mask & ~0x7u) != 0u ||
	    (value.flicker_lamp_voltage != 120u &&
	     value.flicker_lamp_voltage != 230u) ||
	    value.flicker_live_cadence_ms != 1000u ||
	    value.flicker_pst_interval_seconds != 600u ||
	    value.flicker_plt_pst_count != 12u ||
	    value.mains_carrier_millihz == 0u ||
	    value.mains_carrier_millihz >= 12500000u ||
	    value.mains_bandwidth_millihz == 0u ||
	    value.mains_bandwidth_millihz >= value.mains_carrier_millihz ||
	    value.mains_observation_ms != 200u ||
	    value.mains_phase_mask == 0u ||
	    (value.mains_phase_mask & ~0x7u) != 0u ||
	    value.mains_threshold_e4 > 0xffffu)
		return false;
	if ((value.mains_flags & MSAP1_M18_ENGINE_ENABLED) != 0u &&
	    sample_rate_hz != 0u &&
	    static_cast<std::uint64_t>(value.mains_carrier_millihz) * 2u >=
		static_cast<std::uint64_t>(sample_rate_hz) * 1000u)
		return false;
	if ((value.flicker_flags & MSAP1_M18_ENGINE_ENABLED) != 0u &&
	    (value.reference_voltage_microvolts == 0u ||
	     (sample_rate_hz != 0u &&
	      (sample_rate_hz < 2000u || sample_rate_hz % 2000u != 0u))))
		return false;
	for (std::size_t index = 0; index < MSAP1_M18_EVENT_TYPE_COUNT; ++index)
		if (!valid_event_profile(value.event[index], index))
			return false;
	for (std::size_t index = MSAP1_M18_EVENT_CURRENT_SAG;
	     index <= MSAP1_M18_EVENT_CURRENT_UNBALANCE; ++index)
		if ((value.event[index].flags & MSAP1_M18_EVENT_ENABLED) != 0u &&
		    value.reference_current_microamperes == 0u)
			return false;
	return true;
}

} // namespace msap1::m18

#endif // MSAP1_M18_CONFIGURATION_HPP
