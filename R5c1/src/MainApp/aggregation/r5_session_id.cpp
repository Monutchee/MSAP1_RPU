#include "r5_session_id.hpp"

#include "xiltimer.h"

#include <cstdint>

namespace msap1::aggregation {

std::uint64_t generate_r5_session_id() noexcept
{
	XTime first{};
	XTime second{};
	XTime_GetTime(&first);
	XTime_GetTime(&second);
	const auto address_entropy = static_cast<std::uint64_t>(
		reinterpret_cast<std::uintptr_t>(&first));
	const auto entropy = static_cast<std::uint64_t>(first) ^
		(static_cast<std::uint64_t>(second) << 17U) ^
		(static_cast<std::uint64_t>(second) >> 11U) ^ address_entropy ^
		0x4d313752354331ULL;
	return mix_session_entropy(entropy);
}

} // namespace msap1::aggregation
