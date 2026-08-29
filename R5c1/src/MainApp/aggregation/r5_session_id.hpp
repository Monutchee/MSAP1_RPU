#ifndef MSAP1_R5C1_SESSION_ID_HPP
#define MSAP1_R5C1_SESSION_ID_HPP

#include <cstdint>

namespace msap1::aggregation {

struct R5SessionEntropy {
	std::uint64_t shared_system_counter = 0U;
	std::uint64_t local_cycle_counter_before = 0U;
	std::uint64_t local_cycle_counter_after = 0U;
};

[[nodiscard]] constexpr std::uint64_t mix_session_entropy(
	std::uint64_t value) noexcept
{
	value += 0x9e3779b97f4a7c15ULL;
	value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
	value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
	value ^= value >> 31U;
	return value == 0U ? 0x6d736170314d3137ULL : value;
}

[[nodiscard]] constexpr std::uint64_t derive_r5_session_id(
	const R5SessionEntropy &entropy) noexcept
{
	/* SplitMix64 is bijective before the reserved-zero substitution. Keep the
	 * SoC-wide counter as a full-width input, then diffuse the two R5-local
	 * observations around it. */
	const auto local_before =
		(entropy.local_cycle_counter_before << 17U) |
		(entropy.local_cycle_counter_before >> 47U);
	const auto local_after =
		(entropy.local_cycle_counter_after << 41U) |
		(entropy.local_cycle_counter_after >> 23U);
	return mix_session_entropy(entropy.shared_system_counter ^ local_before ^
		local_after ^ 0x4d313752354331ULL);
}

/** Non-security boot nonce used only to distinguish volatile R5C1 sessions. */
[[nodiscard]] std::uint64_t generate_r5_session_id() noexcept;

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_SESSION_ID_HPP
