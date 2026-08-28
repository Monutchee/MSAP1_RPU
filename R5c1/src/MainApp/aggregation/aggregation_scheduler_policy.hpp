#ifndef MSAP1_R5C1_AGGREGATION_SCHEDULER_POLICY_HPP
#define MSAP1_R5C1_AGGREGATION_SCHEDULER_POLICY_HPP

#include <cstddef>
#include <cstdint>

namespace msap1::aggregation::scheduler_policy {

/*
 * The RX task runs above the validator so it can service the hardware FIFO
 * promptly.  Keeping this bound deliberately small guarantees that every
 * non-empty activation reaches the real one-tick blocking point in the
 * shadow service and gives the lower-priority validator time to free slots.
 *
 * This constant is public so host-side scheduling models can prove forward
 * progress using the same production value instead of duplicating it.
 */
inline constexpr std::size_t maximum_input_batch = 4U;

/*
 * Cortex-R5 XTime is backed by the 32-bit PMU cycle counter on this BSP.  The
 * counter wraps roughly every nine minutes, so convert an unsigned tick delta
 * instead of converting each absolute timestamp independently.  Unsigned
 * subtraction remains correct across one wrap for the short scheduler
 * intervals measured here.
 */
[[nodiscard]] inline constexpr std::uint32_t elapsed_counter_ticks(
	std::uint32_t start, std::uint32_t finish) noexcept
{
	return finish - start;
}

[[nodiscard]] inline constexpr std::uint32_t counter_ticks_to_microseconds(
	std::uint32_t ticks, std::uint32_t counts_per_second) noexcept
{
	if (counts_per_second == 0U)
		return 0U;

	const auto seconds = ticks / counts_per_second;
	const auto remainder = ticks % counts_per_second;
	return static_cast<std::uint32_t>(
		static_cast<std::uint64_t>(seconds) * 1000000ULL +
		(static_cast<std::uint64_t>(remainder) * 1000000ULL) /
			counts_per_second);
}

[[nodiscard]] inline constexpr std::uint32_t elapsed_microseconds(
	std::uint32_t start, std::uint32_t finish,
	std::uint32_t counts_per_second) noexcept
{
	return counter_ticks_to_microseconds(
		elapsed_counter_ticks(start, finish), counts_per_second);
}

} // namespace msap1::aggregation::scheduler_policy

#endif // MSAP1_R5C1_AGGREGATION_SCHEDULER_POLICY_HPP
