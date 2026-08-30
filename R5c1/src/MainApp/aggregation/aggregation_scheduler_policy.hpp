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
 * Worst-case private FIFO packet rate at the product's 128 kSPS default.
 * VSB1 is the dominant stream (128000 / 256).  The remaining producers can
 * coincide with it at 60 Hz: one AGG1 per cycle, one PQE1 per half cycle, and
 * one HRM1 family per 200 ms.  Keep ten percent scheduling capacity above the
 * sum so a nominally sufficient tick cannot operate exactly at saturation.
 */
inline constexpr std::uint32_t voltage_sample_packets_per_second = 500U;
inline constexpr std::uint32_t aggregation_packets_per_second = 60U;
inline constexpr std::uint32_t pq_event_packets_per_second = 120U;
inline constexpr std::uint32_t harmonic_packets_per_second = 5U;
inline constexpr std::uint32_t maximum_input_packets_per_second =
	voltage_sample_packets_per_second + aggregation_packets_per_second +
	pq_event_packets_per_second + harmonic_packets_per_second;
inline constexpr std::uint32_t scheduling_margin_percent = 10U;
inline constexpr std::uint32_t minimum_input_capacity_per_second =
	(maximum_input_packets_per_second *
		(100U + scheduling_margin_percent) + 99U) / 100U;

[[nodiscard]] inline constexpr std::uint32_t input_capacity_per_second(
	std::uint32_t tick_rate_hz) noexcept
{
	return static_cast<std::uint32_t>(maximum_input_batch) * tick_rate_hz;
}

[[nodiscard]] inline constexpr bool supports_maximum_input_rate(
	std::uint32_t tick_rate_hz) noexcept
{
	return input_capacity_per_second(tick_rate_hz) >=
		minimum_input_capacity_per_second;
}

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
