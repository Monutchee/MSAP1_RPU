#ifndef MSAP1_R5C1_AGGREGATION_SCHEDULER_POLICY_HPP
#define MSAP1_R5C1_AGGREGATION_SCHEDULER_POLICY_HPP

#include <cstddef>

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

} // namespace msap1::aggregation::scheduler_policy

#endif // MSAP1_R5C1_AGGREGATION_SCHEDULER_POLICY_HPP
