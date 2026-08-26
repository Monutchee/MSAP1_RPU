#ifndef MSAP1_R5C1_AGGREGATION_FRAME_RING_HPP
#define MSAP1_R5C1_AGGREGATION_FRAME_RING_HPP

#include "aggregation_protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/**
 * Static single-producer/single-consumer queue between the FIFO task and the
 * validator task.  Sixty-four complete frames consume about 61 KiB and avoid
 * heap use or partial-frame ownership.
 */
class AggregationFrameRing final {
public:
	static constexpr std::size_t capacity = 64U;

	[[nodiscard]] bool try_push(const AggregationFrame &frame) noexcept;
	[[nodiscard]] bool try_pop(AggregationFrame &frame) noexcept;
	[[nodiscard]] std::size_t size() const noexcept;
	[[nodiscard]] std::size_t available_capacity() const noexcept;

private:
	alignas(64) std::array<AggregationFrame, capacity> frames_{};
	alignas(64) std::uint32_t read_index_{};
	alignas(64) std::uint32_t write_index_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_FRAME_RING_HPP
