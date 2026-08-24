#ifndef MSAP1_R5C1_AGGREGATION_RECORD_RING_HPP
#define MSAP1_R5C1_AGGREGATION_RECORD_RING_HPP

#include "aggregation_meter_record.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/**
 * Static SPSC queue between the future arithmetic engine and FIFO output task.
 * Records are complete before publication; a slow Linux/DMA path can never
 * retain ownership of an accumulator or stall aggregation input.
 */
class AggregationRecordRing final {
public:
	static constexpr std::size_t capacity = 64U;

	[[nodiscard]] bool try_push(const AggregationMeterRecord &record) noexcept;
	[[nodiscard]] bool try_pop(AggregationMeterRecord &record) noexcept;
	[[nodiscard]] std::size_t size() const noexcept;

private:
	alignas(64) std::array<AggregationMeterRecord, capacity> records_{};
	alignas(64) std::uint32_t read_index_{};
	alignas(64) std::uint32_t write_index_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_RECORD_RING_HPP
