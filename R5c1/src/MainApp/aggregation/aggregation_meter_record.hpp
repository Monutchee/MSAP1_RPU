#ifndef MSAP1_R5C1_AGGREGATION_METER_RECORD_HPP
#define MSAP1_R5C1_AGGREGATION_METER_RECORD_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/**
 * One complete meter record for the existing Linux meter DMA stream.
 *
 * `sequence` is diagnostic metadata for the R5 software queue; only `words`
 * are written to the AXI FIFO.  Keeping it outside the record image preserves
 * the existing byte-exact 256-byte MTR contract.
 */
struct AggregationMeterRecord final {
	static constexpr std::size_t word_count = 64U;
	static constexpr std::size_t byte_count = word_count * sizeof(std::uint32_t);

	std::uint32_t sequence{};
	std::array<std::uint32_t, word_count> words{};
};

static_assert(AggregationMeterRecord::byte_count == 256U,
	"meter DMA records must remain exactly 256 bytes");

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_METER_RECORD_HPP
