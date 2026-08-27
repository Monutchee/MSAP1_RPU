#ifndef MSAP1_R5C1_AGGREGATION_PROTOCOL_HPP
#define MSAP1_R5C1_AGGREGATION_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/**
 * Exact co-release wire contract emitted by meter_r5_aggregation_export.vhd.
 *
 * PL and RPU firmware are shipped as one image.  The fixed contract word is
 * therefore only an integrity guard for an accidentally mixed image, not a
 * protocol negotiation mechanism.  There is deliberately no legacy decoder,
 * version selection, or compatibility fallback.
 */
struct AggregationProtocol final {
	static constexpr std::uint32_t magic = 0x31474741U;
	static constexpr std::uint32_t contract_revision = 1U;
	static constexpr std::size_t payload_words = 234U;
	static constexpr std::size_t header_words = 4U;
	static constexpr std::size_t crc_words = 1U;
	static constexpr std::size_t frame_words =
		header_words + payload_words + crc_words;
	static constexpr std::size_t frame_bytes =
		frame_words * sizeof(std::uint32_t);

	static constexpr std::size_t magic_index = 0U;
	static constexpr std::size_t contract_revision_index = 1U;
	static constexpr std::size_t payload_count_index = 2U;
	static constexpr std::size_t transport_sequence_index = 3U;
	static constexpr std::size_t payload_index = 4U;
	static constexpr std::size_t crc_index = frame_words - 1U;

	static constexpr std::size_t single_cycle_words = 221U;
	static constexpr std::size_t context_words = 13U;
	static constexpr std::size_t context_index =
		payload_index + single_cycle_words;

	// Bits outside these masks are reserved by the exact co-release layout.
	// Rejecting them catches a corrupted frame or an accidentally mismatched
	// PL/RPU image; it is not a version-negotiation path.
	static constexpr std::uint32_t control_status_mask = 0x00001FFFU;
	static constexpr std::uint32_t utc_target_status_mask = 0x00000003U;
};

/* HRM1 is the largest private packet sharing the FIFO: four header words,
 * 42 complete 64-word records, and one CRC word. Keep the generic transport
 * buffer independent of either decoder while bounding all static storage. */
inline constexpr std::size_t maximum_transport_frame_words = 2693U;

static_assert(AggregationProtocol::frame_words == 239U);
static_assert(AggregationProtocol::frame_bytes == 956U);
static_assert(AggregationProtocol::single_cycle_words +
	AggregationProtocol::context_words == AggregationProtocol::payload_words);

/** One complete FIFO packet. The fixed extent prevents partial-frame use. */
struct AggregationFrame final {
	std::array<std::uint32_t, maximum_transport_frame_words> words{};
	std::size_t word_count{};
};

/** Context captured atomically when the SingleCycle result begins. */
struct AggregationContext final {
	std::uint32_t configuration_generation{};
	std::uint32_t sample_rate_hz{};
	std::uint32_t control_status{};
	std::uint32_t frequency_status{};
	std::uint32_t frequency_period_q16{};
	std::uint32_t frequency_sequence{};
	std::uint32_t capture_frame_count{};
	std::uint32_t header_error_count{};
	std::uint32_t overflow_count{};
	std::uint32_t alert_status{};
	std::uint64_t utc_target_sample{};
	std::uint32_t utc_target_status{};
};

/** Validated view of one transport frame. It owns no packet storage. */
struct AggregationInputView final {
	std::uint32_t sequence{};
	const std::uint32_t *single_cycle_words{};
	std::size_t single_cycle_word_count{};
	AggregationContext context{};
};

enum class FrameValidationError : std::uint8_t {
	none,
	invalid_length,
	invalid_magic,
	contract_mismatch,
	invalid_payload_count,
	sequence_mismatch,
	crc_mismatch,
	reserved_bits_nonzero,
	invalid_record_geometry,
	provenance_mismatch,
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_PROTOCOL_HPP
