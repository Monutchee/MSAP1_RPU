#ifndef MSAP1_R5C1_HARMONIC_PROTOCOL_HPP
#define MSAP1_R5C1_HARMONIC_PROTOCOL_HPP

#include "aggregation_protocol.hpp"

#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/** Exact co-release HRM1 family packet emitted by MeterCore. */
struct HarmonicProtocol final {
	static constexpr std::uint32_t magic = 0x314D5248U; // "HRM1"
	static constexpr std::uint32_t contract_revision = 1U;
	static constexpr std::size_t record_words = 64U;
	static constexpr std::size_t records_per_family = 42U;
	static constexpr std::size_t payload_words =
		record_words * records_per_family;
	static constexpr std::size_t header_words = 4U;
	static constexpr std::size_t frame_words =
		header_words + payload_words + 1U;
	static constexpr std::size_t frame_bytes = frame_words * sizeof(std::uint32_t);

	static constexpr std::size_t magic_index = 0U;
	static constexpr std::size_t contract_revision_index = 1U;
	static constexpr std::size_t payload_count_index = 2U;
	static constexpr std::size_t transport_sequence_index = 3U;
	static constexpr std::size_t payload_index = 4U;
	static constexpr std::size_t crc_index = frame_words - 1U;

	static constexpr std::uint32_t record_magic = 0x3152544DU;
	static constexpr std::uint32_t base_record_format = 0x00050001U;
	static constexpr std::uint32_t aggregate_record_format = 0x001F0001U;
	static constexpr std::uint32_t record_bytes = 256U;
	static constexpr std::size_t channels = 7U;
	static constexpr std::size_t chunks_per_channel = 6U;
	static constexpr std::size_t orders_per_chunk = 24U;
	static constexpr std::size_t maximum_order = 127U;
};

static_assert(HarmonicProtocol::frame_words == maximum_transport_frame_words);

/** Validated common provenance plus a view of the exact 42 records. */
struct HarmonicInputView final {
	std::uint32_t sequence{};
	std::uint32_t configuration_generation{};
	std::uint32_t sample_rate_hz{};
	std::uint32_t sample_count{};
	std::uint8_t valid_mask{};
	std::uint32_t status{};
	std::uint64_t first_sample{};
	std::uint32_t measured_frequency_millihz{};
	std::uint8_t qualified_max_order{};
	std::uint8_t nominal_frequency_hz{};
	std::uint8_t cycle_count{};
	std::uint8_t filter_profile_id{};
	const std::uint32_t *records{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_HARMONIC_PROTOCOL_HPP
