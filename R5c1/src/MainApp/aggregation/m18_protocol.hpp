#ifndef MSAP1_R5C1_M18_PROTOCOL_HPP
#define MSAP1_R5C1_M18_PROTOCOL_HPP

#include "aggregation_protocol.hpp"

#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

struct M18PacketHeader {
	static constexpr std::size_t header_words = 4U;
	static constexpr std::size_t crc_words = 1U;
	static constexpr std::uint32_t contract_revision = 1U;
	static constexpr std::size_t magic_index = 0U;
	static constexpr std::size_t contract_revision_index = 1U;
	static constexpr std::size_t payload_count_index = 2U;
	static constexpr std::size_t transport_sequence_index = 3U;
	static constexpr std::size_t payload_index = 4U;
};

struct PqEventProtocol final : M18PacketHeader {
	static constexpr std::uint32_t magic = 0x31455150U; // "PQE1"
	static constexpr std::size_t payload_words = 64U;
	static constexpr std::size_t frame_words =
		header_words + payload_words + crc_words;
	static constexpr std::size_t crc_index = frame_words - 1U;
};

struct FlickerProtocol final : M18PacketHeader {
	static constexpr std::uint32_t magic = 0x314B4C46U; // "FLK1"
	static constexpr std::size_t payload_words = 64U;
	static constexpr std::size_t frame_words =
		header_words + payload_words + crc_words;
	static constexpr std::size_t crc_index = frame_words - 1U;
	static constexpr std::size_t histogram_base = 16U;
	static constexpr std::size_t histogram_bins = 16U;
	static constexpr std::size_t phases = 3U;
	static_assert(histogram_base + histogram_bins * phases == payload_words);
};

struct MainsSignalProtocol final : M18PacketHeader {
	static constexpr std::uint32_t magic = 0x3153434DU; // "MCS1"
	static constexpr std::size_t payload_words = 20U;
	static constexpr std::size_t frame_words =
		header_words + payload_words + crc_words;
	static constexpr std::size_t crc_index = frame_words - 1U;
};

static_assert(PqEventProtocol::frame_words <= maximum_transport_frame_words);
static_assert(FlickerProtocol::frame_words <= maximum_transport_frame_words);
static_assert(MainsSignalProtocol::frame_words <=
	maximum_transport_frame_words);

} // namespace msap1::aggregation

#endif
