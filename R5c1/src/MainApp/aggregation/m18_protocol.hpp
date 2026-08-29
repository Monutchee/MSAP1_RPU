#ifndef MSAP1_R5C1_M18_PROTOCOL_HPP
#define MSAP1_R5C1_M18_PROTOCOL_HPP

#include "aggregation_protocol.hpp"

#include <array>
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
	static constexpr std::size_t sequence_word = 0U;
	static constexpr std::size_t generation_word = 1U;
	static constexpr std::size_t sample_rate_word = 2U;
	static constexpr std::size_t status_word = 3U;
	static constexpr std::size_t valid_phases_word = 4U;
	static constexpr std::size_t window_samples_word = 5U;
	static constexpr std::size_t first_sample_word = 6U;
	static constexpr std::size_t last_sample_word = 8U;
	static constexpr std::size_t pl_tick_word = 10U;
	static constexpr std::size_t urms_q16_word = 12U;
	static constexpr std::size_t irms_q16_word = 18U;
	static constexpr std::size_t reference_word = 24U;
	static constexpr std::size_t sag_threshold_word = 25U;
	static constexpr std::size_t swell_threshold_word = 26U;
	static constexpr std::size_t interruption_threshold_word = 27U;
	static constexpr std::size_t hysteresis_word = 28U;
	static constexpr std::size_t apply_word = 29U;
	static constexpr std::size_t reserved_word = 30U;
	static constexpr std::uint32_t status_mask = 0x1FU;
	static constexpr std::uint32_t valid_phases_mask = 0x00000707U;
};

struct PqEventInputView final {
	std::uint32_t sequence{};
	std::uint32_t configuration_generation{};
	std::uint32_t sample_rate_hz{};
	std::uint32_t status{};
	std::uint8_t voltage_valid_mask{};
	std::uint8_t current_valid_mask{};
	std::uint32_t window_samples{};
	std::uint64_t first_sample{};
	std::uint64_t last_sample{};
	std::uint64_t pl_tick{};
	std::array<std::uint64_t, 3U> urms_q16{};
	std::array<std::uint64_t, 3U> irms_q16{};
	std::uint32_t m12_reference_microvolts{};
	std::uint32_t m12_sag_threshold_e4{};
	std::uint32_t m12_swell_threshold_e4{};
	std::uint32_t m12_interruption_threshold_e4{};
	std::uint32_t m12_hysteresis_e4{};
	std::uint32_t apply_toggle{};
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
