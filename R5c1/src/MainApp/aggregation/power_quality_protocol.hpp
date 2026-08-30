#ifndef MSAP1_R5C1_POWER_QUALITY_PROTOCOL_HPP
#define MSAP1_R5C1_POWER_QUALITY_PROTOCOL_HPP

#include "aggregation_protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

struct PowerQualityPacketHeader {
	static constexpr std::size_t header_words = 4U;
	static constexpr std::size_t crc_words = 1U;
	static constexpr std::uint32_t contract_revision = 1U;
	static constexpr std::size_t magic_index = 0U;
	static constexpr std::size_t contract_revision_index = 1U;
	static constexpr std::size_t payload_count_index = 2U;
	static constexpr std::size_t transport_sequence_index = 3U;
	static constexpr std::size_t payload_index = 4U;
};

struct PqEventProtocol final : PowerQualityPacketHeader {
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

struct VoltageSampleProtocol final : PowerQualityPacketHeader {
	static constexpr std::uint32_t magic = 0x31425356U; // "VSB1"
	static constexpr std::size_t payload_words = 1038U;
	static constexpr std::size_t frame_words =
		header_words + payload_words + crc_words;
	static constexpr std::size_t crc_index = frame_words - 1U;
	static constexpr std::size_t sequence_word = 0U;
	static constexpr std::size_t generation_word = 1U;
	static constexpr std::size_t sample_rate_word = 2U;
	static constexpr std::size_t frame_capacity_word = 3U;
	static constexpr std::size_t phase_mask_word = 4U;
	static constexpr std::size_t model_word = 5U;
	static constexpr std::size_t timing_word = 6U;
	static constexpr std::size_t reference_microvolts_word = 7U;
	static constexpr std::size_t first_sample_word = 8U;
	static constexpr std::size_t sample_word = 10U;
	static constexpr std::size_t batch_frames = 256U;
	static constexpr std::size_t words_per_sample = 4U;
	static constexpr std::size_t actual_count_word = 1034U;
	static constexpr std::size_t batch_status_word = 1035U;
	static constexpr std::size_t last_sample_word = 1036U;
	static constexpr std::size_t phases = 3U;
	static constexpr std::size_t classifier_bins = 512U;
	static constexpr std::uint32_t batch_status_mask = 0x3U;
	static constexpr std::uint32_t batch_discontinuity = 1U << 0U;
	static constexpr std::uint32_t batch_source_drop = 1U << 1U;
	static constexpr std::uint32_t sample_valid_a = 1U << 0U;
	static constexpr std::uint32_t sample_valid_b = 1U << 1U;
	static constexpr std::uint32_t sample_valid_c = 1U << 2U;
	static constexpr std::uint32_t sample_malformed = 1U << 3U;
	static constexpr std::uint32_t sample_locked = 1U << 4U;
	static constexpr std::uint32_t sample_fallback = 1U << 5U;
	static constexpr std::uint32_t sample_saturated = 1U << 6U;
	static constexpr std::uint32_t sample_flags_mask = 0x7fU;
	static constexpr std::uint32_t packed_reserved_mask = 0xffffff80U;
	static_assert(sample_word + batch_frames * words_per_sample ==
		actual_count_word);
	static_assert(last_sample_word + 2U == payload_words);
};

struct VoltageSampleInputView final {
	std::uint32_t sequence{};
	std::uint32_t configuration_generation{};
	std::uint32_t sample_rate_hz{};
	std::uint8_t phase_mask{};
	std::uint16_t lamp_voltage{};
	std::uint8_t nominal_hz{};
	std::uint16_t live_cadence_ms{};
	std::uint16_t pst_interval_seconds{};
	std::uint32_t reference_microvolts{};
	std::uint64_t first_sample{};
	std::uint64_t last_sample{};
	std::uint16_t actual_count{};
	std::uint8_t batch_status{};
	const std::uint32_t *packed_sample_words{};
};

static_assert(PqEventProtocol::frame_words <= maximum_transport_frame_words);
static_assert(VoltageSampleProtocol::frame_words <=
	maximum_transport_frame_words);

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_POWER_QUALITY_PROTOCOL_HPP
