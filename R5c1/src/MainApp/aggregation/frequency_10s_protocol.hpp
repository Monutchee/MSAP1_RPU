#ifndef MSAP1_R5C1_FREQUENCY_10S_PROTOCOL_HPP
#define MSAP1_R5C1_FREQUENCY_10S_PROTOCOL_HPP

#include "aggregation_protocol.hpp"

#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/**
 * Exact PL-to-R5C1 contract for one UTC ten-second frequency observation.
 *
 * PL performs only signal conditioning, crossing interpolation, and bounded
 * observation transport.  R5C1 remains the interval authority: it decides
 * which complete cycles are eligible and performs the frequency arithmetic.
 * Crossing timestamps are signed Q16 sample offsets from interval_start_sample
 * so the two guard crossings can sit outside the interval without widening
 * each entry beyond two words.
 */
struct Frequency10sProtocol final {
	static constexpr std::uint32_t magic = 0x31515246U; // "FRQ1"
	static constexpr std::uint32_t contract_revision = 1U;
	static constexpr std::size_t header_words = 4U;
	static constexpr std::size_t crc_words = 1U;
	static constexpr std::size_t metadata_words = 24U;
	static constexpr std::size_t crossing_capacity = 1024U;
	static constexpr std::size_t crossing_words = crossing_capacity * 2U;
	static constexpr std::size_t payload_words =
		metadata_words + crossing_words;
	static constexpr std::size_t frame_words =
		header_words + payload_words + crc_words;

	static constexpr std::size_t magic_index = 0U;
	static constexpr std::size_t contract_revision_index = 1U;
	static constexpr std::size_t payload_count_index = 2U;
	static constexpr std::size_t transport_sequence_index = 3U;
	static constexpr std::size_t payload_index = 4U;
	static constexpr std::size_t crc_index = frame_words - 1U;

	static constexpr std::size_t sequence_word = 0U;
	static constexpr std::size_t generation_word = 1U;
	static constexpr std::size_t sample_rate_hz_word = 2U;
	static constexpr std::size_t measured_sample_rate_millihz_word = 3U;
	static constexpr std::size_t nominal_frequency_hz_word = 4U;
	static constexpr std::size_t profile_word = 5U;
	static constexpr std::size_t status_word = 6U;
	static constexpr std::size_t reason_word = 7U;
	static constexpr std::size_t interval_start_sample_word = 8U;
	static constexpr std::size_t interval_end_sample_word = 10U;
	static constexpr std::size_t utc_start_nanoseconds_word = 12U;
	static constexpr std::size_t utc_end_nanoseconds_word = 14U;
	static constexpr std::size_t utc_uncertainty_nanoseconds_word = 16U;
	static constexpr std::size_t boundary_generation_word = 18U;
	static constexpr std::size_t crossing_count_word = 19U;
	static constexpr std::size_t observer_drop_count_word = 20U;
	static constexpr std::size_t guard_flags_word = 21U;
	static constexpr std::size_t reserved_word = 22U;
	static constexpr std::size_t crossing_base_word = metadata_words;

	static constexpr std::uint32_t profile_reference_mask = 0x000000ffU;
	static constexpr unsigned profile_filter_shift = 8U;
	static constexpr unsigned profile_calibration_shift = 16U;
	static constexpr std::uint32_t profile_reserved_mask = 0xff000000U;

	static constexpr std::uint32_t status_boundary_valid = 1U << 0U;
	static constexpr std::uint32_t status_time_synchronized = 1U << 1U;
	static constexpr std::uint32_t status_sample_rate_valid = 1U << 2U;
	static constexpr std::uint32_t status_filter_ready = 1U << 3U;
	static constexpr std::uint32_t status_reference_valid = 1U << 4U;
	static constexpr std::uint32_t status_source_discontinuity = 1U << 5U;
	static constexpr std::uint32_t status_crossing_overflow = 1U << 6U;
	static constexpr std::uint32_t status_observer_drop = 1U << 7U;
	static constexpr std::uint32_t status_resynchronized = 1U << 8U;
	static constexpr std::uint32_t status_calibration_valid = 1U << 9U;
	static constexpr std::uint32_t status_profile_supported = 1U << 10U;
	static constexpr std::uint32_t status_mask = 0x000007ffU;

	static constexpr std::uint32_t reason_unsupported_profile = 1U << 0U;
	static constexpr std::uint32_t reason_time_unsynchronized = 1U << 1U;
	static constexpr std::uint32_t reason_time_uncertainty = 1U << 2U;
	static constexpr std::uint32_t reason_filter_warmup = 1U << 3U;
	static constexpr std::uint32_t reason_reference_invalid = 1U << 4U;
	static constexpr std::uint32_t reason_discontinuity = 1U << 5U;
	static constexpr std::uint32_t reason_crossing_overflow = 1U << 6U;
	static constexpr std::uint32_t reason_observer_drop = 1U << 7U;
	static constexpr std::uint32_t reason_sample_rate_invalid = 1U << 8U;
	static constexpr std::uint32_t reason_boundary_invalid = 1U << 9U;
	static constexpr std::uint32_t reason_calibration_invalid = 1U << 10U;
	static constexpr std::uint32_t reason_mask = 0x000007ffU;

	static constexpr std::uint32_t guard_before_start = 1U << 0U;
	static constexpr std::uint32_t guard_after_end = 1U << 1U;
	static constexpr std::uint32_t guard_exact_start = 1U << 2U;
	static constexpr std::uint32_t guard_exact_end = 1U << 3U;
	static constexpr std::uint32_t guard_mask = 0x0000000fU;
};

struct Frequency10sInputView final {
	std::uint32_t sequence{};
	std::uint32_t configuration_generation{};
	std::uint32_t sample_rate_hz{};
	std::uint32_t measured_sample_rate_millihz{};
	std::uint8_t nominal_frequency_hz{};
	std::uint8_t reference_channel{};
	std::uint8_t filter_profile{};
	std::uint8_t calibration_profile{};
	std::uint32_t status{};
	std::uint32_t reason{};
	std::uint64_t interval_start_sample{};
	std::uint64_t interval_end_sample{};
	std::uint64_t utc_start_nanoseconds{};
	std::uint64_t utc_end_nanoseconds{};
	std::uint64_t utc_uncertainty_nanoseconds{};
	std::uint32_t boundary_generation{};
	std::uint16_t crossing_count{};
	std::uint32_t observer_drop_count{};
	std::uint8_t guard_flags{};
	const std::uint32_t *crossing_words{};

	[[nodiscard]] std::int64_t crossing_q16(
		std::size_t index) const noexcept;
};

static_assert(Frequency10sProtocol::frame_words <=
	maximum_transport_frame_words);

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_FREQUENCY_10S_PROTOCOL_HPP
