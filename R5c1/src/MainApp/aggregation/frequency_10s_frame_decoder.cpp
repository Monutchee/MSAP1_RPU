#include "frequency_10s_frame_decoder.hpp"

#include "crc32c.hpp"

#include <bit>

namespace msap1::aggregation {
namespace {

std::uint64_t u64(const std::uint32_t *words, std::size_t index) noexcept
{
	return static_cast<std::uint64_t>(words[index]) |
		(static_cast<std::uint64_t>(words[index + 1U]) << 32U);
}

std::int64_t s64(const std::uint32_t *words, std::size_t index) noexcept
{
	return std::bit_cast<std::int64_t>(u64(words, index));
}

bool supported_sample_rate(std::uint32_t rate) noexcept
{
	return rate == 1000U || rate == 2000U || rate == 4000U ||
		rate == 8000U || rate == 16000U || rate == 32000U ||
		rate == 64000U || rate == 128000U;
}

} // namespace

std::int64_t Frequency10sInputView::crossing_q16(
	std::size_t index) const noexcept
{
	if (crossing_words == nullptr || index >= crossing_count)
		return 0;
	return s64(crossing_words, index * 2U);
}

FrameValidationError Frequency10sFrameDecoder::decode(
	const AggregationFrame &frame, Frequency10sInputView &output) const noexcept
{
	using Protocol = Frequency10sProtocol;
	if (frame.word_count != Protocol::frame_words)
		return FrameValidationError::invalid_length;
	const auto &words = frame.words;
	if (words[Protocol::magic_index] != Protocol::magic)
		return FrameValidationError::invalid_magic;
	if (words[Protocol::contract_revision_index] !=
		Protocol::contract_revision)
		return FrameValidationError::contract_mismatch;
	if (words[Protocol::payload_count_index] != Protocol::payload_words)
		return FrameValidationError::invalid_payload_count;
	if (words[Protocol::crc_index] != crc32c_words(words.data(),
			Protocol::crc_index))
		return FrameValidationError::crc_mismatch;

	const auto *payload = &words[Protocol::payload_index];
	if (words[Protocol::transport_sequence_index] !=
		payload[Protocol::sequence_word])
		return FrameValidationError::sequence_mismatch;
	if (payload[Protocol::generation_word] == 0U ||
		!supported_sample_rate(payload[Protocol::sample_rate_hz_word]) ||
		payload[Protocol::measured_sample_rate_millihz_word] == 0U ||
		payload[Protocol::measured_sample_rate_millihz_word] > 200000000U ||
		(payload[Protocol::nominal_frequency_hz_word] != 50U &&
		 payload[Protocol::nominal_frequency_hz_word] != 60U) ||
		(payload[Protocol::profile_word] &
		 Protocol::profile_reserved_mask) != 0U ||
		(payload[Protocol::status_word] & ~Protocol::status_mask) != 0U ||
		(payload[Protocol::reason_word] & ~Protocol::reason_mask) != 0U ||
		payload[Protocol::crossing_count_word] >
		 Protocol::crossing_capacity ||
		(payload[Protocol::guard_flags_word] & ~Protocol::guard_mask) != 0U ||
		payload[Protocol::reserved_word] != 0U ||
		payload[Protocol::reserved_word + 1U] != 0U)
		return FrameValidationError::reserved_bits_nonzero;

	const auto start = u64(payload, Protocol::interval_start_sample_word);
	const auto end = u64(payload, Protocol::interval_end_sample_word);
	if (end <= start || end - start > 2000000U)
		return FrameValidationError::invalid_record_geometry;

	const auto utc_start = u64(payload, Protocol::utc_start_nanoseconds_word);
	const auto utc_end = u64(payload, Protocol::utc_end_nanoseconds_word);
	if ((payload[Protocol::status_word] & Protocol::status_boundary_valid) != 0U &&
		(utc_end <= utc_start || utc_end - utc_start != 10000000000ULL))
		return FrameValidationError::invalid_record_geometry;

	const auto count = static_cast<std::size_t>(
		payload[Protocol::crossing_count_word]);
	const auto *crossings = &payload[Protocol::crossing_base_word];
	std::int64_t previous{};
	bool has_exact_start = false;
	bool has_exact_end = false;
	for (std::size_t index = 0U; index < count; ++index) {
		const auto crossing = s64(crossings, index * 2U);
		if (index != 0U && crossing <= previous)
			return FrameValidationError::invalid_record_geometry;
		has_exact_start = has_exact_start || crossing == 0;
		has_exact_end = has_exact_end ||
			(crossing >= 0 && static_cast<std::uint64_t>(crossing) ==
				((end - start) << 16U));
		previous = crossing;
	}
	const auto guards = payload[Protocol::guard_flags_word];
	const bool has_before = count != 0U && s64(crossings, 0U) < 0;
	const bool has_after = count != 0U && previous >= 0 &&
		static_cast<std::uint64_t>(previous) > ((end - start) << 16U);
	if (has_before != ((guards & Protocol::guard_before_start) != 0U) ||
		has_after != ((guards & Protocol::guard_after_end) != 0U) ||
		has_exact_start != ((guards & Protocol::guard_exact_start) != 0U) ||
		has_exact_end != ((guards & Protocol::guard_exact_end) != 0U))
		return FrameValidationError::invalid_record_geometry;
	for (std::size_t index = count * 2U;
		index < Protocol::crossing_words; ++index)
		if (crossings[index] != 0U)
			return FrameValidationError::reserved_bits_nonzero;

	output = {};
	output.sequence = payload[Protocol::sequence_word];
	output.configuration_generation = payload[Protocol::generation_word];
	output.sample_rate_hz = payload[Protocol::sample_rate_hz_word];
	output.measured_sample_rate_millihz =
		payload[Protocol::measured_sample_rate_millihz_word];
	output.nominal_frequency_hz = static_cast<std::uint8_t>(
		payload[Protocol::nominal_frequency_hz_word]);
	const auto profile = payload[Protocol::profile_word];
	output.reference_channel = static_cast<std::uint8_t>(
		profile & Protocol::profile_reference_mask);
	output.filter_profile = static_cast<std::uint8_t>(
		(profile >> Protocol::profile_filter_shift) & 0xffU);
	output.calibration_profile = static_cast<std::uint8_t>(
		(profile >> Protocol::profile_calibration_shift) & 0xffU);
	output.status = payload[Protocol::status_word];
	output.reason = payload[Protocol::reason_word];
	output.interval_start_sample = start;
	output.interval_end_sample = end;
	output.utc_start_nanoseconds = utc_start;
	output.utc_end_nanoseconds = utc_end;
	output.utc_uncertainty_nanoseconds = u64(payload,
		Protocol::utc_uncertainty_nanoseconds_word);
	output.boundary_generation = payload[Protocol::boundary_generation_word];
	output.crossing_count = static_cast<std::uint16_t>(count);
	output.observer_drop_count = payload[Protocol::observer_drop_count_word];
	output.guard_flags = static_cast<std::uint8_t>(
		payload[Protocol::guard_flags_word]);
	output.crossing_words = crossings;
	return FrameValidationError::none;
}

} // namespace msap1::aggregation
