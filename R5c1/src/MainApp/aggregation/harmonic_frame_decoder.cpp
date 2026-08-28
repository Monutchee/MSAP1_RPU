#include "harmonic_frame_decoder.hpp"

#include "crc32c.hpp"

#include <algorithm>

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t known_status_mask = 0xFFU;
constexpr std::uint64_t magnitude_mask = (std::uint64_t{1} << 40U) - 1U;
constexpr std::uint64_t angle_mask = (std::uint64_t{1} << 20U) - 1U;

std::uint64_t u64(const std::uint32_t *words, std::size_t index) noexcept
{
	return static_cast<std::uint64_t>(words[index]) |
		(static_cast<std::uint64_t>(words[index + 1U]) << 32U);
}

std::uint32_t expected_entry_count(std::size_t chunk) noexcept
{
	const auto first = chunk * HarmonicProtocol::orders_per_chunk + 1U;
	return static_cast<std::uint32_t>(std::min(
		HarmonicProtocol::orders_per_chunk,
		HarmonicProtocol::maximum_order - first + 1U));
}

bool same_provenance(const std::uint32_t *first,
	const std::uint32_t *candidate) noexcept
{
	for (const auto word : {3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U,
		14U, 15U})
		if (candidate[word] != first[word])
			return false;
	return true;
}

} // namespace

FrameValidationError HarmonicFrameDecoder::decode(
	const AggregationFrame &frame, HarmonicInputView &output) const noexcept
{
	if (frame.word_count != HarmonicProtocol::frame_words)
		return FrameValidationError::invalid_length;
	const auto &words = frame.words;
	if (words[HarmonicProtocol::magic_index] != HarmonicProtocol::magic)
		return FrameValidationError::invalid_magic;
	if (words[HarmonicProtocol::contract_revision_index] !=
		HarmonicProtocol::contract_revision)
		return FrameValidationError::contract_mismatch;
	if (words[HarmonicProtocol::payload_count_index] !=
		HarmonicProtocol::payload_words)
		return FrameValidationError::invalid_payload_count;
	if (words[HarmonicProtocol::crc_index] != crc32c_words(words.data(),
		HarmonicProtocol::crc_index))
		return FrameValidationError::crc_mismatch;

	const auto *first = &words[HarmonicProtocol::payload_index];
	if (words[HarmonicProtocol::transport_sequence_index] != first[3U])
		return FrameValidationError::sequence_mismatch;

	for (std::size_t record_index = 0U;
		record_index < HarmonicProtocol::records_per_family; ++record_index) {
		const auto *record = first + record_index * HarmonicProtocol::record_words;
		if (record[0U] != HarmonicProtocol::record_magic ||
			record[1U] != HarmonicProtocol::base_record_format ||
			record[2U] != HarmonicProtocol::record_bytes ||
			(record[7U] & ~0x7FU) != 0U || record[5U] == 0U ||
			record[6U] == 0U || (record[8U] & ~known_status_mask) != 0U ||
			(record[8U] & (1U << 1U)) == 0U)
			return FrameValidationError::invalid_record_geometry;
		if (!same_provenance(first, record))
			return FrameValidationError::provenance_mismatch;

		const auto channel = record_index /
			HarmonicProtocol::chunks_per_channel;
		const auto chunk = record_index % HarmonicProtocol::chunks_per_channel;
		const auto header = record[13U];
		const auto first_order = chunk * HarmonicProtocol::orders_per_chunk + 1U;
		const auto entry_count = expected_entry_count(chunk);
		if ((header & 0x7U) != channel || ((header >> 3U) & 0xFU) != chunk ||
			((header >> 7U) & 0xFFU) != first_order ||
			((header >> 15U) & 0x1FU) != entry_count ||
			((header >> 20U) & 0xFU) !=
				HarmonicProtocol::chunks_per_channel ||
			(header >> 24U) != HarmonicProtocol::maximum_order)
			return FrameValidationError::invalid_record_geometry;

		for (std::size_t entry = 0U;
			entry < HarmonicProtocol::orders_per_chunk; ++entry) {
			const auto packed = u64(record, 16U + entry * 2U);
			if (entry >= entry_count) {
				if (packed != 0U)
					return FrameValidationError::invalid_record_geometry;
				continue;
			}
			if ((packed >> 62U) != 0U)
				return FrameValidationError::reserved_bits_nonzero;
			const bool magnitude_valid = (packed &
				(std::uint64_t{1} << 60U)) != 0U;
			const bool angle_valid = (packed &
				(std::uint64_t{1} << 61U)) != 0U;
			const auto magnitude = packed & magnitude_mask;
			const auto angle = (packed >> 40U) & angle_mask;
			const auto order = first_order + entry;
			const auto qualified = first[15U] & 0xFFU;
			if ((!magnitude_valid && magnitude != 0U) ||
				(!angle_valid && angle != 0U) ||
				(angle_valid && (!magnitude_valid || angle >= 360000U)) ||
				(order > qualified && (magnitude_valid || angle_valid)) ||
				((first[7U] & (1U << channel)) == 0U &&
					(magnitude_valid || angle_valid)))
				return FrameValidationError::invalid_record_geometry;
		}
	}

	output.sequence = first[3U];
	output.configuration_generation = first[4U];
	output.sample_rate_hz = first[5U];
	output.sample_count = first[6U];
	output.valid_mask = static_cast<std::uint8_t>(first[7U]);
	output.status = first[8U];
	output.first_sample = u64(first, 9U);
	output.measured_frequency_millihz = first[14U];
	output.qualified_max_order = static_cast<std::uint8_t>(first[15U]);
	output.nominal_frequency_hz = static_cast<std::uint8_t>(first[15U] >> 8U);
	output.cycle_count = static_cast<std::uint8_t>(first[15U] >> 16U);
	output.filter_profile_id = static_cast<std::uint8_t>(first[15U] >> 24U);
	if (output.qualified_max_order > HarmonicProtocol::maximum_order ||
		!((output.nominal_frequency_hz == 50U && output.cycle_count == 10U) ||
		  (output.nominal_frequency_hz == 60U && output.cycle_count == 12U)))
		return FrameValidationError::invalid_record_geometry;
	output.records = first;
	return FrameValidationError::none;
}

} // namespace msap1::aggregation
