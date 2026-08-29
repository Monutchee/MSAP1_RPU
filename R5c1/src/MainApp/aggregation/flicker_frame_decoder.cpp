#include "flicker_frame_decoder.hpp"

#include "crc32c.hpp"

namespace msap1::aggregation {
namespace {

std::uint64_t u64(const std::uint32_t *words, std::size_t index) noexcept
{
	return static_cast<std::uint64_t>(words[index]) |
		(static_cast<std::uint64_t>(words[index + 1U]) << 32U);
}

bool supported_sample_rate(std::uint32_t rate) noexcept
{
	return rate >= 2000U && rate <= 128000U && rate % 2000U == 0U;
}

} // namespace

FrameValidationError FlickerFrameDecoder::decode(
	const AggregationFrame &frame, FlickerInputView &output) const noexcept
{
	if (frame.word_count != FlickerProtocol::frame_words)
		return FrameValidationError::invalid_length;
	const auto &words = frame.words;
	if (words[FlickerProtocol::magic_index] != FlickerProtocol::magic)
		return FrameValidationError::invalid_magic;
	if (words[FlickerProtocol::contract_revision_index] !=
		FlickerProtocol::contract_revision)
		return FrameValidationError::contract_mismatch;
	if (words[FlickerProtocol::payload_count_index] !=
		FlickerProtocol::payload_words)
		return FrameValidationError::invalid_payload_count;
	if (words[FlickerProtocol::crc_index] != crc32c_words(words.data(),
			FlickerProtocol::crc_index))
		return FrameValidationError::crc_mismatch;

	const auto *payload = &words[FlickerProtocol::payload_index];
	if (words[FlickerProtocol::transport_sequence_index] !=
		payload[FlickerProtocol::sequence_word])
		return FrameValidationError::sequence_mismatch;
	const auto model = payload[FlickerProtocol::model_word];
	const auto timing = payload[FlickerProtocol::timing_word];
	const auto lamp = static_cast<std::uint16_t>(model & 0xffffU);
	const auto nominal = static_cast<std::uint8_t>((model >> 16U) & 0xffU);
	const auto live_ms = static_cast<std::uint16_t>(timing & 0xffffU);
	const auto pst_seconds = static_cast<std::uint16_t>(timing >> 16U);
	const auto kind = payload[FlickerProtocol::kind_word];
	const auto histogram_base = payload[FlickerProtocol::histogram_base_word];
	if (payload[FlickerProtocol::generation_word] == 0U ||
		!supported_sample_rate(payload[FlickerProtocol::sample_rate_word]) ||
		(payload[FlickerProtocol::status_word] &
			~FlickerProtocol::status_mask) != 0U ||
		(payload[FlickerProtocol::phase_mask_word] & ~0x7U) != 0U ||
		kind > FlickerProtocol::kind_histogram ||
		(model & 0xff000000U) != 0U ||
		(lamp != 120U && lamp != 230U) ||
		(nominal != 50U && nominal != 60U) || live_ms != 1000U ||
		pst_seconds != 600U)
		return FrameValidationError::reserved_bits_nonzero;

	const auto first_sample = u64(payload, FlickerProtocol::first_sample_word);
	const auto last_sample = u64(payload, FlickerProtocol::last_sample_word);
	const auto maximum_count = kind == FlickerProtocol::kind_live
		? 2000U : 600U * 2000U;
	const auto interval_samples = static_cast<std::uint64_t>(
		payload[FlickerProtocol::sample_rate_word]) *
		(kind == FlickerProtocol::kind_live ? 1U : 600U);
	if (last_sample < first_sample ||
		last_sample - first_sample + 1U != interval_samples)
		return FrameValidationError::invalid_record_geometry;
	for (std::size_t phase = 0U; phase < FlickerProtocol::phases; ++phase) {
		const auto count = payload[FlickerProtocol::valid_count_word + phase];
		if (count > maximum_count ||
			((payload[FlickerProtocol::phase_mask_word] & (1U << phase)) != 0U &&
				count != maximum_count))
			return FrameValidationError::invalid_record_geometry;
	}

	if (kind == FlickerProtocol::kind_live) {
		if (histogram_base != 0U)
			return FrameValidationError::invalid_record_geometry;
		for (std::size_t word = FlickerProtocol::histogram_word;
			word < FlickerProtocol::payload_words; ++word)
			if (payload[word] != 0U)
				return FrameValidationError::reserved_bits_nonzero;
	} else {
		if (histogram_base >= FlickerProtocol::classifier_bins ||
			histogram_base % FlickerProtocol::histogram_bins != 0U)
			return FrameValidationError::invalid_record_geometry;
		for (std::size_t phase = 0U; phase < FlickerProtocol::phases; ++phase)
			for (std::size_t offset = 0U;
				offset < FlickerProtocol::histogram_bins; ++offset)
				if (histogram_base + offset >= FlickerProtocol::classifier_bins &&
					payload[FlickerProtocol::histogram_word +
						phase * FlickerProtocol::histogram_bins + offset] != 0U)
					return FrameValidationError::reserved_bits_nonzero;
	}

	output = {};
	output.sequence = payload[FlickerProtocol::sequence_word];
	output.configuration_generation =
		payload[FlickerProtocol::generation_word];
	output.sample_rate_hz = payload[FlickerProtocol::sample_rate_word];
	output.status = payload[FlickerProtocol::status_word];
	output.phase_mask = static_cast<std::uint8_t>(
		payload[FlickerProtocol::phase_mask_word]);
	output.kind = static_cast<std::uint8_t>(kind);
	output.lamp_voltage = lamp;
	output.nominal_hz = nominal;
	output.live_cadence_ms = live_ms;
	output.pst_interval_seconds = pst_seconds;
	output.histogram_base = static_cast<std::uint16_t>(histogram_base);
	output.first_sample = first_sample;
	output.last_sample = last_sample;
	for (std::size_t phase = 0U; phase < FlickerProtocol::phases; ++phase) {
		output.valid_count[phase] =
			payload[FlickerProtocol::valid_count_word + phase];
		output.pinst_q16[phase] = payload[FlickerProtocol::pinst_word + phase];
		for (std::size_t offset = 0U;
			offset < FlickerProtocol::histogram_bins; ++offset)
			output.histogram[phase][offset] =
				payload[FlickerProtocol::histogram_word +
					phase * FlickerProtocol::histogram_bins + offset];
	}
	return FrameValidationError::none;
}

} // namespace msap1::aggregation
