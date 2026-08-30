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
	return rate >= 2000U && rate <= 128000U && rate % 2000U == 0U &&
		rate / 2000U <= 64U;
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
	const auto phase_mask = payload[FlickerProtocol::phase_mask_word];
	const auto actual_count = payload[FlickerProtocol::actual_count_word];
	const auto batch_status = payload[FlickerProtocol::batch_status_word];
	if (payload[FlickerProtocol::generation_word] == 0U ||
		!supported_sample_rate(payload[FlickerProtocol::sample_rate_word]) ||
		payload[FlickerProtocol::frame_capacity_word] !=
			FlickerProtocol::batch_frames ||
		phase_mask == 0U || (phase_mask & ~0x7U) != 0U ||
		(model & 0xff000000U) != 0U ||
		(lamp != 120U && lamp != 230U) ||
		(nominal != 50U && nominal != 60U) || live_ms != 1000U ||
		pst_seconds != 600U ||
		payload[FlickerProtocol::reference_microvolts_word] == 0U ||
		(batch_status & ~FlickerProtocol::batch_status_mask) != 0U ||
		((batch_status & FlickerProtocol::batch_source_drop) != 0U &&
			(batch_status & FlickerProtocol::batch_discontinuity) == 0U))
		return FrameValidationError::reserved_bits_nonzero;

	const auto first_sample = u64(payload, FlickerProtocol::first_sample_word);
	const auto last_sample = u64(payload, FlickerProtocol::last_sample_word);
	if (actual_count == 0U || actual_count > FlickerProtocol::batch_frames ||
		last_sample < first_sample ||
		last_sample - first_sample + 1U != actual_count ||
		(actual_count != FlickerProtocol::batch_frames &&
			(batch_status & (FlickerProtocol::batch_discontinuity |
				FlickerProtocol::batch_source_drop)) !=
				(FlickerProtocol::batch_discontinuity |
					FlickerProtocol::batch_source_drop)))
		return FrameValidationError::invalid_record_geometry;
	for (std::size_t sample = 0U; sample < actual_count; ++sample) {
		const auto packed_word = payload[FlickerProtocol::sample_word +
			sample * FlickerProtocol::words_per_sample + 4U];
		if ((packed_word & FlickerProtocol::packed_reserved_mask) != 0U)
			return FrameValidationError::reserved_bits_nonzero;
	}
	for (std::size_t sample = actual_count;
		sample < FlickerProtocol::batch_frames; ++sample)
		for (std::size_t word = 0U;
			word < FlickerProtocol::words_per_sample; ++word)
			if (payload[FlickerProtocol::sample_word +
				sample * FlickerProtocol::words_per_sample + word] != 0U)
				return FrameValidationError::reserved_bits_nonzero;

	output = {};
	output.sequence = payload[FlickerProtocol::sequence_word];
	output.configuration_generation =
		payload[FlickerProtocol::generation_word];
	output.sample_rate_hz = payload[FlickerProtocol::sample_rate_word];
	output.phase_mask = static_cast<std::uint8_t>(phase_mask);
	output.lamp_voltage = lamp;
	output.nominal_hz = nominal;
	output.live_cadence_ms = live_ms;
	output.pst_interval_seconds = pst_seconds;
	output.reference_microvolts =
		payload[FlickerProtocol::reference_microvolts_word];
	output.first_sample = first_sample;
	output.last_sample = last_sample;
	output.actual_count = static_cast<std::uint16_t>(actual_count);
	output.batch_status = static_cast<std::uint8_t>(batch_status);
	output.packed_sample_words = payload + FlickerProtocol::sample_word;
	return FrameValidationError::none;
}

} // namespace msap1::aggregation
