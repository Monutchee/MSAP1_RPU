#include "voltage_sample_frame_decoder.hpp"

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

FrameValidationError VoltageSampleFrameDecoder::decode(
	const AggregationFrame &frame, VoltageSampleInputView &output) const noexcept
{
	if (frame.word_count != VoltageSampleProtocol::frame_words)
		return FrameValidationError::invalid_length;
	const auto &words = frame.words;
	if (words[VoltageSampleProtocol::magic_index] !=
		VoltageSampleProtocol::magic)
		return FrameValidationError::invalid_magic;
	if (words[VoltageSampleProtocol::contract_revision_index] !=
		VoltageSampleProtocol::contract_revision)
		return FrameValidationError::contract_mismatch;
	if (words[VoltageSampleProtocol::payload_count_index] !=
		VoltageSampleProtocol::payload_words)
		return FrameValidationError::invalid_payload_count;
	if (words[VoltageSampleProtocol::crc_index] != crc32c_words(words.data(),
			VoltageSampleProtocol::crc_index))
		return FrameValidationError::crc_mismatch;

	const auto *payload = &words[VoltageSampleProtocol::payload_index];
	if (words[VoltageSampleProtocol::transport_sequence_index] !=
		payload[VoltageSampleProtocol::sequence_word])
		return FrameValidationError::sequence_mismatch;
	const auto model = payload[VoltageSampleProtocol::model_word];
	const auto timing = payload[VoltageSampleProtocol::timing_word];
	const auto lamp = static_cast<std::uint16_t>(model & 0xffffU);
	const auto nominal = static_cast<std::uint8_t>((model >> 16U) & 0xffU);
	const auto live_ms = static_cast<std::uint16_t>(timing & 0xffffU);
	const auto pst_seconds = static_cast<std::uint16_t>(timing >> 16U);
	const auto phase_mask = payload[VoltageSampleProtocol::phase_mask_word];
	const auto actual_count = payload[VoltageSampleProtocol::actual_count_word];
	const auto batch_status = payload[VoltageSampleProtocol::batch_status_word];
	if (payload[VoltageSampleProtocol::generation_word] == 0U ||
		!supported_sample_rate(
			payload[VoltageSampleProtocol::sample_rate_word]) ||
		payload[VoltageSampleProtocol::frame_capacity_word] !=
			VoltageSampleProtocol::batch_frames ||
		phase_mask == 0U || (phase_mask & ~0x7U) != 0U ||
		(model & 0xff000000U) != 0U ||
		(lamp != 120U && lamp != 230U) ||
		(nominal != 50U && nominal != 60U) || live_ms != 1000U ||
		pst_seconds != 600U ||
		payload[VoltageSampleProtocol::reference_microvolts_word] == 0U ||
		(batch_status & ~VoltageSampleProtocol::batch_status_mask) != 0U ||
		((batch_status & VoltageSampleProtocol::batch_source_drop) != 0U &&
			(batch_status & VoltageSampleProtocol::batch_discontinuity) == 0U))
		return FrameValidationError::reserved_bits_nonzero;

	const auto first_sample =
		u64(payload, VoltageSampleProtocol::first_sample_word);
	const auto last_sample =
		u64(payload, VoltageSampleProtocol::last_sample_word);
	if (actual_count == 0U ||
		actual_count > VoltageSampleProtocol::batch_frames ||
		last_sample < first_sample ||
		last_sample - first_sample + 1U != actual_count ||
		(actual_count != VoltageSampleProtocol::batch_frames &&
			(batch_status & (VoltageSampleProtocol::batch_discontinuity |
				VoltageSampleProtocol::batch_source_drop)) !=
				(VoltageSampleProtocol::batch_discontinuity |
					VoltageSampleProtocol::batch_source_drop)))
		return FrameValidationError::invalid_record_geometry;
	for (std::size_t sample = 0U; sample < actual_count; ++sample) {
		const auto flags = payload[VoltageSampleProtocol::sample_word +
			sample * VoltageSampleProtocol::words_per_sample + 3U];
		if ((flags & VoltageSampleProtocol::packed_reserved_mask) != 0U)
			return FrameValidationError::reserved_bits_nonzero;
	}
	for (std::size_t sample = actual_count;
		sample < VoltageSampleProtocol::batch_frames; ++sample)
		for (std::size_t word = 0U;
			word < VoltageSampleProtocol::words_per_sample; ++word)
			if (payload[VoltageSampleProtocol::sample_word +
				sample * VoltageSampleProtocol::words_per_sample + word] != 0U)
				return FrameValidationError::reserved_bits_nonzero;

	output = {};
	output.sequence = payload[VoltageSampleProtocol::sequence_word];
	output.configuration_generation =
		payload[VoltageSampleProtocol::generation_word];
	output.sample_rate_hz = payload[VoltageSampleProtocol::sample_rate_word];
	output.phase_mask = static_cast<std::uint8_t>(phase_mask);
	output.lamp_voltage = lamp;
	output.nominal_hz = nominal;
	output.live_cadence_ms = live_ms;
	output.pst_interval_seconds = pst_seconds;
	output.reference_microvolts =
		payload[VoltageSampleProtocol::reference_microvolts_word];
	output.first_sample = first_sample;
	output.last_sample = last_sample;
	output.actual_count = static_cast<std::uint16_t>(actual_count);
	output.batch_status = static_cast<std::uint8_t>(batch_status);
	output.packed_sample_words =
		payload + VoltageSampleProtocol::sample_word;
	return FrameValidationError::none;
}

} // namespace msap1::aggregation
