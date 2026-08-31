#include "pq_event_frame_decoder.hpp"

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
	return rate == 1000U || rate == 2000U || rate == 4000U ||
		rate == 8000U || rate == 16000U || rate == 32000U ||
		rate == 64000U || rate == 128000U;
}

} // namespace

FrameValidationError PqEventFrameDecoder::decode(
	const AggregationFrame &frame, PqEventInputView &output) const noexcept
{
	if (frame.word_count != PqEventProtocol::frame_words)
		return FrameValidationError::invalid_length;
	const auto &words = frame.words;
	if (words[PqEventProtocol::magic_index] != PqEventProtocol::magic)
		return FrameValidationError::invalid_magic;
	if (words[PqEventProtocol::contract_revision_index] !=
		PqEventProtocol::contract_revision)
		return FrameValidationError::contract_mismatch;
	if (words[PqEventProtocol::payload_count_index] !=
		PqEventProtocol::payload_words)
		return FrameValidationError::invalid_payload_count;
	if (words[PqEventProtocol::crc_index] != crc32c_words(words.data(),
			PqEventProtocol::crc_index))
		return FrameValidationError::crc_mismatch;

	const auto *payload = &words[PqEventProtocol::payload_index];
	if (words[PqEventProtocol::transport_sequence_index] !=
		payload[PqEventProtocol::sequence_word])
		return FrameValidationError::sequence_mismatch;
	if (payload[PqEventProtocol::generation_word] == 0U ||
		!supported_sample_rate(payload[PqEventProtocol::sample_rate_word]) ||
		payload[PqEventProtocol::window_samples_word] == 0U ||
		(payload[PqEventProtocol::status_word] &
			~PqEventProtocol::status_mask) != 0U ||
		(payload[PqEventProtocol::valid_phases_word] &
			~PqEventProtocol::valid_phases_mask) != 0U ||
		payload[PqEventProtocol::apply_word] > 1U)
		return FrameValidationError::reserved_bits_nonzero;
	for (std::size_t word = PqEventProtocol::reserved_word;
		word < PqEventProtocol::payload_words; ++word)
		if (payload[word] != 0U)
			return FrameValidationError::reserved_bits_nonzero;

	const auto first_sample = u64(payload, PqEventProtocol::first_sample_word);
	const auto last_sample = u64(payload, PqEventProtocol::last_sample_word);
	if (last_sample < first_sample ||
		last_sample - first_sample + 1U !=
			payload[PqEventProtocol::window_samples_word])
		return FrameValidationError::invalid_record_geometry;

	output = {};
	output.sequence = payload[PqEventProtocol::sequence_word];
	output.configuration_generation = payload[PqEventProtocol::generation_word];
	output.sample_rate_hz = payload[PqEventProtocol::sample_rate_word];
	output.status = payload[PqEventProtocol::status_word];
	output.voltage_valid_mask = static_cast<std::uint8_t>(
		payload[PqEventProtocol::valid_phases_word] & 0x7U);
	output.current_valid_mask = static_cast<std::uint8_t>(
		(payload[PqEventProtocol::valid_phases_word] >> 8U) & 0x7U);
	output.window_samples = payload[PqEventProtocol::window_samples_word];
	output.first_sample = first_sample;
	output.last_sample = last_sample;
	output.pl_tick = u64(payload, PqEventProtocol::pl_tick_word);
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		output.urms_q16[phase] = u64(payload,
			PqEventProtocol::urms_q16_word + phase * 2U);
		output.irms_q16[phase] = u64(payload,
			PqEventProtocol::irms_q16_word + phase * 2U);
	}
	output.m12_reference_microvolts = payload[PqEventProtocol::reference_word];
	output.m12_sag_threshold_e4 = payload[PqEventProtocol::sag_threshold_word];
	output.m12_swell_threshold_e4 = payload[PqEventProtocol::swell_threshold_word];
	output.m12_interruption_threshold_e4 =
		payload[PqEventProtocol::interruption_threshold_word];
	output.m12_hysteresis_e4 = payload[PqEventProtocol::hysteresis_word];
	output.apply_toggle = payload[PqEventProtocol::apply_word];
	return FrameValidationError::none;
}

} // namespace msap1::aggregation
