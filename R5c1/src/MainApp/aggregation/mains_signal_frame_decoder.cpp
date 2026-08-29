#include "mains_signal_frame_decoder.hpp"

#include "crc32c.hpp"

#include <cstdint>

namespace msap1::aggregation {
namespace {

std::uint64_t u64(const std::uint32_t *words, std::size_t index) noexcept
{
	return static_cast<std::uint64_t>(words[index]) |
		(static_cast<std::uint64_t>(words[index + 1U]) << 32U);
}

bool supported_sample_rate(std::uint32_t rate) noexcept
{
	switch (rate) {
	case 2000U:
	case 4000U:
	case 8000U:
	case 16000U:
	case 32000U:
	case 64000U:
	case 128000U:
		return true;
	default:
		return false;
	}
}

} // namespace

FrameValidationError MainsSignalFrameDecoder::decode(
	const AggregationFrame &frame, MainsSignalInputView &output) const noexcept
{
	if (frame.word_count != MainsSignalProtocol::frame_words)
		return FrameValidationError::invalid_length;
	const auto &words = frame.words;
	if (words[MainsSignalProtocol::magic_index] != MainsSignalProtocol::magic)
		return FrameValidationError::invalid_magic;
	if (words[MainsSignalProtocol::contract_revision_index] !=
		MainsSignalProtocol::contract_revision)
		return FrameValidationError::contract_mismatch;
	if (words[MainsSignalProtocol::payload_count_index] !=
		MainsSignalProtocol::payload_words)
		return FrameValidationError::invalid_payload_count;
	if (words[MainsSignalProtocol::crc_index] != crc32c_words(words.data(),
			MainsSignalProtocol::crc_index))
		return FrameValidationError::crc_mismatch;

	const auto *payload = &words[MainsSignalProtocol::payload_index];
	const auto sequence = payload[MainsSignalProtocol::sequence_word];
	if (sequence == 0U ||
		words[MainsSignalProtocol::transport_sequence_index] != sequence)
		return FrameValidationError::sequence_mismatch;
	const auto generation = payload[MainsSignalProtocol::generation_word];
	const auto sample_rate = payload[MainsSignalProtocol::sample_rate_word];
	const auto status = payload[MainsSignalProtocol::status_word];
	const auto phases = payload[MainsSignalProtocol::phases_word];
	const auto valid_mask = static_cast<std::uint8_t>(phases & 0x7U);
	const auto detected_mask =
		static_cast<std::uint8_t>((phases >> 8U) & 0x7U);
	const auto configured =
		payload[MainsSignalProtocol::configured_millihz_word];
	const auto measured = payload[MainsSignalProtocol::measured_millihz_word];
	const auto bandwidth =
		payload[MainsSignalProtocol::bandwidth_millihz_word];
	const auto observation =
		payload[MainsSignalProtocol::observation_ms_word];
	const auto threshold = payload[MainsSignalProtocol::threshold_e4_word];
	const auto first_sample =
		u64(payload, MainsSignalProtocol::first_sample_word);
	const auto last_sample =
		u64(payload, MainsSignalProtocol::last_sample_word);
	const auto nyquist_millihz =
		static_cast<std::uint64_t>(sample_rate) * 500U;

	if (generation == 0U || !supported_sample_rate(sample_rate) ||
		(status & ~MainsSignalProtocol::status_mask) != 0U ||
		(status & 1U) == 0U ||
		(phases & ~MainsSignalProtocol::phases_mask) != 0U ||
		(detected_mask & ~valid_mask) != 0U || configured == 0U ||
		bandwidth < 4U || bandwidth >= configured || observation != 200U ||
		threshold > 0xffffU ||
		static_cast<std::uint64_t>(configured) + bandwidth >=
			nyquist_millihz ||
		static_cast<std::uint64_t>(configured) + bandwidth >= 12500000U)
		return FrameValidationError::reserved_bits_nonzero;

	const auto expected_samples = static_cast<std::uint64_t>(sample_rate) / 5U;
	if (last_sample < first_sample ||
		last_sample - first_sample + 1U != expected_samples)
		return FrameValidationError::invalid_record_geometry;
	if (detected_mask == 0U) {
		if (measured != configured)
			return FrameValidationError::invalid_record_geometry;
	} else {
		const auto half_bandwidth = bandwidth / 2U;
		const auto lower = configured - half_bandwidth;
		const auto upper = static_cast<std::uint64_t>(configured) +
			half_bandwidth;
		if (measured < lower || measured > upper)
			return FrameValidationError::invalid_record_geometry;
	}

	output = {};
	output.sequence = sequence;
	output.configuration_generation = generation;
	output.sample_rate_hz = sample_rate;
	output.status = status;
	output.valid_phase_mask = valid_mask;
	output.detected_phase_mask = detected_mask;
	output.configured_millihz = configured;
	output.measured_millihz = measured;
	output.bandwidth_millihz = bandwidth;
	output.observation_ms = observation;
	output.first_sample = first_sample;
	output.last_sample = last_sample;
	for (std::size_t phase = 0U;
		phase < MainsSignalProtocol::phases; ++phase) {
		output.magnitude_microvolts[phase] = payload[
			MainsSignalProtocol::magnitude_microvolts_word + phase];
		output.background_microvolts[phase] = payload[
			MainsSignalProtocol::background_microvolts_word + phase];
	}
	output.threshold_e4 = threshold;
	return FrameValidationError::none;
}

} // namespace msap1::aggregation
