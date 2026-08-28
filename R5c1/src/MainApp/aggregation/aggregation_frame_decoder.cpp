#include "aggregation_frame_decoder.hpp"

#include "crc32c.hpp"

namespace msap1::aggregation {

FrameValidationError AggregationFrameDecoder::decode(
	const AggregationFrame &frame, AggregationInputView &output) const noexcept
{
	const auto &words = frame.words;
	if (frame.word_count != AggregationProtocol::frame_words)
		return FrameValidationError::invalid_length;
	if (words[AggregationProtocol::magic_index] != AggregationProtocol::magic)
		return FrameValidationError::invalid_magic;
	if (words[AggregationProtocol::contract_revision_index] !=
		AggregationProtocol::contract_revision)
		return FrameValidationError::contract_mismatch;
	if (words[AggregationProtocol::payload_count_index] !=
		AggregationProtocol::payload_words)
		return FrameValidationError::invalid_payload_count;
	if (words[AggregationProtocol::transport_sequence_index] !=
		words[AggregationProtocol::payload_index])
		return FrameValidationError::sequence_mismatch;

	const std::uint32_t expected_crc = crc32c_words(words.data(),
		AggregationProtocol::crc_index);
	if (words[AggregationProtocol::crc_index] != expected_crc)
		return FrameValidationError::crc_mismatch;

	const std::size_t context = AggregationProtocol::context_index;
	output.sequence = words[AggregationProtocol::transport_sequence_index];
	output.single_cycle_words = &words[AggregationProtocol::payload_index];
	output.single_cycle_word_count = AggregationProtocol::single_cycle_words;
	output.context.configuration_generation = words[context + 0U];
	output.context.sample_rate_hz = words[context + 1U];
	output.context.control_status = words[context + 2U];
	output.context.frequency_status = words[context + 3U];
	output.context.frequency_period_q16 = words[context + 4U];
	output.context.frequency_sequence = words[context + 5U];
	output.context.capture_frame_count = words[context + 6U];
	output.context.header_error_count = words[context + 7U];
	output.context.overflow_count = words[context + 8U];
	output.context.alert_status = words[context + 9U];
	output.context.utc_target_sample =
		static_cast<std::uint64_t>(words[context + 10U]) |
		(static_cast<std::uint64_t>(words[context + 11U]) << 32U);
	output.context.utc_target_status = words[context + 12U];
	if ((output.context.control_status &
		~AggregationProtocol::control_status_mask) != 0U ||
		(output.context.utc_target_status &
		~AggregationProtocol::utc_target_status_mask) != 0U)
		return FrameValidationError::reserved_bits_nonzero;
	return FrameValidationError::none;
}

} // namespace msap1::aggregation
