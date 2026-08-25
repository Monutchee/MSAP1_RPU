#include "r5_aggregation_engine.hpp"

namespace msap1::aggregation {

R5AggregationEngine::R5AggregationEngine(AggregationRecordSink &sink,
	AggregationHealth &health, AggregationOutputMode mode) noexcept
	: sink_(sink), health_(health), mode_(mode)
{
}

bool R5AggregationEngine::initialize() noexcept
{
	assembling_words_ = 0U;
	pass_records_completed_ = 0U;
	last_transport_sequence_ = 0U;
	discontinuity_pending_ = 0U;
	have_transport_sequence_ = false;
	ready_ = true;
	health_.set_engine_ready(true);
	// The health contract must describe the compiled runtime mode.  In shadow
	// mode records are checked but deliberately not returned to PL.  In emit
	// mode every completed record is part of the authoritative DMA stream.
	health_.set_authoritative(mode_ == AggregationOutputMode::emit);
	return true;
}

void R5AggregationEngine::note_transport_discontinuity() noexcept
{
	__atomic_store_n(&discontinuity_pending_, 1U, __ATOMIC_RELEASE);
}

bool R5AggregationEngine::accept_transport_sequence(std::uint32_t sequence,
	bool &discontinuity) noexcept
{
	discontinuity = false;
	if (!have_transport_sequence_) {
		have_transport_sequence_ = true;
		last_transport_sequence_ = sequence;
		return true;
	}

	const auto expected = last_transport_sequence_ + 1U;
	const auto delta = static_cast<std::int32_t>(sequence - expected);
	if (delta < 0)
		return false; // Duplicate or stale input must never be aggregated twice.

	discontinuity = delta > 0;
	last_transport_sequence_ = sequence;
	return true;
}

void R5AggregationEngine::fail_engine() noexcept
{
	ready_ = false;
	assembling_words_ = 0U;
	health_.set_engine_ready(false);
}

void R5AggregationEngine::record_completed_family(
	std::uint32_t format) noexcept
{
	switch (format) {
	case MREC_FORMAT_BASIC_V4:
		health_.record_basic_completed();
		break;
	case MREC_FORMAT_AGG_V3:
		health_.record_aggregate_completed();
		break;
	case MREC_FORMAT_TEN_MINUTE_V1:
		health_.record_ten_minute_completed();
		break;
	case MREC_FORMAT_TWO_HOUR_V1:
		health_.record_two_hour_completed();
		break;
	default:
		break;
	}
}

void R5AggregationEngine::complete_record() noexcept
{
	if (assembling_.words[MREC_MAGIC_WORD] != MREC_MAGIC ||
		assembling_.words[MREC_SIZE_WORD] != AggregationMeterRecord::byte_count) {
		fail_engine();
		return;
	}

	assembling_.sequence = assembling_.words[MREC_SEQUENCE_WORD];
	record_completed_family(assembling_.words[MREC_FORMAT_WORD]);

	if (mode_ == AggregationOutputMode::emit && !sink_.publish(assembling_)) {
		// An authoritative completed record must never be silently discarded.
		// A full output ring means the downstream path can no longer preserve
		// the lossless record contract, so fail closed and expose the fault.
		fail_engine();
		return;
	}

	++pass_records_completed_;

	assembling_ = {};
	assembling_words_ = 0U;
}

void R5AggregationEngine::accept_beat(const record_axis_t &beat) noexcept
{
	if (!ready_)
		return;

	const auto keep = static_cast<std::uint32_t>(beat.keep.to_uint());
	const auto strb = static_cast<std::uint32_t>(beat.strb.to_uint());
	const bool last = beat.last.to_uint() != 0U;
	if (keep != MREC_KEEP_ALL.to_uint() || strb != MREC_KEEP_ALL.to_uint() ||
		assembling_words_ >= AggregationMeterRecord::word_count) {
		fail_engine();
		return;
	}

	assembling_.words[assembling_words_++] = beat.data.to_uint();
	const bool expected_last =
		assembling_words_ == AggregationMeterRecord::word_count;
	if (last != expected_last) {
		fail_engine();
		return;
	}
	if (last)
		complete_record();
}

void R5AggregationEngine::drain(record_axis_stream_t &stream) noexcept
{
	record_axis_t beat{};
	while (ready_ && stream.read_nb(beat))
		accept_beat(beat);
}

std::size_t R5AggregationEngine::run_one_pass() noexcept
{
	pass_records_completed_ = 0U;
	hls_aggregation_engine(input_, basic_output_, aggregate_output_);
	drain(basic_output_);
	drain(aggregate_output_);

	if (input_.overflowed() || input_.underflowed() ||
		basic_output_.overflowed() || basic_output_.underflowed() ||
		aggregate_output_.overflowed() || aggregate_output_.underflowed())
		fail_engine();

	return pass_records_completed_;
}

void R5AggregationEngine::process(const AggregationInputView &input) noexcept
{
	if (!ready_ || input.single_cycle_words == nullptr ||
		input.single_cycle_word_count != AggregationProtocol::single_cycle_words)
		return;

	bool sequence_discontinuity = false;
	if (!accept_transport_sequence(input.sequence, sequence_discontinuity))
		return;
	const bool externally_reported_discontinuity =
		__atomic_exchange_n(&discontinuity_pending_, 0U,
			__ATOMIC_ACQ_REL) != 0U;
	const bool discontinuity = sequence_discontinuity ||
		externally_reported_discontinuity;

	input_.clear_errors();
	basic_output_.clear_errors();
	aggregate_output_.clear_errors();

	for (std::size_t word = 0U; word < input.single_cycle_word_count; ++word) {
		auto value = input.single_cycle_words[word];
		// SCYC word 9 is the single-cycle status word.  Bit 2 is the
		// established first-after-gap flag consumed by every aggregation tier.
		if (word == 9U && discontinuity)
			value |= 1U << 2U;
		input_.write(single_cycle_word_t(value));
	}

	const std::uint32_t context[AggregationProtocol::context_words] = {
		input.context.configuration_generation,
		input.context.sample_rate_hz,
		input.context.control_status,
		input.context.frequency_status,
		input.context.frequency_period_q16,
		input.context.frequency_sequence,
		input.context.capture_frame_count,
		input.context.header_error_count,
		input.context.overflow_count,
		input.context.alert_status,
		static_cast<std::uint32_t>(input.context.utc_target_sample),
		static_cast<std::uint32_t>(input.context.utc_target_sample >> 32U),
		input.context.utc_target_status,
	};
	for (const auto word : context)
		input_.write(single_cycle_word_t(word));

	// One call consumes the cycle.  Deferred 150/180-cycle, ten-minute,
	// two-hour, and preview work is then drained only while the shared engine
	// reports it as pending.  The previous fixed eight-pass drain ran seven or
	// eight empty arbitrary-precision passes after every Basic result and made
	// R5C1 slower than the 50/60 Hz producer, eventually forcing PL to discard
	// whole source packets.  The bound remains a fail-closed guard against a
	// scheduler bug.
	(void)run_one_pass();
	for (std::size_t pass = 0U;
		ready_ && hls_aggregation_engine_has_pending_work() &&
		pass < deferred_pass_count; ++pass)
		(void)run_one_pass();
	if (ready_ && hls_aggregation_engine_has_pending_work())
		fail_engine();

	if (!input_.empty() || assembling_words_ != 0U)
		fail_engine();
}

} // namespace msap1::aggregation
