#include "aggregation_health.hpp"

#include <limits>

namespace msap1::aggregation {
namespace {

std::uint32_t load(const std::uint32_t &value) noexcept
{
	return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void store(std::uint32_t &target, std::uint32_t value) noexcept
{
	__atomic_store_n(&target, value, __ATOMIC_RELEASE);
}

} // namespace

void AggregationHealth::increment(std::uint32_t &counter,
	std::uint32_t amount) noexcept
{
	auto current = load(counter);
	for (;;) {
		const auto maximum = std::numeric_limits<std::uint32_t>::max();
		const auto next = maximum - current < amount ? maximum : current + amount;
		if (__atomic_compare_exchange_n(&counter, &current, next, false,
			__ATOMIC_RELEASE, __ATOMIC_RELAXED))
			return;
	}
}

void AggregationHealth::set_transport_available(bool available) noexcept
{
	store(transport_available_, available ? 1U : 0U);
}

void AggregationHealth::set_transport_initialized(bool initialized) noexcept
{
	store(transport_initialized_, initialized ? 1U : 0U);
}

void AggregationHealth::set_engine_ready(bool ready) noexcept
{
	store(engine_ready_, ready ? 1U : 0U);
}

void AggregationHealth::set_output_ready(bool ready) noexcept
{
	store(output_ready_, ready ? 1U : 0U);
}

void AggregationHealth::set_output_active(bool active) noexcept
{
	store(output_active_, active ? 1U : 0U);
}

void AggregationHealth::set_authoritative(bool authoritative) noexcept
{
	store(authoritative_, authoritative ? 1U : 0U);
}

void AggregationHealth::record_received() noexcept
{
	increment(frames_received_);
}

void AggregationHealth::record_valid(std::uint32_t sequence) noexcept
{
	increment(frames_valid_);
	store(last_sequence_, sequence);
	store(last_validation_error_,
		static_cast<std::uint32_t>(FrameValidationError::none));
}

void AggregationHealth::record_invalid(FrameValidationError error) noexcept
{
	increment(frames_invalid_);
	store(last_validation_error_, static_cast<std::uint32_t>(error));
	if (error == FrameValidationError::crc_mismatch)
		increment(crc_errors_);
	if (error == FrameValidationError::invalid_magic ||
		error == FrameValidationError::contract_mismatch ||
		error == FrameValidationError::invalid_payload_count ||
		error == FrameValidationError::sequence_mismatch ||
		error == FrameValidationError::reserved_bits_nonzero)
		increment(format_errors_);
}

void AggregationHealth::record_sequence(std::uint32_t sequence) noexcept
{
	if (__atomic_exchange_n(&have_sequence_, 1U, __ATOMIC_ACQ_REL) == 0U) {
		store(last_sequence_, sequence);
		store(expected_sequence_, sequence + 1U);
		return;
	}

	const auto expected = load(expected_sequence_);
	const auto previous = load(last_sequence_);
	if (sequence == expected) {
		store(last_sequence_, sequence);
		store(expected_sequence_, sequence + 1U);
		return;
	}
	if (sequence == previous) {
		increment(repeated_frames_);
		return;
	}

	const auto distance = static_cast<std::int32_t>(sequence - expected);
	if (distance > 0) {
		increment(sequence_gaps_, static_cast<std::uint32_t>(distance));
		store(last_sequence_, sequence);
		store(expected_sequence_, sequence + 1U);
	} else {
		increment(out_of_order_frames_);
	}
}

void AggregationHealth::record_ring_overflow() noexcept
{
	increment(ring_overflows_);
}

void AggregationHealth::record_fifo_error(std::uint32_t status) noexcept
{
	increment(fifo_errors_);
	store(last_fifo_error_, status);
}

void AggregationHealth::record_length_error(std::uint32_t length) noexcept
{
	increment(length_errors_);
	store(last_frame_length_, length);
}

void AggregationHealth::record_output_queued() noexcept
{
	increment(records_queued_);
}

void AggregationHealth::record_output_emitted(std::uint32_t sequence,
	std::uint32_t vacancy_words) noexcept
{
	increment(records_emitted_);
	store(last_output_sequence_, sequence);
	store(last_tx_vacancy_, vacancy_words);
}

void AggregationHealth::record_output_error(std::uint32_t vacancy_words) noexcept
{
	increment(output_errors_);
	store(last_tx_vacancy_, vacancy_words);
}

void AggregationHealth::record_output_drop() noexcept
{
	increment(output_drops_);
}

void AggregationHealth::record_basic_completed() noexcept
{
	increment(basic_completed_);
}

void AggregationHealth::record_aggregate_completed() noexcept
{
	increment(aggregate_completed_);
}

void AggregationHealth::record_ten_minute_completed() noexcept
{
	increment(ten_minute_completed_);
}

void AggregationHealth::record_two_hour_completed() noexcept
{
	increment(two_hour_completed_);
}

AggregationHealthSnapshot AggregationHealth::snapshot() const noexcept
{
	AggregationHealthSnapshot result{};
	result.transport_available = load(transport_available_) != 0U;
	result.transport_initialized = load(transport_initialized_) != 0U;
	result.engine_ready = load(engine_ready_) != 0U;
	result.output_ready = load(output_ready_) != 0U;
	result.output_active = load(output_active_) != 0U;
	result.authoritative = load(authoritative_) != 0U;
	result.frames_received = load(frames_received_);
	result.frames_valid = load(frames_valid_);
	result.frames_invalid = load(frames_invalid_);
	result.crc_errors = load(crc_errors_);
	result.format_errors = load(format_errors_);
	result.sequence_gaps = load(sequence_gaps_);
	result.repeated_frames = load(repeated_frames_);
	result.out_of_order_frames = load(out_of_order_frames_);
	result.ring_overflows = load(ring_overflows_);
	result.fifo_errors = load(fifo_errors_);
	result.length_errors = load(length_errors_);
	result.records_queued = load(records_queued_);
	result.records_emitted = load(records_emitted_);
	result.output_errors = load(output_errors_);
	result.output_drops = load(output_drops_);
	result.basic_completed = load(basic_completed_);
	result.aggregate_completed = load(aggregate_completed_);
	result.ten_minute_completed = load(ten_minute_completed_);
	result.two_hour_completed = load(two_hour_completed_);
	result.last_sequence = load(last_sequence_);
	result.expected_sequence = load(expected_sequence_);
	result.last_fifo_error = load(last_fifo_error_);
	result.last_frame_length = load(last_frame_length_);
	result.last_output_sequence = load(last_output_sequence_);
	result.last_tx_vacancy = load(last_tx_vacancy_);
	result.last_validation_error = static_cast<FrameValidationError>(
		load(last_validation_error_));
	return result;
}

} // namespace msap1::aggregation
