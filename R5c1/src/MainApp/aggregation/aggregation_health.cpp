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

RingPressureLevel classify_ring_pressure(std::uint32_t used,
	std::uint32_t capacity) noexcept
{
	if (capacity == 0U || used == 0U)
		return RingPressureLevel::normal;
	if (used >= capacity)
		return RingPressureLevel::full;

	/* Multiplication avoids truncation around percentage boundaries. */
	const auto scaled = static_cast<std::uint64_t>(used) * 100U;
	if (scaled >= static_cast<std::uint64_t>(capacity) * 90U)
		return RingPressureLevel::critical;
	if (scaled >= static_cast<std::uint64_t>(capacity) * 75U)
		return RingPressureLevel::high;
	if (scaled >= static_cast<std::uint64_t>(capacity) * 50U)
		return RingPressureLevel::warning;
	return RingPressureLevel::normal;
}

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

void AggregationHealth::update_maximum(std::uint32_t &maximum,
	std::uint32_t candidate) noexcept
{
	auto current = load(maximum);
	while (candidate > current &&
		!__atomic_compare_exchange_n(&maximum, &current, candidate, false,
			__ATOMIC_RELEASE, __ATOMIC_RELAXED)) {
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
		const auto dropped = static_cast<std::uint32_t>(distance);
		increment(sequence_gaps_, dropped);
		/*
		 * The sequence gap is the only authoritative evidence that complete
		 * input records were discarded before validation.  Preserve the first
		 * and most recent missing sequence so field diagnostics can correlate a
		 * loss with software-ring or hardware-FIFO pressure.
		 */
		if (load(input_records_dropped_) == 0U)
			store(first_dropped_sequence_, expected);
		increment(input_records_dropped_, dropped);
		store(last_dropped_sequence_, sequence - 1U);
		store(last_sequence_, sequence);
		store(expected_sequence_, sequence + 1U);
	} else {
		increment(out_of_order_frames_);
	}
}

void AggregationHealth::record_ring_overflow() noexcept
{
	increment(ring_overflows_);
	increment(software_ring_push_failures_);
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

void AggregationHealth::observe_software_ring(std::uint32_t used,
	std::uint32_t capacity) noexcept
{
	store(software_ring_current_, used);
	store(software_ring_capacity_, capacity);
	update_maximum(software_ring_high_water_, used);

	const auto next = classify_ring_pressure(used, capacity);
	const auto previous = static_cast<RingPressureLevel>(
		__atomic_exchange_n(&software_ring_pressure_,
			static_cast<std::uint32_t>(next), __ATOMIC_ACQ_REL));
	if (static_cast<std::uint32_t>(next) <=
		static_cast<std::uint32_t>(previous))
		return;

	/* Count every newly crossed edge, including a direct normal-to-full jump. */
	if (previous < RingPressureLevel::warning &&
		next >= RingPressureLevel::warning)
		increment(software_ring_warning_entries_);
	if (previous < RingPressureLevel::high && next >= RingPressureLevel::high)
		increment(software_ring_high_entries_);
	if (previous < RingPressureLevel::critical &&
		next >= RingPressureLevel::critical)
		increment(software_ring_critical_entries_);
	if (previous < RingPressureLevel::full && next >= RingPressureLevel::full)
		increment(software_ring_full_entries_);
}

void AggregationHealth::observe_hardware_fifo(
	std::uint32_t occupancy_words) noexcept
{
	store(hardware_fifo_current_words_, occupancy_words);
	update_maximum(hardware_fifo_high_water_words_, occupancy_words);
}

void AggregationHealth::record_hardware_fifo_full_events(
	std::uint32_t count) noexcept
{
	increment(hardware_fifo_full_events_, count);
}

void AggregationHealth::record_input_activation(
	std::uint32_t records_processed, std::uint32_t runtime_us) noexcept
{
	increment(input_wake_count_);
	increment(input_records_processed_, records_processed);
	update_maximum(input_max_batch_, records_processed);
	update_maximum(input_max_runtime_us_, runtime_us);
}

void AggregationHealth::record_validator_activation(
	std::uint32_t records_processed, std::uint32_t runtime_us,
	std::uint32_t schedule_gap_us) noexcept
{
	increment(validator_wake_count_);
	increment(validator_records_processed_, records_processed);
	update_maximum(validator_max_runtime_us_, runtime_us);
	update_maximum(validator_max_schedule_gap_us_, schedule_gap_us);
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
	result.software_ring_push_failures = load(software_ring_push_failures_);
	result.input_records_dropped = load(input_records_dropped_);
	result.first_dropped_sequence = load(first_dropped_sequence_);
	result.last_dropped_sequence = load(last_dropped_sequence_);
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
	result.software_ring_current = load(software_ring_current_);
	result.software_ring_high_water = load(software_ring_high_water_);
	result.software_ring_capacity = load(software_ring_capacity_);
	result.software_ring_pressure = static_cast<RingPressureLevel>(
		load(software_ring_pressure_));
	result.software_ring_warning_entries = load(
		software_ring_warning_entries_);
	result.software_ring_high_entries = load(software_ring_high_entries_);
	result.software_ring_critical_entries = load(
		software_ring_critical_entries_);
	result.software_ring_full_entries = load(software_ring_full_entries_);
	result.hardware_fifo_current_words = load(hardware_fifo_current_words_);
	result.hardware_fifo_high_water_words = load(
		hardware_fifo_high_water_words_);
	result.hardware_fifo_full_events = load(hardware_fifo_full_events_);
	result.input_wake_count = load(input_wake_count_);
	result.input_records_processed = load(input_records_processed_);
	result.input_max_batch = load(input_max_batch_);
	result.input_max_runtime_us = load(input_max_runtime_us_);
	result.validator_wake_count = load(validator_wake_count_);
	result.validator_records_processed = load(validator_records_processed_);
	result.validator_max_runtime_us = load(validator_max_runtime_us_);
	result.validator_max_schedule_gap_us = load(
		validator_max_schedule_gap_us_);
	return result;
}

} // namespace msap1::aggregation
