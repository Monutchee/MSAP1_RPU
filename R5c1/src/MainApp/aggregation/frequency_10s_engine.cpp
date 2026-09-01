#include "frequency_10s_engine.hpp"

#include <bit>
#include <limits>

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t certified_sample_rate_hz = 128000U;
constexpr std::uint8_t certified_reference_channel = 6U;
constexpr std::uint8_t certified_filter_profile = 1U;
constexpr std::uint8_t certified_calibration_profile = 1U;
constexpr std::uint64_t maximum_time_uncertainty_nanoseconds = 1000000ULL;
constexpr std::uint32_t maximum_gap_placeholders = 64U;

void put_u64(AggregationMeterRecord &record, std::size_t low_word,
	std::uint64_t value) noexcept
{
	record.words[low_word] = static_cast<std::uint32_t>(value);
	record.words[low_word + 1U] = static_cast<std::uint32_t>(value >> 32U);
}

void put_s64(AggregationMeterRecord &record, std::size_t low_word,
	std::int64_t value) noexcept
{
	put_u64(record, low_word, std::bit_cast<std::uint64_t>(value));
}

std::uint32_t minimum_frequency_millihz(std::uint8_t nominal_hz) noexcept
{
	return nominal_hz == 50U ? 42500U : 51000U;
}

std::uint32_t maximum_frequency_millihz(std::uint8_t nominal_hz) noexcept
{
	return nominal_hz == 50U ? 57500U : 69000U;
}

bool cycle_in_range(std::uint64_t duration_q16,
	std::uint32_t measured_rate_millihz, std::uint8_t nominal_hz) noexcept
{
	const auto scaled_rate =
		static_cast<std::uint64_t>(measured_rate_millihz) << 16U;
	const auto minimum = minimum_frequency_millihz(nominal_hz);
	const auto maximum = maximum_frequency_millihz(nominal_hz);
	return duration_q16 * maximum >= scaled_rate &&
		duration_q16 * minimum <= scaled_rate;
}

std::uint64_t rounded_divide_ties_to_even(std::uint64_t numerator,
	std::uint64_t denominator, bool &error) noexcept
{
	if (denominator == 0U) {
		error = true;
		return 0U;
	}
	const auto quotient = numerator / denominator;
	const auto remainder = numerator % denominator;
	/* Compare 2 * remainder with denominator without overflowing. */
	const auto complement = denominator - remainder;
	const bool round_up = remainder > complement ||
		(remainder == complement && (quotient & 1U) != 0U);
	if (round_up) {
		if (quotient == std::numeric_limits<std::uint64_t>::max()) {
			error = true;
			return quotient;
		}
		return quotient + 1U;
	}
	return quotient;
}

} // namespace

Frequency10sEngine::Frequency10sEngine(AggregationRecordSink &sink,
	AggregationHealth &health, bool emit) noexcept
	: sink_(sink), health_(health), emit_(emit)
{
}

bool Frequency10sEngine::initialize() noexcept
{
	ready_ = true;
	have_sequence_ = false;
	have_interval_ = false;
	transport_discontinuity_pending_ = false;
	last_sequence_ = 0U;
	last_interval_ = {};
	return true;
}

void Frequency10sEngine::note_transport_discontinuity() noexcept
{
	transport_discontinuity_pending_ = true;
}

void Frequency10sEngine::fail() noexcept
{
	ready_ = false;
	health_.set_engine_ready(false);
}

AggregationMeterRecord Frequency10sEngine::build_record(
	const Frequency10sInputView &input,
	std::uint32_t additional_reason) const noexcept
{
	using Protocol = Frequency10sProtocol;
	using Record = Frequency10sRecord;
	AggregationMeterRecord record{};
	record.sequence = input.sequence;
	record.words[0U] = Record::magic;
	record.words[1U] = Record::format;
	record.words[2U] = Record::bytes;
	record.words[3U] = input.sequence;
	record.words[4U] = input.configuration_generation;
	record.words[5U] = input.sample_rate_hz;

	std::uint32_t reasons = input.reason | additional_reason;
	const auto sample_span = input.interval_end_sample >
		input.interval_start_sample
		? input.interval_end_sample - input.interval_start_sample : 0U;
	if (sample_span == 0U ||
		sample_span > std::numeric_limits<std::uint32_t>::max())
		reasons |= Record::reason_time_geometry;
	record.words[6U] = static_cast<std::uint32_t>(sample_span);
	record.words[9U] = static_cast<std::uint32_t>(input.interval_start_sample);
	record.words[10U] = static_cast<std::uint32_t>(
		input.interval_start_sample >> 32U);
	record.words[11U] = input.observer_drop_count;

	record.words[13U] = static_cast<std::uint32_t>(
		input.nominal_frequency_hz) |
		(static_cast<std::uint32_t>(input.reference_channel) << 8U) |
		(static_cast<std::uint32_t>(input.filter_profile) << 16U) |
		(static_cast<std::uint32_t>(input.calibration_profile) << 24U);
	put_u64(record, 14U, input.interval_end_sample == 0U
		? 0U : input.interval_end_sample - 1U);
	put_u64(record, Record::interval_end_sample_word,
		input.interval_end_sample);
	put_u64(record, Record::utc_start_nanoseconds_word,
		input.utc_start_nanoseconds);
	put_u64(record, Record::utc_end_nanoseconds_word,
		input.utc_end_nanoseconds);
	put_u64(record, Record::utc_uncertainty_nanoseconds_word,
		input.utc_uncertainty_nanoseconds);
	record.words[Record::measured_sample_rate_millihz_word] =
		input.measured_sample_rate_millihz;
	record.words[Record::source_sequence_word] = input.sequence;
	record.words[Record::boundary_generation_word] =
		input.boundary_generation;
	record.words[Record::source_status_word] = input.status;
	record.words[Record::observer_drop_count_word] =
		input.observer_drop_count;
	record.words[Record::guard_flags_word] = input.guard_flags;
	record.words[Record::observed_crossing_count_word] = input.crossing_count;

	if (input.sample_rate_hz != certified_sample_rate_hz ||
		input.reference_channel != certified_reference_channel ||
		input.filter_profile != certified_filter_profile ||
		input.calibration_profile != certified_calibration_profile ||
		(input.status & Protocol::status_profile_supported) == 0U)
		reasons |= Protocol::reason_unsupported_profile;
	if ((input.status & Protocol::status_boundary_valid) == 0U)
		reasons |= Protocol::reason_boundary_invalid;
	if ((input.status & Protocol::status_time_synchronized) == 0U)
		reasons |= Protocol::reason_time_unsynchronized;
	if ((input.status & Protocol::status_sample_rate_valid) == 0U)
		reasons |= Protocol::reason_sample_rate_invalid;
	if ((input.status & Protocol::status_filter_ready) == 0U)
		reasons |= Protocol::reason_filter_warmup;
	if ((input.status & Protocol::status_reference_valid) == 0U)
		reasons |= Protocol::reason_reference_invalid;
	if ((input.status & Protocol::status_calibration_valid) == 0U)
		reasons |= Protocol::reason_calibration_invalid;
	if (input.utc_end_nanoseconds <= input.utc_start_nanoseconds ||
		input.utc_end_nanoseconds - input.utc_start_nanoseconds !=
			10000000000ULL)
		reasons |= Record::reason_time_geometry;
	if (input.utc_uncertainty_nanoseconds >
		maximum_time_uncertainty_nanoseconds)
		reasons |= Protocol::reason_time_uncertainty;
	const auto expected_span_millisamples =
		static_cast<std::uint64_t>(input.measured_sample_rate_millihz) * 10U;
	const auto actual_span_millisamples = sample_span * 1000U;
	const auto span_error_millisamples = expected_span_millisamples >
		actual_span_millisamples
		? expected_span_millisamples - actual_span_millisamples
		: actual_span_millisamples - expected_span_millisamples;
	if (span_error_millisamples > 2000U)
		reasons |= Record::reason_time_geometry;
	if ((input.status & (Protocol::status_source_discontinuity |
		Protocol::status_resynchronized)) != 0U)
		reasons |= Protocol::reason_discontinuity;
	if ((input.status & Protocol::status_crossing_overflow) != 0U)
		reasons |= Protocol::reason_crossing_overflow;
	if ((input.status & Protocol::status_observer_drop) != 0U ||
		input.observer_drop_count != 0U)
		reasons |= Protocol::reason_observer_drop;

	const auto interval_q16 = sample_span <=
		(std::numeric_limits<std::uint64_t>::max() >> 16U)
		? sample_span << 16U : 0U;
	bool have_crossing = false;
	std::int64_t first_crossing{};
	std::int64_t previous_crossing{};
	std::int64_t last_crossing{};
	std::uint32_t included_crossings = 0U;
	std::uint32_t rejected_cycles = 0U;
	for (std::size_t index = 0U; index < input.crossing_count; ++index) {
		const auto crossing = input.crossing_q16(index);
		if (crossing < 0 || static_cast<std::uint64_t>(crossing) > interval_q16)
			continue;
		if (!have_crossing) {
			have_crossing = true;
			first_crossing = crossing;
			previous_crossing = crossing;
			last_crossing = crossing;
			included_crossings = 1U;
			continue;
		}
		const auto duration = static_cast<std::uint64_t>(
			crossing - previous_crossing);
		if (duration == 0U || !cycle_in_range(duration,
				input.measured_sample_rate_millihz,
				input.nominal_frequency_hz))
			++rejected_cycles;
		previous_crossing = crossing;
		last_crossing = crossing;
		++included_crossings;
	}

	const auto cycle_count = included_crossings > 0U
		? included_crossings - 1U : 0U;
	const auto total_duration = included_crossings >= 2U
		? static_cast<std::uint64_t>(last_crossing - first_crossing) : 0U;
	record.words[Record::cycle_count_word] = cycle_count;
	put_u64(record, Record::duration_q16_word, total_duration);
	put_s64(record, Record::first_crossing_q16_word, first_crossing);
	put_s64(record, Record::last_crossing_q16_word, last_crossing);
	record.words[Record::included_crossing_count_word] = included_crossings;
	record.words[Record::rejected_cycle_count_word] = rejected_cycles;
	if (cycle_count == 0U || total_duration == 0U)
		reasons |= Record::reason_insufficient_crossings;
	if (rejected_cycles != 0U)
		reasons |= Record::reason_cycle_geometry;

	bool arithmetic_error = false;
	std::uint64_t frequency_millihz = 0U;
	if (cycle_count != 0U && total_duration != 0U) {
		const auto scaled_rate =
			static_cast<std::uint64_t>(
				input.measured_sample_rate_millihz) << 16U;
		if (scaled_rate > std::numeric_limits<std::uint64_t>::max() /
				cycle_count) {
			arithmetic_error = true;
		} else {
			frequency_millihz = rounded_divide_ties_to_even(
				scaled_rate * cycle_count, total_duration,
				arithmetic_error);
		}
	}
	if (arithmetic_error ||
		frequency_millihz > std::numeric_limits<std::uint32_t>::max())
		reasons |= Record::reason_arithmetic;
	if (frequency_millihz != 0U &&
		(frequency_millihz < minimum_frequency_millihz(
			input.nominal_frequency_hz) ||
		 frequency_millihz > maximum_frequency_millihz(
			input.nominal_frequency_hz)))
		reasons |= Record::reason_out_of_range;

	std::uint32_t status = 0U;
	if ((input.status & Protocol::status_boundary_valid) != 0U)
		status |= Record::status_time_aligned;
	if ((input.status & Protocol::status_profile_supported) != 0U)
		status |= Record::status_profile_supported;
	if ((input.status & Protocol::status_time_synchronized) != 0U)
		status |= Record::status_time_synchronized;
	if ((input.status & Protocol::status_filter_ready) != 0U)
		status |= Record::status_filter_ready;
	if ((input.status & Protocol::status_reference_valid) != 0U)
		status |= Record::status_reference_valid;
	if ((input.status & Protocol::status_calibration_valid) != 0U)
		status |= Record::status_calibration_valid;
	if ((input.status & Protocol::status_sample_rate_valid) != 0U)
		status |= Record::status_sample_rate_valid;
	if ((input.status & (Protocol::status_source_discontinuity |
		Protocol::status_resynchronized)) != 0U)
		status |= Record::status_discontinuity;
	if ((input.status & Protocol::status_resynchronized) != 0U)
		status |= Record::status_resynchronized;
	if ((input.status & Protocol::status_crossing_overflow) != 0U)
		status |= Record::status_crossing_overflow;
	if ((input.status & Protocol::status_observer_drop) != 0U ||
		input.observer_drop_count != 0U)
		status |= Record::status_observer_drop;
	if ((reasons & Record::reason_insufficient_crossings) != 0U)
		status |= Record::status_insufficient_crossings;
	if ((reasons & (Record::reason_out_of_range |
		Record::reason_cycle_geometry)) != 0U)
		status |= Record::status_out_of_range;
	if ((reasons & Record::reason_transport_gap) != 0U)
		status |= Record::status_transport_gap;
	if ((reasons & Record::reason_arithmetic) != 0U)
		status |= Record::status_arithmetic_error;

	const bool valid = reasons == 0U;
	if (valid) {
		status |= Record::status_result_valid;
		record.words[7U] = 1U << certified_reference_channel;
		record.words[Record::frequency_millihz_word] =
			static_cast<std::uint32_t>(frequency_millihz);
	}
	record.words[8U] = status;
	record.words[Record::reason_word] = reasons;
	return record;
}

bool Frequency10sEngine::publish(
	const AggregationMeterRecord &record) noexcept
{
	const bool valid = (record.words[8U] &
		Frequency10sRecord::status_result_valid) != 0U;
	health_.record_frequency_completed(record.sequence, valid);
	if (!emit_)
		return true;
	if (sink_.publish(record))
		return true;
	health_.record_output_drop();
	fail();
	return false;
}

bool Frequency10sEngine::publish_gap(std::uint32_t sequence,
	std::uint32_t ordinal) noexcept
{
	if (!have_interval_)
		return true;
	const auto span = last_interval_.end_sample - last_interval_.start_sample;
	Frequency10sInputView placeholder{};
	placeholder.sequence = sequence;
	placeholder.configuration_generation =
		last_interval_.configuration_generation;
	placeholder.sample_rate_hz = last_interval_.sample_rate_hz;
	placeholder.measured_sample_rate_millihz =
		last_interval_.measured_sample_rate_millihz;
	placeholder.nominal_frequency_hz = last_interval_.nominal_frequency_hz;
	placeholder.reference_channel = last_interval_.reference_channel;
	placeholder.filter_profile = last_interval_.filter_profile;
	placeholder.calibration_profile = last_interval_.calibration_profile;
	placeholder.status = last_interval_.status;
	placeholder.interval_start_sample = last_interval_.end_sample +
		span * static_cast<std::uint64_t>(ordinal - 1U);
	placeholder.interval_end_sample = placeholder.interval_start_sample + span;
	placeholder.utc_start_nanoseconds =
		last_interval_.utc_end_nanoseconds +
		10000000000ULL * static_cast<std::uint64_t>(ordinal - 1U);
	placeholder.utc_end_nanoseconds =
		placeholder.utc_start_nanoseconds + 10000000000ULL;
	placeholder.utc_uncertainty_nanoseconds =
		last_interval_.utc_uncertainty_nanoseconds;
	placeholder.boundary_generation = last_interval_.boundary_generation;
	const auto record = build_record(placeholder,
		Frequency10sRecord::reason_transport_gap);
	health_.record_frequency_placeholder();
	return publish(record);
}

void Frequency10sEngine::remember(const Frequency10sInputView &input) noexcept
{
	last_interval_ = {
		input.configuration_generation,
		input.sample_rate_hz,
		input.measured_sample_rate_millihz,
		input.nominal_frequency_hz,
		input.reference_channel,
		input.filter_profile,
		input.calibration_profile,
		input.status,
		input.interval_start_sample,
		input.interval_end_sample,
		input.utc_start_nanoseconds,
		input.utc_end_nanoseconds,
		input.utc_uncertainty_nanoseconds,
		input.boundary_generation,
	};
	have_interval_ = true;
}

void Frequency10sEngine::process(const Frequency10sInputView &input) noexcept
{
	if (!ready_)
		return;
	std::uint32_t extra_reason = 0U;
	if (transport_discontinuity_pending_) {
		extra_reason |= Frequency10sRecord::reason_transport_gap;
		transport_discontinuity_pending_ = false;
	}

	if (have_sequence_) {
		const auto expected = last_sequence_ + 1U;
		const auto distance = static_cast<std::int32_t>(input.sequence - expected);
		if (distance < 0)
			return;
		if (distance > 0) {
			const auto gaps = static_cast<std::uint32_t>(distance);
			health_.record_frequency_transport_gap(gaps);
			if (gaps > maximum_gap_placeholders) {
				fail();
				return;
			}
			for (std::uint32_t gap = 1U; gap <= gaps; ++gap)
				if (!publish_gap(expected + gap - 1U, gap))
					return;
			extra_reason |= Frequency10sRecord::reason_transport_gap;
		}
	}

	const auto record = build_record(input, extra_reason);
	if (!publish(record))
		return;
	last_sequence_ = input.sequence;
	have_sequence_ = true;
	remember(input);
}

} // namespace msap1::aggregation
