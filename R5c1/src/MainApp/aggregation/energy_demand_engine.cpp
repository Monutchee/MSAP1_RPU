#include "energy_demand_engine.hpp"

#include "measurement_record.hpp"
#include "metering_types.hpp"

#include <ap_int.h>

#include <climits>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

namespace {

constexpr std::uint64_t maximum_energy_value =
	static_cast<std::uint64_t>(INT64_MAX);
constexpr std::uint32_t energy_tick_rate_hz = 128000U;
// pW * seconds / 3.6e9 = micro-watt-hours.  All supported sample rates
// divide 128 kHz, so conversion-domain samples can first be represented as
// exact 1/128000-second ticks and one denominator survives rate changes.
constexpr std::uint64_t pico_tick_per_micro_hour =
	static_cast<std::uint64_t>(energy_tick_rate_hz) * 3600000000ULL;
constexpr std::uint64_t pico_units_per_micro_unit = 1000000ULL;
constexpr std::uint32_t demand_bucket_seconds =
	DEMAND_SLIDING_UPDATE_SECONDS;

constexpr std::uint8_t phase_current_lanes[3] = {
	MET_LANE_IA, MET_LANE_IB, MET_LANE_IC};
constexpr std::uint8_t phase_voltage_lanes[3] = {
	MET_LANE_VA, MET_LANE_VB, MET_LANE_VC};

void increment_saturating(std::uint32_t &value) noexcept
{
	if (value != UINT32_MAX)
		++value;
}

void add_saturating(std::uint64_t &value, std::uint64_t amount) noexcept
{
	if (UINT64_MAX - value < amount)
		value = UINT64_MAX;
	else
		value += amount;
}

void add_saturating(std::uint32_t &value, std::uint32_t amount) noexcept
{
	if (UINT32_MAX - value < amount)
		value = UINT32_MAX;
	else
		value += amount;
}

} // namespace

EnergyDemandEngine::EnergyDemandEngine(std::uint64_t session_id) noexcept
	: session_id_(session_id == 0U ? 1U : session_id)
{
}

bool EnergyDemandEngine::valid_demand_configuration(DemandMethod method,
	std::uint32_t window_seconds, std::uint32_t update_seconds) noexcept
{
	if (method == DemandMethod::fixed_block)
		return window_seconds == DEMAND_FIXED_INTERVAL_SECONDS &&
			update_seconds == DEMAND_FIXED_INTERVAL_SECONDS;
	if (method != DemandMethod::sliding ||
		update_seconds != DEMAND_SLIDING_UPDATE_SECONDS)
		return false;
	return window_seconds == 60U || window_seconds == 300U ||
		window_seconds == 600U || window_seconds == 900U ||
		window_seconds == DEMAND_MAX_WINDOW_SECONDS;
}

bool EnergyDemandEngine::configure_demand(DemandMethod method,
	std::uint32_t window_seconds, std::uint32_t update_seconds,
	std::uint32_t profile_generation) noexcept
{
	if (!valid_demand_configuration(method, window_seconds, update_seconds) ||
		profile_generation == 0U)
		return false;
	if (method == demand_method_ && window_seconds == demand_window_seconds_ &&
		update_seconds == demand_update_seconds_ &&
		profile_generation == demand_profile_generation_)
		return true;
	demand_method_ = method;
	demand_window_seconds_ = window_seconds;
	demand_update_seconds_ = update_seconds;
	demand_profile_generation_ = profile_generation;
	clear_demand_profile_state();
	return true;
}

void EnergyDemandEngine::initialize(std::uint64_t session_id) noexcept
{
	session_id_ = session_id == 0U ? 1U : session_id;
	clear_state();
}

void EnergyDemandEngine::clear_state() noexcept
{
	for (auto &counter : active_import_)
		counter.value = counter.remainder = 0U;
	for (auto &counter : active_export_)
		counter.value = counter.remainder = 0U;
	for (auto &counter : apparent_)
		counter.value = counter.remainder = 0U;
	for (auto &quadrant : reactive_quadrants_)
		for (auto &counter : quadrant)
			counter.value = counter.remainder = 0U;
	accepted_samples_ = 0U;
	skipped_samples_ = 0U;
	accepted_blocks_ = 0U;
	skipped_blocks_ = 0U;
	energy_saturated_ = false;
	energy_incomplete_ = false;
	energy_discontinuity_ = false;
	energy_started_ = false;
	last_basic_identity_ = {};
	have_last_basic_identity_ = false;
	clear_demand_profile_state();
	clear_basic_pending();
	energy_summary_record_.words.fill(0U);
	energy_quadrant_record_.words.fill(0U);
	demand_record_.words.fill(0U);
}

void EnergyDemandEngine::clear_demand_window() noexcept
{
	demand_bucket_head_ = 0U;
	demand_bucket_count_ = 0U;
}

void EnergyDemandEngine::clear_demand_profile_state() noexcept
{
	for (std::size_t index = 0U; index < phase_total_count; ++index) {
		demand_power_picowatts_[index] = 0;
		demand_current_[index] = 0;
		demand_import_peak_[index] = 0U;
		demand_export_peak_[index] = 0U;
		demand_import_anchor_[index] = 0U;
		demand_export_anchor_[index] = 0U;
	}
	demand_saturated_ = false;
	demand_incomplete_ = false;
	clear_demand_window();
	clear_demand_pending();
}

void EnergyDemandEngine::remember_basic_source() noexcept
{
	last_basic_identity_ = basic_identity_;
	have_last_basic_identity_ = true;
}

bool EnergyDemandEngine::reject_duplicate_or_stale_basic() noexcept
{
	if (!have_last_basic_identity_)
		return false;
	const auto advance = basic_identity_.sequence - last_basic_identity_.sequence;
	if (advance != 0U && advance < 0x80000000U)
		return false;

	// A duplicate or out-of-order family represents no new elapsed interval.
	// Do not add its samples a second time; expose the rejected source block on
	// the next cumulative family instead.
	increment_saturating(skipped_blocks_);
	energy_incomplete_ = true;
	energy_discontinuity_ = true;
	return true;
}

void EnergyDemandEngine::note_basic_gap() noexcept
{
	if (!have_last_basic_identity_)
		return;
	const auto advance = basic_identity_.sequence - last_basic_identity_.sequence;
	if (advance <= 1U || advance >= 0x80000000U)
		return;

	add_saturating(skipped_blocks_, advance - 1U);
	if (last_basic_identity_.last_sample != UINT64_MAX &&
		basic_identity_.first_sample > last_basic_identity_.last_sample + 1U)
		add_saturating(skipped_samples_, basic_identity_.first_sample -
			last_basic_identity_.last_sample - 1U);
	energy_incomplete_ = true;
	energy_discontinuity_ = true;
}

void EnergyDemandEngine::clear_basic_pending() noexcept
{
	basic_identity_.sequence = 0U;
	basic_identity_.generation = 0U;
	basic_identity_.sample_rate_hz = 0U;
	basic_identity_.sample_count = 0U;
	basic_identity_.valid_mask = 0U;
	basic_identity_.status = 0U;
	basic_identity_.first_sample = 0U;
	basic_identity_.last_sample = 0U;
	for (std::size_t index = 0U; index < phase_total_count; ++index) {
		basic_active_power_[index] = 0;
		basic_apparent_power_[index] = 0U;
		basic_reactive_power_[index] = 0;
	}
	basic_power_status_ = 0U;
	basic_phasor_status_ = 0U;
	basic_summary_valid_ = 0U;
	basic_quadrant_valid_ = 0U;
	basic_seen_ = false;
	basic_power_seen_ = false;
	basic_phasor_seen_ = false;
}

void EnergyDemandEngine::clear_demand_pending() noexcept
{
	demand_identity_.sequence = 0U;
	demand_identity_.generation = 0U;
	demand_identity_.sample_rate_hz = 0U;
	demand_identity_.sample_count = 0U;
	demand_identity_.valid_mask = 0U;
	demand_identity_.status = 0U;
	demand_identity_.first_sample = 0U;
	demand_identity_.last_sample = 0U;
	demand_interval_anchor_sample_ = 0U;
	demand_source_interval_count_ = 0U;
	demand_source_status_ = 0U;
	demand_valid_ = 0U;
	demand_seen_ = false;
	demand_power_seen_ = false;
	demand_phasor_seen_ = false;
	for (std::size_t index = 0U; index < phase_total_count; ++index)
		demand_power_picowatts_[index] = 0;
}

std::uint64_t EnergyDemandEngine::read_unsigned64(
	const AggregationMeterRecord &record, std::size_t low_word) noexcept
{
	return static_cast<std::uint64_t>(record.words[low_word]) |
		(static_cast<std::uint64_t>(record.words[low_word + 1U]) << 32U);
}

std::int64_t EnergyDemandEngine::read_signed64(
	const AggregationMeterRecord &record, std::size_t low_word) noexcept
{
	const auto bits = read_unsigned64(record, low_word);
	if (bits <= static_cast<std::uint64_t>(INT64_MAX))
		return static_cast<std::int64_t>(bits);
	// Decode the wire's two's-complement representation without relying on an
	// implementation-defined out-of-range unsigned-to-signed conversion.
	return -1 - static_cast<std::int64_t>(~bits);
}

std::uint64_t EnergyDemandEngine::magnitude(std::int64_t value) noexcept
{
	// Signed-to-unsigned conversion is defined modulo 2^64, including INT64_MIN.
	const auto bits = static_cast<std::uint64_t>(value);
	return value < 0 ? 0U - bits : bits;
}

bool EnergyDemandEngine::supported_sample_rate(
	std::uint32_t sample_rate_hz) noexcept
{
	return sample_rate_hz >= 1000U && sample_rate_hz <= energy_tick_rate_hz &&
		energy_tick_rate_hz % sample_rate_hz == 0U;
}

std::uint8_t EnergyDemandEngine::phase_total_valid_mask(
	std::uint8_t channel_mask) noexcept
{
	std::uint8_t result = 0U;
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		const auto current = static_cast<std::uint8_t>(1U << phase_current_lanes[phase]);
		const auto voltage = static_cast<std::uint8_t>(1U << phase_voltage_lanes[phase]);
		if ((channel_mask & current) != 0U && (channel_mask & voltage) != 0U)
			result |= static_cast<std::uint8_t>(1U << phase);
	}
	if ((result & 0x07U) == 0x07U)
		result |= 0x08U;
	return result;
}

bool EnergyDemandEngine::same_family(const AggregationMeterRecord &record,
	const FamilyIdentity &identity) noexcept
{
	return record.words[MREC_SEQUENCE_WORD] == identity.sequence &&
		record.words[MREC_GENERATION_WORD] == identity.generation &&
		record.words[MREC_SAMPLE_RATE_WORD] == identity.sample_rate_hz &&
		record.words[MREC_SAMPLE_COUNT_WORD] == identity.sample_count &&
		static_cast<std::uint8_t>(record.words[MREC_VALID_MASK_WORD]) ==
			identity.valid_mask &&
		read_unsigned64(record, MREC_FIRST_SAMPLE_LOW_WORD) ==
			identity.first_sample;
}

void EnergyDemandEngine::begin_basic(const AggregationMeterRecord &record) noexcept
{
	if (basic_seen_ && energy_started_) {
		add_saturating(skipped_samples_, basic_identity_.sample_count);
		increment_saturating(skipped_blocks_);
		energy_incomplete_ = true;
		energy_discontinuity_ = true;
		remember_basic_source();
	}
	clear_basic_pending();
	basic_identity_.sequence = record.words[MREC_SEQUENCE_WORD];
	basic_identity_.generation = record.words[MREC_GENERATION_WORD];
	basic_identity_.sample_rate_hz = record.words[MREC_SAMPLE_RATE_WORD];
	basic_identity_.sample_count = record.words[MREC_SAMPLE_COUNT_WORD];
	basic_identity_.valid_mask =
		static_cast<std::uint8_t>(record.words[MREC_VALID_MASK_WORD]);
	basic_identity_.status = record.words[MREC_STATUS_WORD];
	basic_identity_.first_sample =
		read_unsigned64(record, MREC_FIRST_SAMPLE_LOW_WORD);
	basic_identity_.last_sample =
		read_unsigned64(record, BASIC_LAST_SAMPLE_LOW_WORD);
	basic_seen_ = true;
}

void EnergyDemandEngine::accept_basic_power(
	const AggregationMeterRecord &record) noexcept
{
	if (!basic_seen_ || !same_family(record, basic_identity_)) {
		if (energy_started_)
			energy_incomplete_ = true;
		return;
	}
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		const auto base = static_cast<std::size_t>(POWER_PHASE_BASE_WORD) +
			phase * static_cast<std::size_t>(POWER_PHASE_STRIDE);
		basic_active_power_[phase] =
			read_signed64(record, base + POWER_PHASE_P_LOW);
		basic_apparent_power_[phase] =
			read_unsigned64(record, base + POWER_PHASE_S_LOW);
	}
	basic_active_power_[3U] = read_signed64(record, POWER_TOTAL_P_LOW_WORD);
	basic_apparent_power_[3U] =
		read_unsigned64(record, POWER_TOTAL_S_LOW_WORD);
	basic_power_status_ = record.words[MREC_STATUS_WORD];
	basic_summary_valid_ = phase_total_valid_mask(basic_identity_.valid_mask);
	basic_power_seen_ = true;
}

void EnergyDemandEngine::accept_basic_phasor(
	const AggregationMeterRecord &record) noexcept
{
	if (!basic_seen_ || !same_family(record, basic_identity_)) {
		if (energy_started_)
			energy_incomplete_ = true;
		return;
	}
	for (std::size_t phase = 0U; phase < 3U; ++phase)
		basic_reactive_power_[phase] = read_signed64(record,
			static_cast<std::size_t>(PHASOR_Q1_BASE_WORD) + phase * 2U);
	basic_reactive_power_[3U] =
		read_signed64(record, PHASOR_Q1_TOTAL_LOW_WORD);
	basic_phasor_status_ = record.words[MREC_STATUS_WORD];
	basic_quadrant_valid_ = phase_total_valid_mask(basic_identity_.valid_mask);
	if ((basic_phasor_status_ & (1U << PHASOR_STATUS_INVALID_BIT)) != 0U)
		basic_quadrant_valid_ = 0U;
	basic_phasor_seen_ = true;
}

void EnergyDemandEngine::integrate(FractionalCounter &counter,
	std::uint64_t pico_units, std::uint32_t samples,
	std::uint32_t sample_rate_hz) noexcept
{
	if (pico_units == 0U || samples == 0U ||
		!supported_sample_rate(sample_rate_hz))
		return;

	const auto tick_scale = energy_tick_rate_hz / sample_rate_hz;
	ap_uint<128> numerator = ap_uint<128>(pico_units) * samples;
	numerator *= tick_scale;
	numerator += counter.remainder;
	const ap_uint<128> delta = numerator / pico_tick_per_micro_hour;
	counter.remainder =
		(numerator % pico_tick_per_micro_hour).to_uint64();
	const ap_uint<128> available = maximum_energy_value - counter.value;
	if (delta > available) {
		counter.value = maximum_energy_value;
		counter.remainder = 0U;
		energy_saturated_ = true;
		return;
	}
	counter.value += delta.to_uint64();
}

void EnergyDemandEngine::write_counter(AggregationMeterRecord &output,
	std::size_t low_word, std::uint64_t value) noexcept
{
	output.words[low_word] = static_cast<std::uint32_t>(value);
	output.words[low_word + 1U] = static_cast<std::uint32_t>(value >> 32U);
}

void EnergyDemandEngine::write_common(AggregationMeterRecord &output,
	const FamilyIdentity &identity, std::uint32_t format,
	std::uint32_t status) noexcept
{
	output.sequence = identity.sequence;
	output.words.fill(0U);
	output.words[MREC_MAGIC_WORD] = MREC_MAGIC;
	output.words[MREC_FORMAT_WORD] = format;
	output.words[MREC_SIZE_WORD] = AggregationMeterRecord::byte_count;
	output.words[MREC_SEQUENCE_WORD] = identity.sequence;
	output.words[MREC_GENERATION_WORD] = identity.generation;
	output.words[MREC_SAMPLE_RATE_WORD] = identity.sample_rate_hz;
	output.words[MREC_SAMPLE_COUNT_WORD] = identity.sample_count;
	output.words[MREC_VALID_MASK_WORD] = identity.valid_mask;
	output.words[MREC_STATUS_WORD] = status;
	write_counter(output, MREC_FIRST_SAMPLE_LOW_WORD, identity.first_sample);
}

void EnergyDemandEngine::write_energy_metadata(
	AggregationMeterRecord &output) noexcept
{
	write_counter(output, ENERGY_SESSION_ID_LOW_WORD, session_id_);
	write_counter(output, ENERGY_ACCEPTED_SAMPLES_LOW_WORD, accepted_samples_);
	write_counter(output, ENERGY_SKIPPED_SAMPLES_LOW_WORD, skipped_samples_);
	output.words[ENERGY_ACCEPTED_BLOCKS_WORD] = accepted_blocks_;
	output.words[ENERGY_SKIPPED_BLOCKS_WORD] = skipped_blocks_;
}

bool EnergyDemandEngine::emit_energy(AggregationRecordSink &sink,
	bool emit) noexcept
{
	const std::uint32_t status =
		(1U << ENERGY_STATUS_COMPLETE_BIT) |
		(static_cast<std::uint32_t>(energy_incomplete_)
			<< ENERGY_STATUS_INCOMPLETE_INPUT_BIT) |
		(static_cast<std::uint32_t>(energy_saturated_)
			<< ENERGY_STATUS_SATURATED_BIT) |
		(static_cast<std::uint32_t>(energy_discontinuity_)
			<< ENERGY_STATUS_DISCONTINUITY_BIT) |
		(static_cast<std::uint32_t>(energy_saturated_)
			<< MREC_STATUS_ARITHMETIC_BIT);

	write_common(energy_summary_record_, basic_identity_,
		MREC_FORMAT_ENERGY_V1, status);
	energy_summary_record_.words[MREC_FORMAT_HEADER_WORD] =
		(static_cast<std::uint32_t>(ENERGY_PART_SUMMARY)
			<< ENERGY_HEADER_PART_LSB) |
		(static_cast<std::uint32_t>(ENERGY_PART_COUNT)
			<< ENERGY_HEADER_PART_COUNT_LSB) |
		(1U << ENERGY_HEADER_FAMILY_COMPLETE_BIT) |
		(static_cast<std::uint32_t>(basic_summary_valid_)
			<< ENERGY_HEADER_VALID_LSB);
	write_counter(energy_summary_record_, ENERGY_LAST_SAMPLE_LOW_WORD,
		basic_identity_.last_sample);
	for (std::size_t index = 0U; index < phase_total_count; ++index) {
		write_counter(energy_summary_record_,
			ENERGY_SUMMARY_IMPORT_BASE_WORD + index * ENERGY_VALUE_STRIDE,
			active_import_[index].value);
		write_counter(energy_summary_record_,
			ENERGY_SUMMARY_EXPORT_BASE_WORD + index * ENERGY_VALUE_STRIDE,
			active_export_[index].value);
		write_counter(energy_summary_record_,
			ENERGY_SUMMARY_APPARENT_BASE_WORD + index * ENERGY_VALUE_STRIDE,
			apparent_[index].value);
	}
	write_energy_metadata(energy_summary_record_);

	write_common(energy_quadrant_record_, basic_identity_,
		MREC_FORMAT_ENERGY_V1, status);
	energy_quadrant_record_.words[MREC_FORMAT_HEADER_WORD] =
		(static_cast<std::uint32_t>(ENERGY_PART_QUADRANTS)
			<< ENERGY_HEADER_PART_LSB) |
		(static_cast<std::uint32_t>(ENERGY_PART_COUNT)
			<< ENERGY_HEADER_PART_COUNT_LSB) |
		(1U << ENERGY_HEADER_FAMILY_COMPLETE_BIT) |
		(static_cast<std::uint32_t>(basic_quadrant_valid_)
			<< ENERGY_HEADER_VALID_LSB);
	write_counter(energy_quadrant_record_, ENERGY_LAST_SAMPLE_LOW_WORD,
		basic_identity_.last_sample);
	constexpr std::size_t quadrant_bases[quadrant_count] = {
		ENERGY_QUADRANT_I_BASE_WORD, ENERGY_QUADRANT_II_BASE_WORD,
		ENERGY_QUADRANT_III_BASE_WORD, ENERGY_QUADRANT_IV_BASE_WORD};
	for (std::size_t quadrant = 0U; quadrant < quadrant_count; ++quadrant)
		for (std::size_t index = 0U; index < phase_total_count; ++index)
			write_counter(energy_quadrant_record_,
				quadrant_bases[quadrant] + index * ENERGY_VALUE_STRIDE,
				reactive_quadrants_[quadrant][index].value);
	write_energy_metadata(energy_quadrant_record_);

	return !emit ||
		(sink.publish(energy_summary_record_) &&
		 sink.publish(energy_quadrant_record_));
}

bool EnergyDemandEngine::finish_basic(const AggregationMeterRecord &record,
	AggregationRecordSink &sink, bool emit) noexcept
{
	if (!basic_seen_ || !same_family(record, basic_identity_) ||
		!basic_power_seen_ || !basic_phasor_seen_) {
		if (energy_started_) {
			energy_incomplete_ = true;
			energy_discontinuity_ = true;
			if (basic_seen_) {
				add_saturating(skipped_samples_, basic_identity_.sample_count);
				increment_saturating(skipped_blocks_);
				remember_basic_source();
			}
		}
		clear_basic_pending();
		return true;
	}

	const bool common_eligible = basic_identity_.sample_count != 0U &&
		supported_sample_rate(basic_identity_.sample_rate_hz) &&
		(basic_identity_.status & (1U << MREC_STATUS_ARITHMETIC_BIT)) == 0U &&
		(basic_identity_.status & (1U << 2U)) == 0U &&
		basic_power_status_ == basic_identity_.status &&
		record.words[MREC_EMIT_DROPS_WORD] == 0U &&
		record.words[MREC_RESULT_DROPS_WORD] == 0U;
	const bool fully_valid = basic_summary_valid_ == 0x0fU &&
		basic_quadrant_valid_ == 0x0fU;

	// A boot, PL reset, or initial APPLY can finish one Basic family before the
	// complete typed pipeline has a trustworthy session baseline. Keep all such
	// input outside the energy session: publish nothing, count no skipped time,
	// and begin only on the first coherent family which could be integrated in
	// full. After this point every rejection remains sticky and observable.
	if (!energy_started_) {
		if (!common_eligible || !fully_valid) {
			clear_basic_pending();
			return true;
		}
		energy_started_ = true;
	}

	if (reject_duplicate_or_stale_basic()) {
		clear_basic_pending();
		return true;
	}
	note_basic_gap();
	if ((basic_identity_.status & (1U << 2U)) != 0U)
		energy_discontinuity_ = true;

	if (!common_eligible) {
		add_saturating(skipped_samples_, basic_identity_.sample_count);
		increment_saturating(skipped_blocks_);
		energy_incomplete_ = true;
		basic_summary_valid_ = 0U;
		basic_quadrant_valid_ = 0U;
	} else {
		add_saturating(accepted_samples_, basic_identity_.sample_count);
		increment_saturating(accepted_blocks_);
		if (basic_summary_valid_ != 0x0fU ||
			basic_quadrant_valid_ != 0x0fU)
			energy_incomplete_ = true;

		for (std::size_t index = 0U; index < phase_total_count; ++index) {
			const auto bit = static_cast<std::uint8_t>(1U << index);
			if ((basic_summary_valid_ & bit) != 0U) {
				const auto active = basic_active_power_[index];
				if (active >= 0)
					integrate(active_import_[index], magnitude(active),
						basic_identity_.sample_count,
						basic_identity_.sample_rate_hz);
				else
					integrate(active_export_[index], magnitude(active),
						basic_identity_.sample_count,
						basic_identity_.sample_rate_hz);
				integrate(apparent_[index], basic_apparent_power_[index],
					basic_identity_.sample_count,
					basic_identity_.sample_rate_hz);
			}
			if ((basic_quadrant_valid_ & bit) != 0U) {
				const auto quadrant = met_energy_quadrant(
					basic_active_power_[index], basic_reactive_power_[index]);
				if (quadrant != EnergyQuadrant::none)
					integrate(reactive_quadrants_[
						static_cast<std::size_t>(quadrant)][index],
						magnitude(basic_reactive_power_[index]),
						basic_identity_.sample_count,
						basic_identity_.sample_rate_hz);
			}
		}
	}

	remember_basic_source();
	const bool published = emit_energy(sink, emit);
	clear_basic_pending();
	return published;
}

void EnergyDemandEngine::begin_demand(
	const AggregationMeterRecord &record) noexcept
{
	if (demand_seen_) {
		demand_incomplete_ = true;
		if (demand_method_ == DemandMethod::sliding)
			clear_demand_window();
	}
	clear_demand_pending();
	demand_identity_.sequence = record.words[MREC_SEQUENCE_WORD];
	demand_identity_.generation = record.words[MREC_GENERATION_WORD];
	demand_identity_.sample_rate_hz = record.words[MREC_SAMPLE_RATE_WORD];
	demand_identity_.sample_count = record.words[MREC_SAMPLE_COUNT_WORD];
	demand_identity_.valid_mask =
		static_cast<std::uint8_t>(record.words[MREC_VALID_MASK_WORD]);
	demand_identity_.status = record.words[MREC_STATUS_WORD];
	demand_identity_.first_sample =
		read_unsigned64(record, MREC_FIRST_SAMPLE_LOW_WORD);
	demand_identity_.last_sample =
		read_unsigned64(record, AGG_LAST_SAMPLE_LOW_WORD);
	demand_interval_anchor_sample_ = demand_method_ == DemandMethod::fixed_block
		? read_unsigned64(record, TEN_MINUTE_TARGET_SAMPLE_LOW_WORD)
		: demand_identity_.first_sample;
	demand_source_interval_count_ = demand_method_ == DemandMethod::fixed_block
		? record.words[AGGREGATE_SHAPE_WORD] & 0xffffU : 1U;
	demand_source_status_ = record.words[MREC_STATUS_WORD];
	demand_seen_ = true;
}

void EnergyDemandEngine::accept_demand_power(
	const AggregationMeterRecord &record) noexcept
{
	if (!demand_seen_ || !same_family(record, demand_identity_)) {
		demand_incomplete_ = true;
		return;
	}
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		const auto base = static_cast<std::size_t>(POWER_PHASE_BASE_WORD) +
			phase * static_cast<std::size_t>(POWER_PHASE_STRIDE);
		demand_power_picowatts_[phase] =
			read_signed64(record, base + POWER_PHASE_P_LOW);
	}
	demand_power_picowatts_[3U] =
		read_signed64(record, POWER_TOTAL_P_LOW_WORD);
	demand_power_seen_ = true;
}

void EnergyDemandEngine::accept_demand_phasor(
	const AggregationMeterRecord &record) noexcept
{
	if (!demand_seen_ || !same_family(record, demand_identity_)) {
		demand_incomplete_ = true;
		return;
	}
	demand_phasor_seen_ = true;
}

bool EnergyDemandEngine::emit_demand(AggregationRecordSink &sink,
	bool emit) noexcept
{
	const std::uint32_t source = demand_identity_.status;
	const bool fixed = demand_method_ == DemandMethod::fixed_block;
	const bool time_aligned = fixed &&
		(source & (1U << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT)) != 0U;
	const bool source_contaminated = fixed &&
		(source & (1U << TEN_MINUTE_STATUS_CONTAMINATED_BIT)) != 0U;
	const bool boundary_valid = fixed
		? (source & (1U << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT)) != 0U
		: demand_valid_ == 0x0fU;
	const std::uint32_t status =
		(1U << DEMAND_STATUS_COMPLETE_BIT) |
		(static_cast<std::uint32_t>(time_aligned)
			<< DEMAND_STATUS_TIME_ALIGNED_BIT) |
		(static_cast<std::uint32_t>(source_contaminated || demand_incomplete_)
			<< DEMAND_STATUS_CONTAMINATED_BIT) |
		(static_cast<std::uint32_t>(boundary_valid)
			<< DEMAND_STATUS_BOUNDARY_VALID_BIT) |
		(static_cast<std::uint32_t>(demand_saturated_)
			<< DEMAND_STATUS_SATURATED_BIT) |
		(static_cast<std::uint32_t>(demand_incomplete_)
			<< DEMAND_STATUS_INCOMPLETE_INPUT_BIT) |
		(static_cast<std::uint32_t>(demand_saturated_)
			<< MREC_STATUS_ARITHMETIC_BIT);
	write_common(demand_record_, demand_output_identity_, MREC_FORMAT_DEMAND_V1,
		status);
	demand_record_.words[MREC_FORMAT_HEADER_WORD] =
		(demand_window_seconds_
			<< DEMAND_HEADER_INTERVAL_SECONDS_LSB) |
		(static_cast<std::uint32_t>(demand_valid_)
			<< DEMAND_HEADER_VALID_LSB) |
		(static_cast<std::uint32_t>(demand_method_)
			<< DEMAND_HEADER_METHOD_LSB) |
		(demand_update_seconds_ << DEMAND_HEADER_UPDATE_SECONDS_LSB);
	write_counter(demand_record_, DEMAND_LAST_SAMPLE_LOW_WORD,
		demand_output_identity_.last_sample);
	for (std::size_t index = 0U; index < phase_total_count; ++index) {
		write_counter(demand_record_,
			DEMAND_CURRENT_BASE_WORD + index * DEMAND_VALUE_STRIDE,
			static_cast<std::uint64_t>(demand_current_[index]));
		write_counter(demand_record_,
			DEMAND_IMPORT_PEAK_BASE_WORD + index * DEMAND_VALUE_STRIDE,
			demand_import_peak_[index]);
		write_counter(demand_record_,
			DEMAND_EXPORT_PEAK_BASE_WORD + index * DEMAND_VALUE_STRIDE,
			demand_export_peak_[index]);
		write_counter(demand_record_,
			DEMAND_IMPORT_PEAK_ANCHOR_BASE_WORD + index * 2U,
			demand_import_anchor_[index]);
		write_counter(demand_record_,
			DEMAND_EXPORT_PEAK_ANCHOR_BASE_WORD + index * 2U,
			demand_export_anchor_[index]);
	}
	write_counter(demand_record_, DEMAND_SESSION_ID_LOW_WORD, session_id_);
	write_counter(demand_record_, DEMAND_INTERVAL_ANCHOR_SAMPLE_LOW_WORD,
		demand_interval_anchor_sample_);
	demand_record_.words[DEMAND_SOURCE_INTERVAL_COUNT_WORD] =
		demand_source_interval_count_;
	demand_record_.words[DEMAND_SOURCE_STATUS_WORD] = demand_source_status_;
	demand_record_.words[DEMAND_PROFILE_GENERATION_WORD] =
		demand_profile_generation_;
	return !emit || sink.publish(demand_record_);
}

void EnergyDemandEngine::update_demand_peaks() noexcept
{
	for (std::size_t index = 0U; index < phase_total_count; ++index) {
		const auto bit = static_cast<std::uint8_t>(1U << index);
		if ((demand_valid_ & bit) == 0U)
			continue;
		const auto current = demand_current_[index];
		if (current > 0) {
			const auto value = static_cast<std::uint64_t>(current);
			if (value > demand_import_peak_[index]) {
				demand_import_peak_[index] = value;
				demand_import_anchor_[index] =
					demand_output_identity_.last_sample;
			}
		} else if (current < 0) {
			const auto value = magnitude(current);
			if (value > demand_export_peak_[index]) {
				demand_export_peak_[index] = value;
				demand_export_anchor_[index] =
					demand_output_identity_.last_sample;
			}
		}
	}
}

bool EnergyDemandEngine::finish_fixed_demand(
	const AggregationMeterRecord &record,
	AggregationRecordSink &sink, bool emit) noexcept
{
	if (!demand_seen_ || !same_family(record, demand_identity_) ||
		!demand_power_seen_ || !demand_phasor_seen_) {
		demand_incomplete_ = true;
		clear_demand_pending();
		return true;
	}
	const auto status = demand_identity_.status;
	const bool valid_interval =
		(status & (1U << TEN_MINUTE_STATUS_COMPLETE_BIT)) != 0U &&
		(status & (1U << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT)) != 0U &&
		(status & (1U << TEN_MINUTE_STATUS_CONTAMINATED_BIT)) == 0U &&
		(status & (1U << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT)) != 0U &&
		(status & (1U << MREC_STATUS_ARITHMETIC_BIT)) == 0U &&
		record.words[MREC_EMIT_DROPS_WORD] == 0U &&
		record.words[MREC_RESULT_DROPS_WORD] == 0U;
	demand_valid_ = valid_interval ?
		phase_total_valid_mask(demand_identity_.valid_mask) : 0U;
	demand_incomplete_ = !valid_interval || demand_valid_ != 0x0fU;
	demand_output_identity_ = demand_identity_;

	for (std::size_t index = 0U; index < phase_total_count; ++index) {
		const auto bit = static_cast<std::uint8_t>(1U << index);
		if ((demand_valid_ & bit) == 0U) {
			demand_current_[index] = 0;
			continue;
		}
		demand_current_[index] = demand_power_picowatts_[index] /
			static_cast<std::int64_t>(pico_units_per_micro_unit);
	}
	update_demand_peaks();

	const bool published = emit_demand(sink, emit);
	clear_demand_pending();
	return published;
}

bool EnergyDemandEngine::append_sliding_bucket() noexcept
{
	const auto target = static_cast<std::size_t>(
		demand_window_seconds_ / demand_bucket_seconds);
	if (target == 0U || target > maximum_demand_buckets)
		return false;

	if (demand_bucket_count_ != 0U) {
		const auto newest_index =
			(demand_bucket_head_ + demand_bucket_count_ - 1U) % target;
		const auto &newest = demand_buckets_[newest_index];
		const bool contiguous = newest.last_sample != UINT64_MAX &&
			demand_identity_.first_sample == newest.last_sample + 1U;
		if (!contiguous || newest.generation != demand_identity_.generation ||
			newest.sample_rate_hz != demand_identity_.sample_rate_hz)
			clear_demand_window();
	}

	std::size_t index = 0U;
	if (demand_bucket_count_ < target) {
		index = (demand_bucket_head_ + demand_bucket_count_) % target;
		++demand_bucket_count_;
	} else {
		index = demand_bucket_head_;
		demand_bucket_head_ = (demand_bucket_head_ + 1U) % target;
	}
	auto &bucket = demand_buckets_[index];
	for (std::size_t phase = 0U; phase < phase_total_count; ++phase)
		bucket.active_power_picowatts[phase] =
			demand_power_picowatts_[phase];
	bucket.sample_count = demand_identity_.sample_count;
	bucket.source_status = demand_identity_.status;
	bucket.first_sample = demand_identity_.first_sample;
	bucket.last_sample = demand_identity_.last_sample;
	bucket.generation = demand_identity_.generation;
	bucket.sample_rate_hz = demand_identity_.sample_rate_hz;
	return demand_bucket_count_ == target;
}

void EnergyDemandEngine::calculate_sliding_demand() noexcept
{
	const auto target = static_cast<std::size_t>(
		demand_window_seconds_ / demand_bucket_seconds);
	const auto newest_index =
		(demand_bucket_head_ + demand_bucket_count_ - 1U) % target;
	const auto &oldest = demand_buckets_[demand_bucket_head_];
	const auto &newest = demand_buckets_[newest_index];

	std::uint64_t total_samples = 0U;
	std::uint32_t source_status = 0U;
	for (std::size_t offset = 0U; offset < demand_bucket_count_; ++offset) {
		const auto &bucket = demand_buckets_[
			(demand_bucket_head_ + offset) % target];
		total_samples += bucket.sample_count;
		source_status |= bucket.source_status;
	}

	for (std::size_t phase = 0U; phase < phase_total_count; ++phase) {
		ap_int<128> numerator = 0;
		for (std::size_t offset = 0U; offset < demand_bucket_count_; ++offset) {
			const auto &bucket = demand_buckets_[
				(demand_bucket_head_ + offset) % target];
			numerator += ap_int<128>(bucket.active_power_picowatts[phase]) *
				bucket.sample_count;
		}
		if (total_samples == 0U) {
			demand_current_[phase] = 0;
			continue;
		}
		const ap_int<128> micro_watts =
			(numerator / total_samples) / pico_units_per_micro_unit;
		if (micro_watts > INT64_MAX) {
			demand_current_[phase] = INT64_MAX;
			demand_saturated_ = true;
		} else if (micro_watts < INT64_MIN) {
			demand_current_[phase] = INT64_MIN;
			demand_saturated_ = true;
		} else {
			demand_current_[phase] = micro_watts.to_int64();
		}
	}

	demand_output_identity_.sequence = demand_identity_.sequence;
	demand_output_identity_.generation = newest.generation;
	demand_output_identity_.sample_rate_hz = newest.sample_rate_hz;
	demand_output_identity_.sample_count = total_samples > UINT32_MAX
		? UINT32_MAX : static_cast<std::uint32_t>(total_samples);
	demand_output_identity_.valid_mask = demand_identity_.valid_mask;
	demand_output_identity_.status = source_status;
	demand_output_identity_.first_sample = oldest.first_sample;
	demand_output_identity_.last_sample = newest.last_sample;
	demand_interval_anchor_sample_ = oldest.first_sample;
	demand_source_interval_count_ =
		static_cast<std::uint32_t>(demand_bucket_count_);
	demand_source_status_ = source_status;
}

bool EnergyDemandEngine::finish_sliding_demand(
	const AggregationMeterRecord &record, AggregationRecordSink &sink,
	bool emit) noexcept
{
	const bool family_complete = demand_seen_ &&
		same_family(record, demand_identity_) && demand_power_seen_ &&
		demand_phasor_seen_;
	const auto shape = demand_seen_ ? record.words[AGGREGATE_SHAPE_WORD] : 0U;
	const auto nominal = (shape >> AGGREGATE_SHAPE_NOMINAL_LSB) & 0xffU;
	const bool valid_interval = family_complete &&
		demand_identity_.sample_count != 0U &&
		(shape & 0xffU) == 15U && (nominal == 50U || nominal == 60U) &&
		(demand_identity_.status & (1U << AGGREGATE_STATUS_COMPLETE_BIT)) != 0U &&
		(demand_identity_.status & (1U << MREC_STATUS_ARITHMETIC_BIT)) == 0U &&
		record.words[MREC_EMIT_DROPS_WORD] == 0U &&
		record.words[MREC_RESULT_DROPS_WORD] == 0U &&
		phase_total_valid_mask(demand_identity_.valid_mask) == 0x0fU;

	if (!valid_interval) {
		clear_demand_window();
		demand_incomplete_ = true;
		demand_valid_ = 0U;
		demand_output_identity_ = demand_identity_;
		demand_interval_anchor_sample_ = demand_identity_.first_sample;
		demand_source_interval_count_ = 0U;
		for (auto &current : demand_current_)
			current = 0;
		const bool published = demand_seen_ ? emit_demand(sink, emit) : true;
		clear_demand_pending();
		return published;
	}

	const bool window_complete = append_sliding_bucket();
	if (!window_complete) {
		demand_incomplete_ = false;
		clear_demand_pending();
		return true;
	}
	calculate_sliding_demand();
	demand_valid_ = 0x0fU;
	demand_incomplete_ = false;
	update_demand_peaks();
	const bool published = emit_demand(sink, emit);
	clear_demand_pending();
	return published;
}

bool EnergyDemandEngine::finish_demand(const AggregationMeterRecord &record,
	AggregationRecordSink &sink, bool emit) noexcept
{
	return demand_method_ == DemandMethod::fixed_block
		? finish_fixed_demand(record, sink, emit)
		: finish_sliding_demand(record, sink, emit);
}

bool EnergyDemandEngine::observe(const AggregationMeterRecord &record,
	AggregationRecordSink &sink, bool emit) noexcept
{
	switch (record.words[MREC_FORMAT_WORD]) {
	case MREC_FORMAT_BASIC_V4:
		begin_basic(record);
		return true;
	case MREC_FORMAT_POWER_V1:
		accept_basic_power(record);
		return true;
	case MREC_FORMAT_PHASOR_V2:
		accept_basic_phasor(record);
		return true;
	case MREC_FORMAT_UNBAL_V2:
		return finish_basic(record, sink, emit);
	case MREC_FORMAT_AGG_V3:
		if (demand_method_ != DemandMethod::sliding)
			return true;
		begin_demand(record);
		return true;
	case MREC_FORMAT_AGG_POWER_V1:
		if (demand_method_ == DemandMethod::sliding)
			accept_demand_power(record);
		return true;
	case MREC_FORMAT_AGG_PHASOR_V2:
		if (demand_method_ == DemandMethod::sliding)
			accept_demand_phasor(record);
		return true;
	case MREC_FORMAT_AGG_UNBAL_V2:
		return demand_method_ == DemandMethod::sliding
			? finish_demand(record, sink, emit) : true;
	case MREC_FORMAT_TEN_MINUTE_V1:
		if (demand_method_ != DemandMethod::fixed_block)
			return true;
		begin_demand(record);
		return true;
	case MREC_FORMAT_TEN_MINUTE_POWER_V1:
		if (demand_method_ == DemandMethod::fixed_block)
			accept_demand_power(record);
		return true;
	case MREC_FORMAT_TEN_MINUTE_PHASOR_V2:
		if (demand_method_ == DemandMethod::fixed_block)
			accept_demand_phasor(record);
		return true;
	case MREC_FORMAT_TEN_MINUTE_UNBAL_V2:
		return demand_method_ == DemandMethod::fixed_block
			? finish_demand(record, sink, emit) : true;
	default:
		return true;
	}
}

} // namespace msap1::aggregation
