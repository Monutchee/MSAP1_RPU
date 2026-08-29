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
	for (std::size_t index = 0U; index < phase_total_count; ++index) {
		demand_current_[index] = 0;
		demand_import_peak_[index] = 0U;
		demand_export_peak_[index] = 0U;
		demand_import_anchor_[index] = 0U;
		demand_export_anchor_[index] = 0U;
	}
	demand_saturated_ = false;
	demand_incomplete_ = false;
	clear_basic_pending();
	clear_demand_pending();
	energy_summary_record_.words.fill(0U);
	energy_quadrant_record_.words.fill(0U);
	demand_record_.words.fill(0U);
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
	demand_target_sample_ = 0U;
	demand_source_interval_count_ = 0U;
	demand_source_status_ = 0U;
	demand_valid_ = 0U;
	demand_seen_ = false;
	demand_power_seen_ = false;
	demand_phasor_seen_ = false;
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
	if (demand_seen_)
		demand_incomplete_ = true;
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
	demand_target_sample_ =
		read_unsigned64(record, TEN_MINUTE_TARGET_SAMPLE_LOW_WORD);
	demand_source_interval_count_ =
		record.words[MTR2_SHAPE_WORD] & 0xffffU;
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
		demand_current_[phase] =
			read_signed64(record, base + POWER_PHASE_P_LOW) /
			static_cast<std::int64_t>(pico_units_per_micro_unit);
	}
	demand_current_[3U] =
		read_signed64(record, POWER_TOTAL_P_LOW_WORD) /
		static_cast<std::int64_t>(pico_units_per_micro_unit);
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
	const std::uint32_t status =
		(1U << DEMAND_STATUS_COMPLETE_BIT) |
		(((source >> TEN_MINUTE_STATUS_TIME_ALIGNED_BIT) & 1U)
			<< DEMAND_STATUS_TIME_ALIGNED_BIT) |
		(((source >> TEN_MINUTE_STATUS_CONTAMINATED_BIT) & 1U)
			<< DEMAND_STATUS_CONTAMINATED_BIT) |
		(((source >> TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT) & 1U)
			<< DEMAND_STATUS_BOUNDARY_VALID_BIT) |
		(static_cast<std::uint32_t>(demand_saturated_)
			<< DEMAND_STATUS_SATURATED_BIT) |
		(static_cast<std::uint32_t>(demand_incomplete_)
			<< DEMAND_STATUS_INCOMPLETE_INPUT_BIT) |
		(static_cast<std::uint32_t>(demand_saturated_)
			<< MREC_STATUS_ARITHMETIC_BIT);
	write_common(demand_record_, demand_identity_, MREC_FORMAT_DEMAND_V1,
		status);
	demand_record_.words[MREC_FORMAT_HEADER_WORD] =
		(static_cast<std::uint32_t>(DEMAND_INTERVAL_SECONDS)
			<< DEMAND_HEADER_INTERVAL_SECONDS_LSB) |
		(static_cast<std::uint32_t>(demand_valid_)
			<< DEMAND_HEADER_VALID_LSB);
	write_counter(demand_record_, DEMAND_LAST_SAMPLE_LOW_WORD,
		demand_identity_.last_sample);
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
	write_counter(demand_record_, DEMAND_TARGET_SAMPLE_LOW_WORD,
		demand_target_sample_);
	demand_record_.words[DEMAND_SOURCE_INTERVAL_COUNT_WORD] =
		demand_source_interval_count_;
	demand_record_.words[DEMAND_SOURCE_STATUS_WORD] = demand_source_status_;
	return !emit || sink.publish(demand_record_);
}

bool EnergyDemandEngine::finish_demand(const AggregationMeterRecord &record,
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
	if (!valid_interval || demand_valid_ != 0x0fU)
		demand_incomplete_ = true;

	for (std::size_t index = 0U; index < phase_total_count; ++index) {
		const auto bit = static_cast<std::uint8_t>(1U << index);
		if ((demand_valid_ & bit) == 0U) {
			demand_current_[index] = 0;
			continue;
		}
		const auto current = demand_current_[index];
		if (current > 0) {
			const auto value = static_cast<std::uint64_t>(current);
			if (value > demand_import_peak_[index]) {
				demand_import_peak_[index] = value;
				demand_import_anchor_[index] = demand_identity_.last_sample;
			}
		} else if (current < 0) {
			const auto value = magnitude(current);
			if (value > demand_export_peak_[index]) {
				demand_export_peak_[index] = value;
				demand_export_anchor_[index] = demand_identity_.last_sample;
			}
		}
	}

	const bool published = emit_demand(sink, emit);
	clear_demand_pending();
	return published;
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
	case MREC_FORMAT_TEN_MINUTE_V1:
		begin_demand(record);
		return true;
	case MREC_FORMAT_TEN_MINUTE_POWER_V1:
		accept_demand_power(record);
		return true;
	case MREC_FORMAT_TEN_MINUTE_PHASOR_V2:
		accept_demand_phasor(record);
		return true;
	case MREC_FORMAT_TEN_MINUTE_UNBAL_V2:
		return finish_demand(record, sink, emit);
	default:
		return true;
	}
}

} // namespace msap1::aggregation
