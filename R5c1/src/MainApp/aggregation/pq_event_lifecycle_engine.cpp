#include "pq_event_lifecycle_engine.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t meter_record_magic = 0x3152544DU;
constexpr std::uint32_t pq_event_record_format = 0x00060001U;
constexpr std::uint32_t record_bytes = 256U;
constexpr std::uint8_t lifecycle_start = 0U;
constexpr std::uint8_t lifecycle_update = 1U;
constexpr std::uint8_t lifecycle_end = 2U;
constexpr std::uint8_t lifecycle_abort = 3U;

std::uint32_t low(std::uint64_t value) noexcept
{
	return static_cast<std::uint32_t>(value);
}

std::uint32_t high(std::uint64_t value) noexcept
{
	return static_cast<std::uint32_t>(value >> 32U);
}

std::uint32_t clamp_u32(std::uint64_t value) noexcept
{
	return value > std::numeric_limits<std::uint32_t>::max()
		? std::numeric_limits<std::uint32_t>::max()
		: static_cast<std::uint32_t>(value);
}

std::uint32_t fnv_mix(std::uint32_t hash, std::uint32_t value) noexcept
{
	for (std::size_t byte = 0U; byte < 4U; ++byte) {
		hash ^= (value >> (byte * 8U)) & 0xffU;
		hash *= 16777619U;
	}
	return hash;
}

bool is_voltage(std::size_t type) noexcept
{
	return type <= MSAP1_M18_EVENT_VOLTAGE_UNBALANCE ||
		type == MSAP1_M18_EVENT_TRANSIENT_VOLTAGE;
}

bool is_sag(std::size_t type) noexcept
{
	return type == MSAP1_M18_EVENT_VOLTAGE_SAG ||
		type == MSAP1_M18_EVENT_VOLTAGE_INTERRUPTION ||
		type == MSAP1_M18_EVENT_CURRENT_SAG;
}

bool is_swell(std::size_t type) noexcept
{
	return type == MSAP1_M18_EVENT_VOLTAGE_SWELL ||
		type == MSAP1_M18_EVENT_CURRENT_SWELL ||
		type == MSAP1_M18_EVENT_TRANSIENT_VOLTAGE;
}

bool is_unbalance(std::size_t type) noexcept
{
	return type == MSAP1_M18_EVENT_VOLTAGE_UNBALANCE ||
		type == MSAP1_M18_EVENT_CURRENT_UNBALANCE;
}

} // namespace

PqEventLifecycleEngine::PqEventLifecycleEngine(AggregationRecordSink &sink,
	AggregationHealth &health, std::uint64_t session_id) noexcept
	: sink_(sink), health_(health), session_id_(session_id == 0U ? 1U : session_id)
{
}

bool PqEventLifecycleEngine::configure_session_id(
	std::uint64_t session_id) noexcept
{
	if (ready_ || session_id == 0U)
		return false;
	session_id_ = session_id;
	return true;
}

bool PqEventLifecycleEngine::configure(
	const msap1_m18_config_payload &configuration) noexcept
{
	if (!msap1::power_quality::valid_configuration(configuration))
		return false;
	(void)__atomic_add_fetch(&staged_revision_, 1U, __ATOMIC_ACQ_REL);
	for (std::size_t word = 0U; word < configuration_words; ++word) {
		std::uint32_t value{};
		std::memcpy(&value,
			reinterpret_cast<const std::uint8_t *>(&configuration) +
				word * sizeof(value), sizeof(value));
		__atomic_store_n(&staged_words_[word], value, __ATOMIC_RELAXED);
	}
	(void)__atomic_add_fetch(&staged_revision_, 1U, __ATOMIC_RELEASE);
	return true;
}

bool PqEventLifecycleEngine::load_staged(
	msap1_m18_config_payload &configuration) const noexcept
{
	for (std::size_t attempt = 0U; attempt < 3U; ++attempt) {
		const auto before = __atomic_load_n(&staged_revision_, __ATOMIC_ACQUIRE);
		if (before == 0U || (before & 1U) != 0U)
			return false;
		for (std::size_t word = 0U; word < configuration_words; ++word) {
			const auto value = __atomic_load_n(&staged_words_[word],
				__ATOMIC_RELAXED);
			std::memcpy(reinterpret_cast<std::uint8_t *>(&configuration) +
				word * sizeof(value), &value, sizeof(value));
		}
		if (__atomic_load_n(&staged_revision_, __ATOMIC_ACQUIRE) == before)
			return true;
	}
	return false;
}

bool PqEventLifecycleEngine::initialize() noexcept
{
	for (auto &type : state_)
		for (auto &state : type)
			state = {};
	previous_half_cycle_voltage_.fill(0U);
	previous_half_cycle_voltage_valid_.fill(0U);
	one_cycle_ago_voltage_.fill(0U);
	one_cycle_ago_voltage_valid_.fill(0U);
	rvc_baseline_ready_.fill(0U);
	output_sequence_ = 0U;
	last_input_sequence_ = 0U;
	event_counter_ = 0U;
	external_discontinuity_ = 0U;
	have_input_sequence_ = false;
	have_active_configuration_ = false;
	first_after_discontinuity_ = true;
	ready_ = true;
	return true;
}

void PqEventLifecycleEngine::note_transport_discontinuity() noexcept
{
	__atomic_store_n(&external_discontinuity_, 1U, __ATOMIC_RELEASE);
}

std::array<std::uint32_t, 4U> PqEventLifecycleEngine::fingerprint(
	std::size_t type, std::uint32_t generation, std::uint32_t reference,
	const msap1_m18_event_profile &profile) noexcept
{
	const std::uint32_t values[] = {
		static_cast<std::uint32_t>(type), generation, reference, profile.flags,
		profile.threshold_e4, profile.hysteresis_e4, profile.phase_mask,
		profile.waveform_pretrigger_ms, profile.waveform_posttrigger_ms,
		profile.waveform_decimation};
	std::array<std::uint32_t, 4U> result{};
	for (std::size_t lane = 0U; lane < result.size(); ++lane) {
		auto hash = 2166136261U ^ static_cast<std::uint32_t>(
			0x9e3779b9U * (lane + 1U));
		for (const auto value : values)
			hash = fnv_mix(hash, value);
		result[lane] = hash;
	}
	return result;
}

void PqEventLifecycleEngine::fail() noexcept
{
	ready_ = false;
	health_.set_engine_ready(false);
}

void PqEventLifecycleEngine::emit(EventState &state, std::size_t type,
	std::uint8_t lifecycle, const PqEventInputView &input) noexcept
{
	if (!ready_)
		return;
	AggregationMeterRecord record{};
	record.sequence = ++output_sequence_;
	auto &words = record.words;
	words[0U] = meter_record_magic;
	words[1U] = pq_event_record_format;
	words[2U] = record_bytes;
	words[3U] = record.sequence;
	words[4U] = state.profile_generation;
	words[5U] = input.sample_rate_hz;
	words[6U] = clamp_u32(state.last_sample - state.first_sample + 1U);
	const bool voltage = is_voltage(type);
	words[7U] = voltage
		? static_cast<std::uint32_t>(state.phase_mask) << 4U
		: static_cast<std::uint32_t>(state.phase_mask);
	words[8U] = (1U << 1U) | (1U << 3U);
	if (first_after_discontinuity_) {
		words[8U] |= 1U << 2U;
		first_after_discontinuity_ = false;
	}
	if ((input.status & (1U << 3U)) != 0U)
		words[8U] |= 1U;
	words[9U] = low(state.first_sample);
	words[10U] = high(state.first_sample);
	words[12U] = state.discontinuities;
	const std::uint32_t trigger_source = is_unbalance(type) ? 2U :
		(type == MSAP1_M18_EVENT_RAPID_VOLTAGE_CHANGE ? 1U : 0U);
	words[13U] = static_cast<std::uint32_t>(lifecycle) |
		(static_cast<std::uint32_t>(type) << 4U) |
		(static_cast<std::uint32_t>(state.phase_mask) << 8U) |
		(trigger_source << 16U);
	words[14U] = low(state.last_sample);
	words[15U] = high(state.last_sample);
	words[16U] = low(session_id_);
	words[17U] = high(session_id_);
	words[18U] = low(state.id_counter);
	words[19U] = high(state.id_counter);
	words[20U] = state.profile_generation;
	words[21U] = state.profile.threshold_e4;
	words[22U] = state.profile.hysteresis_e4;
	words[23U] =
		((state.profile.flags & MSAP1_M18_EVENT_WAVEFORM_ENABLED) != 0U ? 1U : 0U) |
		((state.profile.flags & MSAP1_M18_EVENT_PER_PHASE) != 0U ? 1U << 1U : 0U) |
		((state.profile.flags & MSAP1_M18_EVENT_IEC_CLASSIFICATION) != 0U ?
			1U << 2U : 0U) |
		(state.profile.waveform_decimation << 8U);
	words[24U] = state.profile.waveform_pretrigger_ms;
	words[25U] = state.profile.waveform_posttrigger_ms;
	words[26U] = state.reference;
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		words[28U + phase] = state.minimum[phase];
		words[31U + phase] = state.maximum[phase];
		words[34U + phase] = state.current[phase];
	}
	const auto duration = state.last_sample - state.first_sample;
	words[37U] = low(duration);
	words[38U] = high(duration);
	words[39U] = low(state.trigger_sample);
	words[40U] = high(state.trigger_sample);
	// UTC/TAI correlation is resolved by the APU measurement timebase. Zero
	// timestamps plus quality 0 are explicit "unresolved", never wall-clock
	// estimates fabricated by firmware.
	words[45U] = 0U;
	words[46U] = state.discontinuities;
	words[47U] = state.updates;
	for (std::size_t lane = 0U; lane < state.settings_fingerprint.size(); ++lane)
		words[48U + lane] = state.settings_fingerprint[lane];
	if (!sink_.publish(record))
		fail();
}

void PqEventLifecycleEngine::abort_all(const PqEventInputView &input) noexcept
{
	for (std::size_t type = 0U; type < event_types; ++type) {
		for (std::size_t slot = 0U; slot < phases; ++slot) {
			auto &state = state_[type][slot];
			if (!state.active)
				continue;
			state.last_sample = input.last_sample;
			++state.discontinuities;
			emit(state, type, lifecycle_abort, input);
			state.active = false;
		}
	}
	first_after_discontinuity_ = true;
}

void PqEventLifecycleEngine::apply_matching_configuration(
	const PqEventInputView &input) noexcept
{
	if (!load_staged(candidate_configuration_) ||
		candidate_configuration_.generation != input.configuration_generation)
		return;
	if (have_active_configuration_ &&
		active_configuration_.generation == candidate_configuration_.generation)
		return;
	if (have_active_configuration_)
		abort_all(input);
	active_configuration_ = candidate_configuration_;
	have_active_configuration_ = true;
	previous_half_cycle_voltage_valid_.fill(0U);
	one_cycle_ago_voltage_valid_.fill(0U);
	rvc_baseline_ready_.fill(0U);
	first_after_discontinuity_ = true;
}

PqEventLifecycleEngine::Evaluation PqEventLifecycleEngine::evaluate(
	std::size_t type, std::size_t slot,
	const msap1_m18_event_profile &profile,
	const PqEventInputView &input) const noexcept
{
	Evaluation result{};
	const bool voltage = is_voltage(type);
	const auto &source = voltage ? input.urms_q16 : input.irms_q16;
	const auto validity = voltage ? input.voltage_valid_mask :
		input.current_valid_mask;
	for (std::size_t phase = 0U; phase < phases; ++phase)
		result.values[phase] = clamp_u32(source[phase] >> 16U);

	const bool per_phase = (profile.flags & MSAP1_M18_EVENT_PER_PHASE) != 0U;
	const auto configured_mask = static_cast<std::uint8_t>(profile.phase_mask);
	result.phase_mask = per_phase ? static_cast<std::uint8_t>(1U << slot) :
		configured_mask;
	if ((result.phase_mask & configured_mask) != result.phase_mask ||
		(validity & result.phase_mask) != result.phase_mask)
		return result;

	std::uint64_t metric = result.values[slot];
	std::uint64_t reference = voltage
		? active_configuration_.reference_voltage_microvolts
		: active_configuration_.reference_current_microamperes;
	if (is_unbalance(type)) {
		std::uint64_t total = 0U;
		std::uint32_t count = 0U;
		for (std::size_t phase = 0U; phase < phases; ++phase) {
			if ((configured_mask & (1U << phase)) == 0U)
				continue;
			if ((validity & (1U << phase)) == 0U)
				return result;
			total += result.values[phase];
			++count;
		}
		if (count < 2U || total == 0U)
			return result;
		const auto average = total / count;
		std::uint64_t maximum_deviation = 0U;
		for (std::size_t phase = 0U; phase < phases; ++phase) {
			if ((configured_mask & (1U << phase)) == 0U)
				continue;
			const auto value = result.values[phase];
			const auto deviation = value > average ? value - average : average - value;
			if (deviation > maximum_deviation)
				maximum_deviation = deviation;
		}
		metric = maximum_deviation * 10000U / average;
		reference = 10000U;
	} else if (type == MSAP1_M18_EVENT_RAPID_VOLTAGE_CHANGE) {
		if (one_cycle_ago_voltage_valid_[slot] == 0U || reference == 0U)
			return result;
		const auto previous = one_cycle_ago_voltage_[slot];
		const auto value = result.values[slot];
		const auto delta = value > previous ? value - previous : previous - value;
		metric = static_cast<std::uint64_t>(delta) * 10000U / reference;
		reference = 10000U;
	}
	if (reference == 0U)
		return result;

	result.valid = true;
	if (is_unbalance(type) ||
		type == MSAP1_M18_EVENT_RAPID_VOLTAGE_CHANGE) {
		result.start = metric > profile.threshold_e4;
		const auto recovery = profile.threshold_e4 > profile.hysteresis_e4
			? profile.threshold_e4 - profile.hysteresis_e4 : 0U;
		result.recovered = metric <= recovery;
	} else {
		const auto scaled = metric * 10000U;
		if (is_sag(type)) {
			result.start = scaled < reference * profile.threshold_e4;
			result.recovered = scaled >= reference *
				(profile.threshold_e4 + profile.hysteresis_e4);
		} else if (is_swell(type)) {
			result.start = scaled > reference * profile.threshold_e4;
			const auto recovery = profile.threshold_e4 > profile.hysteresis_e4
				? profile.threshold_e4 - profile.hysteresis_e4 : 0U;
			result.recovered = scaled <= reference * recovery;
		}
	}
	return result;
}

void PqEventLifecycleEngine::start_state(EventState &state, std::size_t type,
	const msap1_m18_event_profile &profile, const Evaluation &evaluation,
	const PqEventInputView &input) noexcept
{
	state = {};
	state.active = true;
	state.phase_mask = evaluation.phase_mask;
	if (++event_counter_ == 0U)
		++event_counter_;
	state.id_counter = event_counter_;
	state.first_sample = input.last_sample;
	state.trigger_sample = input.last_sample;
	state.last_sample = input.last_sample;
	state.last_emit_sample = input.last_sample;
	state.minimum = evaluation.values;
	state.maximum = evaluation.values;
	state.current = evaluation.values;
	state.profile = profile;
	state.profile_generation = active_configuration_.generation;
	state.reference = is_voltage(type)
		? active_configuration_.reference_voltage_microvolts
		: active_configuration_.reference_current_microamperes;
	state.updates = 1U;
	state.settings_fingerprint = fingerprint(type, state.profile_generation,
		state.reference, profile);
	emit(state, type, lifecycle_start, input);
}

void PqEventLifecycleEngine::process_state(std::size_t type, std::size_t slot,
	const msap1_m18_event_profile &profile,
	const PqEventInputView &input) noexcept
{
	auto &state = state_[type][slot];
	const auto evaluation = evaluate(type, slot, profile, input);
	/* RVC is a transition between steady states. The first usable
	 * one-cycle delta after a reset can still span RMS-window startup
	 * settling, so it may establish readiness only when it is already
	 * inside the configured recovery band. Do not emit the comparison that
	 * arms the detector itself. */
	if (type == MSAP1_M18_EVENT_RAPID_VOLTAGE_CHANGE &&
	    rvc_baseline_ready_[slot] == 0U) {
		if (evaluation.valid && evaluation.recovered)
			rvc_baseline_ready_[slot] = 1U;
		return;
	}
	if (!state.active) {
		if (evaluation.valid && evaluation.start)
			start_state(state, type, profile, evaluation, input);
		return;
	}
	if (!evaluation.valid) {
		state.last_sample = input.last_sample;
		++state.discontinuities;
		emit(state, type, lifecycle_abort, input);
		state.active = false;
		return;
	}
	state.last_sample = input.last_sample;
	state.current = evaluation.values;
	++state.updates;
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		state.minimum[phase] = std::min(state.minimum[phase],
			evaluation.values[phase]);
		state.maximum[phase] = std::max(state.maximum[phase],
			evaluation.values[phase]);
	}
	if (evaluation.recovered) {
		emit(state, type, lifecycle_end, input);
		state.active = false;
	} else if (input.last_sample - state.last_emit_sample >= input.sample_rate_hz) {
		state.last_emit_sample = input.last_sample;
		emit(state, type, lifecycle_update, input);
	}
}

void PqEventLifecycleEngine::process(const PqEventInputView &input) noexcept
{
	if (!ready_)
		return;
	apply_matching_configuration(input);
	if (!have_active_configuration_ ||
		active_configuration_.generation != input.configuration_generation) {
		if (have_active_configuration_)
			abort_all(input);
		return;
	}

	bool discontinuity = (input.status & (1U << 2U)) != 0U ||
		__atomic_exchange_n(&external_discontinuity_, 0U,
			__ATOMIC_ACQ_REL) != 0U;
	if (have_input_sequence_) {
		const auto delta = static_cast<std::int32_t>(input.sequence -
			(last_input_sequence_ + 1U));
		if (delta < 0)
			return;
		discontinuity = discontinuity || delta > 0;
	}
	have_input_sequence_ = true;
	last_input_sequence_ = input.sequence;
	if (discontinuity) {
		abort_all(input);
		previous_half_cycle_voltage_valid_.fill(0U);
		one_cycle_ago_voltage_valid_.fill(0U);
		rvc_baseline_ready_.fill(0U);
	}

	for (std::size_t type = 0U; type < event_types; ++type) {
		const auto &profile = active_configuration_.event[type];
		if ((profile.flags & MSAP1_M18_EVENT_ENABLED) == 0U)
			continue;
		if ((profile.flags & MSAP1_M18_EVENT_PER_PHASE) != 0U) {
			for (std::size_t phase = 0U; phase < phases; ++phase)
				if ((profile.phase_mask & (1U << phase)) != 0U)
					process_state(type, phase, profile, input);
		} else {
			process_state(type, 0U, profile, input);
		}
	}

	for (std::size_t phase = 0U; phase < phases; ++phase) {
		one_cycle_ago_voltage_[phase] = previous_half_cycle_voltage_[phase];
		one_cycle_ago_voltage_valid_[phase] =
			previous_half_cycle_voltage_valid_[phase];
		previous_half_cycle_voltage_[phase] =
			clamp_u32(input.urms_q16[phase] >> 16U);
		previous_half_cycle_voltage_valid_[phase] = static_cast<std::uint8_t>(
			(input.voltage_valid_mask >> phase) & 1U);
	}
}

} // namespace msap1::aggregation
