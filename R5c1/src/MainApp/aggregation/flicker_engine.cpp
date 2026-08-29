#include "flicker_engine.hpp"

#include <algorithm>
#include <cstring>
#include <initializer_list>
#include <limits>

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t meter_record_magic = 0x3152544DU;
constexpr std::uint32_t flicker_record_format = 0x000E0001U;
constexpr std::uint32_t record_bytes = 256U;
constexpr std::uint8_t record_live = 0U;
constexpr std::uint8_t record_pst = 1U;
constexpr std::uint8_t record_plt = 2U;
constexpr std::uint32_t internal_rate_hz = 2000U;
constexpr std::uint32_t classifier_samples = 600U * internal_rate_hz;
constexpr std::uint32_t invalid_interval_status =
	(1U << 3U) | (1U << 4U) | (1U << 5U) | (1U << 6U);

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

std::uint32_t integer_sqrt(std::uint64_t value) noexcept
{
	std::uint64_t result = 0U;
	std::uint64_t bit = std::uint64_t{1U} << 62U;
	while (bit > value)
		bit >>= 2U;
	while (bit != 0U) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1U) + bit;
		} else {
			result >>= 1U;
		}
		bit >>= 2U;
	}
	return clamp_u32(result);
}

std::uint64_t cube_q16(std::uint32_t value) noexcept
{
	const auto square_q16 =
		(static_cast<std::uint64_t>(value) * value) >> 16U;
	return (square_q16 * value) >> 16U;
}

std::uint32_t average(std::initializer_list<std::uint32_t> values) noexcept
{
	std::uint64_t sum = 0U;
	for (const auto value : values)
		sum += value;
	return static_cast<std::uint32_t>(sum / values.size());
}

} // namespace

FlickerEngine::FlickerEngine(AggregationRecordSink &sink,
	AggregationHealth &health) noexcept : sink_(sink), health_(health)
{
}

bool FlickerEngine::configure(
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

bool FlickerEngine::load_staged(
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

bool FlickerEngine::initialize() noexcept
{
	for (auto &phase : histogram_)
		phase.fill(0U);
	for (auto &phase : rolling_pst_)
		phase.fill(0U);
	rolling_first_sample_.fill(0U);
	interval_valid_count_.fill(0U);
	interval_peak_.fill(0U);
	output_sequence_ = 0U;
	last_input_sequence_ = 0U;
	have_active_configuration_ = false;
	assembling_histogram_ = false;
	have_input_sequence_ = false;
	external_discontinuity_ = false;
	first_after_discontinuity_ = true;
	rolling_count_ = 0U;
	rolling_position_ = 0U;
	ready_ = true;
	return true;
}

void FlickerEngine::note_transport_discontinuity() noexcept
{
	external_discontinuity_ = true;
}

void FlickerEngine::fail() noexcept
{
	ready_ = false;
	health_.set_engine_ready(false);
}

void FlickerEngine::reset_plt() noexcept
{
	for (auto &phase : rolling_pst_)
		phase.fill(0U);
	rolling_first_sample_.fill(0U);
	rolling_count_ = 0U;
	rolling_position_ = 0U;
}

void FlickerEngine::clear_histogram() noexcept
{
	for (auto &phase : histogram_)
		phase.fill(0U);
	assembling_histogram_ = false;
	expected_histogram_base_ = 0U;
}

void FlickerEngine::abandon_histogram() noexcept
{
	clear_histogram();
	first_after_discontinuity_ = true;
}

bool FlickerEngine::apply_matching_configuration(
	const FlickerInputView &input) noexcept
{
	if (have_active_configuration_ &&
		active_configuration_.generation == input.configuration_generation)
		return active_configuration_.flicker_lamp_voltage ==
				input.lamp_voltage &&
			active_configuration_.flicker_live_cadence_ms ==
				input.live_cadence_ms &&
			active_configuration_.flicker_pst_interval_seconds ==
				input.pst_interval_seconds;
	if (!load_staged(candidate_configuration_) ||
		candidate_configuration_.generation != input.configuration_generation ||
		(candidate_configuration_.flicker_flags &
			MSAP1_M18_ENGINE_ENABLED) == 0U ||
		candidate_configuration_.flicker_lamp_voltage != input.lamp_voltage ||
		candidate_configuration_.flicker_live_cadence_ms !=
			input.live_cadence_ms ||
		candidate_configuration_.flicker_pst_interval_seconds !=
			input.pst_interval_seconds)
		return false;
	active_configuration_ = candidate_configuration_;
	have_active_configuration_ = true;
	abandon_histogram();
	reset_plt();
	return true;
}

std::uint32_t FlickerEngine::percentile_q16(
	const std::array<std::uint32_t, bins> &histogram,
	std::uint32_t total, std::uint32_t exceedance_tenths) noexcept
{
	if (total == 0U)
		return 0U;
	const std::uint64_t target =
		(static_cast<std::uint64_t>(total) * exceedance_tenths + 999U) /
		1000U;
	std::uint64_t accumulated = 0U;
	for (std::size_t reverse = bins; reverse != 0U; --reverse) {
		const auto bin = reverse - 1U;
		accumulated += histogram[bin];
		if (accumulated < target)
			continue;
		const auto octave = static_cast<std::uint32_t>(bin / 32U);
		const auto fraction = static_cast<std::uint32_t>(bin % 32U);
		const auto base = std::uint32_t{1U} << (octave + 8U);
		const auto width = std::uint32_t{1U} << (octave + 3U);
		return base + fraction * width + width / 2U;
	}
	return 1U << 8U;
}

std::uint32_t FlickerEngine::pst_q16(
	const std::array<std::uint32_t, bins> &histogram,
	std::uint32_t total) noexcept
{
	const auto p01 = percentile_q16(histogram, total, 1U);
	const auto p1s = average({percentile_q16(histogram, total, 7U),
		percentile_q16(histogram, total, 10U),
		percentile_q16(histogram, total, 15U)});
	const auto p3s = average({percentile_q16(histogram, total, 22U),
		percentile_q16(histogram, total, 30U),
		percentile_q16(histogram, total, 40U)});
	const auto p10s = average({percentile_q16(histogram, total, 60U),
		percentile_q16(histogram, total, 80U),
		percentile_q16(histogram, total, 100U),
		percentile_q16(histogram, total, 130U),
		percentile_q16(histogram, total, 170U)});
	const auto p50s = average({percentile_q16(histogram, total, 300U),
		percentile_q16(histogram, total, 500U),
		percentile_q16(histogram, total, 800U)});
	const std::uint64_t weighted_q16 =
		(314ULL * p01 + 525ULL * p1s + 657ULL * p3s +
			2800ULL * p10s + 800ULL * p50s) / 10000ULL;
	return integer_sqrt(weighted_q16 << 16U);
}

std::uint32_t FlickerEngine::plt_q16(
	const std::array<std::uint32_t, plt_periods> &pst) noexcept
{
	std::uint64_t mean_cube_q16 = 0U;
	for (const auto value : pst)
		mean_cube_q16 += cube_q16(value);
	mean_cube_q16 /= plt_periods;
	std::uint32_t low_bound = 0U;
	std::uint32_t high_bound =
		*std::max_element(pst.begin(), pst.end());
	while (low_bound < high_bound) {
		const auto middle = low_bound + (high_bound - low_bound + 1U) / 2U;
		if (cube_q16(middle) <= mean_cube_q16)
			low_bound = middle;
		else
			high_bound = middle - 1U;
	}
	return low_bound;
}

void FlickerEngine::emit(std::uint8_t kind, const FlickerInputView &input,
	std::uint8_t valid_mask,
	const std::array<std::uint32_t, phases> &pst,
	const std::array<std::uint32_t, phases> &plt,
	std::uint64_t first_sample, std::uint32_t interval_seconds) noexcept
{
	if (!ready_)
		return;
	AggregationMeterRecord record{};
	record.sequence = ++output_sequence_;
	auto &words = record.words;
	words[0U] = meter_record_magic;
	words[1U] = flicker_record_format;
	words[2U] = record_bytes;
	words[3U] = record.sequence;
	words[4U] = input.configuration_generation;
	words[5U] = input.sample_rate_hz;
	const auto maximum_valid = *std::max_element(
		input.valid_count.begin(), input.valid_count.end());
	words[6U] = kind == record_plt
		? clamp_u32(static_cast<std::uint64_t>(interval_seconds) *
			input.sample_rate_hz)
		: clamp_u32(static_cast<std::uint64_t>(maximum_valid) *
			(input.sample_rate_hz / internal_rate_hz));
	words[7U] = static_cast<std::uint32_t>(valid_mask) << 4U;
	if ((input.status & (1U << 4U)) != 0U)
		words[8U] |= 1U;
	if ((input.status & ((1U << 3U) | (1U << 6U))) != 0U ||
		first_after_discontinuity_) {
		words[8U] |= 1U << 2U;
		first_after_discontinuity_ = false;
	}
	words[9U] = low(first_sample);
	words[10U] = high(first_sample);
	words[13U] = static_cast<std::uint32_t>(kind) |
		(static_cast<std::uint32_t>(valid_mask) << 8U);
	words[14U] = low(input.last_sample);
	words[15U] = high(input.last_sample);
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		words[16U + phase] = input.pinst_q16[phase];
		words[19U + phase] = pst[phase];
		words[22U + phase] = plt[phase];
		words[25U + phase] = kind == record_plt &&
			(valid_mask & (1U << phase)) != 0U
			? classifier_samples * plt_periods
			: input.valid_count[phase];
	}
	words[28U] = interval_seconds;
	words[29U] = input.configuration_generation;
	words[30U] = static_cast<std::uint32_t>(input.lamp_voltage) |
		(static_cast<std::uint32_t>(input.nominal_hz) << 16U);
	words[31U] = input.status;
	words[32U] = low(first_sample);
	words[33U] = high(first_sample);
	if (!sink_.publish(record))
		fail();
}

void FlickerEngine::process_live(const FlickerInputView &input) noexcept
{
	std::array<std::uint32_t, phases> zero{};
	auto valid = input.phase_mask;
	if ((input.status & (invalid_interval_status | (1U << 7U))) != 0U)
		valid = 0U;
	emit(record_live, input, valid, zero, zero, input.first_sample, 1U);
}

void FlickerEngine::process_histogram(const FlickerInputView &input) noexcept
{
	if (input.histogram_base == 0U) {
		clear_histogram();
		assembling_histogram_ = true;
		interval_first_sample_ = input.first_sample;
		interval_last_sample_ = input.last_sample;
		interval_status_ = input.status;
		interval_generation_ = input.configuration_generation;
		interval_sample_rate_ = input.sample_rate_hz;
		interval_lamp_voltage_ = input.lamp_voltage;
		interval_nominal_hz_ = input.nominal_hz;
		interval_phase_mask_ = input.phase_mask;
		interval_valid_count_ = input.valid_count;
		interval_peak_ = input.pinst_q16;
		expected_histogram_base_ = 0U;
	}
	if (!assembling_histogram_ ||
		input.histogram_base != expected_histogram_base_ ||
		input.configuration_generation != interval_generation_ ||
		input.sample_rate_hz != interval_sample_rate_ ||
		input.status != interval_status_ ||
		input.phase_mask != interval_phase_mask_ ||
		input.valid_count != interval_valid_count_ ||
		input.pinst_q16 != interval_peak_ ||
		input.first_sample != interval_first_sample_ ||
		input.last_sample != interval_last_sample_ ||
		input.lamp_voltage != interval_lamp_voltage_ ||
		input.nominal_hz != interval_nominal_hz_) {
		abandon_histogram();
		reset_plt();
		return;
	}
	for (std::size_t phase = 0U; phase < phases; ++phase)
		for (std::size_t offset = 0U;
			offset < FlickerProtocol::histogram_bins; ++offset) {
			const auto bin = input.histogram_base + offset;
			if (bin < bins)
				histogram_[phase][bin] = input.histogram[phase][offset];
		}
	expected_histogram_base_ = static_cast<std::uint16_t>(
		input.histogram_base + FlickerProtocol::histogram_bins);
	if (expected_histogram_base_ < bins)
		return;

	std::array<std::uint32_t, phases> pst{};
	std::array<std::uint32_t, phases> plt{};
	std::uint8_t valid_mask = 0U;
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		std::uint64_t total = 0U;
		for (const auto count : histogram_[phase])
			total += count;
		if ((interval_phase_mask_ & (1U << phase)) != 0U &&
			(interval_status_ & invalid_interval_status) == 0U &&
			interval_valid_count_[phase] == classifier_samples &&
			total == classifier_samples) {
			valid_mask |= static_cast<std::uint8_t>(1U << phase);
			pst[phase] = pst_q16(histogram_[phase], classifier_samples);
		}
	}

	const auto configured_mask = static_cast<std::uint8_t>(
		active_configuration_.flicker_phase_mask & 0x7U);
	std::uint8_t plt_mask = 0U;
	std::uint64_t plt_first = interval_first_sample_;
	if (valid_mask == configured_mask && configured_mask != 0U) {
		for (std::size_t phase = 0U; phase < phases; ++phase)
			rolling_pst_[phase][rolling_position_] = pst[phase];
		rolling_first_sample_[rolling_position_] = interval_first_sample_;
		rolling_position_ = (rolling_position_ + 1U) % plt_periods;
		if (rolling_count_ < plt_periods)
			++rolling_count_;
		if (rolling_count_ == plt_periods) {
			plt_first = rolling_first_sample_[rolling_position_];
			plt_mask = configured_mask;
			for (std::size_t phase = 0U; phase < phases; ++phase)
				if ((plt_mask & (1U << phase)) != 0U)
					plt[phase] = plt_q16(rolling_pst_[phase]);
		}
	} else {
		reset_plt();
	}

	FlickerInputView completed = input;
	completed.pinst_q16 = interval_peak_;
	completed.valid_count = interval_valid_count_;
	completed.first_sample = interval_first_sample_;
	completed.last_sample = interval_last_sample_;
	completed.status = interval_status_;
	const std::array<std::uint32_t, phases> no_plt{};
	emit(record_pst, completed, valid_mask, pst, no_plt,
		interval_first_sample_, 600U);
	if (plt_mask != 0U)
		emit(record_plt, completed, plt_mask, pst, plt, plt_first, 7200U);
	clear_histogram();
}

void FlickerEngine::process(const FlickerInputView &input) noexcept
{
	if (!ready_ || !apply_matching_configuration(input)) {
		note_transport_discontinuity();
		return;
	}
	const bool sequence_gap = have_input_sequence_ &&
		input.sequence != last_input_sequence_ + 1U;
	last_input_sequence_ = input.sequence;
	have_input_sequence_ = true;
	if (external_discontinuity_ || sequence_gap) {
		external_discontinuity_ = false;
		abandon_histogram();
		reset_plt();
	}
	if ((input.status & ((1U << 3U) | (1U << 6U))) != 0U) {
		first_after_discontinuity_ = true;
		reset_plt();
	}
	if (input.kind == FlickerProtocol::kind_live)
		process_live(input);
	else
		process_histogram(input);
}

} // namespace msap1::aggregation
