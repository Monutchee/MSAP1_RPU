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
constexpr std::uint32_t settling_seconds = 10U;
constexpr std::uint32_t classifier_samples = 600U * internal_rate_hz;
constexpr std::uint32_t invalid_interval_status =
	(1U << 3U) | (1U << 4U) | (1U << 5U) | (1U << 6U);

struct BiquadCoefficients final {
	std::int64_t b0;
	std::int64_t b1;
	std::int64_t b2;
	std::int64_t a1;
	std::int64_t a2;
};

constexpr std::array<BiquadCoefficients, 7U> coefficients_230{{
	{1073657499LL, -1073657499LL, 0LL, -1073573174LL, 0LL},
	{3152648LL, 6305296LL, 3152648LL, -2075566063LL, 1014434831LL},
	{3008728LL, 6017457LL, 3008728LL, -1980815729LL, 919108819LL},
	{2931466LL, 5862932LL, 2931466LL, -1929949505LL, 867933545LL},
	{26645808LL, 0LL, -26645808LL, -2119567684LL, 1046702695LL},
	{19224623LL, 137199LL, -19087424LL, -2071940580LL, 998473154LL},
	{894040LL, 894040LL, 0LL, -1071953744LL, 0LL},
}};

constexpr std::array<BiquadCoefficients, 7U> coefficients_120{{
	{1073657499LL, -1073657499LL, 0LL, -1073573174LL, 0LL},
	{4513006LL, 9026012LL, 4513006LL, -2058714883LL, 1003025083LL},
	{4269489LL, 8538978LL, 4269489LL, -1947628912LL, 890965045LL},
	{4140500LL, 8280999LL, 4140500LL, -1888787185LL, 831607359LL},
	{24713697LL, 0LL, -24713697LL, -2118875559LL, 1045995452LL},
	{13518150LL, 124279LL, -13393871LL, -2085928193LL, 1012434928LL},
	{894040LL, 894040LL, 0LL, -1071953744LL, 0LL},
}};

constexpr std::uint64_t calibration_230_q16 = 5072324577ULL;
constexpr std::uint64_t calibration_120_q16 = 5040534274ULL;

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

struct WideUnsigned final {
	std::uint64_t high{};
	std::uint64_t low{};
};

std::uint64_t magnitude(std::int64_t value) noexcept
{
	return value < 0
		? static_cast<std::uint64_t>(-(value + 1)) + 1U
		: static_cast<std::uint64_t>(value);
}

WideUnsigned multiply_wide(std::uint64_t lhs, std::uint64_t rhs) noexcept
{
	const auto lhs_low = static_cast<std::uint64_t>(
		static_cast<std::uint32_t>(lhs));
	const auto lhs_high = lhs >> 32U;
	const auto rhs_low = static_cast<std::uint64_t>(
		static_cast<std::uint32_t>(rhs));
	const auto rhs_high = rhs >> 32U;
	const auto low_low = lhs_low * rhs_low;
	const auto low_high = lhs_low * rhs_high;
	const auto high_low = lhs_high * rhs_low;
	const auto high_high = lhs_high * rhs_high;
	const auto middle = (low_low >> 32U) +
		static_cast<std::uint32_t>(low_high) +
		static_cast<std::uint32_t>(high_low);
	return {
		high_high + (low_high >> 32U) + (high_low >> 32U) +
			(middle >> 32U),
		(middle << 32U) | static_cast<std::uint32_t>(low_low)};
}

void add_wide(WideUnsigned &value, std::uint64_t addend) noexcept
{
	const auto before = value.low;
	value.low += addend;
	value.high += value.low < before ? 1U : 0U;
}

int compare_wide(const WideUnsigned &lhs,
	const WideUnsigned &rhs) noexcept
{
	if (lhs.high != rhs.high)
		return lhs.high < rhs.high ? -1 : 1;
	if (lhs.low != rhs.low)
		return lhs.low < rhs.low ? -1 : 1;
	return 0;
}

WideUnsigned subtract_wide(const WideUnsigned &lhs,
	const WideUnsigned &rhs) noexcept
{
	return {lhs.high - rhs.high - (lhs.low < rhs.low ? 1U : 0U),
		lhs.low - rhs.low};
}

std::uint64_t shift_wide_saturating(const WideUnsigned &value,
	unsigned shift, std::uint64_t limit, bool &overflow) noexcept
{
	if ((value.high >> shift) != 0U) {
		overflow = true;
		return limit;
	}
	const auto shifted = (value.high << (64U - shift)) |
		(value.low >> shift);
	if (shifted > limit) {
		overflow = true;
		return limit;
	}
	return shifted;
}

std::uint64_t multiply_shift_saturating(std::uint64_t lhs,
	std::uint64_t rhs, unsigned shift, std::uint64_t limit,
	bool &overflow, std::uint64_t rounding = 0U) noexcept
{
	constexpr auto narrow_limit = static_cast<std::uint64_t>(
		std::numeric_limits<std::uint32_t>::max());
	if (lhs <= narrow_limit && rhs <= narrow_limit) {
		const auto product = lhs * rhs;
		if (product <= std::numeric_limits<std::uint64_t>::max() - rounding) {
			const auto shifted = (product + rounding) >> shift;
			if (shifted > limit) {
				overflow = true;
				return limit;
			}
			return shifted;
		}
	}
	auto product = multiply_wide(lhs, rhs);
	add_wide(product, rounding);
	return shift_wide_saturating(product, shift, limit, overflow);
}

inline __attribute__((always_inline)) std::int64_t
normalize_microvolts_q16_fast(std::int32_t sample,
	std::uint64_t reciprocal_q46, bool &overflow) noexcept
{
	constexpr std::uint64_t limit = std::uint64_t{8U} << 16U;
	constexpr std::uint64_t divisor = std::uint64_t{1U} << 30U;
	const auto absolute = magnitude(sample);
	std::uint64_t normalized{};
	if (reciprocal_q46 <= std::numeric_limits<std::uint32_t>::max()) {
		/* The wire sample is an integer number of microvolts. Multiplying it
		 * by 2^16 before applying the Q46 reciprocal and shifting by 46 is
		 * exactly equivalent to shifting the unscaled sample by 30. Normal
		 * 120/230 V references therefore need one UMULL, not a four-limb
		 * 64x64 multiply, for every phase of every 128 kSPS input sample. */
		auto product = absolute * reciprocal_q46;
		if (sample < 0)
			product += divisor - 1U;
		normalized = product >> 30U;
		if (normalized > limit) {
			normalized = limit;
			overflow = true;
		}
	} else {
		normalized = multiply_shift_saturating(absolute, reciprocal_q46,
			30U, limit, overflow, sample < 0 ? divisor - 1U : 0U);
	}
	return sample < 0 ? -static_cast<std::int64_t>(normalized)
		: static_cast<std::int64_t>(normalized);
}

void accumulate_signed(WideUnsigned &positive, WideUnsigned &negative,
	std::int64_t value, bool negate) noexcept
{
	if ((value < 0) != negate)
		add_wide(negative, magnitude(value));
	else
		add_wide(positive, magnitude(value));
}

std::int64_t saturating_expression(std::int64_t first, bool negate_first,
	std::int64_t second, bool negate_second, std::int64_t third,
	bool negate_third, bool &overflow) noexcept
{
	WideUnsigned positive{};
	WideUnsigned negative{};
	accumulate_signed(positive, negative, first, negate_first);
	accumulate_signed(positive, negative, second, negate_second);
	accumulate_signed(positive, negative, third, negate_third);
	const bool result_negative = compare_wide(positive, negative) < 0;
	const auto difference = result_negative
		? subtract_wide(negative, positive)
		: subtract_wide(positive, negative);
	const auto limit = result_negative
		? std::uint64_t{1U} << 63U
		: static_cast<std::uint64_t>(
			std::numeric_limits<std::int64_t>::max());
	if (difference.high != 0U || difference.low > limit) {
		overflow = true;
		return result_negative ? std::numeric_limits<std::int64_t>::min()
			: std::numeric_limits<std::int64_t>::max();
	}
	if (!result_negative)
		return static_cast<std::int64_t>(difference.low);
	if (difference.low == (std::uint64_t{1U} << 63U))
		return std::numeric_limits<std::int64_t>::min();
	return -static_cast<std::int64_t>(difference.low);
}

std::int64_t multiply_q30(std::int64_t lhs, std::int64_t rhs,
	bool &overflow) noexcept
{
	const bool negative = (lhs < 0) != (rhs < 0);
	const auto limit = negative ? std::uint64_t{1U} << 63U
		: static_cast<std::uint64_t>(
			std::numeric_limits<std::int64_t>::max());
	const auto absolute_lhs = magnitude(lhs);
	const auto absolute_rhs = magnitude(rhs);
	std::uint64_t rounded{};
	if (absolute_lhs <= std::numeric_limits<std::uint32_t>::max() &&
		absolute_rhs <= std::numeric_limits<std::uint32_t>::max()) {
		const auto product = absolute_lhs * absolute_rhs +
			(std::uint64_t{1U} << 29U);
		rounded = product >> 30U;
		if (rounded > limit) {
			rounded = limit;
			overflow = true;
		}
	} else {
		rounded = multiply_shift_saturating(absolute_lhs, absolute_rhs,
			30U, limit, overflow, std::uint64_t{1U} << 29U);
	}
	if (!negative)
		return static_cast<std::int64_t>(rounded);
	if (rounded == (std::uint64_t{1U} << 63U))
		return std::numeric_limits<std::int64_t>::min();
	return -static_cast<std::int64_t>(rounded);
}

std::int64_t run_biquad(std::int64_t input,
	const BiquadCoefficients &coefficient, std::int64_t &z1,
	std::int64_t &z2, bool &overflow) noexcept
{
	const auto output = saturating_expression(
		multiply_q30(input, coefficient.b0, overflow), false,
		z1, false, 0, false, overflow);
	const auto next_z1 = saturating_expression(
		multiply_q30(input, coefficient.b1, overflow), false,
		multiply_q30(output, coefficient.a1, overflow), true,
		z2, false, overflow);
	const auto next_z2 = saturating_expression(
		multiply_q30(input, coefficient.b2, overflow), false,
		multiply_q30(output, coefficient.a2, overflow), true,
		0, false, overflow);
	z1 = next_z1;
	z2 = next_z2;
	return output;
}

struct PackedSample final {
	std::array<std::int32_t, 3U> voltage_microvolts{};
	std::uint8_t flags{};
};

PackedSample unpack_sample(const std::uint32_t *words) noexcept
{
	PackedSample sample{};
	sample.voltage_microvolts = {
		static_cast<std::int32_t>(words[0U]),
		static_cast<std::int32_t>(words[1U]),
		static_cast<std::int32_t>(words[2U])};
	sample.flags = static_cast<std::uint8_t>(words[3U] &
		VoltageSampleProtocol::sample_flags_mask);
	return sample;
}

std::uint16_t histogram_bin_q16(std::uint32_t pinst_q16,
	bool &outside) noexcept
{
	outside = false;
	if (pinst_q16 < (1U << 8U)) {
		outside = true;
		return 0U;
	}
	if (pinst_q16 >= (1U << 24U)) {
		outside = true;
		return static_cast<std::uint16_t>(
			VoltageSampleProtocol::classifier_bins - 1U);
	}
	const auto msb = static_cast<unsigned>(31U -
		static_cast<unsigned>(__builtin_clz(pinst_q16)));
	const auto fraction = (pinst_q16 >> (msb - 5U)) & 0x1fU;
	return static_cast<std::uint16_t>((msb - 8U) * 32U + fraction);
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

std::int64_t FlickerEngine::normalize_microvolts_q16(std::int32_t sample,
	std::uint64_t reciprocal_q46, bool &overflow) noexcept
{
	return normalize_microvolts_q16_fast(sample, reciprocal_q46, overflow);
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
	have_active_configuration_ = false;
	have_input_sequence_ = false;
	external_discontinuity_ = false;
	output_sequence_ = 0U;
	last_input_sequence_ = 0U;
	sample_rate_hz_ = 0U;
	nominal_hz_ = 0U;
	decimation_divisor_ = 1U;
	reference_reciprocal_q46_ = 0U;
	reset_runtime(false);
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
}

void FlickerEngine::reset_signal_path(bool contaminated) noexcept
{
	raw_square_sum_.fill(0U);
	adapter_q32_.fill(std::uint64_t{1U} << 32U);
	adapter_reciprocal_q30_.fill(std::uint64_t{1U} << 30U);
	for (auto &phase : filter_z1_)
		phase.fill(0);
	for (auto &phase : filter_z2_)
		phase.fill(0);
	raw_count_ = 0U;
	raw_valid_mask_ = 0U;
	settling_ticks_ = internal_rate_hz * settling_seconds;
	adapter_reciprocal_ticks_ = 0U;
	have_last_input_sample_ = false;
	live_discontinuity_ = true;
	interval_discontinuity_ = true;
	if (contaminated) {
		live_contaminated_ = true;
		interval_contaminated_ = true;
	}
	first_after_discontinuity_ = true;
	reset_plt();
}

void FlickerEngine::reset_runtime(bool contaminated) noexcept
{
	clear_histogram();
	reset_plt();
	live_valid_count_.fill(0U);
	live_peak_.fill(0U);
	interval_valid_count_.fill(0U);
	interval_peak_.fill(0U);
	live_ticks_ = 0U;
	interval_ticks_ = 0U;
	live_first_sample_ = 0U;
	interval_first_sample_ = 0U;
	live_contaminated_ = contaminated;
	interval_contaminated_ = contaminated;
	live_classifier_overflow_ = false;
	interval_classifier_overflow_ = false;
	arithmetic_overflow_ = false;
	locked_ = false;
	fallback_ = false;
	reset_signal_path(contaminated);
}

bool FlickerEngine::apply_matching_configuration(
	const VoltageSampleInputView &input) noexcept
{
	if (have_active_configuration_ &&
		active_configuration_.generation == input.configuration_generation) {
		if (active_configuration_.flicker_lamp_voltage != input.lamp_voltage ||
			active_configuration_.flicker_live_cadence_ms !=
				input.live_cadence_ms ||
			active_configuration_.flicker_pst_interval_seconds !=
				input.pst_interval_seconds ||
			(active_configuration_.flicker_phase_mask &
				~static_cast<std::uint32_t>(input.phase_mask)) != 0U ||
			active_configuration_.reference_voltage_microvolts !=
				input.reference_microvolts)
			return false;
		if (sample_rate_hz_ != input.sample_rate_hz ||
			nominal_hz_ != input.nominal_hz) {
			sample_rate_hz_ = input.sample_rate_hz;
			nominal_hz_ = input.nominal_hz;
			decimation_divisor_ = static_cast<std::uint16_t>(
				sample_rate_hz_ / internal_rate_hz);
			reference_reciprocal_q46_ =
				(std::uint64_t{1U} << 46U) / input.reference_microvolts;
			reset_runtime(false);
		}
		return true;
	}
	if (!load_staged(candidate_configuration_) ||
		candidate_configuration_.generation != input.configuration_generation ||
		(candidate_configuration_.flicker_flags &
			MSAP1_M18_ENGINE_ENABLED) == 0U ||
		candidate_configuration_.flicker_lamp_voltage != input.lamp_voltage ||
		candidate_configuration_.flicker_live_cadence_ms !=
			input.live_cadence_ms ||
		candidate_configuration_.flicker_pst_interval_seconds !=
			input.pst_interval_seconds ||
		(candidate_configuration_.flicker_phase_mask &
			~static_cast<std::uint32_t>(input.phase_mask)) != 0U ||
		candidate_configuration_.reference_voltage_microvolts !=
			input.reference_microvolts)
		return false;
	active_configuration_ = candidate_configuration_;
	have_active_configuration_ = true;
	sample_rate_hz_ = input.sample_rate_hz;
	nominal_hz_ = input.nominal_hz;
	decimation_divisor_ = static_cast<std::uint16_t>(
		sample_rate_hz_ / internal_rate_hz);
	reference_reciprocal_q46_ =
		(std::uint64_t{1U} << 46U) / input.reference_microvolts;
	have_input_sequence_ = false;
	reset_runtime(false);
	return true;
}

std::uint32_t FlickerEngine::status_word(bool discontinuity,
	bool classifier_overflow, bool contaminated, bool settling) const noexcept
{
	return 1U |
		(static_cast<std::uint32_t>(locked_) << 1U) |
		(static_cast<std::uint32_t>(fallback_) << 2U) |
		(static_cast<std::uint32_t>(discontinuity) << 3U) |
		(static_cast<std::uint32_t>(arithmetic_overflow_) << 4U) |
		(static_cast<std::uint32_t>(classifier_overflow) << 5U) |
		(static_cast<std::uint32_t>(contaminated) << 6U) |
		(static_cast<std::uint32_t>(settling) << 7U);
}

void FlickerEngine::process_sample(const VoltageSampleInputView &input,
	std::size_t offset, std::uint64_t sample_index) noexcept
{
	const auto sample = unpack_sample(input.packed_sample_words +
		offset * VoltageSampleProtocol::words_per_sample);
	locked_ = (sample.flags & VoltageSampleProtocol::sample_locked) != 0U;
	fallback_ = (sample.flags & VoltageSampleProtocol::sample_fallback) != 0U;
	const bool sequence_gap = have_last_input_sample_ &&
		sample_index != last_input_sample_ + 1U;
	if (sequence_gap)
		reset_runtime(true);
	last_input_sample_ = sample_index;
	have_last_input_sample_ = true;
	if ((sample.flags & VoltageSampleProtocol::sample_malformed) != 0U) {
		reset_runtime(true);
		last_input_sample_ = sample_index;
		have_last_input_sample_ = true;
		return;
	}
	if ((sample.flags & VoltageSampleProtocol::sample_saturated) != 0U)
		arithmetic_overflow_ = true;
	if (raw_count_ == 0U)
		raw_valid_mask_ = 0x7U;
	constexpr std::array<std::uint32_t, phases> valid_bits{
		VoltageSampleProtocol::sample_valid_a,
		VoltageSampleProtocol::sample_valid_b,
		VoltageSampleProtocol::sample_valid_c};
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		const bool valid =
			(active_configuration_.flicker_phase_mask & (1U << phase)) != 0U &&
			(sample.flags & valid_bits[phase]) != 0U;
		if (!valid) {
			raw_valid_mask_ &= static_cast<std::uint8_t>(~(1U << phase));
			continue;
		}
		const auto value = normalize_microvolts_q16_fast(
			sample.voltage_microvolts[phase],
			reference_reciprocal_q46_, arithmetic_overflow_);
		const auto magnitude = static_cast<std::uint64_t>(
			value < 0 ? -value : value);
		const auto square = magnitude * magnitude;
		if (raw_square_sum_[phase] >
			std::numeric_limits<std::uint64_t>::max() - square) {
			raw_square_sum_[phase] =
				std::numeric_limits<std::uint64_t>::max();
			arithmetic_overflow_ = true;
		} else {
			raw_square_sum_[phase] += square;
		}
	}
	++raw_count_;
	if (raw_count_ >= decimation_divisor_)
		process_decimated(sample_index);
}

void FlickerEngine::process_decimated(std::uint64_t sample_index) noexcept
{
	const auto first_sample = sample_index - decimation_divisor_ + 1U;
	if (live_ticks_ == 0U)
		live_first_sample_ = first_sample;
	if (interval_ticks_ == 0U)
		interval_first_sample_ = first_sample;
	const bool was_settling = settling_ticks_ != 0U;
	if (settling_ticks_ != 0U)
		--settling_ticks_;
	++adapter_reciprocal_ticks_;
	const auto &coefficients = active_configuration_.flicker_lamp_voltage == 120U
		? coefficients_120 : coefficients_230;
	const auto calibration = active_configuration_.flicker_lamp_voltage == 120U
		? calibration_120_q16 : calibration_230_q16;
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		std::uint32_t pinst_q16 = 0U;
		if ((raw_valid_mask_ & (1U << phase)) != 0U) {
			const auto mean_square_q32 =
				raw_square_sum_[phase] / decimation_divisor_;
			const auto difference = mean_square_q32 >= adapter_q32_[phase]
				? static_cast<std::int64_t>(
					mean_square_q32 - adapter_q32_[phase])
				: -static_cast<std::int64_t>(
					adapter_q32_[phase] - mean_square_q32);
			const auto step = difference / 54600;
			auto next = step < 0
				? adapter_q32_[phase] - magnitude(step)
				: adapter_q32_[phase] + static_cast<std::uint64_t>(step);
			if (next < (std::uint64_t{1U} << 16U)) {
				next = std::uint64_t{1U} << 16U;
				arithmetic_overflow_ = true;
			}
			adapter_q32_[phase] = next;
			if (adapter_reciprocal_ticks_ >= internal_rate_hz)
				adapter_reciprocal_q30_[phase] =
					(std::uint64_t{1U} << 62U) / adapter_q32_[phase];
			const auto ratio_q30 = multiply_shift_saturating(
				mean_square_q32, adapter_reciprocal_q30_[phase], 32U,
				static_cast<std::uint64_t>(
					std::numeric_limits<std::int64_t>::max()),
				arithmetic_overflow_);
			auto filtered = saturating_expression(
				static_cast<std::int64_t>(ratio_q30), false,
				std::int64_t{1} << 30U, true, 0, false,
				arithmetic_overflow_);
			for (std::size_t stage = 0U; stage < 6U; ++stage)
				filtered = run_biquad(filtered, coefficients[stage],
					filter_z1_[phase][stage], filter_z2_[phase][stage],
					arithmetic_overflow_);
			const auto squared_q30 = static_cast<std::int64_t>(
				multiply_shift_saturating(magnitude(filtered),
					magnitude(filtered), 30U,
					static_cast<std::uint64_t>(
						std::numeric_limits<std::int64_t>::max()),
					arithmetic_overflow_, std::uint64_t{1U} << 29U));
			auto memory_q30 = run_biquad(squared_q30, coefficients[6U],
				filter_z1_[phase][6U], filter_z2_[phase][6U],
				arithmetic_overflow_);
			if (memory_q30 < 0)
				memory_q30 = 0;
			pinst_q16 = static_cast<std::uint32_t>(
				multiply_shift_saturating(
					static_cast<std::uint64_t>(memory_q30), calibration,
					30U, std::numeric_limits<std::uint32_t>::max(),
					arithmetic_overflow_));
			++live_valid_count_[phase];
			live_peak_[phase] = std::max(live_peak_[phase], pinst_q16);
			if (!was_settling) {
				++interval_valid_count_[phase];
				interval_peak_[phase] =
					std::max(interval_peak_[phase], pinst_q16);
				bool outside = false;
				const auto bin = histogram_bin_q16(pinst_q16, outside);
				if (outside) {
					interval_classifier_overflow_ = true;
					live_classifier_overflow_ = true;
				}
				auto &count = histogram_[phase][bin];
				if (count == std::numeric_limits<std::uint32_t>::max()) {
					interval_classifier_overflow_ = true;
					live_classifier_overflow_ = true;
				} else {
					++count;
				}
			}
		} else {
			live_contaminated_ = true;
			interval_contaminated_ = true;
		}
	}
	if (adapter_reciprocal_ticks_ >= internal_rate_hz)
		adapter_reciprocal_ticks_ = 0U;
	raw_count_ = 0U;
	raw_valid_mask_ = 0U;
	raw_square_sum_.fill(0U);
	++live_ticks_;
	if (!was_settling)
		++interval_ticks_;
	if (live_ticks_ >= internal_rate_hz)
		complete_live(sample_index);
	if (interval_ticks_ >= classifier_samples)
		complete_interval(sample_index);
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
	std::uint32_t high_bound = *std::max_element(pst.begin(), pst.end());
	while (low_bound < high_bound) {
		const auto middle = low_bound + (high_bound - low_bound + 1U) / 2U;
		if (cube_q16(middle) <= mean_cube_q16)
			low_bound = middle;
		else
			high_bound = middle - 1U;
	}
	return low_bound;
}

void FlickerEngine::emit(std::uint8_t kind, const RecordContext &context,
	std::uint8_t valid_mask, const std::array<std::uint32_t, phases> &pst,
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
	words[4U] = context.generation;
	words[5U] = context.sample_rate_hz;
	const auto maximum_valid = *std::max_element(
		context.valid_count.begin(), context.valid_count.end());
	words[6U] = kind == record_plt
		? clamp_u32(static_cast<std::uint64_t>(interval_seconds) *
			context.sample_rate_hz)
		: clamp_u32(static_cast<std::uint64_t>(maximum_valid) *
			(context.sample_rate_hz / internal_rate_hz));
	words[7U] = static_cast<std::uint32_t>(valid_mask) << 4U;
	if ((context.status & (1U << 4U)) != 0U)
		words[8U] |= 1U;
	if ((context.status & ((1U << 3U) | (1U << 6U))) != 0U ||
		first_after_discontinuity_) {
		words[8U] |= 1U << 2U;
		first_after_discontinuity_ = false;
	}
	words[9U] = low(first_sample);
	words[10U] = high(first_sample);
	words[13U] = static_cast<std::uint32_t>(kind) |
		(static_cast<std::uint32_t>(valid_mask) << 8U);
	words[14U] = low(context.last_sample);
	words[15U] = high(context.last_sample);
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		words[16U + phase] = context.pinst_q16[phase];
		words[19U + phase] = pst[phase];
		words[22U + phase] = plt[phase];
		words[25U + phase] = kind == record_plt &&
			(valid_mask & (1U << phase)) != 0U
			? classifier_samples * plt_periods
			: context.valid_count[phase];
	}
	words[28U] = interval_seconds;
	words[29U] = context.generation;
	words[30U] = static_cast<std::uint32_t>(context.lamp_voltage) |
		(static_cast<std::uint32_t>(context.nominal_hz) << 16U);
	words[31U] = context.status;
	words[32U] = low(first_sample);
	words[33U] = high(first_sample);
	if (!sink_.publish(record))
		fail();
}

void FlickerEngine::complete_live(std::uint64_t last_sample) noexcept
{
	RecordContext context{};
	context.generation = active_configuration_.generation;
	context.sample_rate_hz = sample_rate_hz_;
	context.status = status_word(live_discontinuity_,
		live_classifier_overflow_, live_contaminated_, settling_ticks_ != 0U);
	context.lamp_voltage = active_configuration_.flicker_lamp_voltage;
	context.nominal_hz = nominal_hz_;
	context.first_sample = live_first_sample_;
	context.last_sample = last_sample;
	context.pinst_q16 = live_peak_;
	context.valid_count = live_valid_count_;
	const auto configured_mask = static_cast<std::uint8_t>(
		active_configuration_.flicker_phase_mask & 0x7U);
	for (std::size_t phase = 0U; phase < phases; ++phase)
		if ((configured_mask & (1U << phase)) != 0U &&
			live_valid_count_[phase] == live_ticks_)
			context.phase_mask |= static_cast<std::uint8_t>(1U << phase);
	if ((context.status & (invalid_interval_status | (1U << 7U))) != 0U)
		context.phase_mask = 0U;
	const std::array<std::uint32_t, phases> zero{};
	emit(record_live, context, context.phase_mask, zero, zero,
		live_first_sample_, 1U);
	live_ticks_ = 0U;
	live_valid_count_.fill(0U);
	live_peak_.fill(0U);
	live_discontinuity_ = false;
	live_contaminated_ = false;
	live_classifier_overflow_ = false;
}

void FlickerEngine::complete_interval(std::uint64_t last_sample) noexcept
{
	RecordContext context{};
	context.generation = active_configuration_.generation;
	context.sample_rate_hz = sample_rate_hz_;
	context.status = status_word(interval_discontinuity_,
		interval_classifier_overflow_, interval_contaminated_, false);
	context.phase_mask = static_cast<std::uint8_t>(
		active_configuration_.flicker_phase_mask & 0x7U);
	context.lamp_voltage = active_configuration_.flicker_lamp_voltage;
	context.nominal_hz = nominal_hz_;
	context.first_sample = interval_first_sample_;
	context.last_sample = last_sample;
	context.pinst_q16 = interval_peak_;
	context.valid_count = interval_valid_count_;
	std::array<std::uint32_t, phases> pst{};
	std::array<std::uint32_t, phases> plt{};
	std::uint8_t valid_mask = 0U;
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		std::uint64_t total = 0U;
		for (const auto count : histogram_[phase])
			total += count;
		if ((context.phase_mask & (1U << phase)) != 0U &&
			(context.status & invalid_interval_status) == 0U &&
			interval_valid_count_[phase] == classifier_samples &&
			total == classifier_samples) {
			valid_mask |= static_cast<std::uint8_t>(1U << phase);
			pst[phase] = pst_q16(histogram_[phase], classifier_samples);
		}
	}
	std::uint8_t plt_mask = 0U;
	std::uint64_t plt_first = interval_first_sample_;
	if (valid_mask == context.phase_mask && context.phase_mask != 0U) {
		for (std::size_t phase = 0U; phase < phases; ++phase)
			rolling_pst_[phase][rolling_position_] = pst[phase];
		rolling_first_sample_[rolling_position_] = interval_first_sample_;
		rolling_position_ = (rolling_position_ + 1U) % plt_periods;
		if (rolling_count_ < plt_periods)
			++rolling_count_;
		if (rolling_count_ == plt_periods) {
			plt_first = rolling_first_sample_[rolling_position_];
			plt_mask = context.phase_mask;
			for (std::size_t phase = 0U; phase < phases; ++phase)
				if ((plt_mask & (1U << phase)) != 0U)
					plt[phase] = plt_q16(rolling_pst_[phase]);
		}
	} else {
		reset_plt();
	}
	const std::array<std::uint32_t, phases> no_plt{};
	emit(record_pst, context, valid_mask, pst, no_plt,
		interval_first_sample_, 600U);
	if (plt_mask != 0U)
		emit(record_plt, context, plt_mask, pst, plt, plt_first, 7200U);
	clear_histogram();
	interval_ticks_ = 0U;
	interval_valid_count_.fill(0U);
	interval_peak_.fill(0U);
	interval_discontinuity_ = false;
	interval_contaminated_ = false;
	interval_classifier_overflow_ = false;
}

void FlickerEngine::process(const VoltageSampleInputView &input) noexcept
{
	if (!ready_ || !apply_matching_configuration(input)) {
		note_transport_discontinuity();
		return;
	}
	const bool packet_sequence_gap = have_input_sequence_ &&
		input.sequence != last_input_sequence_ + 1U;
	last_input_sequence_ = input.sequence;
	have_input_sequence_ = true;
	const bool batch_discontinuity =
		(input.batch_status & VoltageSampleProtocol::batch_discontinuity) != 0U;
	const bool source_drop =
		(input.batch_status & VoltageSampleProtocol::batch_source_drop) != 0U;
	if (external_discontinuity_ || packet_sequence_gap || batch_discontinuity)
		reset_runtime(external_discontinuity_ || packet_sequence_gap ||
			source_drop);
	external_discontinuity_ = false;
	for (std::size_t offset = 0U; offset < input.actual_count; ++offset)
		process_sample(input, offset, input.first_sample + offset);
	if (source_drop ||
		input.actual_count != VoltageSampleProtocol::batch_frames)
		reset_runtime(true);
}

} // namespace msap1::aggregation
