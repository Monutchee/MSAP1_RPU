#include "harmonic_aggregation_engine.hpp"

#include "metrology_sine_lut.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t meter_record_magic = 0x3152544DU;
constexpr std::uint32_t aggregate_harmonic_format = 0x001F0001U;
constexpr std::size_t aggregate_orders_per_chunk = 23U;
constexpr std::uint32_t status_complete = 1U << 1U;
constexpr std::uint32_t status_aligned = 1U << 2U;
constexpr std::uint32_t status_valid = 1U << 3U;
constexpr std::uint32_t status_magnitude_valid = 1U << 4U;
constexpr std::uint32_t status_full_range = 1U << 5U;
constexpr std::uint32_t status_first_after_discontinuity = 1U << 6U;
constexpr std::uint32_t status_rate_limited = 1U << 7U;
constexpr std::uint64_t magnitude_mask = (std::uint64_t{1} << 40U) - 1U;
constexpr std::uint64_t angle_mask = (std::uint64_t{1} << 20U) - 1U;
constexpr std::array<std::uint32_t, 30U> cordic_atan_turns{
	536870912U, 316933406U, 167458907U, 85004756U, 42667331U,
	21354465U, 10679838U, 5340245U, 2670163U, 1335087U,
	667544U, 333772U, 166886U, 83443U, 41722U, 20861U, 10430U,
	5215U, 2608U, 1304U, 652U, 326U, 163U, 81U, 41U, 20U, 10U,
	5U, 3U, 1U};

std::int64_t arithmetic_shift_right(std::int64_t value,
	unsigned shift) noexcept
{
	if (shift == 0U)
		return value;
	return value >= 0 ? value >> shift :
		-1 - ((-1 - value) >> shift);
}

std::int64_t signed_word(std::uint64_t value) noexcept
{
	std::int64_t result{};
	static_assert(sizeof(result) == sizeof(value));
	std::memcpy(&result, &value, sizeof(result));
	return result;
}

std::uint32_t low(std::uint64_t value) noexcept
{
	return static_cast<std::uint32_t>(value);
}

std::uint32_t high(std::uint64_t value) noexcept
{
	return static_cast<std::uint32_t>(value >> 32U);
}

std::uint64_t saturating_add(std::uint64_t left, std::uint64_t right,
	bool &overflow) noexcept
{
	if (std::numeric_limits<std::uint64_t>::max() - left < right) {
		overflow = true;
		return std::numeric_limits<std::uint64_t>::max();
	}
	return left + right;
}

} // namespace

HarmonicAggregationEngine::HarmonicAggregationEngine(
	AggregationRecordSink &sink, AggregationHealth &health) noexcept
	: sink_(sink), health_(health)
{
}

bool HarmonicAggregationEngine::initialize() noexcept
{
	clear_tier(three_second_, true);
	clear_tier(ten_minute_, true);
	clear_tier(two_hour_, true);
	finalized_magnitude_.fill(0U);
	finalized_valid_.fill(0U);
	finalized_angle_.fill(0U);
	finalized_angle_valid_.fill(0U);
	output_sequence_.fill(0U);
	ten_minute_target_sample_ = 0U;
	ten_minute_target_toggle_ = 0U;
	have_ten_minute_target_toggle_ = false;
	ten_minute_target_valid_ = false;
	ten_minute_contaminated_ = true;
	have_input_sequence_ = false;
	discontinuity_pending_ = true;
	ready_ = true;
	return true;
}

void HarmonicAggregationEngine::clear_tier(TierAccumulator &tier,
	bool first_after_discontinuity) noexcept
{
	/*
	 * TierAccumulator is larger than an aggregation worker's stack.  A whole
	 * object `tier = {}` assignment makes the ARM compiler materialize a full
	 * temporary on that stack before copying it into place.  Clear every field
	 * in situ so resets remain bounded regardless of the accumulator size.
	 */
	for (auto &sum : tier.square_sum) {
		sum.high = 0U;
		sum.low = 0U;
	}
	tier.valid.fill(0U);
	for (auto &sum : tier.phase_real)
		sum = {};
	for (auto &sum : tier.phase_imag)
		sum = {};
	tier.angle_valid.fill(0U);
	tier.contributors = 0U;
	tier.configuration_generation = 0U;
	tier.sample_rate_hz = 0U;
	tier.sample_count = 0U;
	tier.valid_mask = 0U;
	tier.first_sample = 0U;
	tier.last_sample = 0U;
	tier.first_source_sequence = 0U;
	tier.last_source_sequence = 0U;
	tier.qualified_max_order = 0U;
	tier.nominal_frequency_hz = 0U;
	tier.cycle_count = 0U;
	tier.filter_profile_id = 0U;
	tier.arithmetic_error = false;
	tier.active = false;
	tier.first_after_discontinuity = first_after_discontinuity;
}

void HarmonicAggregationEngine::note_transport_discontinuity() noexcept
{
	discontinuity_pending_ = true;
}

void HarmonicAggregationEngine::observe_timing_context(
	const AggregationContext &context) noexcept
{
	const auto toggle = static_cast<std::uint8_t>(
		(context.utc_target_status >> 1U) & 1U);
	if (!have_ten_minute_target_toggle_) {
		have_ten_minute_target_toggle_ = true;
		ten_minute_target_toggle_ = toggle;
		ten_minute_target_sample_ = context.utc_target_sample;
		ten_minute_target_valid_ = (context.utc_target_status & 1U) != 0U;
		return;
	}
	if (toggle == ten_minute_target_toggle_)
		return;

	ten_minute_target_toggle_ = toggle;
	ten_minute_target_sample_ = context.utc_target_sample;
	ten_minute_target_valid_ = (context.utc_target_status & 1U) != 0U;
	reset_tier(ten_minute_, true);
	reset_tier(two_hour_, true);
	ten_minute_contaminated_ = true;
}

void HarmonicAggregationEngine::reset_tier(TierAccumulator &tier,
	bool discontinuity) noexcept
{
	const bool first = discontinuity || tier.first_after_discontinuity;
	clear_tier(tier, first);
}

void HarmonicAggregationEngine::reset_all(bool discontinuity) noexcept
{
	reset_tier(three_second_, discontinuity);
	reset_tier(ten_minute_, discontinuity);
	reset_tier(two_hour_, discontinuity);
	ten_minute_contaminated_ = true;
}

void HarmonicAggregationEngine::begin_tier(TierAccumulator &tier,
	const HarmonicInputView &input) noexcept
{
	const bool first_after = tier.first_after_discontinuity;
	clear_tier(tier, first_after);
	tier.active = true;
	tier.valid.fill(1U);
	tier.angle_valid.fill(1U);
	tier.configuration_generation = input.configuration_generation;
	tier.sample_rate_hz = input.sample_rate_hz;
	tier.valid_mask = input.valid_mask;
	tier.first_sample = input.first_sample;
	tier.last_sample = input.first_sample + input.sample_count - 1U;
	tier.first_source_sequence = input.sequence;
	tier.last_source_sequence = input.sequence;
	tier.qualified_max_order = input.qualified_max_order;
	tier.nominal_frequency_hz = input.nominal_frequency_hz;
	tier.cycle_count = input.cycle_count;
	tier.filter_profile_id = input.filter_profile_id;
}

HarmonicAggregationEngine::HarmonicPoint
HarmonicAggregationEngine::base_point(
	const HarmonicInputView &input, std::size_t channel,
	std::size_t order_index) noexcept
{
	const auto chunk = order_index / HarmonicProtocol::orders_per_chunk;
	const auto entry = order_index % HarmonicProtocol::orders_per_chunk;
	const auto *record = input.records +
		(channel * HarmonicProtocol::chunks_per_channel + chunk) *
			HarmonicProtocol::record_words;
	const auto word = 16U + entry * 2U;
	const auto packed = static_cast<std::uint64_t>(record[word]) |
		(static_cast<std::uint64_t>(record[word + 1U]) << 32U);
	return {
		packed & magnitude_mask,
		static_cast<std::uint32_t>((packed >> 40U) & angle_mask),
		(packed & (std::uint64_t{1} << 60U)) != 0U,
		(packed & (std::uint64_t{1} << 61U)) != 0U,
	};
}

void HarmonicAggregationEngine::accumulate_phase(TierAccumulator &tier,
	std::size_t point, std::uint64_t magnitude,
	std::uint32_t angle_millidegrees) noexcept
{
	/* Base angles are relative angles on a circle. Preserve their sufficient
	 * statistics as a magnitude-weighted vector instead of averaging degree
	 * numbers, which would turn 359 and 1 degrees into 180 degrees. The input
	 * magnitude is 40 bits and Q1.37 trig is 39 bits; even UINT32_MAX base
	 * contributors remain safely inside the signed 128-bit accumulator. */
	const auto turns = static_cast<std::uint32_t>((
		static_cast<std::uint64_t>(angle_millidegrees) << 32U) / 360000U);
	add_phase_product(tier.phase_real[point], magnitude,
		sine_q37(turns + 0x40000000U));
	add_phase_product(tier.phase_imag[point], magnitude, sine_q37(turns));
}

std::int64_t HarmonicAggregationEngine::sine_q37(
	std::uint32_t phase) noexcept
{
	const auto point = phase >> 20U;
	const auto fraction = phase & 0xFFFFFU;
	const auto sample = [](std::uint32_t index) noexcept {
		const auto quadrant = index >> 10U;
		const auto offset = index & 0x3FFU;
		const auto table_index = (quadrant & 1U) == 0U ?
			offset : 1024U - offset;
		const auto value = static_cast<std::int64_t>(
			MET_SINE_QLUT[table_index]);
		return (quadrant & 2U) == 0U ? value : -value;
	};
	const auto first = sample(point);
	const auto second = sample((point + 1U) & 0xFFFU);
	return first * (std::int64_t{1} << 20U) +
		(second - first) * fraction;
}

void HarmonicAggregationEngine::add_phase_product(WideSigned &sum,
	std::uint64_t magnitude, std::int64_t component) noexcept
{
	const auto absolute = component < 0 ?
		static_cast<std::uint64_t>(-component) :
		static_cast<std::uint64_t>(component);
	const auto product = multiply_u64(magnitude, absolute);
	if (component >= 0) {
		const auto previous = sum.low;
		sum.low += product.low;
		const auto carry = sum.low < previous ? 1U : 0U;
		sum.high += product.high;
		sum.high += carry;
	} else {
		const auto borrow = sum.low < product.low ? 1U : 0U;
		sum.low -= product.low;
		sum.high -= product.high;
		sum.high -= borrow;
	}
}

void HarmonicAggregationEngine::add_phase_sum(WideSigned &sum,
	const WideSigned &source) noexcept
{
	const auto previous = sum.low;
	sum.low += source.low;
	const auto carry = sum.low < previous ? 1U : 0U;
	sum.high += source.high;
	sum.high += carry;
}

HarmonicAggregationEngine::WideUnsigned
HarmonicAggregationEngine::phase_absolute(const WideSigned &value) noexcept
{
	if ((value.high & (std::uint64_t{1} << 63U)) == 0U)
		return {value.high, value.low};
	const auto low = ~value.low + 1U;
	return {~value.high + (low == 0U ? 1U : 0U), low};
}

std::int64_t HarmonicAggregationEngine::shifted_phase(
	const WideSigned &value, unsigned shift) noexcept
{
	std::uint64_t word = value.low;
	if (shift != 0U && shift < 64U)
		word = (value.low >> shift) | (value.high << (64U - shift));
	else if (shift >= 64U && shift < 128U)
		word = static_cast<std::uint64_t>(arithmetic_shift_right(
			signed_word(value.high), shift - 64U));
	else if (shift >= 128U)
		word = (value.high & (std::uint64_t{1} << 63U)) != 0U ?
			std::numeric_limits<std::uint64_t>::max() : 0U;
	return signed_word(word);
}

bool HarmonicAggregationEngine::finalize_phase(const WideSigned &real,
	const WideSigned &imag, std::uint32_t &angle_millidegrees) noexcept
{
	const auto real_abs = phase_absolute(real);
	const auto imag_abs = phase_absolute(imag);
	const auto imag_dominant = less_equal(real_abs, imag_abs);
	const auto dominant = imag_dominant ? imag_abs : real_abs;
	if (dominant.high == 0U && dominant.low == 0U) {
		angle_millidegrees = 0U;
		return false;
	}

	/* met_atan2_turns accepts signed 64-bit operands. Shift both components
	 * by one common power of two so their ratio and quadrant are unchanged
	 * while the dominant magnitude fits below the sign bit. */
	unsigned most_significant_bit = 0U;
	if (dominant.high != 0U) {
		for (int bit = 63; bit >= 0; --bit) {
			if ((dominant.high &
				(std::uint64_t{1} << static_cast<unsigned>(bit))) != 0U) {
				most_significant_bit = 64U + static_cast<unsigned>(bit);
				break;
			}
		}
	} else {
		for (int bit = 63; bit >= 0; --bit) {
			if ((dominant.low &
				(std::uint64_t{1} << static_cast<unsigned>(bit))) != 0U) {
				most_significant_bit = static_cast<unsigned>(bit);
				break;
			}
		}
	}
	const auto shift = most_significant_bit > 61U ?
		most_significant_bit - 61U : 0U;
	const auto scaled_real = shifted_phase(real, shift);
	const auto scaled_imag = shifted_phase(imag, shift);
	if (scaled_real == 0 && scaled_imag == 0) {
		angle_millidegrees = 0U;
		return false;
	}
	angle_millidegrees = atan2_millidegrees(scaled_imag, scaled_real);
	return true;
}

std::uint32_t HarmonicAggregationEngine::atan2_millidegrees(
	std::int64_t imaginary, std::int64_t real) noexcept
{
	if (imaginary == 0 && real == 0)
		return 0U;

	/* This is the R5-native equivalent of met_atan2_turns.  The shared
	 * ap_int implementation is appropriate for HLS, but synthesizes hundreds
	 * of software helper operations on Arm for every published harmonic. */
	auto x = real;
	auto y = imaginary;
	std::uint32_t angle = 0U;
	if (x < 0) {
		x = -x;
		y = -y;
		angle = 0x80000000U;
	}
	const auto absolute = [](std::int64_t value) noexcept {
		return value < 0 ?
			static_cast<std::uint64_t>(-(value + 1)) + 1U :
			static_cast<std::uint64_t>(value);
	};
	auto magnitude = std::max(absolute(x), absolute(y));
	while (magnitude < (std::uint64_t{1} << 44U)) {
		magnitude <<= 1U;
		x *= 2;
		y *= 2;
	}
	while (magnitude >= (std::uint64_t{1} << 45U)) {
		magnitude >>= 1U;
		x = arithmetic_shift_right(x, 1U);
		y = arithmetic_shift_right(y, 1U);
	}

	for (unsigned iteration = 0U; iteration < cordic_atan_turns.size();
		++iteration) {
		const auto shifted_x = arithmetic_shift_right(x, iteration);
		const auto shifted_y = arithmetic_shift_right(y, iteration);
		if (y >= 0) {
			x += shifted_y;
			y -= shifted_x;
			angle += cordic_atan_turns[iteration];
		} else {
			x -= shifted_y;
			y += shifted_x;
			angle -= cordic_atan_turns[iteration];
		}
	}
	return static_cast<std::uint32_t>(
		(static_cast<std::uint64_t>(angle) * 360000U) >> 32U);
}

void HarmonicAggregationEngine::accumulate_base(TierAccumulator &tier,
	const HarmonicInputView &input) noexcept
{
	if (!tier.active)
		begin_tier(tier, input);
	tier.valid_mask &= input.valid_mask;
	tier.last_sample = input.first_sample + input.sample_count - 1U;
	tier.last_source_sequence = input.sequence;
	tier.qualified_max_order = std::min(tier.qualified_max_order,
		input.qualified_max_order);
	tier.sample_count = saturating_add(tier.sample_count, input.sample_count,
		tier.arithmetic_error);
	++tier.contributors;

	for (std::size_t channel = 0U; channel < HarmonicProtocol::channels;
		++channel) {
		for (std::size_t order = 0U;
			order < HarmonicProtocol::maximum_order; ++order) {
			const auto point = channel * HarmonicProtocol::maximum_order + order;
			const auto source = base_point(input, channel, order);
			tier.valid[point] &= source.magnitude_valid ? 1U : 0U;
			tier.angle_valid[point] &= source.angle_valid ? 1U : 0U;
			if (!source.magnitude_valid)
				continue;
			const auto square = multiply_u64(source.magnitude, source.magnitude);
			if (!add_checked(tier.square_sum[point], square)) {
				tier.square_sum[point] = {
					std::numeric_limits<std::uint64_t>::max(),
					std::numeric_limits<std::uint64_t>::max()};
				tier.arithmetic_error = true;
			}
			if (source.angle_valid)
				accumulate_phase(tier, point, source.magnitude,
					source.angle_millidegrees);
		}
	}
}

HarmonicAggregationEngine::WideUnsigned
HarmonicAggregationEngine::multiply_u64(std::uint64_t left,
	std::uint64_t right) noexcept
{
	const auto left_low = static_cast<std::uint64_t>(
		static_cast<std::uint32_t>(left));
	const auto left_high = left >> 32U;
	const auto right_low = static_cast<std::uint64_t>(
		static_cast<std::uint32_t>(right));
	const auto right_high = right >> 32U;
	const auto product_low = left_low * right_low;
	const auto product_cross_0 = left_low * right_high;
	const auto product_cross_1 = left_high * right_low;
	const auto product_high = left_high * right_high;
	const auto middle = (product_low >> 32U) +
		static_cast<std::uint32_t>(product_cross_0) +
		static_cast<std::uint32_t>(product_cross_1);
	return {
		product_high + (product_cross_0 >> 32U) +
			(product_cross_1 >> 32U) + (middle >> 32U),
		(middle << 32U) | static_cast<std::uint32_t>(product_low)};
}

bool HarmonicAggregationEngine::add_checked(WideUnsigned &left,
	WideUnsigned right) noexcept
{
	const auto low = left.low + right.low;
	const auto carry = static_cast<std::uint64_t>(low < left.low);
	const auto high_without_carry = left.high + right.high;
	const bool overflow = high_without_carry < left.high ||
		(high_without_carry == std::numeric_limits<std::uint64_t>::max() &&
			carry != 0U);
	left.low = low;
	left.high = high_without_carry + carry;
	return !overflow;
}

HarmonicAggregationEngine::WideUnsigned
HarmonicAggregationEngine::divide_u32(WideUnsigned value,
	std::uint32_t divisor) noexcept
{
	if (divisor == 0U)
		return {};
	const std::array<std::uint32_t, 4U> dividend{
		static_cast<std::uint32_t>(value.high >> 32U),
		static_cast<std::uint32_t>(value.high),
		static_cast<std::uint32_t>(value.low >> 32U),
		static_cast<std::uint32_t>(value.low)};
	std::array<std::uint32_t, 4U> quotient{};
	std::uint64_t remainder = 0U;
	for (std::size_t index = 0U; index < dividend.size(); ++index) {
		const auto step = (remainder << 32U) | dividend[index];
		quotient[index] = static_cast<std::uint32_t>(step / divisor);
		remainder = step % divisor;
	}
	return {
		(static_cast<std::uint64_t>(quotient[0U]) << 32U) | quotient[1U],
		(static_cast<std::uint64_t>(quotient[2U]) << 32U) | quotient[3U]};
}

bool HarmonicAggregationEngine::less_equal(WideUnsigned left,
	WideUnsigned right) noexcept
{
	return left.high < right.high ||
		(left.high == right.high && left.low <= right.low);
}

std::uint64_t HarmonicAggregationEngine::integer_sqrt(
	WideUnsigned value) noexcept
{
	WideUnsigned remainder{};
	std::uint64_t result = 0U;
	for (std::uint32_t digit = 0U; digit < 64U; ++digit) {
		const auto input_pair = value.high >> 62U;
		value.high = (value.high << 2U) | (value.low >> 62U);
		value.low <<= 2U;

		remainder.high = (remainder.high << 2U) |
			(remainder.low >> 62U);
		remainder.low = (remainder.low << 2U) | input_pair;
		result <<= 1U;

		const WideUnsigned trial{
			result >> 63U,
			(result << 1U) | 1U};
		if (less_equal(trial, remainder)) {
			const auto borrow = remainder.low < trial.low ? 1U : 0U;
			remainder.low -= trial.low;
			remainder.high -= trial.high + borrow;
			++result;
		}
	}
	return result;
}

void HarmonicAggregationEngine::finalize_values(
	const TierAccumulator &tier) noexcept
{
	for (std::size_t point = 0U; point < point_count; ++point) {
		finalized_valid_[point] = tier.active && tier.contributors != 0U &&
			tier.valid[point] != 0U && !tier.arithmetic_error;
		finalized_magnitude_[point] = finalized_valid_[point]
			? integer_sqrt(divide_u32(tier.square_sum[point], tier.contributors))
			: 0U;
		finalized_angle_[point] = 0U;
		finalized_angle_valid_[point] = finalized_valid_[point] != 0U &&
			tier.angle_valid[point] != 0U &&
			finalize_phase(tier.phase_real[point], tier.phase_imag[point],
				finalized_angle_[point]);
	}
}

void HarmonicAggregationEngine::accumulate_finalized(
	TierAccumulator &tier, const TierAccumulator &source) noexcept
{
	if (!tier.active) {
		const bool first_after = tier.first_after_discontinuity;
		clear_tier(tier, first_after);
		tier.active = true;
		tier.valid.fill(1U);
		tier.angle_valid.fill(1U);
		tier.configuration_generation = source.configuration_generation;
		tier.sample_rate_hz = source.sample_rate_hz;
		tier.valid_mask = source.valid_mask;
		tier.first_sample = source.first_sample;
		tier.first_source_sequence = source.first_source_sequence;
		tier.qualified_max_order = source.qualified_max_order;
		tier.nominal_frequency_hz = source.nominal_frequency_hz;
		tier.cycle_count = source.cycle_count;
		tier.filter_profile_id = source.filter_profile_id;
	}
	tier.arithmetic_error = tier.arithmetic_error || source.arithmetic_error;
	tier.valid_mask &= source.valid_mask;
	tier.last_sample = source.last_sample;
	tier.last_source_sequence = source.last_source_sequence;
	tier.qualified_max_order = std::min(tier.qualified_max_order,
		source.qualified_max_order);
	tier.sample_count = saturating_add(tier.sample_count, source.sample_count,
		tier.arithmetic_error);
	++tier.contributors;
	for (std::size_t point = 0U; point < point_count; ++point) {
		tier.valid[point] &= finalized_valid_[point];
		const bool source_angle_valid = source.active &&
			source.contributors != 0U && source.angle_valid[point] != 0U &&
			!source.arithmetic_error;
		tier.angle_valid[point] &= source_angle_valid ? 1U : 0U;
		if (finalized_valid_[point] == 0U)
			continue;
		const auto magnitude = finalized_magnitude_[point];
		const auto square = multiply_u64(magnitude, magnitude);
		if (!add_checked(tier.square_sum[point], square)) {
			tier.square_sum[point] = {
				std::numeric_limits<std::uint64_t>::max(),
				std::numeric_limits<std::uint64_t>::max()};
			tier.arithmetic_error = true;
		}
		/* The source's raw vector sum is the sufficient statistic. Cascading
		 * it directly makes the 2-hour angle exactly the circular aggregate of
		 * all base families, without quantizing through each 10-minute angle. */
		if (source_angle_valid && tier.angle_valid[point] != 0U) {
			add_phase_sum(tier.phase_real[point], source.phase_real[point]);
			add_phase_sum(tier.phase_imag[point], source.phase_imag[point]);
		}
	}
}

bool HarmonicAggregationEngine::emit_family(const TierAccumulator &tier,
	OutputPeriod period, std::uint64_t target_sample,
	std::uint32_t overshoot_samples, bool aligned,
	bool contaminated) noexcept
{
	finalize_values(tier);
	const auto period_index = static_cast<std::size_t>(period);
	const auto sequence = output_sequence_[period_index]++;
	const auto contributors = std::min<std::uint32_t>(tier.contributors, 0xFFFU);
	const auto overshoot = std::min<std::uint32_t>(overshoot_samples, 0xFFFFU);
	const auto shape = static_cast<std::uint32_t>(period) |
		(contributors << 2U) | (overshoot << 14U) |
		(aligned ? (1U << 30U) : 0U) |
		(contaminated ? (1U << 31U) : 0U);

	std::uint32_t status = status_complete;
	if (aligned)
		status |= status_aligned;
	if (!contaminated && !tier.arithmetic_error)
		status |= status_valid | status_magnitude_valid;
	if (tier.qualified_max_order == HarmonicProtocol::maximum_order)
		status |= status_full_range;
	else
		status |= status_rate_limited;
	if (tier.first_after_discontinuity)
		status |= status_first_after_discontinuity;
	if (tier.arithmetic_error)
		status |= 1U;

	for (std::size_t channel = 0U; channel < HarmonicProtocol::channels;
		++channel) {
		for (std::size_t chunk = 0U;
			chunk < HarmonicProtocol::chunks_per_channel; ++chunk) {
			AggregationMeterRecord record{};
			record.sequence = sequence;
			auto &words = record.words;
			words[0U] = meter_record_magic;
			words[1U] = aggregate_harmonic_format;
			words[2U] = AggregationMeterRecord::byte_count;
			words[3U] = sequence;
			words[4U] = tier.configuration_generation;
			words[5U] = tier.sample_rate_hz;
			words[6U] = static_cast<std::uint32_t>(tier.sample_count);
			words[7U] = tier.valid_mask;
			words[8U] = status;
			words[9U] = low(tier.first_sample);
			words[10U] = high(tier.first_sample);
			words[11U] = low(target_sample);
			words[12U] = high(target_sample);

			const auto first_order = chunk * aggregate_orders_per_chunk + 1U;
			const auto entry_count = std::min(aggregate_orders_per_chunk,
				HarmonicProtocol::maximum_order - first_order + 1U);
			words[13U] = static_cast<std::uint32_t>(channel) |
				(static_cast<std::uint32_t>(chunk) << 3U) |
				(static_cast<std::uint32_t>(first_order) << 7U) |
				(static_cast<std::uint32_t>(entry_count) << 15U) |
				(static_cast<std::uint32_t>(
					HarmonicProtocol::chunks_per_channel) << 20U) |
				(static_cast<std::uint32_t>(
					HarmonicProtocol::maximum_order) << 24U);
			words[14U] = shape;
			words[15U] = tier.qualified_max_order |
				(static_cast<std::uint32_t>(tier.nominal_frequency_hz) << 8U) |
				(static_cast<std::uint32_t>(tier.cycle_count) << 16U) |
				(static_cast<std::uint32_t>(tier.filter_profile_id) << 24U);

			for (std::size_t entry = 0U; entry < entry_count; ++entry) {
				const auto order = first_order + entry - 1U;
				const auto point = channel * HarmonicProtocol::maximum_order + order;
				/* Invalid packed entries must be all-zero.  In particular, a
				 * contaminated long interval deliberately clears the family and
				 * entry validity bits; retaining its diagnostic RMS magnitude while
				 * clearing bit 60 creates a self-contradictory wire image that every
				 * conforming HARMONIC-v1 decoder must reject. */
				std::uint64_t packed = 0U;
				if (!contaminated && finalized_valid_[point] != 0U) {
					packed = finalized_magnitude_[point] |
						(std::uint64_t{1} << 60U);
					if (finalized_angle_valid_[point] != 0U) {
						packed |= static_cast<std::uint64_t>(
							finalized_angle_[point]) << 40U;
						packed |= std::uint64_t{1} << 61U;
					}
				}
				words[16U + entry * 2U] = low(packed);
				words[17U + entry * 2U] = high(packed);
			}
			words[62U] = tier.first_source_sequence;
			words[63U] = tier.last_source_sequence;
			if (!sink_.publish(record)) {
				ready_ = false;
				health_.set_engine_ready(false);
				return false;
			}
		}
	}
	return true;
}

bool HarmonicAggregationEngine::accept_sequence(
	const HarmonicInputView &input) noexcept
{
	if (!have_input_sequence_) {
		have_input_sequence_ = true;
		last_input_sequence_ = input.sequence;
		expected_first_sample_ = input.first_sample + input.sample_count;
		return true;
	}
	const auto expected_sequence = last_input_sequence_ + 1U;
	const auto delta = static_cast<std::int32_t>(input.sequence -
		expected_sequence);
	if (delta < 0)
		return false;
	/* The conditioner retains the exact nominal L/25 spectral lattice, while
	 * the two independently quantized source-block endpoints may move the next
	 * first sample by two accepted ADC frames.  Normalize only that bounded
	 * displacement; a larger displacement or sequence gap remains a real
	 * discontinuity. */
	constexpr std::uint64_t endpoint_quantization_frames = 2U;
	const auto sample_delta = input.first_sample >= expected_first_sample_
		? input.first_sample - expected_first_sample_
		: expected_first_sample_ - input.first_sample;
	const bool adjacent_sample = sample_delta <= endpoint_quantization_frames;
	if (delta > 0 || !adjacent_sample)
		discontinuity_pending_ = true;
	last_input_sequence_ = input.sequence;
	expected_first_sample_ = input.first_sample + input.sample_count;
	return true;
}

void HarmonicAggregationEngine::process(const HarmonicInputView &input) noexcept
{
	if (!ready_ || input.records == nullptr || !accept_sequence(input))
		return;
	if ((input.status & (1U << 6U)) != 0U)
		discontinuity_pending_ = true;
	if (discontinuity_pending_) {
		reset_all(true);
		discontinuity_pending_ = false;
	}

	const bool eligible = (input.status & 1U) == 0U &&
		(input.status & ((1U << 2U) | (1U << 3U) | (1U << 4U))) ==
			((1U << 2U) | (1U << 3U) | (1U << 4U));
	if (!eligible) {
		reset_all(true);
		return;
	}
	const auto incompatible = [&input](const TierAccumulator &tier) noexcept {
		return tier.active &&
			(tier.configuration_generation != input.configuration_generation ||
			 tier.sample_rate_hz != input.sample_rate_hz ||
			 tier.nominal_frequency_hz != input.nominal_frequency_hz ||
			 tier.cycle_count != input.cycle_count ||
			 tier.filter_profile_id != input.filter_profile_id);
	};
	if (incompatible(three_second_) || incompatible(ten_minute_))
		reset_all(true);

	accumulate_base(three_second_, input);
	if (three_second_.contributors == 15U) {
		const auto target = three_second_.last_sample + 1U;
		(void)emit_family(three_second_, OutputPeriod::cycles_150_180,
			target, 0U, true, false);
		reset_tier(three_second_, false);
		three_second_.first_after_discontinuity = false;
	}

	if (!ten_minute_target_valid_)
		return;
	accumulate_base(ten_minute_, input);
	const auto end_sample = ten_minute_.last_sample + 1U;
	if (end_sample < ten_minute_target_sample_)
		return;

	const auto overshoot64 = end_sample - ten_minute_target_sample_;
	const auto overshoot = static_cast<std::uint32_t>(std::min<std::uint64_t>(
		overshoot64, std::numeric_limits<std::uint32_t>::max()));
	const bool contaminated = ten_minute_contaminated_;
	(void)emit_family(ten_minute_, OutputPeriod::minutes_10,
		ten_minute_target_sample_, overshoot, true, contaminated);

	if (!contaminated && ready_) {
		/* emit_family left this tier's finalized values in the scratch arrays. */
		accumulate_finalized(two_hour_, ten_minute_);
		if (two_hour_.contributors == 12U) {
			(void)emit_family(two_hour_, OutputPeriod::hours_2,
				ten_minute_target_sample_, overshoot, true, false);
			reset_tier(two_hour_, false);
			two_hour_.first_after_discontinuity = false;
		}
	}

	const auto interval_samples =
		static_cast<std::uint64_t>(input.sample_rate_hz) * 600U;
	ten_minute_target_sample_ += interval_samples;
	reset_tier(ten_minute_, false);
	ten_minute_.first_after_discontinuity = false;
	ten_minute_contaminated_ = false;
}

} // namespace msap1::aggregation
