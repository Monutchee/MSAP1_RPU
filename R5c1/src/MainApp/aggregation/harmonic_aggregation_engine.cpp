#include "harmonic_aggregation_engine.hpp"

#include <algorithm>
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
	three_second_ = {};
	ten_minute_ = {};
	two_hour_ = {};
	finalized_magnitude_.fill(0U);
	finalized_valid_.fill(0U);
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
	tier = {};
	tier.first_after_discontinuity = first;
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
	tier = {};
	tier.active = true;
	tier.first_after_discontinuity = first_after;
	tier.valid.fill(1U);
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

std::uint64_t HarmonicAggregationEngine::base_magnitude(
	const HarmonicInputView &input, std::size_t channel,
	std::size_t order_index, bool &valid) noexcept
{
	const auto chunk = order_index / HarmonicProtocol::orders_per_chunk;
	const auto entry = order_index % HarmonicProtocol::orders_per_chunk;
	const auto *record = input.records +
		(channel * HarmonicProtocol::chunks_per_channel + chunk) *
			HarmonicProtocol::record_words;
	const auto word = 16U + entry * 2U;
	const auto packed = static_cast<std::uint64_t>(record[word]) |
		(static_cast<std::uint64_t>(record[word + 1U]) << 32U);
	valid = (packed & (std::uint64_t{1} << 60U)) != 0U;
	return packed & ((std::uint64_t{1} << 40U) - 1U);
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
			bool valid = false;
			const auto magnitude = base_magnitude(input, channel, order, valid);
			tier.valid[point] &= valid ? 1U : 0U;
			if (!valid)
				continue;
			const auto square = multiply_u64(magnitude, magnitude);
			if (!add_checked(tier.square_sum[point], square)) {
				tier.square_sum[point] = {
					std::numeric_limits<std::uint64_t>::max(),
					std::numeric_limits<std::uint64_t>::max()};
				tier.arithmetic_error = true;
			}
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
	std::uint64_t result = 0U;
	for (int bit = 63; bit >= 0; --bit) {
		const auto candidate = result |
			(std::uint64_t{1} << static_cast<unsigned>(bit));
		if (less_equal(multiply_u64(candidate, candidate), value))
			result = candidate;
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
	}
}

void HarmonicAggregationEngine::accumulate_finalized(
	TierAccumulator &tier, const TierAccumulator &source) noexcept
{
	if (!tier.active) {
		const bool first_after = tier.first_after_discontinuity;
		tier = {};
		tier.active = true;
		tier.first_after_discontinuity = first_after;
		tier.valid.fill(1U);
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
				std::uint64_t packed = finalized_magnitude_[point];
				if (!contaminated && finalized_valid_[point] != 0U)
					packed |= std::uint64_t{1} << 60U;
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
	if (delta > 0 || input.first_sample != expected_first_sample_)
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
