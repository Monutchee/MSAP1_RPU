#ifndef MSAP1_R5C1_HARMONIC_AGGREGATION_ENGINE_HPP
#define MSAP1_R5C1_HARMONIC_AGGREGATION_ENGINE_HPP

#include "aggregation_health.hpp"
#include "aggregation_record_sink.hpp"
#include "harmonic_protocol.hpp"
#include "metering_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/** R5C1 owner of 3-second, 10-minute, and 2-hour harmonic spectra. */
class HarmonicAggregationEngine final {
public:
	HarmonicAggregationEngine(AggregationRecordSink &sink,
		AggregationHealth &health) noexcept;

	bool initialize() noexcept;
	void observe_timing_context(const AggregationContext &context) noexcept;
	void process(const HarmonicInputView &input) noexcept;
	void note_transport_discontinuity() noexcept;
	[[nodiscard]] bool ready() const noexcept { return ready_; }

private:
	struct WideUnsigned final {
		std::uint64_t high{};
		std::uint64_t low{};
	};
	struct HarmonicPoint final {
		std::uint64_t magnitude{};
		std::uint32_t angle_millidegrees{};
		bool magnitude_valid{};
		bool angle_valid{};
	};
	using PhaseSum = ap_int<128>;
	static constexpr std::size_t point_count =
		HarmonicProtocol::channels * HarmonicProtocol::maximum_order;

	enum class OutputPeriod : std::uint8_t {
		cycles_150_180 = 1U,
		minutes_10 = 2U,
		hours_2 = 3U,
	};

	struct TierAccumulator final {
		TierAccumulator() noexcept = default;
		TierAccumulator(const TierAccumulator &) = delete;
		TierAccumulator &operator=(const TierAccumulator &) = delete;
		TierAccumulator(TierAccumulator &&) = delete;
		TierAccumulator &operator=(TierAccumulator &&) = delete;

		std::array<WideUnsigned, point_count> square_sum{};
		std::array<std::uint8_t, point_count> valid{};
		std::array<PhaseSum, point_count> phase_real{};
		std::array<PhaseSum, point_count> phase_imag{};
		std::array<std::uint8_t, point_count> angle_valid{};
		std::uint32_t contributors{};
		std::uint32_t configuration_generation{};
		std::uint32_t sample_rate_hz{};
		std::uint64_t sample_count{};
		std::uint8_t valid_mask{};
		std::uint64_t first_sample{};
		std::uint64_t last_sample{};
		std::uint32_t first_source_sequence{};
		std::uint32_t last_source_sequence{};
		std::uint8_t qualified_max_order{};
		std::uint8_t nominal_frequency_hz{};
		std::uint8_t cycle_count{};
		std::uint8_t filter_profile_id{};
		bool arithmetic_error{};
		bool active{};
		bool first_after_discontinuity{true};
	};

	static void clear_tier(TierAccumulator &tier,
		bool first_after_discontinuity) noexcept;
	void reset_tier(TierAccumulator &tier, bool discontinuity) noexcept;
	void reset_all(bool discontinuity) noexcept;
	void begin_tier(TierAccumulator &tier,
		const HarmonicInputView &input) noexcept;
	void accumulate_base(TierAccumulator &tier,
		const HarmonicInputView &input) noexcept;
	void accumulate_finalized(TierAccumulator &tier,
		const TierAccumulator &source) noexcept;
	void finalize_values(const TierAccumulator &tier) noexcept;
	bool emit_family(const TierAccumulator &tier, OutputPeriod period,
		std::uint64_t target_sample, std::uint32_t overshoot_samples,
		bool aligned, bool contaminated) noexcept;
	[[nodiscard]] static std::uint64_t integer_sqrt(
		WideUnsigned value) noexcept;
	[[nodiscard]] static WideUnsigned multiply_u64(
		std::uint64_t left, std::uint64_t right) noexcept;
	[[nodiscard]] static bool add_checked(WideUnsigned &left,
		WideUnsigned right) noexcept;
	[[nodiscard]] static WideUnsigned divide_u32(WideUnsigned value,
		std::uint32_t divisor) noexcept;
	[[nodiscard]] static bool less_equal(WideUnsigned left,
		WideUnsigned right) noexcept;
	[[nodiscard]] static HarmonicPoint base_point(
		const HarmonicInputView &input, std::size_t channel,
		std::size_t order_index) noexcept;
	static void accumulate_phase(TierAccumulator &tier, std::size_t point,
		std::uint64_t magnitude, std::uint32_t angle_millidegrees) noexcept;
	[[nodiscard]] static bool finalize_phase(const PhaseSum &real,
		const PhaseSum &imag, std::uint32_t &angle_millidegrees) noexcept;
	[[nodiscard]] bool accept_sequence(const HarmonicInputView &input) noexcept;

	AggregationRecordSink &sink_;
	AggregationHealth &health_;
	TierAccumulator three_second_{};
	TierAccumulator ten_minute_{};
	TierAccumulator two_hour_{};
	std::array<std::uint64_t, point_count> finalized_magnitude_{};
	std::array<std::uint8_t, point_count> finalized_valid_{};
	std::array<std::uint32_t, point_count> finalized_angle_{};
	std::array<std::uint8_t, point_count> finalized_angle_valid_{};
	std::array<std::uint32_t, 4U> output_sequence_{};
	std::uint64_t ten_minute_target_sample_{};
	std::uint8_t ten_minute_target_toggle_{};
	bool have_ten_minute_target_toggle_{};
	bool ten_minute_target_valid_{};
	bool ten_minute_contaminated_{true};
	std::uint32_t last_input_sequence_{};
	std::uint64_t expected_first_sample_{};
	bool have_input_sequence_{};
	bool discontinuity_pending_{true};
	bool ready_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_HARMONIC_AGGREGATION_ENGINE_HPP
