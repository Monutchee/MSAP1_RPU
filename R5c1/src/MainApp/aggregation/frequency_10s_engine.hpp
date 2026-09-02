#ifndef MSAP1_R5C1_FREQUENCY_10S_ENGINE_HPP
#define MSAP1_R5C1_FREQUENCY_10S_ENGINE_HPP

#include "aggregation_health.hpp"
#include "aggregation_record_sink.hpp"
#include "frequency_10s_protocol.hpp"

#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/** Public FREQUENCY-10S-v1 record identity and fixed word map. */
struct Frequency10sRecord final {
	static constexpr std::uint32_t magic = 0x3152544DU;
	static constexpr std::uint32_t format = 0x00280001U;
	static constexpr std::uint32_t bytes = 256U;

	static constexpr std::size_t frequency_millihz_word = 16U;
	static constexpr std::size_t cycle_count_word = 17U;
	static constexpr std::size_t duration_q16_word = 18U;
	static constexpr std::size_t first_crossing_q16_word = 20U;
	static constexpr std::size_t last_crossing_q16_word = 22U;
	static constexpr std::size_t interval_end_sample_word = 24U;
	static constexpr std::size_t utc_start_nanoseconds_word = 26U;
	static constexpr std::size_t utc_end_nanoseconds_word = 28U;
	static constexpr std::size_t utc_uncertainty_nanoseconds_word = 30U;
	static constexpr std::size_t measured_sample_rate_millihz_word = 32U;
	static constexpr std::size_t source_sequence_word = 33U;
	static constexpr std::size_t boundary_generation_word = 34U;
	static constexpr std::size_t source_status_word = 35U;
	static constexpr std::size_t reason_word = 36U;
	static constexpr std::size_t observer_drop_count_word = 37U;
	static constexpr std::size_t guard_flags_word = 38U;
	static constexpr std::size_t observed_crossing_count_word = 39U;
	static constexpr std::size_t included_crossing_count_word = 40U;
	static constexpr std::size_t rejected_cycle_count_word = 41U;

	static constexpr std::uint32_t status_arithmetic_error = 1U << 0U;
	static constexpr std::uint32_t status_result_valid = 1U << 1U;
	static constexpr std::uint32_t status_time_aligned = 1U << 2U;
	static constexpr std::uint32_t status_profile_supported = 1U << 3U;
	static constexpr std::uint32_t status_time_synchronized = 1U << 4U;
	static constexpr std::uint32_t status_filter_ready = 1U << 5U;
	static constexpr std::uint32_t status_reference_valid = 1U << 6U;
	static constexpr std::uint32_t status_discontinuity = 1U << 7U;
	static constexpr std::uint32_t status_crossing_overflow = 1U << 8U;
	static constexpr std::uint32_t status_observer_drop = 1U << 9U;
	static constexpr std::uint32_t status_insufficient_crossings = 1U << 10U;
	static constexpr std::uint32_t status_out_of_range = 1U << 11U;
	static constexpr std::uint32_t status_transport_gap = 1U << 12U;
	static constexpr std::uint32_t status_calibration_valid = 1U << 13U;
	static constexpr std::uint32_t status_sample_rate_valid = 1U << 14U;
	static constexpr std::uint32_t status_resynchronized = 1U << 15U;

	static constexpr std::uint32_t reason_insufficient_crossings = 1U << 16U;
	static constexpr std::uint32_t reason_out_of_range = 1U << 17U;
	static constexpr std::uint32_t reason_arithmetic = 1U << 18U;
	static constexpr std::uint32_t reason_transport_gap = 1U << 19U;
	static constexpr std::uint32_t reason_cycle_geometry = 1U << 20U;
	static constexpr std::uint32_t reason_time_geometry = 1U << 21U;
};

/** R5C1 authority for IEC 61000-4-30 ten-second frequency results. */
class Frequency10sEngine final {
public:
	Frequency10sEngine(AggregationRecordSink &sink,
		AggregationHealth &health, bool emit) noexcept;

	bool initialize() noexcept;
	void process(const Frequency10sInputView &input) noexcept;
	void note_transport_discontinuity() noexcept;

	[[nodiscard]] bool ready() const noexcept { return ready_; }

private:
	struct IntervalMetadata final {
		std::uint32_t configuration_generation{};
		std::uint32_t sample_rate_hz{};
		std::uint32_t measured_sample_rate_millihz{};
		std::uint8_t nominal_frequency_hz{};
		std::uint8_t reference_channel{};
		std::uint8_t filter_profile{};
		std::uint8_t calibration_profile{};
		std::uint32_t status{};
		std::uint64_t start_sample{};
		std::uint64_t end_sample{};
		std::uint64_t utc_start_nanoseconds{};
		std::uint64_t utc_end_nanoseconds{};
		std::uint64_t utc_uncertainty_nanoseconds{};
		std::uint32_t boundary_generation{};
	};

	[[nodiscard]] AggregationMeterRecord build_record(
		const Frequency10sInputView &input,
		std::uint32_t additional_reason) const noexcept;
	[[nodiscard]] bool publish(const AggregationMeterRecord &record) noexcept;
	[[nodiscard]] bool publish_gap(std::uint32_t sequence,
		std::uint32_t ordinal) noexcept;
	void remember(const Frequency10sInputView &input) noexcept;
	void fail() noexcept;

	AggregationRecordSink &sink_;
	AggregationHealth &health_;
	bool emit_{};
	bool ready_{};
	bool have_sequence_{};
	bool have_interval_{};
	bool transport_discontinuity_pending_{};
	std::uint32_t last_sequence_{};
	IntervalMetadata last_interval_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_FREQUENCY_10S_ENGINE_HPP
