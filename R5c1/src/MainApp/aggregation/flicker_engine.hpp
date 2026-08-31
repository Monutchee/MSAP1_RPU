#ifndef MSAP1_R5C1_FLICKER_ENGINE_HPP
#define MSAP1_R5C1_FLICKER_ENGINE_HPP

#include "aggregation_health.hpp"
#include "aggregation_record_sink.hpp"
#include "power_quality_protocol.hpp"

#include "power_quality_configuration.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/** Complete R5C1 IEC flickermeter fed by packed PL voltage batches. */
class FlickerEngine final {
public:
	FlickerEngine(AggregationRecordSink &sink,
		AggregationHealth &health) noexcept;

	[[nodiscard]] bool configure(
		const msap1_m18_config_payload &configuration) noexcept;
	bool initialize() noexcept;
	void process(const VoltageSampleInputView &input) noexcept;
	void note_transport_discontinuity() noexcept;

private:
	friend struct FlickerEngineTestAccess;
	static constexpr std::size_t phases = VoltageSampleProtocol::phases;
	static constexpr std::size_t bins = VoltageSampleProtocol::classifier_bins;
	static constexpr std::size_t filter_stages = 7U;
	static constexpr std::size_t plt_periods = 12U;
	static constexpr std::size_t configuration_words =
		sizeof(msap1_m18_config_payload) / sizeof(std::uint32_t);
	struct RecordContext final {
		std::uint32_t generation{};
		std::uint32_t sample_rate_hz{};
		std::uint32_t status{};
		std::uint8_t phase_mask{};
		std::uint16_t lamp_voltage{};
		std::uint8_t nominal_hz{};
		std::uint64_t first_sample{};
		std::uint64_t last_sample{};
		std::array<std::uint32_t, phases> pinst_q16{};
		std::array<std::uint32_t, phases> valid_count{};
	};

	[[nodiscard]] bool load_staged(
		msap1_m18_config_payload &configuration) const noexcept;
	[[nodiscard]] bool apply_matching_configuration(
		const VoltageSampleInputView &input) noexcept;
	void reset_runtime(bool contaminated) noexcept;
	void reset_signal_path(bool contaminated) noexcept;
	void process_sample(const VoltageSampleInputView &input, std::size_t offset,
		std::uint64_t sample_index) noexcept;
	void process_decimated(std::uint64_t sample_index) noexcept;
	void complete_live(std::uint64_t last_sample) noexcept;
	void complete_interval(std::uint64_t last_sample) noexcept;
	[[nodiscard]] std::uint32_t status_word(bool discontinuity,
		bool classifier_overflow, bool contaminated,
		bool settling) const noexcept;
	void clear_histogram() noexcept;
	void reset_plt() noexcept;
	void emit(std::uint8_t kind, const RecordContext &context,
		std::uint8_t valid_mask,
		const std::array<std::uint32_t, phases> &pst,
		const std::array<std::uint32_t, phases> &plt,
		std::uint64_t first_sample, std::uint32_t interval_seconds) noexcept;
	void fail() noexcept;

	[[nodiscard]] static std::uint32_t percentile_q16(
		const std::array<std::uint32_t, bins> &histogram,
		std::uint32_t total, std::uint32_t exceedance_tenths) noexcept;
	[[nodiscard]] static std::uint32_t pst_q16(
		const std::array<std::uint32_t, bins> &histogram,
		std::uint32_t total) noexcept;
	[[nodiscard]] static std::uint32_t plt_q16(
		const std::array<std::uint32_t, plt_periods> &pst) noexcept;
	[[nodiscard]] static std::int64_t normalize_microvolts_q16(
		std::int32_t sample, std::uint64_t reciprocal_q46,
		bool &overflow) noexcept;

	AggregationRecordSink &sink_;
	AggregationHealth &health_;
	alignas(4) std::array<std::uint32_t, configuration_words> staged_words_{};
	std::uint32_t staged_revision_{};
	msap1_m18_config_payload candidate_configuration_{};
	msap1_m18_config_payload active_configuration_{};
	std::array<std::array<std::uint32_t, bins>, phases> histogram_{};
	std::array<std::array<std::uint32_t, plt_periods>, phases> rolling_pst_{};
	std::array<std::uint64_t, plt_periods> rolling_first_sample_{};
	std::array<std::uint64_t, phases> raw_square_sum_{};
	std::array<std::uint64_t, phases> adapter_q32_{};
	std::array<std::uint64_t, phases> adapter_reciprocal_q30_{};
	std::array<std::array<std::int64_t, filter_stages>, phases> filter_z1_{};
	std::array<std::array<std::int64_t, filter_stages>, phases> filter_z2_{};
	std::array<std::uint32_t, phases> live_valid_count_{};
	std::array<std::uint32_t, phases> live_peak_{};
	std::array<std::uint32_t, phases> interval_valid_count_{};
	std::array<std::uint32_t, phases> interval_peak_{};
	std::uint64_t reference_reciprocal_q46_{};
	std::uint64_t last_input_sample_{};
	std::uint64_t live_first_sample_{};
	std::uint64_t interval_first_sample_{};
	std::uint32_t sample_rate_hz_{};
	std::uint32_t live_ticks_{};
	std::uint32_t interval_ticks_{};
	std::uint32_t settling_ticks_{};
	std::uint32_t adapter_reciprocal_ticks_{};
	std::uint32_t output_sequence_{};
	std::uint32_t last_input_sequence_{};
	std::uint16_t decimation_divisor_{1U};
	std::uint16_t raw_count_{};
	std::uint8_t raw_valid_mask_{};
	std::uint8_t nominal_hz_{};
	std::size_t rolling_count_{};
	std::size_t rolling_position_{};
	bool have_active_configuration_{};
	bool have_input_sequence_{};
	bool have_last_input_sample_{};
	bool external_discontinuity_{};
	bool first_after_discontinuity_{true};
	bool arithmetic_overflow_{};
	bool locked_{};
	bool fallback_{};
	bool live_discontinuity_{true};
	bool live_contaminated_{};
	bool live_classifier_overflow_{};
	bool interval_discontinuity_{true};
	bool interval_contaminated_{};
	bool interval_classifier_overflow_{};
	bool ready_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_FLICKER_ENGINE_HPP
