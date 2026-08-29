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

/** R5C1 percentile, Pst, and Plt authority for the PL flickermeter. */
class FlickerEngine final {
public:
	FlickerEngine(AggregationRecordSink &sink,
		AggregationHealth &health) noexcept;

	[[nodiscard]] bool configure(
		const msap1_m18_config_payload &configuration) noexcept;
	bool initialize() noexcept;
	void process(const FlickerInputView &input) noexcept;
	void note_transport_discontinuity() noexcept;

private:
	static constexpr std::size_t phases = FlickerProtocol::phases;
	static constexpr std::size_t bins = FlickerProtocol::classifier_bins;
	static constexpr std::size_t plt_periods = 12U;
	static constexpr std::size_t configuration_words =
		sizeof(msap1_m18_config_payload) / sizeof(std::uint32_t);

	[[nodiscard]] bool load_staged(
		msap1_m18_config_payload &configuration) const noexcept;
	[[nodiscard]] bool apply_matching_configuration(
		const FlickerInputView &input) noexcept;
	void process_live(const FlickerInputView &input) noexcept;
	void process_histogram(const FlickerInputView &input) noexcept;
	void clear_histogram() noexcept;
	void abandon_histogram() noexcept;
	void reset_plt() noexcept;
	void emit(std::uint8_t kind, const FlickerInputView &input,
		std::uint8_t valid_mask,
		const std::array<std::uint32_t, phases> &pst,
		const std::array<std::uint32_t, phases> &plt,
		std::uint64_t first_sample,
		std::uint32_t interval_seconds) noexcept;
	void fail() noexcept;

	[[nodiscard]] static std::uint32_t percentile_q16(
		const std::array<std::uint32_t, bins> &histogram,
		std::uint32_t total, std::uint32_t exceedance_tenths) noexcept;
	[[nodiscard]] static std::uint32_t pst_q16(
		const std::array<std::uint32_t, bins> &histogram,
		std::uint32_t total) noexcept;
	[[nodiscard]] static std::uint32_t plt_q16(
		const std::array<std::uint32_t, plt_periods> &pst) noexcept;

	AggregationRecordSink &sink_;
	AggregationHealth &health_;
	alignas(4) std::array<std::uint32_t, configuration_words> staged_words_{};
	std::uint32_t staged_revision_{};
	msap1_m18_config_payload candidate_configuration_{};
	msap1_m18_config_payload active_configuration_{};
	std::array<std::array<std::uint32_t, bins>, phases> histogram_{};
	std::array<std::array<std::uint32_t, plt_periods>, phases> rolling_pst_{};
	std::array<std::uint64_t, plt_periods> rolling_first_sample_{};
	std::array<std::uint32_t, phases> interval_valid_count_{};
	std::array<std::uint32_t, phases> interval_peak_{};
	std::uint64_t interval_first_sample_{};
	std::uint64_t interval_last_sample_{};
	std::uint32_t interval_status_{};
	std::uint32_t interval_generation_{};
	std::uint32_t interval_sample_rate_{};
	std::uint16_t interval_lamp_voltage_{};
	std::uint8_t interval_nominal_hz_{};
	std::uint8_t interval_phase_mask_{};
	std::uint16_t expected_histogram_base_{};
	std::uint32_t output_sequence_{};
	std::uint32_t last_input_sequence_{};
	std::size_t rolling_count_{};
	std::size_t rolling_position_{};
	bool have_active_configuration_{};
	bool assembling_histogram_{};
	bool have_input_sequence_{};
	bool external_discontinuity_{};
	bool first_after_discontinuity_{true};
	bool ready_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_FLICKER_ENGINE_HPP
