#ifndef MSAP1_R5C1_MAINS_SIGNAL_ENGINE_HPP
#define MSAP1_R5C1_MAINS_SIGNAL_ENGINE_HPP

#include "aggregation_health.hpp"
#include "aggregation_record_sink.hpp"
#include "power_quality_protocol.hpp"

#include "power_quality_configuration.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/** Complete R5C1 seven-probe mains-signalling estimator. */
class MainsSignalEngine final {
public:
	MainsSignalEngine(AggregationRecordSink &sink,
		AggregationHealth &health) noexcept;

	[[nodiscard]] bool configure(
		const msap1_m18_config_payload &configuration) noexcept;
	bool initialize() noexcept;
	void process(const VoltageSampleInputView &input) noexcept;
	void note_transport_discontinuity() noexcept;

private:
	static constexpr std::size_t phases = 3U;
	static constexpr std::size_t probes = 7U;
	static constexpr std::size_t configuration_words =
		sizeof(msap1_m18_config_payload) / sizeof(std::uint32_t);

	[[nodiscard]] bool load_staged(
		msap1_m18_config_payload &configuration) const noexcept;
	[[nodiscard]] bool apply_matching_configuration(
		const VoltageSampleInputView &input) noexcept;
	void reset_signal_path(bool contaminated) noexcept;
	void clear_window() noexcept;
	void process_sample(const VoltageSampleInputView &input,
		std::size_t offset, std::uint64_t sample_index) noexcept;
	void complete_window(std::uint64_t last_sample) noexcept;
	void emit(std::uint32_t status, std::uint8_t valid_mask,
		std::uint8_t detected_mask, std::uint32_t measured_millihz,
		const std::array<std::uint32_t, phases> &magnitude_microvolts,
		const std::array<std::uint32_t, phases> &background_microvolts,
		std::uint64_t last_sample) noexcept;
	void fail() noexcept;

	AggregationRecordSink &sink_;
	AggregationHealth &health_;
	alignas(4) std::array<std::uint32_t, configuration_words> staged_words_{};
	std::uint32_t staged_revision_{};
	msap1_m18_config_payload candidate_configuration_{};
	msap1_m18_config_payload active_configuration_{};
	std::array<std::array<std::int64_t, probes>, phases> real_sum_{};
	std::array<std::array<std::int64_t, probes>, phases> imaginary_sum_{};
	std::array<std::uint32_t, probes> probe_phase_{};
	std::array<std::uint32_t, probes> probe_step_{};
	std::uint64_t window_first_sample_{};
	std::uint64_t last_input_sample_{};
	std::uint32_t sample_rate_hz_{};
	std::uint32_t observation_samples_{};
	std::uint32_t window_count_{};
	std::uint32_t output_sequence_{};
	std::uint32_t last_input_sequence_{};
	std::uint8_t window_valid_mask_{};
	bool have_active_configuration_{};
	bool have_input_sequence_{};
	bool have_last_input_sample_{};
	bool external_discontinuity_{};
	bool pending_discontinuity_{true};
	bool window_locked_{true};
	bool window_fallback_{};
	bool arithmetic_overflow_{};
	bool ready_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_MAINS_SIGNAL_ENGINE_HPP
