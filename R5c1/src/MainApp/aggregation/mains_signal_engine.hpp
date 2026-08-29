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

/** R5C1 authority for finalized MAINS-SIGNAL-v1 records. */
class MainsSignalEngine final {
public:
	MainsSignalEngine(AggregationRecordSink &sink,
		AggregationHealth &health) noexcept;

	[[nodiscard]] bool configure(
		const msap1_m18_config_payload &configuration) noexcept;
	bool initialize() noexcept;
	void process(const MainsSignalInputView &input) noexcept;
	void note_transport_discontinuity() noexcept;

private:
	static constexpr std::size_t configuration_words =
		sizeof(msap1_m18_config_payload) / sizeof(std::uint32_t);

	[[nodiscard]] bool load_staged(
		msap1_m18_config_payload &configuration) const noexcept;
	[[nodiscard]] bool apply_matching_configuration(
		const MainsSignalInputView &input) noexcept;
	void emit(const MainsSignalInputView &input,
		bool sequence_discontinuity) noexcept;
	void fail() noexcept;

	AggregationRecordSink &sink_;
	AggregationHealth &health_;
	alignas(4) std::array<std::uint32_t, configuration_words> staged_words_{};
	std::uint32_t staged_revision_{};
	msap1_m18_config_payload candidate_configuration_{};
	msap1_m18_config_payload active_configuration_{};
	std::uint32_t output_sequence_{};
	std::uint32_t last_input_sequence_{};
	bool have_active_configuration_{};
	bool have_input_sequence_{};
	bool external_discontinuity_{};
	bool first_after_discontinuity_{true};
	bool ready_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_MAINS_SIGNAL_ENGINE_HPP
