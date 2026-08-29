#ifndef MSAP1_R5C1_PQ_EVENT_LIFECYCLE_ENGINE_HPP
#define MSAP1_R5C1_PQ_EVENT_LIFECYCLE_ENGINE_HPP

#include "aggregation_health.hpp"
#include "aggregation_record_sink.hpp"
#include "m18_protocol.hpp"

#include "m18_configuration.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/** R5C1 authority for all M18 power-quality event lifecycles. */
class PqEventLifecycleEngine final {
public:
	PqEventLifecycleEngine(AggregationRecordSink &sink,
		AggregationHealth &health, std::uint64_t session_id = 1U) noexcept;

	[[nodiscard]] bool configure_session_id(std::uint64_t session_id) noexcept;
	[[nodiscard]] bool configure(
		const msap1_m18_config_payload &configuration) noexcept;
	bool initialize() noexcept;
	void process(const PqEventInputView &input) noexcept;
	void note_transport_discontinuity() noexcept;

private:
	static constexpr std::size_t event_types = MSAP1_M18_EVENT_TYPE_COUNT;
	static constexpr std::size_t phases = 3U;
	static constexpr std::size_t configuration_words =
		sizeof(msap1_m18_config_payload) / sizeof(std::uint32_t);

	struct EventState final {
		bool active{};
		std::uint8_t phase_mask{};
		std::uint64_t id_counter{};
		std::uint64_t first_sample{};
		std::uint64_t trigger_sample{};
		std::uint64_t last_sample{};
		std::uint64_t last_emit_sample{};
		std::array<std::uint32_t, phases> minimum{};
		std::array<std::uint32_t, phases> maximum{};
		std::array<std::uint32_t, phases> current{};
		std::array<std::uint32_t, 4U> settings_fingerprint{};
		msap1_m18_event_profile profile{};
		std::uint32_t profile_generation{};
		std::uint32_t reference{};
		std::uint32_t updates{};
		std::uint32_t discontinuities{};
	};

	struct Evaluation final {
		bool valid{};
		bool start{};
		bool recovered{};
		std::uint8_t phase_mask{};
		std::array<std::uint32_t, phases> values{};
	};

	[[nodiscard]] bool load_staged(
		msap1_m18_config_payload &configuration) const noexcept;
	void apply_matching_configuration(const PqEventInputView &input) noexcept;
	[[nodiscard]] Evaluation evaluate(std::size_t type, std::size_t slot,
		const msap1_m18_event_profile &profile,
		const PqEventInputView &input) const noexcept;
	void process_state(std::size_t type, std::size_t slot,
		const msap1_m18_event_profile &profile,
		const PqEventInputView &input) noexcept;
	void start_state(EventState &state, std::size_t type,
		const msap1_m18_event_profile &profile,
		const Evaluation &evaluation, const PqEventInputView &input) noexcept;
	void abort_all(const PqEventInputView &input) noexcept;
	void emit(EventState &state, std::size_t type, std::uint8_t lifecycle,
		const PqEventInputView &input) noexcept;
	void fail() noexcept;
	[[nodiscard]] static std::array<std::uint32_t, 4U> fingerprint(
		std::size_t type, std::uint32_t generation, std::uint32_t reference,
		const msap1_m18_event_profile &profile) noexcept;

	AggregationRecordSink &sink_;
	AggregationHealth &health_;
	std::uint64_t session_id_;
	alignas(4) std::array<std::uint32_t, configuration_words> staged_words_{};
	std::uint32_t staged_revision_{};
	msap1_m18_config_payload candidate_configuration_{};
	msap1_m18_config_payload active_configuration_{};
	std::array<std::array<EventState, phases>, event_types> state_{};
	std::array<std::uint32_t, phases> previous_voltage_{};
	std::array<std::uint8_t, phases> previous_voltage_valid_{};
	std::uint32_t output_sequence_{};
	std::uint32_t last_input_sequence_{};
	std::uint64_t event_counter_{};
	std::uint32_t external_discontinuity_{};
	bool have_input_sequence_{};
	bool have_active_configuration_{};
	bool first_after_discontinuity_{true};
	bool ready_{};
};

static_assert(sizeof(msap1_m18_config_payload) % sizeof(std::uint32_t) == 0U);

} // namespace msap1::aggregation

#endif
