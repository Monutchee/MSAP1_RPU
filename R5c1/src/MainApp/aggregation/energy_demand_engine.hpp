#ifndef MSAP1_R5C1_ENERGY_DEMAND_ENGINE_HPP
#define MSAP1_R5C1_ENERGY_DEMAND_ENGINE_HPP

#include "aggregation_meter_record.hpp"
#include "aggregation_record_sink.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/**
 * R5C1-owned volatile M17 energy and demand state.
 *
 * The containing R5AggregationEngine is a namespace-scope object, so every
 * byte below lives in .bss/.data rather than a FreeRTOS worker stack.  The
 * class is deliberately non-copyable: reset is always performed in place.
 */
class EnergyDemandEngine final {
public:
	explicit EnergyDemandEngine(std::uint64_t session_id = 1U) noexcept;
	EnergyDemandEngine(const EnergyDemandEngine &) = delete;
	EnergyDemandEngine &operator=(const EnergyDemandEngine &) = delete;
	EnergyDemandEngine(EnergyDemandEngine &&) = delete;
	EnergyDemandEngine &operator=(EnergyDemandEngine &&) = delete;

	void initialize(std::uint64_t session_id) noexcept;

	/** Observe one completed R5 aggregation record and append any M17 output. */
	[[nodiscard]] bool observe(const AggregationMeterRecord &record,
		AggregationRecordSink &sink, bool emit) noexcept;

	[[nodiscard]] std::uint64_t session_id() const noexcept
	{
		return session_id_;
	}

private:
	static constexpr std::size_t phase_total_count = 4U;
	static constexpr std::size_t quadrant_count = 4U;

	struct FractionalCounter {
		std::uint64_t value{};
		std::uint64_t remainder{};
	};

	struct FamilyIdentity {
		std::uint32_t sequence{};
		std::uint32_t generation{};
		std::uint32_t sample_rate_hz{};
		std::uint32_t sample_count{};
		std::uint8_t valid_mask{};
		std::uint32_t status{};
		std::uint64_t first_sample{};
		std::uint64_t last_sample{};
	};

	void clear_state() noexcept;
	void clear_basic_pending() noexcept;
	void clear_demand_pending() noexcept;
	void remember_basic_source() noexcept;
	[[nodiscard]] bool reject_duplicate_or_stale_basic() noexcept;
	void note_basic_gap() noexcept;
	void begin_basic(const AggregationMeterRecord &record) noexcept;
	void accept_basic_power(const AggregationMeterRecord &record) noexcept;
	void accept_basic_phasor(const AggregationMeterRecord &record) noexcept;
	[[nodiscard]] bool finish_basic(const AggregationMeterRecord &record,
		AggregationRecordSink &sink, bool emit) noexcept;
	void begin_demand(const AggregationMeterRecord &record) noexcept;
	void accept_demand_power(const AggregationMeterRecord &record) noexcept;
	void accept_demand_phasor(const AggregationMeterRecord &record) noexcept;
	[[nodiscard]] bool finish_demand(const AggregationMeterRecord &record,
		AggregationRecordSink &sink, bool emit) noexcept;

	[[nodiscard]] static bool same_family(const AggregationMeterRecord &record,
		const FamilyIdentity &identity) noexcept;
	[[nodiscard]] static std::uint64_t read_unsigned64(
		const AggregationMeterRecord &record, std::size_t low_word) noexcept;
	[[nodiscard]] static std::int64_t read_signed64(
		const AggregationMeterRecord &record, std::size_t low_word) noexcept;
	[[nodiscard]] static std::uint8_t phase_total_valid_mask(
		std::uint8_t channel_mask) noexcept;
	[[nodiscard]] static bool supported_sample_rate(
		std::uint32_t sample_rate_hz) noexcept;
	[[nodiscard]] static std::uint64_t magnitude(std::int64_t value) noexcept;

	void integrate(FractionalCounter &counter, std::uint64_t pico_units,
		std::uint32_t samples, std::uint32_t sample_rate_hz) noexcept;
	void write_common(AggregationMeterRecord &output,
		const FamilyIdentity &identity, std::uint32_t format,
		std::uint32_t status) noexcept;
	void write_energy_metadata(AggregationMeterRecord &output) noexcept;
	void write_counter(AggregationMeterRecord &output, std::size_t low_word,
		std::uint64_t value) noexcept;
	[[nodiscard]] bool emit_energy(AggregationRecordSink &sink,
		bool emit) noexcept;
	[[nodiscard]] bool emit_demand(AggregationRecordSink &sink,
		bool emit) noexcept;

	std::uint64_t session_id_{1U};
	std::array<FractionalCounter, phase_total_count> active_import_{};
	std::array<FractionalCounter, phase_total_count> active_export_{};
	std::array<FractionalCounter, phase_total_count> apparent_{};
	std::array<std::array<FractionalCounter, phase_total_count>,
		quadrant_count> reactive_quadrants_{};
	std::uint64_t accepted_samples_{};
	std::uint64_t skipped_samples_{};
	std::uint32_t accepted_blocks_{};
	std::uint32_t skipped_blocks_{};
	bool energy_saturated_{};
	bool energy_incomplete_{};
	bool energy_discontinuity_{};
	// Startup input is priming until one complete, fully valid Basic family
	// establishes the session's first authoritative integration window.
	bool energy_started_{};
	FamilyIdentity last_basic_identity_{};
	bool have_last_basic_identity_{};

	FamilyIdentity basic_identity_{};
	std::array<std::int64_t, phase_total_count> basic_active_power_{};
	std::array<std::uint64_t, phase_total_count> basic_apparent_power_{};
	std::array<std::int64_t, phase_total_count> basic_reactive_power_{};
	std::uint32_t basic_power_status_{};
	std::uint32_t basic_phasor_status_{};
	std::uint8_t basic_summary_valid_{};
	std::uint8_t basic_quadrant_valid_{};
	bool basic_seen_{};
	bool basic_power_seen_{};
	bool basic_phasor_seen_{};

	FamilyIdentity demand_identity_{};
	std::array<std::int64_t, phase_total_count> demand_current_{};
	std::array<std::uint64_t, phase_total_count> demand_import_peak_{};
	std::array<std::uint64_t, phase_total_count> demand_export_peak_{};
	std::array<std::uint64_t, phase_total_count> demand_import_anchor_{};
	std::array<std::uint64_t, phase_total_count> demand_export_anchor_{};
	std::uint64_t demand_target_sample_{};
	std::uint32_t demand_source_interval_count_{};
	std::uint32_t demand_source_status_{};
	std::uint8_t demand_valid_{};
	bool demand_seen_{};
	bool demand_power_seen_{};
	bool demand_phasor_seen_{};
	bool demand_saturated_{};
	bool demand_incomplete_{};

	// Reused output images keep 3 x 260-byte records off the worker stack.
	AggregationMeterRecord energy_summary_record_{};
	AggregationMeterRecord energy_quadrant_record_{};
	AggregationMeterRecord demand_record_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_ENERGY_DEMAND_ENGINE_HPP
