#ifndef MSAP1_R5C1_AGGREGATION_HEALTH_HPP
#define MSAP1_R5C1_AGGREGATION_HEALTH_HPP

#include "aggregation_protocol.hpp"

#include <cstdint>

namespace msap1::aggregation {

/** Edge-triggered software-ring pressure state used by diagnostics. */
enum class RingPressureLevel : std::uint32_t {
	normal = 0U,
	warning = 1U,
	high = 2U,
	critical = 3U,
	full = 4U,
};

[[nodiscard]] RingPressureLevel classify_ring_pressure(
	std::uint32_t used, std::uint32_t capacity) noexcept;

/** Coherent diagnostic copy returned to future RPMsg health reporting. */
struct AggregationHealthSnapshot final {
	bool transport_available{};
	bool transport_initialized{};
	bool engine_ready{};
	bool output_ready{};
	bool output_active{};
	bool authoritative{};
	std::uint32_t frames_received{};
	std::uint32_t frames_valid{};
	std::uint32_t frames_invalid{};
	std::uint32_t crc_errors{};
	std::uint32_t format_errors{};
	std::uint32_t sequence_gaps{};
	std::uint32_t repeated_frames{};
	std::uint32_t out_of_order_frames{};
	std::uint32_t ring_overflows{};
	std::uint32_t software_ring_push_failures{};
	std::uint32_t input_records_dropped{};
	std::uint32_t first_dropped_sequence{};
	std::uint32_t last_dropped_sequence{};
	std::uint32_t fifo_errors{};
	std::uint32_t length_errors{};
	std::uint32_t records_queued{};
	std::uint32_t records_emitted{};
	std::uint32_t output_errors{};
	std::uint32_t output_drops{};
	std::uint32_t basic_completed{};
	std::uint32_t aggregate_completed{};
	std::uint32_t ten_minute_completed{};
	std::uint32_t two_hour_completed{};
	std::uint32_t last_sequence{};
	std::uint32_t expected_sequence{};
	std::uint32_t last_fifo_error{};
	std::uint32_t last_frame_length{};
	std::uint32_t last_output_sequence{};
	std::uint32_t last_tx_vacancy{};
	std::uint32_t software_ring_current{};
	std::uint32_t software_ring_high_water{};
	std::uint32_t software_ring_capacity{};
	RingPressureLevel software_ring_pressure{RingPressureLevel::normal};
	std::uint32_t software_ring_warning_entries{};
	std::uint32_t software_ring_high_entries{};
	std::uint32_t software_ring_critical_entries{};
	std::uint32_t software_ring_full_entries{};
	std::uint32_t hardware_fifo_current_words{};
	std::uint32_t hardware_fifo_high_water_words{};
	std::uint32_t hardware_fifo_full_events{};
	std::uint32_t input_wake_count{};
	std::uint32_t input_records_processed{};
	std::uint32_t input_max_batch{};
	std::uint32_t input_max_runtime_us{};
	std::uint32_t validator_wake_count{};
	std::uint32_t validator_records_processed{};
	std::uint32_t validator_max_runtime_us{};
	std::uint32_t validator_max_schedule_gap_us{};
	FrameValidationError last_validation_error{FrameValidationError::none};
};

/**
 * Saturating telemetry for the authoritative aggregation receive pipeline.
 *
 * The input and validation tasks update different counters.  GCC atomic
 * builtins keep snapshots race-free without heap allocation or libatomic.
 */
class AggregationHealth final {
public:
	void set_transport_available(bool available) noexcept;
	void set_transport_initialized(bool initialized) noexcept;
	void set_engine_ready(bool ready) noexcept;
	void set_output_ready(bool ready) noexcept;
	void set_output_active(bool active) noexcept;
	void set_authoritative(bool authoritative) noexcept;
	void record_received() noexcept;
	void record_valid(std::uint32_t sequence) noexcept;
	void record_invalid(FrameValidationError error) noexcept;
	void record_sequence(std::uint32_t sequence) noexcept;
	void record_ring_overflow() noexcept;
	void record_fifo_error(std::uint32_t status) noexcept;
	void record_length_error(std::uint32_t length) noexcept;
	void record_output_queued() noexcept;
	void record_output_emitted(std::uint32_t sequence,
		std::uint32_t vacancy_words) noexcept;
	void record_output_error(std::uint32_t vacancy_words) noexcept;
	void record_output_drop() noexcept;
	void record_basic_completed() noexcept;
	void record_aggregate_completed() noexcept;
	void record_ten_minute_completed() noexcept;
	void record_two_hour_completed() noexcept;
	void observe_software_ring(std::uint32_t used,
		std::uint32_t capacity) noexcept;
	void observe_hardware_fifo(std::uint32_t occupancy_words) noexcept;
	void record_hardware_fifo_full_events(std::uint32_t count) noexcept;
	void record_input_activation(std::uint32_t records_processed,
		std::uint32_t runtime_us) noexcept;
	void record_validator_activation(std::uint32_t records_processed,
		std::uint32_t runtime_us, std::uint32_t schedule_gap_us) noexcept;

	[[nodiscard]] AggregationHealthSnapshot snapshot() const noexcept;

private:
	static void increment(std::uint32_t &counter,
		std::uint32_t amount = 1U) noexcept;
	static void update_maximum(std::uint32_t &maximum,
		std::uint32_t candidate) noexcept;

	std::uint32_t transport_available_{};
	std::uint32_t transport_initialized_{};
	std::uint32_t engine_ready_{};
	std::uint32_t output_ready_{};
	std::uint32_t output_active_{};
	std::uint32_t authoritative_{};
	std::uint32_t frames_received_{};
	std::uint32_t frames_valid_{};
	std::uint32_t frames_invalid_{};
	std::uint32_t crc_errors_{};
	std::uint32_t format_errors_{};
	std::uint32_t sequence_gaps_{};
	std::uint32_t repeated_frames_{};
	std::uint32_t out_of_order_frames_{};
	std::uint32_t ring_overflows_{};
	std::uint32_t software_ring_push_failures_{};
	std::uint32_t input_records_dropped_{};
	std::uint32_t first_dropped_sequence_{};
	std::uint32_t last_dropped_sequence_{};
	std::uint32_t fifo_errors_{};
	std::uint32_t length_errors_{};
	std::uint32_t records_queued_{};
	std::uint32_t records_emitted_{};
	std::uint32_t output_errors_{};
	std::uint32_t output_drops_{};
	std::uint32_t basic_completed_{};
	std::uint32_t aggregate_completed_{};
	std::uint32_t ten_minute_completed_{};
	std::uint32_t two_hour_completed_{};
	std::uint32_t last_sequence_{};
	std::uint32_t expected_sequence_{};
	std::uint32_t last_fifo_error_{};
	std::uint32_t last_frame_length_{};
	std::uint32_t last_output_sequence_{};
	std::uint32_t last_tx_vacancy_{};
	std::uint32_t last_validation_error_{};
	std::uint32_t have_sequence_{};
	std::uint32_t software_ring_current_{};
	std::uint32_t software_ring_high_water_{};
	std::uint32_t software_ring_capacity_{};
	std::uint32_t software_ring_pressure_{};
	std::uint32_t software_ring_warning_entries_{};
	std::uint32_t software_ring_high_entries_{};
	std::uint32_t software_ring_critical_entries_{};
	std::uint32_t software_ring_full_entries_{};
	std::uint32_t hardware_fifo_current_words_{};
	std::uint32_t hardware_fifo_high_water_words_{};
	std::uint32_t hardware_fifo_full_events_{};
	std::uint32_t input_wake_count_{};
	std::uint32_t input_records_processed_{};
	std::uint32_t input_max_batch_{};
	std::uint32_t input_max_runtime_us_{};
	std::uint32_t validator_wake_count_{};
	std::uint32_t validator_records_processed_{};
	std::uint32_t validator_max_runtime_us_{};
	std::uint32_t validator_max_schedule_gap_us_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_HEALTH_HPP
