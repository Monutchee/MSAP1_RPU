#ifndef MSAP1_R5C1_AGGREGATION_HEALTH_HPP
#define MSAP1_R5C1_AGGREGATION_HEALTH_HPP

#include "aggregation_protocol.hpp"

#include <cstdint>

namespace msap1::aggregation {

/** Coherent diagnostic copy returned to future RPMsg health reporting. */
struct AggregationHealthSnapshot final {
	bool transport_available{};
	bool transport_initialized{};
	std::uint32_t frames_received{};
	std::uint32_t frames_valid{};
	std::uint32_t frames_invalid{};
	std::uint32_t crc_errors{};
	std::uint32_t format_errors{};
	std::uint32_t sequence_gaps{};
	std::uint32_t repeated_frames{};
	std::uint32_t out_of_order_frames{};
	std::uint32_t ring_overflows{};
	std::uint32_t fifo_errors{};
	std::uint32_t length_errors{};
	std::uint32_t last_sequence{};
	std::uint32_t expected_sequence{};
	std::uint32_t last_fifo_error{};
	std::uint32_t last_frame_length{};
	FrameValidationError last_validation_error{FrameValidationError::none};
};

/**
 * Saturating telemetry for the observational shadow receiver.
 *
 * The input and validation tasks update different counters.  GCC atomic
 * builtins keep snapshots race-free without heap allocation or libatomic.
 */
class AggregationHealth final {
public:
	void set_transport_available(bool available) noexcept;
	void set_transport_initialized(bool initialized) noexcept;
	void record_received() noexcept;
	void record_valid(std::uint32_t sequence) noexcept;
	void record_invalid(FrameValidationError error) noexcept;
	void record_sequence(std::uint32_t sequence) noexcept;
	void record_ring_overflow() noexcept;
	void record_fifo_error(std::uint32_t status) noexcept;
	void record_length_error(std::uint32_t length) noexcept;

	[[nodiscard]] AggregationHealthSnapshot snapshot() const noexcept;

private:
	static void increment(std::uint32_t &counter,
		std::uint32_t amount = 1U) noexcept;

	std::uint32_t transport_available_{};
	std::uint32_t transport_initialized_{};
	std::uint32_t frames_received_{};
	std::uint32_t frames_valid_{};
	std::uint32_t frames_invalid_{};
	std::uint32_t crc_errors_{};
	std::uint32_t format_errors_{};
	std::uint32_t sequence_gaps_{};
	std::uint32_t repeated_frames_{};
	std::uint32_t out_of_order_frames_{};
	std::uint32_t ring_overflows_{};
	std::uint32_t fifo_errors_{};
	std::uint32_t length_errors_{};
	std::uint32_t last_sequence_{};
	std::uint32_t expected_sequence_{};
	std::uint32_t last_fifo_error_{};
	std::uint32_t last_frame_length_{};
	std::uint32_t last_validation_error_{};
	std::uint32_t have_sequence_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_HEALTH_HPP
