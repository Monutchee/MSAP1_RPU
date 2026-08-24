#ifndef MSAP1_R5C1_AGGREGATION_SHADOW_SERVICE_HPP
#define MSAP1_R5C1_AGGREGATION_SHADOW_SERVICE_HPP

#include "aggregation_frame_decoder.hpp"
#include "aggregation_frame_ring.hpp"
#include "aggregation_health.hpp"
#include "aggregation_transport.hpp"

#include "FreeRTOS.h"
#include "task.h"

namespace msap1::aggregation {

/**
 * Observational PL-to-R5C1 shadow pipeline.
 *
 * The high-priority input task owns the FIFO and copies only complete frames
 * into the static ring.  The low-priority validator owns parsing, CRC, and
 * sequence checks.  No path can stall PL or alter the authoritative HLS
 * aggregation output during this migration phase.
 */
class AggregationShadowService final {
public:
	AggregationShadowService(AggregationTransport &transport,
		AggregationFrameRing &ring, const AggregationFrameDecoder &decoder,
		AggregationHealth &health) noexcept;

	bool initialize(TaskHandle_t input_task,
		TaskHandle_t validator_task) noexcept;
	[[noreturn]] void run_input() noexcept;
	[[noreturn]] void run_validator() noexcept;

private:
	void record_transport_errors() noexcept;

	AggregationTransport &transport_;
	AggregationFrameRing &ring_;
	const AggregationFrameDecoder &decoder_;
	AggregationHealth &health_;
	TaskHandle_t validator_task_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_SHADOW_SERVICE_HPP
