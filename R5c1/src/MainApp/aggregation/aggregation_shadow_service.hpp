#ifndef MSAP1_R5C1_AGGREGATION_SHADOW_SERVICE_HPP
#define MSAP1_R5C1_AGGREGATION_SHADOW_SERVICE_HPP

#include "aggregation_frame_decoder.hpp"
#include "aggregation_frame_ring.hpp"
#include "aggregation_health.hpp"
#include "flicker_engine.hpp"
#include "flicker_frame_decoder.hpp"
#include "harmonic_aggregation_engine.hpp"
#include "harmonic_frame_decoder.hpp"
#include "pq_event_frame_decoder.hpp"
#include "pq_event_lifecycle_engine.hpp"
#include "r5_aggregation_engine.hpp"
#include "aggregation_transport.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cstdint>

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
		R5AggregationEngine &engine,
		const HarmonicFrameDecoder &harmonic_decoder,
		HarmonicAggregationEngine &harmonic_engine,
		const PqEventFrameDecoder &pq_event_decoder,
		PqEventLifecycleEngine &pq_event_engine,
		const FlickerFrameDecoder &flicker_decoder,
		FlickerEngine &flicker_engine,
		AggregationHealth &health) noexcept;

	bool initialize(TaskHandle_t input_task,
		TaskHandle_t validator_task) noexcept;
	[[noreturn]] void run_input() noexcept;
	[[noreturn]] void run_validator() noexcept;

private:
	void record_transport_errors() noexcept;
	void notify_validator() noexcept;

	AggregationTransport &transport_;
	AggregationFrameRing &ring_;
	const AggregationFrameDecoder &decoder_;
	R5AggregationEngine &engine_;
	const HarmonicFrameDecoder &harmonic_decoder_;
	HarmonicAggregationEngine &harmonic_engine_;
	const PqEventFrameDecoder &pq_event_decoder_;
	PqEventLifecycleEngine &pq_event_engine_;
	const FlickerFrameDecoder &flicker_decoder_;
	FlickerEngine &flicker_engine_;
	AggregationHealth &health_;
	/* Largest HRM1 packets cannot live on the 8 KiB worker stacks. */
	AggregationFrame input_frame_{};
	AggregationFrame validator_frame_{};
	TaskHandle_t validator_task_{};
	/*
	 * The input task records the first outstanding notification time.  The
	 * validator atomically consumes it when it runs, producing a direct
	 * measurement of lower-priority scheduling latency without adding a queue
	 * or taking a lock in the high-priority path.
	 */
	std::uint32_t validator_notification_ticks_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_SHADOW_SERVICE_HPP
