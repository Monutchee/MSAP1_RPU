/*
 * MSAP1 RPU firmware - R5 core 1 entry point.
 *
 * This file only composes the long-lived objects and FreeRTOS tasks.  FIFO
 * ownership, buffering, validation, and health are separate classes under
 * MainApp/aggregation; RPMsg remains in msap1::ControlService.
 */

#include "FreeRTOS.h"
#include "task.h"

#include "control_service.hpp"
#include "r5c1_service.hpp"

#include "aggregation/aggregation_frame_decoder.hpp"
#include "aggregation/aggregation_frame_ring.hpp"
#include "aggregation/aggregation_health.hpp"
#include "aggregation/harmonic_aggregation_engine.hpp"
#include "aggregation/harmonic_frame_decoder.hpp"
#include "aggregation/flicker_engine.hpp"
#include "aggregation/flicker_frame_decoder.hpp"
#include "aggregation/pq_event_frame_decoder.hpp"
#include "aggregation/pq_event_lifecycle_engine.hpp"
#include "aggregation/aggregation_output_service.hpp"
#include "aggregation/aggregation_record_ring.hpp"
#include "aggregation/aggregation_runtime.hpp"
#include "aggregation/r5_aggregation_engine.hpp"
#include "aggregation/r5_session_id.hpp"
#include "aggregation/aggregation_shadow_service.hpp"
#include "aggregation/axi_fifo_aggregation_transport.hpp"

#ifndef MNC_R5_AGGREGATION_EMIT_OUTPUT
#define MNC_R5_AGGREGATION_EMIT_OUTPUT 1
#endif

namespace {

constexpr auto aggregation_output_mode =
	MNC_R5_AGGREGATION_EMIT_OUTPUT != 0
		? msap1::aggregation::AggregationOutputMode::emit
		: msap1::aggregation::AggregationOutputMode::shadow;

} // namespace

static msap1::aggregation::AggregationHealth aggregation_health;
static msap1::aggregation::AxiFifoAggregationTransport aggregation_transport;
static msap1::aggregation::AggregationFrameRing aggregation_ring;
static msap1::aggregation::AggregationFrameDecoder aggregation_decoder;
static msap1::aggregation::HarmonicFrameDecoder harmonic_decoder;
static msap1::aggregation::PqEventFrameDecoder pq_event_decoder;
static msap1::aggregation::FlickerFrameDecoder flicker_decoder;
static msap1::aggregation::AggregationRecordRing aggregation_output_ring;
static msap1::aggregation::AggregationOutputService aggregation_output(
	aggregation_transport, aggregation_output_ring, aggregation_health);
static msap1::aggregation::R5AggregationEngine aggregation_engine(
	aggregation_output, aggregation_health,
	aggregation_output_mode);
static msap1::aggregation::HarmonicAggregationEngine harmonic_engine(
	aggregation_output, aggregation_health);
static msap1::aggregation::PqEventLifecycleEngine pq_event_engine(
	aggregation_output, aggregation_health);
static msap1::aggregation::FlickerEngine flicker_engine(
	aggregation_output, aggregation_health);
static msap1::aggregation::AggregationShadowService aggregation_shadow(
	aggregation_transport, aggregation_ring, aggregation_decoder,
	aggregation_engine, harmonic_decoder, harmonic_engine, pq_event_decoder,
	pq_event_engine, flicker_decoder, flicker_engine, aggregation_health);
static msap1::aggregation::AggregationRuntime aggregation_runtime(
	aggregation_shadow, aggregation_output, aggregation_health);
static R5c1Service service(msap1::CoreConfig::current(), aggregation_health,
	aggregation_runtime, aggregation_engine, pq_event_engine, flicker_engine);

static TaskHandle_t comm_task_handle;

static void comm_task(void *)
{
	service.run();
}

static void aggregation_bootstrap_task(void *)
{
	/*
	 * Start FIFO service as soon as the scheduler runs.  Waiting for Linux to
	 * announce an RPMsg endpoint can leave the PL producer filling the FIFO for
	 * an unbounded interval during boot.  start() is idempotent, so the RPMsg
	 * callback remains a safe recovery path if this first attempt cannot create
	 * all workers.
	 */
	(void)aggregation_runtime.start();
	vTaskDelete(nullptr);
}

int main(void)
{
	/* All BSP constructors, including timer setup, have completed by main().
	 * Generating this from another global constructor made its inputs
	 * deterministic and repeated the session across full device reboots. */
	const auto session_id = msap1::aggregation::generate_r5_session_id();
	if (!aggregation_engine.configure_session_id(session_id) ||
		!pq_event_engine.configure_session_id(session_id))
		return -1;

	/* Keep the control plane independent so FIFO failure cannot remove Linux
	 * diagnostics. */
	if (xTaskCreate(comm_task, "RPMSG", 2048, NULL, 4,
			&comm_task_handle) != pdPASS)
		return -1;

	/*
	 * This task outranks RPMsg only for its short, one-shot start transaction.
	 * A creation failure is non-fatal: the endpoint callback retries later and
	 * exposes the failure through the aggregation-health response.
	 */
	(void)xTaskCreate(aggregation_bootstrap_task, "AGG_BOOT", 1024, nullptr,
		5U, nullptr);

	vTaskStartScheduler();

	while (1)
		;

	return 0;
}
