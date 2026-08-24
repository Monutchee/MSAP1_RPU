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
#include "aggregation/aggregation_output_service.hpp"
#include "aggregation/aggregation_record_ring.hpp"
#include "aggregation/r5_aggregation_engine.hpp"
#include "aggregation/aggregation_shadow_service.hpp"
#include "aggregation/axi_fifo_aggregation_transport.hpp"

#ifndef MNC_R5_AGGREGATION_EMIT_OUTPUT
#define MNC_R5_AGGREGATION_EMIT_OUTPUT 0
#endif

namespace {

constexpr auto aggregation_output_mode =
	MNC_R5_AGGREGATION_EMIT_OUTPUT != 0
		? msap1::aggregation::AggregationOutputMode::emit
		: msap1::aggregation::AggregationOutputMode::shadow;

} // namespace

static msap1::aggregation::AggregationHealth aggregation_health;
static R5c1Service service(msap1::CoreConfig::current(), aggregation_health);
static msap1::aggregation::AxiFifoAggregationTransport aggregation_transport;
static msap1::aggregation::AggregationFrameRing aggregation_ring;
static msap1::aggregation::AggregationFrameDecoder aggregation_decoder;
static msap1::aggregation::AggregationRecordRing aggregation_output_ring;
static msap1::aggregation::AggregationOutputService aggregation_output(
	aggregation_transport, aggregation_output_ring, aggregation_health);
static msap1::aggregation::R5AggregationEngine aggregation_engine(
	aggregation_output, aggregation_health,
	aggregation_output_mode);
static msap1::aggregation::AggregationShadowService aggregation_shadow(
	aggregation_transport, aggregation_ring, aggregation_decoder,
	aggregation_engine, aggregation_health);

static TaskHandle_t comm_task_handle;
static TaskHandle_t aggregation_input_task_handle;
static TaskHandle_t aggregation_output_task_handle;
static TaskHandle_t aggregation_validator_task_handle;

static void comm_task(void *)
{
	service.run();
}

static void aggregation_input_task(void *)
{
	aggregation_shadow.run_input();
}

static void aggregation_validator_task(void *)
{
	aggregation_shadow.run_validator();
}

static void aggregation_output_task(void *)
{
	aggregation_output.run();
}

int main(void)
{
	if (xTaskCreate(comm_task, "RPMSG", 2048, NULL, 2,
			&comm_task_handle) != pdPASS)
		return -1;
	if (xTaskCreate(aggregation_input_task, "AGG_RX", 2048, NULL, 4,
			&aggregation_input_task_handle) != pdPASS)
		return -1;
	if (xTaskCreate(aggregation_output_task, "AGG_TX", 2048, NULL, 3,
			&aggregation_output_task_handle) != pdPASS)
		return -1;
	if (xTaskCreate(aggregation_validator_task, "AGG_VAL", 2048, NULL, 1,
			&aggregation_validator_task_handle) != pdPASS)
		return -1;

	/*
	 * A stale XSA has no FIFO driver/base address.  In that case initialize()
	 * returns false and the two shadow tasks remain harmlessly idle while RPMsg
	 * continues to run.  The refreshed XSA selects the hardware path at compile
	 * time; no runtime protocol compatibility mode exists.
	 */
	(void)aggregation_shadow.initialize(aggregation_input_task_handle,
		aggregation_validator_task_handle);
	aggregation_output.initialize();

	vTaskStartScheduler();

	while (1)
		;

	return 0;
}
