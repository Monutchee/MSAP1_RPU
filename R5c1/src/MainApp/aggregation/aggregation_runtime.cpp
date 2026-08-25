#include "aggregation_runtime.hpp"

namespace msap1::aggregation {

namespace {

constexpr configSTACK_DEPTH_TYPE worker_stack_depth = 2048U;

/*
 * The pipeline uses downstream-first priority.  The output owner is highest:
 * when a completed record is queued it preempts arithmetic long enough to move
 * that record into the AXI FIFO.  The validator is next so it can drain the
 * 64-frame input ring before the receiver accepts another burst.  The receiver
 * runs whenever both downstream stages are blocked.
 *
 * This ordering is required even though completed intervals are infrequent:
 * each Basic boundary emits a four-record family and may also emit live
 * previews.  With output below an always-runnable validator, the 64-record
 * output ring filled and the authoritative engine correctly failed closed.
 * RPMsg remains priority 4 and retains control/health responsiveness.
 */
constexpr UBaseType_t output_priority = 3U;
constexpr UBaseType_t validator_priority = 2U;
constexpr UBaseType_t input_priority = 1U;

} // namespace

AggregationRuntime::AggregationRuntime(AggregationShadowService &shadow,
	AggregationOutputService &output, AggregationHealth &health) noexcept
	: shadow_(shadow), output_(output), health_(health)
{
}

bool AggregationRuntime::start() noexcept
{
	if (started_)
		return true;

	/*
	 * This method runs in the higher-priority RPMsg task.  The new workers
	 * cannot execute until this transaction has either completed or deleted
	 * every partially created task.
	 */
	if (xTaskCreate(input_task_entry, "AGG_RX", worker_stack_depth, this,
			input_priority, &input_task_) != pdPASS ||
		xTaskCreate(output_task_entry, "AGG_TX", worker_stack_depth, this,
			output_priority, &output_task_) != pdPASS ||
		xTaskCreate(validator_task_entry, "AGG_VAL", worker_stack_depth, this,
			validator_priority, &validator_task_) != pdPASS) {
		discard_partial_start();
		health_.set_transport_initialized(false);
		health_.set_engine_ready(false);
		health_.set_output_ready(false);
		health_.set_output_active(false);
		health_.set_authoritative(false);
		return false;
	}

	started_ = true;
	xTaskNotifyGive(input_task_);
	return true;
}

void AggregationRuntime::discard_partial_start() noexcept
{
	if (validator_task_ != nullptr) {
		vTaskDelete(validator_task_);
		validator_task_ = nullptr;
	}
	if (output_task_ != nullptr) {
		vTaskDelete(output_task_);
		output_task_ = nullptr;
	}
	if (input_task_ != nullptr) {
		vTaskDelete(input_task_);
		input_task_ = nullptr;
	}
}

void AggregationRuntime::input_task_entry(void *context) noexcept
{
	static_cast<AggregationRuntime *>(context)->run_input();
}

void AggregationRuntime::output_task_entry(void *context) noexcept
{
	static_cast<AggregationRuntime *>(context)->run_output();
}

void AggregationRuntime::validator_task_entry(void *context) noexcept
{
	static_cast<AggregationRuntime *>(context)->run_validator();
}

[[noreturn]] void AggregationRuntime::run_input() noexcept
{
	(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	const bool initialized = shadow_.initialize(input_task_, validator_task_);
	output_.initialize(output_task_);
	if (!initialized)
		vTaskSuspend(nullptr);

	xTaskNotifyGive(output_task_);
	shadow_.run_input();
}

[[noreturn]] void AggregationRuntime::run_output() noexcept
{
	(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	output_.run();
}

[[noreturn]] void AggregationRuntime::run_validator() noexcept
{
	shadow_.run_validator();
}

} // namespace msap1::aggregation
