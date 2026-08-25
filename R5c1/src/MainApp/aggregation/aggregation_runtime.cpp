#include "aggregation_runtime.hpp"

namespace msap1::aggregation {

namespace {

constexpr configSTACK_DEPTH_TYPE worker_stack_depth = 2048U;
constexpr UBaseType_t input_priority = 3U;
constexpr UBaseType_t output_priority = 2U;
constexpr UBaseType_t validator_priority = 1U;

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
	output_.initialize();
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
