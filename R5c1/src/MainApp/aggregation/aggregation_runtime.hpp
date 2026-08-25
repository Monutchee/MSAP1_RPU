#ifndef MSAP1_R5C1_AGGREGATION_RUNTIME_HPP
#define MSAP1_R5C1_AGGREGATION_RUNTIME_HPP

#include "aggregation_health.hpp"
#include "aggregation_output_service.hpp"
#include "aggregation_shadow_service.hpp"

#include "FreeRTOS.h"
#include "task.h"

namespace msap1::aggregation {

/**
 * Owns the FreeRTOS lifecycle of the optional R5C1 aggregation workers.
 *
 * A one-shot bootstrap task starts the runtime independently of RPMsg endpoint
 * discovery.  The RPMsg endpoint callback repeats the idempotent start as a
 * recovery path, so an exhausted FreeRTOS heap or unavailable AXI FIFO still
 * leaves the control endpoint alive for Linux health diagnostics.
 */
class AggregationRuntime final {
public:
	AggregationRuntime(AggregationShadowService &shadow,
		AggregationOutputService &output,
		AggregationHealth &health) noexcept;

	/** Create and release the aggregation workers. Safe to call repeatedly. */
	[[nodiscard]] bool start() noexcept;
	[[nodiscard]] bool started() const noexcept { return started_; }

private:
	static void input_task_entry(void *context) noexcept;
	static void output_task_entry(void *context) noexcept;
	static void validator_task_entry(void *context) noexcept;

	[[noreturn]] void run_input() noexcept;
	[[noreturn]] void run_output() noexcept;
	[[noreturn]] void run_validator() noexcept;
	void discard_partial_start() noexcept;

	AggregationShadowService &shadow_;
	AggregationOutputService &output_;
	AggregationHealth &health_;
	TaskHandle_t input_task_{};
	TaskHandle_t output_task_{};
	TaskHandle_t validator_task_{};
	bool started_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_RUNTIME_HPP
