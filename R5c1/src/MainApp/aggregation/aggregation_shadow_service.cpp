#include "aggregation_shadow_service.hpp"

namespace msap1::aggregation {

AggregationShadowService::AggregationShadowService(
	AggregationTransport &transport, AggregationFrameRing &ring,
	const AggregationFrameDecoder &decoder, AggregationHealth &health) noexcept
	: transport_(transport), ring_(ring), decoder_(decoder), health_(health)
{
}

bool AggregationShadowService::initialize(TaskHandle_t input_task,
	TaskHandle_t validator_task) noexcept
{
	validator_task_ = validator_task;
	health_.set_transport_available(transport_.hardware_available());
	const bool initialized = transport_.initialize(input_task);
	health_.set_transport_initialized(initialized);
	return initialized;
}

void AggregationShadowService::record_transport_errors() noexcept
{
	const auto errors = transport_.take_interrupt_errors();
	if (errors != 0U)
		health_.record_fifo_error(errors);
}

[[noreturn]] void AggregationShadowService::run_input() noexcept
{
	AggregationFrame frame{};
	for (;;) {
		(void)transport_.wait_for_frame(pdMS_TO_TICKS(100U));
		record_transport_errors();

		while (transport_.frame_available()) {
			switch (transport_.read(frame)) {
			case TransportReadResult::frame:
				health_.record_received();
				if (!ring_.try_push(frame)) {
					health_.record_ring_overflow();
					break;
				}
				if (validator_task_ != nullptr)
					xTaskNotifyGive(validator_task_);
				break;
			case TransportReadResult::malformed_frame:
				health_.record_length_error(
					transport_.last_frame_length());
				break;
			case TransportReadResult::hardware_error:
				{
					const auto status = transport_.take_interrupt_errors();
					// Preserve transport failures without coupling this service
					// to a particular FIFO driver's interrupt representation.
					health_.record_fifo_error(
						status == 0U ? 0x80000000U : status);
				}
				break;
			case TransportReadResult::no_frame:
				break;
			}
		}
	}
}

[[noreturn]] void AggregationShadowService::run_validator() noexcept
{
	AggregationFrame frame{};
	AggregationInputView input{};
	for (;;) {
		(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		while (ring_.try_pop(frame)) {
			const auto error = decoder_.decode(frame, input);
			if (error != FrameValidationError::none) {
				health_.record_invalid(error);
				continue;
			}
			health_.record_sequence(input.sequence);
			health_.record_valid(input.sequence);
		}
	}
}

} // namespace msap1::aggregation
