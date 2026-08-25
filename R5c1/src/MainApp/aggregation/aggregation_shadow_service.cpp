#include "aggregation_shadow_service.hpp"

namespace msap1::aggregation {

namespace {

/*
 * The RX task has a higher priority than the validator so it can respond to
 * the FIFO interrupt promptly.  It must not, however, drain an unbounded
 * number of packets: only the lower-priority validator can release software
 * ring slots.  A small batch retains ample input throughput while providing a
 * deterministic scheduling point for validation and aggregation.
 */
constexpr std::size_t maximum_input_batch = 4U;

TickType_t validator_handoff_delay() noexcept
{
	const auto ticks = pdMS_TO_TICKS(1U);
	return ticks == 0U ? 1U : ticks;
}

} // namespace

AggregationShadowService::AggregationShadowService(
	AggregationTransport &transport, AggregationFrameRing &ring,
	const AggregationFrameDecoder &decoder, R5AggregationEngine &engine,
	AggregationHealth &health) noexcept
	: transport_(transport), ring_(ring), decoder_(decoder), engine_(engine),
	  health_(health)
{
}

bool AggregationShadowService::initialize(TaskHandle_t input_task,
	TaskHandle_t validator_task) noexcept
{
	validator_task_ = validator_task;
	health_.set_transport_available(transport_.hardware_available());
	const bool initialized = transport_.initialize(input_task);
	health_.set_transport_initialized(initialized);
	if (!initialized) {
		health_.set_engine_ready(false);
		health_.set_authoritative(false);
		return false;
	}
	return engine_.initialize();
}

void AggregationShadowService::record_transport_errors() noexcept
{
	const auto errors = transport_.take_interrupt_errors();
	if (errors != 0U) {
		health_.record_fifo_error(errors);
		engine_.note_transport_discontinuity();
	}
}

[[noreturn]] void AggregationShadowService::run_input() noexcept
{
	AggregationFrame frame{};
	for (;;) {
		(void)transport_.wait_for_frame(pdMS_TO_TICKS(100U));
		record_transport_errors();

		std::size_t processed = 0U;
		std::size_t queued = 0U;
		while (processed < maximum_input_batch &&
			transport_.frame_available()) {
			/*
			 * Do not remove a complete packet from the hardware FIFO unless
			 * ownership can be transferred to the software ring.  When the ring
			 * is full, leave the packet intact in hardware and let the PL exporter
			 * apply its nonblocking whole-packet discard policy upstream.
			 */
			if (ring_.available_capacity() == 0U)
				break;

			++processed;
			switch (transport_.read(frame)) {
			case TransportReadResult::frame:
				health_.record_received();
				if (!ring_.try_push(frame)) {
					/*
					 * This can only occur if the SPSC capacity observation is
					 * violated.  Retain it as a hard diagnostic rather than
					 * silently losing a frame.
					 */
					health_.record_ring_overflow();
					engine_.note_transport_discontinuity();
					break;
				}
				++queued;
				break;
			case TransportReadResult::malformed_frame:
				health_.record_length_error(
					transport_.last_frame_length());
				engine_.note_transport_discontinuity();
				break;
			case TransportReadResult::hardware_error:
				{
					const auto status = transport_.take_interrupt_errors();
					// Preserve transport failures without coupling this service
					// to a particular FIFO driver's interrupt representation.
					health_.record_fifo_error(
						status == 0U ? 0x80000000U : status);
					engine_.note_transport_discontinuity();
				}
				break;
			case TransportReadResult::no_frame:
				break;
			}
		}

		if ((queued != 0U || ring_.available_capacity() == 0U) &&
			validator_task_ != nullptr)
			xTaskNotifyGive(validator_task_);

		/*
		 * taskYIELD() only selects peers at the same priority.  Block RX for a
		 * real tick after every nonempty batch so the lower-priority validator
		 * can consume the frames and free ring capacity.
		 */
		if (processed != 0U || ring_.available_capacity() == 0U)
			vTaskDelay(validator_handoff_delay());
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
				engine_.note_transport_discontinuity();
				continue;
			}
			health_.record_sequence(input.sequence);
			health_.record_valid(input.sequence);
			engine_.process(input);
		}
	}
}

} // namespace msap1::aggregation
