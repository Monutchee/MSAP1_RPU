#include "aggregation_shadow_service.hpp"
#include "aggregation_scheduler_policy.hpp"

#include "xiltimer.h"

#include <cstdint>

namespace msap1::aggregation {

namespace {

TickType_t validator_handoff_delay() noexcept
{
	const auto ticks = pdMS_TO_TICKS(1U);
	return ticks == 0U ? 1U : ticks;
}

std::uint32_t monotonic_counter_ticks() noexcept
{
	XTime ticks{};
	XTime_GetTime(&ticks);
	return static_cast<std::uint32_t>(ticks);
}

std::uint32_t elapsed_microseconds(
	std::uint32_t start, std::uint32_t finish) noexcept
{
	return scheduler_policy::elapsed_microseconds(start, finish,
		static_cast<std::uint32_t>(COUNTS_PER_SECOND));
}

} // namespace

AggregationShadowService::AggregationShadowService(
	AggregationTransport &transport, AggregationFrameRing &ring,
	const AggregationFrameDecoder &decoder, R5AggregationEngine &engine,
	const HarmonicFrameDecoder &harmonic_decoder,
	HarmonicAggregationEngine &harmonic_engine,
	const PqEventFrameDecoder &pq_event_decoder,
	PqEventLifecycleEngine &pq_event_engine,
	AggregationHealth &health) noexcept
	: transport_(transport), ring_(ring), decoder_(decoder), engine_(engine),
	  harmonic_decoder_(harmonic_decoder), harmonic_engine_(harmonic_engine),
	  pq_event_decoder_(pq_event_decoder), pq_event_engine_(pq_event_engine),
	  health_(health)
{
}

bool AggregationShadowService::initialize(TaskHandle_t input_task,
	TaskHandle_t validator_task) noexcept
{
	validator_task_ = validator_task;
	__atomic_store_n(&validator_notification_ticks_, 0U, __ATOMIC_RELEASE);
	health_.set_transport_available(transport_.hardware_available());
	const bool initialized = transport_.initialize(input_task);
	health_.set_transport_initialized(initialized);
	if (!initialized) {
		health_.set_engine_ready(false);
		health_.set_authoritative(false);
		return false;
	}
	health_.observe_software_ring(static_cast<std::uint32_t>(ring_.size()),
		static_cast<std::uint32_t>(AggregationFrameRing::capacity));
	health_.observe_hardware_fifo(transport_.input_occupancy_words());
	return engine_.initialize() && harmonic_engine_.initialize() &&
		pq_event_engine_.initialize();
}

void AggregationShadowService::record_transport_errors() noexcept
{
	const auto errors = transport_.take_interrupt_errors();
	if (errors != 0U) {
		health_.record_fifo_error(errors);
		engine_.note_transport_discontinuity();
		harmonic_engine_.note_transport_discontinuity();
		pq_event_engine_.note_transport_discontinuity();
	}
	const auto full_events = transport_.take_input_full_events();
	if (full_events != 0U)
		health_.record_hardware_fifo_full_events(full_events);
}

void AggregationShadowService::notify_validator() noexcept
{
	if (validator_task_ == nullptr)
		return;

	auto notification_time = monotonic_counter_ticks();
	/* Zero is reserved for "no notification pending". */
	if (notification_time == 0U)
		notification_time = 1U;
	std::uint32_t expected = 0U;
	(void)__atomic_compare_exchange_n(&validator_notification_ticks_,
		&expected, notification_time, false, __ATOMIC_RELEASE,
		__ATOMIC_RELAXED);
	xTaskNotifyGive(validator_task_);
}

[[noreturn]] void AggregationShadowService::run_input() noexcept
{
	for (;;) {
		(void)transport_.wait_for_frame(pdMS_TO_TICKS(100U));
		const auto activation_start = monotonic_counter_ticks();
		record_transport_errors();
		health_.observe_hardware_fifo(transport_.input_occupancy_words());
		health_.observe_software_ring(
			static_cast<std::uint32_t>(ring_.size()),
			static_cast<std::uint32_t>(AggregationFrameRing::capacity));

		std::size_t processed = 0U;
		std::size_t queued = 0U;
		while (processed < scheduler_policy::maximum_input_batch &&
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
			switch (transport_.read(input_frame_)) {
			case TransportReadResult::frame:
				health_.record_received();
				if (!ring_.try_push(input_frame_)) {
					/*
					 * This can only occur if the SPSC capacity observation is
					 * violated.  Retain it as a hard diagnostic rather than
					 * silently losing a frame.
					 */
					health_.record_ring_overflow();
					engine_.note_transport_discontinuity();
					harmonic_engine_.note_transport_discontinuity();
					pq_event_engine_.note_transport_discontinuity();
					break;
				}
				++queued;
				break;
			case TransportReadResult::malformed_frame:
				health_.record_length_error(
					transport_.last_frame_length());
				engine_.note_transport_discontinuity();
				harmonic_engine_.note_transport_discontinuity();
				pq_event_engine_.note_transport_discontinuity();
				break;
			case TransportReadResult::hardware_error:
				{
					const auto status = transport_.take_interrupt_errors();
					// Preserve transport failures without coupling this service
					// to a particular FIFO driver's interrupt representation.
					health_.record_fifo_error(
						status == 0U ? 0x80000000U : status);
					engine_.note_transport_discontinuity();
					harmonic_engine_.note_transport_discontinuity();
					pq_event_engine_.note_transport_discontinuity();
				}
				break;
			case TransportReadResult::no_frame:
				break;
			}
		}

		health_.observe_hardware_fifo(transport_.input_occupancy_words());
		health_.observe_software_ring(
			static_cast<std::uint32_t>(ring_.size()),
			static_cast<std::uint32_t>(AggregationFrameRing::capacity));
		if (queued != 0U || ring_.available_capacity() == 0U)
			notify_validator();

		health_.record_input_activation(
			static_cast<std::uint32_t>(processed),
			elapsed_microseconds(activation_start,
				monotonic_counter_ticks()));

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
	AggregationInputView input{};
	HarmonicInputView harmonic_input{};
	PqEventInputView pq_event_input{};
	for (;;) {
		(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		const auto activation_start = monotonic_counter_ticks();
		const auto notification_time = __atomic_exchange_n(
			&validator_notification_ticks_, 0U, __ATOMIC_ACQ_REL);
		const auto schedule_gap = notification_time == 0U ? 0U :
			elapsed_microseconds(notification_time, activation_start);
		std::uint32_t processed = 0U;
		while (ring_.try_pop(validator_frame_)) {
			++processed;
			if (validator_frame_.word_count != 0U &&
				validator_frame_.words[0U] == AggregationProtocol::magic) {
				const auto error = decoder_.decode(validator_frame_, input);
				if (error != FrameValidationError::none) {
					health_.record_invalid(error);
					engine_.note_transport_discontinuity();
					continue;
				}
				health_.record_sequence(input.sequence);
				health_.record_valid(input.sequence);
				harmonic_engine_.observe_timing_context(input.context);
				engine_.process(input);
				continue;
			}

			if (validator_frame_.word_count != 0U &&
				validator_frame_.words[0U] == HarmonicProtocol::magic) {
				const auto error = harmonic_decoder_.decode(
					validator_frame_, harmonic_input);
				if (error != FrameValidationError::none) {
					health_.record_invalid(error);
					harmonic_engine_.note_transport_discontinuity();
					continue;
				}
				health_.record_auxiliary_valid();
				harmonic_engine_.process(harmonic_input);
				continue;
			}

			if (validator_frame_.word_count != 0U &&
				validator_frame_.words[0U] == PqEventProtocol::magic) {
				const auto error = pq_event_decoder_.decode(
					validator_frame_, pq_event_input);
				if (error != FrameValidationError::none) {
					health_.record_invalid(error);
					pq_event_engine_.note_transport_discontinuity();
					continue;
				}
				health_.record_auxiliary_valid();
				pq_event_engine_.process(pq_event_input);
				continue;
			}

			health_.record_invalid(FrameValidationError::invalid_magic);
		}
		health_.observe_software_ring(
			static_cast<std::uint32_t>(ring_.size()),
			static_cast<std::uint32_t>(AggregationFrameRing::capacity));
		health_.record_validator_activation(processed,
			elapsed_microseconds(activation_start,
				monotonic_counter_ticks()), schedule_gap);
	}
}

} // namespace msap1::aggregation
