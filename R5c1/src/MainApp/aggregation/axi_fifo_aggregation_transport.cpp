#include "axi_fifo_aggregation_transport.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

namespace msap1::aggregation {

bool AxiFifoAggregationTransport::initialize(TaskHandle_t input_task) noexcept
{
	input_task_ = input_task;
#if MSAP1_HAVE_R5_AGGREGATION_FIFO
	XLlFifo_Initialize(&fifo_, MSAP1_R5_AGGREGATION_FIFO_BASEADDR);
	XLlFifo_Reset(&fifo_);
	XLlFifo_IntDisable(&fifo_, XLLF_INT_ALL_MASK);
	XLlFifo_IntClear(&fifo_, XLLF_INT_ALL_MASK);

#if defined(MSAP1_R5_AGGREGATION_FIFO_INTERRUPT_ID)
	if (xPortInstallInterruptHandler(
		static_cast<std::uint16_t>(
			MSAP1_R5_AGGREGATION_FIFO_INTERRUPT_ID),
		&interrupt_handler, this) == pdPASS) {
		XLlFifo_IntEnable(&fifo_, XLLF_INT_RC_MASK | XLLF_INT_ERROR_MASK);
		vPortEnableInterrupt(static_cast<std::uint16_t>(
			MSAP1_R5_AGGREGATION_FIFO_INTERRUPT_ID));
		interrupt_enabled_ = true;
	}
#endif
	initialized_ = true;
	return true;
#else
	(void)input_task;
	initialized_ = false;
	return false;
#endif
}

bool AxiFifoAggregationTransport::hardware_available() const noexcept
{
	return MSAP1_HAVE_R5_AGGREGATION_FIFO != 0;
}

bool AxiFifoAggregationTransport::frame_available() const noexcept
{
#if MSAP1_HAVE_R5_AGGREGATION_FIFO
	return initialized_ && XLlFifo_IsRxEmpty(
		const_cast<XLlFifo *>(&fifo_)) == FALSE;
#else
	return false;
#endif
}

bool AxiFifoAggregationTransport::wait_for_frame(TickType_t timeout) noexcept
{
	if (frame_available())
		return true;
	if (!initialized_) {
		vTaskDelay(timeout == portMAX_DELAY ? pdMS_TO_TICKS(1000U) : timeout);
		return false;
	}
	if (interrupt_enabled_) {
		/*
		 * The ISR masks receive-complete before waking the owner task.  Rearm
		 * only after that task has drained every complete packet.  The second
		 * occupancy check closes the arrival race between the first check and
		 * clearing/re-enabling the interrupt.
		 */
		rearm_receive_interrupt();
		if (frame_available())
			return true;
		(void)ulTaskNotifyTake(pdTRUE, timeout);
	} else {
		vTaskDelay(timeout == portMAX_DELAY ? pdMS_TO_TICKS(1U) : timeout);
	}
	return frame_available();
}

TransportReadResult AxiFifoAggregationTransport::read(
	AggregationFrame &frame) noexcept
{
#if MSAP1_HAVE_R5_AGGREGATION_FIFO
	if (!initialized_ || XLlFifo_IsRxEmpty(&fifo_) != FALSE)
		return TransportReadResult::no_frame;

	const auto bytes = XLlFifo_RxGetLen(&fifo_);
	__atomic_store_n(&last_frame_length_, bytes, __ATOMIC_RELEASE);
	if (bytes != AggregationProtocol::frame_bytes) {
		drain(bytes);
		return TransportReadResult::malformed_frame;
	}

	XLlFifo_Read(&fifo_, frame.words.data(), bytes);
	return TransportReadResult::frame;
#else
	(void)frame;
	return TransportReadResult::no_frame;
#endif
}

std::uint32_t AxiFifoAggregationTransport::take_interrupt_errors() noexcept
{
	return __atomic_exchange_n(&interrupt_errors_, 0U, __ATOMIC_ACQ_REL);
}

std::uint32_t AxiFifoAggregationTransport::last_frame_length() const noexcept
{
	return __atomic_load_n(&last_frame_length_, __ATOMIC_ACQUIRE);
}

#if MSAP1_HAVE_R5_AGGREGATION_FIFO
void AxiFifoAggregationTransport::interrupt_handler(void *reference) noexcept
{
	static_cast<AxiFifoAggregationTransport *>(reference)->handle_interrupt();
}

void AxiFifoAggregationTransport::handle_interrupt() noexcept
{
	const auto pending = XLlFifo_IntPending(&fifo_);
	const auto errors = pending & XLLF_INT_ERROR_MASK;
	if (errors != 0U) {
		(void)__atomic_fetch_or(&interrupt_errors_, errors, __ATOMIC_RELEASE);
		XLlFifo_IntClear(&fifo_, errors);
	}

	/*
	 * RC describes completion on the stream side; it is not a software-read
	 * completion flag.  Mask and acknowledge it here to avoid an interrupt
	 * storm, then let the sole FIFO-owner task drain all complete packets.
	 */
	if ((pending & XLLF_INT_RC_MASK) != 0U) {
		XLlFifo_IntDisable(&fifo_, XLLF_INT_RC_MASK);
		XLlFifo_IntClear(&fifo_, XLLF_INT_RC_MASK);
	}

	const auto other = pending & ~(XLLF_INT_RC_MASK | XLLF_INT_ERROR_MASK);
	if (other != 0U)
		XLlFifo_IntClear(&fifo_, other);

	if ((pending & (XLLF_INT_RC_MASK | XLLF_INT_ERROR_MASK)) != 0U &&
		input_task_ != nullptr) {
		BaseType_t higher_priority_task_woken = pdFALSE;
		vTaskNotifyGiveFromISR(input_task_, &higher_priority_task_woken);
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}

void AxiFifoAggregationTransport::rearm_receive_interrupt() noexcept
{
	XLlFifo_IntClear(&fifo_, XLLF_INT_RC_MASK);
	XLlFifo_IntEnable(&fifo_, XLLF_INT_RC_MASK | XLLF_INT_ERROR_MASK);
}

void AxiFifoAggregationTransport::drain(std::uint32_t bytes) noexcept
{
	std::array<std::uint32_t, 32U> discard{};
	while (bytes != 0U) {
		const auto chunk = std::min<std::uint32_t>(bytes,
			static_cast<std::uint32_t>(discard.size() * sizeof(discard[0])));
		XLlFifo_Read(&fifo_, discard.data(), chunk);
		bytes -= chunk;
	}
}
#endif

} // namespace msap1::aggregation
