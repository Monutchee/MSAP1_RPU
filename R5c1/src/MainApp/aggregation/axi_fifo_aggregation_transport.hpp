#ifndef MSAP1_R5C1_AXI_FIFO_AGGREGATION_TRANSPORT_HPP
#define MSAP1_R5C1_AXI_FIFO_AGGREGATION_TRANSPORT_HPP

#include "aggregation_transport.hpp"

#include "xparameters.h"

#if __has_include("xllfifo.h")
#include "xllfifo.h"
#define MSAP1_HAVE_XLLFIFO_DRIVER 1
#else
#define MSAP1_HAVE_XLLFIFO_DRIVER 0
#endif

/* Prefer the stable BD instance name; accept canonical driver aliases. */
#if defined(XPAR_R5_AGGREGATION_FIFO_BASEADDR)
#define MSAP1_R5_AGGREGATION_FIFO_BASEADDR XPAR_R5_AGGREGATION_FIFO_BASEADDR
#elif defined(XPAR_METERLOGIC_R5_AGGREGATION_FIFO_BASEADDR)
#define MSAP1_R5_AGGREGATION_FIFO_BASEADDR \
	XPAR_METERLOGIC_R5_AGGREGATION_FIFO_BASEADDR
#elif defined(XPAR_XLLFIFO_0_BASEADDR) && defined(XPAR_XLLFIFO_NUM_INSTANCES) && \
	(XPAR_XLLFIFO_NUM_INSTANCES == 1)
#define MSAP1_R5_AGGREGATION_FIFO_BASEADDR XPAR_XLLFIFO_0_BASEADDR
#endif

/* Interrupt names vary slightly between Vivado hierarchy and SDT output. */
#if defined(XPAR_FABRIC_R5_AGGREGATION_FIFO_INTERRUPT_INTR)
#define MSAP1_R5_AGGREGATION_FIFO_INTERRUPT_ID \
	XPAR_FABRIC_R5_AGGREGATION_FIFO_INTERRUPT_INTR
#elif defined(XPAR_FABRIC_METERLOGIC_R5_AGGREGATION_FIFO_INTERRUPT_INTR)
#define MSAP1_R5_AGGREGATION_FIFO_INTERRUPT_ID \
	XPAR_FABRIC_METERLOGIC_R5_AGGREGATION_FIFO_INTERRUPT_INTR
#elif defined(XPAR_FABRIC_METERLOGIC_MTR_BUFFER_R5_AGGREGATION_FIFO_INTR)
#define MSAP1_R5_AGGREGATION_FIFO_INTERRUPT_ID \
	XPAR_FABRIC_METERLOGIC_MTR_BUFFER_R5_AGGREGATION_FIFO_INTR
#elif defined(XPAR_FABRIC_XLLFIFO_0_INTR)
#define MSAP1_R5_AGGREGATION_FIFO_INTERRUPT_ID \
	XPAR_FABRIC_XLLFIFO_0_INTR
#elif defined(XPAR_FABRIC_AXI_FIFO_MM_S_0_INTERRUPT_INTR)
#define MSAP1_R5_AGGREGATION_FIFO_INTERRUPT_ID \
	XPAR_FABRIC_AXI_FIFO_MM_S_0_INTERRUPT_INTR
#endif

#if MSAP1_HAVE_XLLFIFO_DRIVER && \
	defined(MSAP1_R5_AGGREGATION_FIFO_BASEADDR)
#define MSAP1_HAVE_R5_AGGREGATION_FIFO 1
#else
#define MSAP1_HAVE_R5_AGGREGATION_FIFO 0
#endif

namespace msap1::aggregation {

/**
 * AMD AXI4-Stream FIFO MM-S receive adapter.
 *
 * The current checked-in XSA intentionally builds the unavailable stub.  Once
 * the user adds R5_Aggregation_FIFO to the block design, xparameters.h and the
 * XLlFifo BSP driver select the hardware implementation automatically.
 */
class AxiFifoAggregationTransport final : public AggregationTransport {
public:
	bool initialize(TaskHandle_t input_task) noexcept override;
	[[nodiscard]] bool hardware_available() const noexcept override;
	[[nodiscard]] bool frame_available() const noexcept override;
	bool wait_for_frame(TickType_t timeout) noexcept override;
	TransportReadResult read(AggregationFrame &frame) noexcept override;
	std::uint32_t take_interrupt_errors() noexcept override;
	[[nodiscard]] std::uint32_t last_frame_length() const noexcept override;
	[[nodiscard]] bool output_available() const noexcept override;
	[[nodiscard]] std::uint32_t output_vacancy_words() const noexcept override;
	TransportWriteResult write(
		const AggregationMeterRecord &record) noexcept override;

private:
#if MSAP1_HAVE_R5_AGGREGATION_FIFO
	static void interrupt_handler(void *reference) noexcept;
	void handle_interrupt() noexcept;
	void rearm_receive_interrupt() noexcept;
	void drain(std::uint32_t bytes) noexcept;

	XLlFifo fifo_{};
#endif
	TaskHandle_t input_task_{};
	std::uint32_t interrupt_errors_{};
	std::uint32_t last_frame_length_{};
	std::uint32_t last_tx_vacancy_{};
	bool initialized_{};
	bool interrupt_enabled_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AXI_FIFO_AGGREGATION_TRANSPORT_HPP
