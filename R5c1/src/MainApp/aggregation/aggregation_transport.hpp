#ifndef MSAP1_R5C1_AGGREGATION_TRANSPORT_HPP
#define MSAP1_R5C1_AGGREGATION_TRANSPORT_HPP

#include "aggregation_protocol.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cstdint>

namespace msap1::aggregation {

enum class TransportReadResult : std::uint8_t {
	no_frame,
	frame,
	malformed_frame,
	hardware_error,
};

/** Hardware-independent source of complete aggregation frames. */
class AggregationTransport {
public:
	virtual ~AggregationTransport() = default;

	virtual bool initialize(TaskHandle_t input_task) noexcept = 0;
	[[nodiscard]] virtual bool hardware_available() const noexcept = 0;
	[[nodiscard]] virtual bool frame_available() const noexcept = 0;
	virtual bool wait_for_frame(TickType_t timeout) noexcept = 0;
	virtual TransportReadResult read(AggregationFrame &frame) noexcept = 0;
	virtual std::uint32_t take_interrupt_errors() noexcept = 0;
	[[nodiscard]] virtual std::uint32_t last_frame_length() const noexcept = 0;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_TRANSPORT_HPP
