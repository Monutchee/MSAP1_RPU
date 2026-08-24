#ifndef MSAP1_R5C1_AGGREGATION_OUTPUT_SERVICE_HPP
#define MSAP1_R5C1_AGGREGATION_OUTPUT_SERVICE_HPP

#include "aggregation_health.hpp"
#include "aggregation_record_sink.hpp"
#include "aggregation_record_ring.hpp"
#include "aggregation_transport.hpp"

#include "FreeRTOS.h"
#include "task.h"

namespace msap1::aggregation {

/** Serializes complete 256-byte records onto the FIFO TX stream. */
class AggregationOutputService final : public AggregationRecordSink {
public:
	AggregationOutputService(AggregationTransport &transport,
		AggregationRecordRing &ring, AggregationHealth &health) noexcept;

	void initialize() noexcept;
	[[nodiscard]] bool try_enqueue(const AggregationMeterRecord &record) noexcept;
	[[nodiscard]] bool publish(
		const AggregationMeterRecord &record) noexcept override;
	[[noreturn]] void run() noexcept;

private:
	AggregationTransport &transport_;
	AggregationRecordRing &ring_;
	AggregationHealth &health_;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_OUTPUT_SERVICE_HPP
