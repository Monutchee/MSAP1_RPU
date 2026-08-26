#ifndef MSAP1_R5C1_AGGREGATION_RECORD_SINK_HPP
#define MSAP1_R5C1_AGGREGATION_RECORD_SINK_HPP

#include "aggregation_meter_record.hpp"

namespace msap1::aggregation {

/** Destination for one complete byte-exact meter record. */
class AggregationRecordSink {
public:
	virtual ~AggregationRecordSink() = default;
	[[nodiscard]] virtual bool publish(
		const AggregationMeterRecord &record) noexcept = 0;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_RECORD_SINK_HPP
