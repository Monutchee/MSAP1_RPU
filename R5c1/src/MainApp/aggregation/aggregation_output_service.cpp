#include "aggregation_output_service.hpp"

namespace msap1::aggregation {

AggregationOutputService::AggregationOutputService(
	AggregationTransport &transport, AggregationRecordRing &ring,
	AggregationHealth &health) noexcept
	: transport_(transport), ring_(ring), health_(health)
{
}

void AggregationOutputService::initialize() noexcept
{
	health_.set_output_ready(transport_.output_available());
	health_.set_output_active(false);
}

bool AggregationOutputService::try_enqueue(
	const AggregationMeterRecord &record) noexcept
{
	if (!ring_.try_push(record)) {
		health_.record_output_drop();
		return false;
	}
	health_.record_output_queued();
	return true;
}

bool AggregationOutputService::publish(
	const AggregationMeterRecord &record) noexcept
{
	return try_enqueue(record);
}

[[noreturn]] void AggregationOutputService::run() noexcept
{
	AggregationMeterRecord record{};
	bool output_fault_reported = false;
	for (;;) {
		if (!ring_.try_pop(record)) {
			vTaskDelay(pdMS_TO_TICKS(1U));
			continue;
		}

		for (;;) {
			const auto vacancy = transport_.output_vacancy_words();
			switch (transport_.write(record)) {
			case TransportWriteResult::written:
				health_.set_output_ready(true);
				health_.set_output_active(true);
				health_.record_output_emitted(record.sequence, vacancy);
				output_fault_reported = false;
				break;
			case TransportWriteResult::would_block:
				vTaskDelay(pdMS_TO_TICKS(1U));
				continue;
			case TransportWriteResult::hardware_error:
				// Keep ownership of this record and retry it.  Dropping a record
				// here would create an invisible hole in the authoritative meter
				// stream.  If the fault persists, the producer ring eventually
				// fills and the aggregation engine fails closed.
				health_.set_output_ready(false);
				health_.set_output_active(false);
				if (!output_fault_reported) {
					health_.record_output_error(vacancy);
					output_fault_reported = true;
				}
				vTaskDelay(pdMS_TO_TICKS(1U));
				continue;
			}
			break;
		}
	}
}

} // namespace msap1::aggregation
