#ifndef MSAP1_R5C1_SERVICE_HPP
#define MSAP1_R5C1_SERVICE_HPP

/*
 * R5 core 1 control service.
 *
 * On the KR260 board R5c1 has no LED, so it uses the comm-only ControlService
 * behaviour unchanged: SET_LED is accepted as an ACK no-op and the LED status
 * fields report zero (the base-class defaults). This subclass exists for
 * symmetry with R5c0Service and reports aggregation health. Meter records
 * themselves remain on the AXI FIFO/DMA path and never use RPMsg.
 */

#include "control_service.hpp"
#include "aggregation/aggregation_health.hpp"
#include "aggregation/r5_aggregation_engine.hpp"
#include "aggregation/aggregation_runtime.hpp"
#include "aggregation/pq_event_lifecycle_engine.hpp"

class R5c1Service : public msap1::ControlService {
public:
	R5c1Service(const msap1::CoreConfig &config,
		msap1::aggregation::AggregationHealth &health,
		msap1::aggregation::AggregationRuntime &runtime,
		msap1::aggregation::R5AggregationEngine &engine,
		msap1::aggregation::PqEventLifecycleEngine &pq_event_engine) noexcept;

protected:
	bool handle_custom(const msap1_rpu_msg_header &request,
		const void *payload, std::uint16_t payload_len,
		std::uint32_t src) override;
	void on_endpoint_ready() override;

private:
	msap1::aggregation::AggregationHealth &health_;
	msap1::aggregation::AggregationRuntime &runtime_;
	msap1::aggregation::R5AggregationEngine &engine_;
	msap1::aggregation::PqEventLifecycleEngine &pq_event_engine_;
};

#endif /* MSAP1_R5C1_SERVICE_HPP */
