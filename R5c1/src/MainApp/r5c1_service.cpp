#include "r5c1_service.hpp"

#include "power_quality_configuration.hpp"

#include <cstring>

static_assert(sizeof(msap1_aggregation_health_payload) == 200U,
	"R5C1 aggregation-health wire payload changed unexpectedly");
static_assert(sizeof(msap1_demand_config_payload) == 12U,
	"R5C1 demand-configuration wire payload changed unexpectedly");
static_assert(sizeof(msap1_demand_config_ack_payload) == 16U,
	"R5C1 demand-configuration ACK changed unexpectedly");
static_assert(sizeof(msap1_m18_config_payload) == 316U,
	"R5C1 M18-configuration wire payload changed unexpectedly");

R5c1Service::R5c1Service(const msap1::CoreConfig &config,
	msap1::aggregation::AggregationHealth &health,
	msap1::aggregation::AggregationRuntime &runtime,
	msap1::aggregation::R5AggregationEngine &engine,
	msap1::aggregation::PqEventLifecycleEngine &pq_event_engine,
	msap1::aggregation::FlickerEngine &flicker_engine,
	msap1::aggregation::MainsSignalEngine &mains_signal_engine) noexcept
	: msap1::ControlService(config), health_(health), runtime_(runtime),
	  engine_(engine), pq_event_engine_(pq_event_engine),
	  flicker_engine_(flicker_engine), mains_signal_engine_(mains_signal_engine)
{
}

void R5c1Service::on_endpoint_ready()
{
	/* Aggregation is deliberately non-fatal to the RPMsg control plane. */
	(void)runtime_.start();
}

bool R5c1Service::handle_custom(const msap1_rpu_msg_header &request,
	const void *payload, std::uint16_t payload_len, std::uint32_t src)
{
	if (request.type == MSAP1_RPU_MSG_M18_CONFIG_SET) {
		if (payload_len != sizeof(msap1_m18_config_payload)) {
			(void)send_response(&request, src, MSAP1_RPU_MSG_ERROR,
				MSAP1_RPU_STATUS_BAD_PAYLOAD, nullptr, 0U);
			return true;
		}
		msap1_m18_config_payload configuration{};
		std::memcpy(&configuration, payload, sizeof(configuration));
		if (!msap1::power_quality::valid_configuration(configuration)) {
			(void)send_response(&request, src, MSAP1_RPU_MSG_ERROR,
				MSAP1_RPU_STATUS_BAD_PAYLOAD, nullptr, 0U);
			return true;
		}
		if (!pq_event_engine_.configure(configuration) ||
			!flicker_engine_.configure(configuration) ||
			!mains_signal_engine_.configure(configuration)) {
			(void)send_response(&request, src, MSAP1_RPU_MSG_ERROR,
				MSAP1_RPU_STATUS_BAD_PAYLOAD, nullptr, 0U);
			return true;
		}
		const msap1_m18_config_ack_payload response{
			configuration.generation, 0U};
		(void)send_response(&request, src, MSAP1_RPU_MSG_M18_CONFIG,
			MSAP1_RPU_STATUS_OK, &response, sizeof(response));
		return true;
	}
	if (request.type == MSAP1_RPU_MSG_DEMAND_CONFIG_SET) {
		if (payload_len != sizeof(msap1_demand_config_payload)) {
			(void)send_response(&request, src, MSAP1_RPU_MSG_ERROR,
				MSAP1_RPU_STATUS_BAD_PAYLOAD, nullptr, 0U);
			return true;
		}
		const auto &configuration =
			*static_cast<const msap1_demand_config_payload *>(payload);
		const auto method = configuration.method ==
			MSAP1_DEMAND_METHOD_FIXED_BLOCK
			? msap1::aggregation::DemandMethod::fixed_block
			: msap1::aggregation::DemandMethod::sliding;
		std::uint32_t generation = 0U;
		if (configuration.method > MSAP1_DEMAND_METHOD_SLIDING ||
			!engine_.configure_demand(method, configuration.window_seconds,
				configuration.update_seconds, generation)) {
			(void)send_response(&request, src, MSAP1_RPU_MSG_ERROR,
				MSAP1_RPU_STATUS_BAD_PAYLOAD, nullptr, 0U);
			return true;
		}
		const msap1_demand_config_ack_payload response{
			configuration.method, configuration.window_seconds,
			configuration.update_seconds, generation};
		(void)send_response(&request, src, MSAP1_RPU_MSG_DEMAND_CONFIG,
			MSAP1_RPU_STATUS_OK, &response, sizeof(response));
		return true;
	}
	if (request.type != MSAP1_RPU_MSG_AGGREGATION_HEALTH_GET)
		return false;
	if (payload_len != 0U) {
		(void)send_response(&request, src, MSAP1_RPU_MSG_ERROR,
			MSAP1_RPU_STATUS_BAD_PAYLOAD, nullptr, 0U);
		return true;
	}

	const auto value = health_.snapshot();
	msap1_aggregation_health_payload response{};
	if (value.transport_available)
		response.health_flags |=
			MSAP1_AGGREGATION_HEALTH_TRANSPORT_AVAILABLE;
	if (value.transport_initialized)
		response.health_flags |=
			MSAP1_AGGREGATION_HEALTH_TRANSPORT_INITIALIZED;
	/* Historical counters are diagnostic. The current receive path is healthy
	 * once a valid frame has cleared the latest validation error. */
	if (value.transport_available && value.transport_initialized &&
		value.frames_valid != 0U &&
		value.last_validation_error ==
			msap1::aggregation::FrameValidationError::none)
		response.health_flags |= MSAP1_AGGREGATION_HEALTH_INPUT_HEALTHY;
	if (value.engine_ready)
		response.health_flags |= MSAP1_AGGREGATION_HEALTH_ENGINE_READY;
	if (value.output_ready)
		response.health_flags |= MSAP1_AGGREGATION_HEALTH_OUTPUT_READY;
	if (value.output_active)
		response.health_flags |= MSAP1_AGGREGATION_HEALTH_OUTPUT_ACTIVE;
	if (value.authoritative)
		response.health_flags |= MSAP1_AGGREGATION_HEALTH_AUTHORITATIVE;

	response.frames_received = value.frames_received;
	response.frames_valid = value.frames_valid;
	response.frames_invalid = value.frames_invalid;
	response.crc_errors = value.crc_errors;
	response.format_errors = value.format_errors;
	response.sequence_gaps = value.sequence_gaps;
	response.repeated_frames = value.repeated_frames;
	response.out_of_order_frames = value.out_of_order_frames;
	response.ring_overflows = value.ring_overflows;
	response.software_ring_push_failures =
		value.software_ring_push_failures;
	response.input_records_dropped = value.input_records_dropped;
	response.first_dropped_sequence = value.first_dropped_sequence;
	response.last_dropped_sequence = value.last_dropped_sequence;
	response.fifo_errors = value.fifo_errors;
	response.length_errors = value.length_errors;
	response.records_queued = value.records_queued;
	response.records_emitted = value.records_emitted;
	response.output_errors = value.output_errors;
	response.output_drops = value.output_drops;
	response.basic_completed = value.basic_completed;
	response.aggregate_completed = value.aggregate_completed;
	response.ten_minute_completed = value.ten_minute_completed;
	response.two_hour_completed = value.two_hour_completed;
	response.last_input_sequence = value.last_sequence;
	response.expected_input_sequence = value.expected_sequence;
	response.last_output_sequence = value.last_output_sequence;
	response.last_fifo_error = value.last_fifo_error;
	response.last_frame_length = value.last_frame_length;
	response.last_validation_error =
		static_cast<std::uint32_t>(value.last_validation_error);
	response.last_tx_vacancy = value.last_tx_vacancy;
	response.software_ring_current = value.software_ring_current;
	response.software_ring_high_water = value.software_ring_high_water;
	response.software_ring_capacity = value.software_ring_capacity;
	response.software_ring_pressure =
		static_cast<std::uint32_t>(value.software_ring_pressure);
	response.software_ring_warning_entries =
		value.software_ring_warning_entries;
	response.software_ring_high_entries = value.software_ring_high_entries;
	response.software_ring_critical_entries =
		value.software_ring_critical_entries;
	response.software_ring_full_entries = value.software_ring_full_entries;
	response.hardware_fifo_current_words = value.hardware_fifo_current_words;
	response.hardware_fifo_high_water_words =
		value.hardware_fifo_high_water_words;
	response.hardware_fifo_full_events = value.hardware_fifo_full_events;
	response.input_wake_count = value.input_wake_count;
	response.input_records_processed = value.input_records_processed;
	response.input_max_batch = value.input_max_batch;
	response.input_max_runtime_us = value.input_max_runtime_us;
	response.validator_wake_count = value.validator_wake_count;
	response.validator_records_processed = value.validator_records_processed;
	response.validator_max_runtime_us = value.validator_max_runtime_us;
	response.validator_max_schedule_gap_us =
		value.validator_max_schedule_gap_us;

	(void)send_response(&request, src, MSAP1_RPU_MSG_AGGREGATION_HEALTH,
		MSAP1_RPU_STATUS_OK, &response, sizeof(response));
	return true;
}
