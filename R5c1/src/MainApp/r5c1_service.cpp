#include "r5c1_service.hpp"

R5c1Service::R5c1Service(const msap1::CoreConfig &config,
	msap1::aggregation::AggregationHealth &health,
	msap1::aggregation::AggregationRuntime &runtime) noexcept
	: msap1::ControlService(config), health_(health), runtime_(runtime)
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
	(void)payload;
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

	(void)send_response(&request, src, MSAP1_RPU_MSG_AGGREGATION_HEALTH,
		MSAP1_RPU_STATUS_OK, &response, sizeof(response));
	return true;
}
