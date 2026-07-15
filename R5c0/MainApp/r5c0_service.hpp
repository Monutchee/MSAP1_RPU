#ifndef MSAP1_R5C0_SERVICE_HPP
#define MSAP1_R5C0_SERVICE_HPP

/*
 * R5 core 0 control service.
 *
 * R5c0 owns the board's single LED ("UF2_LED", bit 0x01 of AXI_GPIO_0) and runs
 * the heartbeat. It extends the comm-only ControlService with LED behaviour by
 * composing a LedController and overriding the LED hooks.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include "FreeRTOS.h"
#include "xparameters.h"

#include "ad7771.hpp"
#include "control_service.hpp"
#include "led_controller.hpp"

class R5c0Service : public msap1::ControlService {
public:
	explicit R5c0Service(const msap1::CoreConfig &config)
		: msap1::ControlService(config),
		  led_(XPAR_XGPIO_0_BASEADDR, /*led_mask=*/0x01u,
		       /*heartbeat_period_ms=*/200u)
	{
	}

	/* Initialise the LED GPIO. Call before starting the scheduler. */
	bool init_led() { return led_.init(); }

	/* Heartbeat task body. */
	void run_heartbeat() { led_.run_heartbeat(); }

	/* Advertise the full-rate ADC clock to visualization subscribers. */
	void configure_adc_stream(std::uint32_t sample_rate_hz)
	{
		taskENTER_CRITICAL();
		adc_sample_rate_hz_ = sample_rate_hz;
		if (stream_active_ &&
		    (display_rate_hz_ == 0u ||
		     sample_rate_hz % display_rate_hz_ != 0u)) {
			stream_active_ = false;
			++stream_generation_;
		}
		taskEXIT_CRITICAL();
	}

	/*
	 * Publish only the frames selected by the APU's requested display rate.
	 * The original full-rate buffer remains owned by the ADC/DSP data path.
	 */
	void publish_adc_packet(const msap1::adc::SampleFrame *frames,
				std::size_t frame_count,
				std::uint64_t first_frame_index,
				std::uint32_t capture_flags)
	{
		if (frames == nullptr || frame_count == 0u)
			return;

		std::uint32_t destination;
		std::uint32_t sample_rate;
		std::uint32_t display_rate;
		std::uint32_t stride;
		std::uint32_t generation;
		std::uint32_t event_sequence;
		std::uint64_t next_frame;

		taskENTER_CRITICAL();
		if (!stream_active_) {
			taskEXIT_CRITICAL();
			return;
		}
		destination = subscriber_address_;
		sample_rate = adc_sample_rate_hz_;
		display_rate = display_rate_hz_;
		stride = sample_rate / display_rate;
		generation = stream_generation_;
		event_sequence = stream_sequence_;
		next_frame = next_frame_index_;
		taskEXIT_CRITICAL();

		if (next_frame == std::numeric_limits<std::uint64_t>::max())
			next_frame = first_frame_index;
		while (next_frame < first_frame_index)
			next_frame += stride;

		const std::uint64_t packet_end = first_frame_index + frame_count;
		msap1_adc_sample_batch batch = {};
		batch.adc_sample_rate_hz = sample_rate;
		batch.display_rate_hz = display_rate;
		batch.capture_flags = capture_flags;
		batch.channel_count = MSAP1_ADC_CHANNEL_COUNT;

		auto send_batch = [&]() {
			if (batch.frame_count == 0u)
				return;
			msap1_rpu_msg_header event = {};
			event.sequence = event_sequence++;
			const auto payload_size = static_cast<std::uint16_t>(
				MSAP1_ADC_SAMPLE_BATCH_HEADER_SIZE +
				batch.frame_count * sizeof(msap1_adc_sample_frame));
			send_response(&event, destination,
				      MSAP1_RPU_MSG_ADC_SAMPLE_BATCH,
				      MSAP1_RPU_STATUS_OK, &batch, payload_size);
			batch.frame_count = 0u;
		};

		while (next_frame < packet_end) {
			const auto packet_index = static_cast<std::size_t>(
				next_frame - first_frame_index);
			if (batch.frame_count == 0u)
				batch.first_frame_index = next_frame;
			std::memcpy(&batch.frames[batch.frame_count],
				    &frames[packet_index], sizeof(msap1_adc_sample_frame));
			++batch.frame_count;
			next_frame += stride;
			if (batch.frame_count == MSAP1_ADC_MAX_BATCH_FRAMES)
				send_batch();
		}
		send_batch();

		taskENTER_CRITICAL();
		if (stream_active_ && stream_generation_ == generation) {
			next_frame_index_ = next_frame;
			stream_sequence_ = event_sequence;
		}
		taskEXIT_CRITICAL();
	}

protected:
	std::uint32_t on_set_led(std::uint8_t mode) override
	{
		return led_.set_mode(mode);
	}

	void on_fill_status(msap1_rpu_status_payload &status) override
	{
		led_.fill_status(status);
	}

	bool handle_custom(const msap1_rpu_msg_header &request,
			   const void *payload, std::uint16_t payload_len,
			   std::uint32_t src) override
	{
		switch (request.type) {
		case MSAP1_RPU_MSG_ADC_STREAM_START: {
			msap1_adc_stream_request stream_request = {};
			if (payload_len != sizeof(stream_request)) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}
			std::memcpy(&stream_request, payload, sizeof(stream_request));

			std::uint32_t sample_rate;
			taskENTER_CRITICAL();
			sample_rate = adc_sample_rate_hz_;
			taskEXIT_CRITICAL();
			if (sample_rate == 0u) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_ADC_UNAVAILABLE,
					      nullptr, 0);
				return true;
			}
			if (stream_request.display_rate_hz == 0u ||
			    stream_request.display_rate_hz > maximum_display_rate_hz ||
			    sample_rate % stream_request.display_rate_hz != 0u) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}

			if (!send_response(&request, src, MSAP1_RPU_MSG_ACK,
					   MSAP1_RPU_STATUS_OK, nullptr, 0))
				return true;
			taskENTER_CRITICAL();
			subscriber_address_ = src;
			display_rate_hz_ = stream_request.display_rate_hz;
			next_frame_index_ = std::numeric_limits<std::uint64_t>::max();
			stream_sequence_ = 0u;
			stream_active_ = true;
			++stream_generation_;
			taskEXIT_CRITICAL();
			return true;
		}
		case MSAP1_RPU_MSG_ADC_STREAM_STOP:
			if (payload_len != 0u) {
				send_response(&request, src, MSAP1_RPU_MSG_ERROR,
					      MSAP1_RPU_STATUS_BAD_PAYLOAD,
					      nullptr, 0);
				return true;
			}
			taskENTER_CRITICAL();
			stream_active_ = false;
			++stream_generation_;
			taskEXIT_CRITICAL();
			send_response(&request, src, MSAP1_RPU_MSG_ACK,
				      MSAP1_RPU_STATUS_OK, nullptr, 0);
			return true;
		default:
			return false;
		}
	}

	void on_transport_unbind() override
	{
		taskENTER_CRITICAL();
		stream_active_ = false;
		++stream_generation_;
		taskEXIT_CRITICAL();
	}

private:
	static constexpr std::uint32_t maximum_display_rate_hz = 4000u;
	static_assert(sizeof(msap1_adc_sample_frame) ==
		      sizeof(msap1::adc::SampleFrame),
		      "ADC wire and capture frame layouts must match");
	static_assert(offsetof(msap1_adc_sample_batch, frames) ==
		      MSAP1_ADC_SAMPLE_BATCH_HEADER_SIZE,
		      "ADC batch header layout must match the wire ABI");
	static_assert(sizeof(msap1_rpu_msg_header) +
		      sizeof(msap1_adc_sample_batch) <= MSAP1_RPU_MAX_FRAME_SIZE,
		      "ADC RPMsg batch exceeds protocol frame size");

	msap1::LedController led_;
	std::uint32_t adc_sample_rate_hz_ = 0u;
	std::uint32_t display_rate_hz_ = 0u;
	std::uint32_t subscriber_address_ = 0xffffffffu;
	std::uint32_t stream_generation_ = 0u;
	std::uint32_t stream_sequence_ = 0u;
	std::uint64_t next_frame_index_ =
		std::numeric_limits<std::uint64_t>::max();
	bool stream_active_ = false;
};

#endif /* MSAP1_R5C0_SERVICE_HPP */
