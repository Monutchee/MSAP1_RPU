/*
 * MSAP1 RPU firmware - R5 core 0 entry point.
 *
 * This file owns the FreeRTOS tasks and their handles. The rpmsg communication
 * lives in msap1::ControlService (../../common); R5c0Service adds the LED
 * behaviour for this core. R5c0 drives the board's single LED ("UF2_LED") and
 * runs the heartbeat.
 */

#include "FreeRTOS.h"
#include "task.h"

#include "ad7771.hpp"
#include "control_service.hpp"
#include "r5c0_service.hpp"

static msap1::adc::Ad7771 adc;
static R5c0Service service(msap1::CoreConfig::current(), adc);

static constexpr std::size_t adc_packet_frames = 256;
alignas(64) static msap1::adc::SampleFrame
	adc_buffers[2][adc_packet_frames];

static TaskHandle_t comm_task_handle;
static TaskHandle_t led_task_handle;
static TaskHandle_t adc_task_handle;

static void comm_task(void *)
{
	service.run();
}

static void led_task(void *)
{
	service.run_heartbeat();
}

static void adc_task(void *)
{
	auto *completed_buffer = adc_buffers[0];
	auto *next_buffer = adc_buffers[1];
	auto error = adc.enable_capture_interrupt();
	if (error == msap1::adc::Error::None)
		error = adc.start_capture(completed_buffer, adc_packet_frames);
	std::uint32_t completed_packets = 0;
	std::uint32_t capture_flags = 0;

	while (error == msap1::adc::Error::None) {
		error = adc.wait_for_capture();
		if (error != msap1::adc::Error::None)
			break;

		error = adc.finish_capture(completed_buffer);
		if (error != msap1::adc::Error::None)
			break;

		// Re-arm before examining the completed packet. The PL FIFO absorbs
		// samples during the short simple-DMA descriptor gap.
		error = adc.rearm_capture(next_buffer, adc_packet_frames);
		if (error != msap1::adc::Error::None)
			break;

		const auto first_frame_index =
			static_cast<std::uint64_t>(completed_packets) *
			adc_packet_frames;
		service.publish_adc_packet(completed_buffer, adc_packet_frames,
					   first_frame_index, capture_flags);
		++completed_packets;
		if ((completed_packets & 0x7fu) == 0u)
			capture_flags = adc.status().flags;

		auto *old_buffer = completed_buffer;
		completed_buffer = next_buffer;
		next_buffer = old_buffer;
	}

	adc.stop_capture();
	vTaskDelete(nullptr);
}

int main(void)
{
	if (!service.init_led())
		return -1;

	msap1::adc::Configuration adc_configuration;
	adc_configuration.sample_rate = msap1::adc::SampleRate::Sps32000;
	adc_configuration.filter = msap1::adc::Filter::Sinc5;
	adc_configuration.power_mode = msap1::adc::PowerMode::HighResolution;
	adc_configuration.master_clock_hz = 8192000;
	adc_configuration.frames_per_packet = adc_packet_frames;
	const auto adc_error = adc.initialize(adc_configuration);
	if (adc_error == msap1::adc::Error::None) {
		service.configure_adc_stream(msap1::adc::sample_rate_hz(
			adc_configuration.sample_rate));
	}

	if (xTaskCreate(comm_task, "RPMSG", 2048, NULL, 2,
			&comm_task_handle) != pdPASS)
		return -1;

	if (xTaskCreate(led_task, "LED", 1024, NULL, 1,
			&led_task_handle) != pdPASS)
		return -1;

	if (adc_error == msap1::adc::Error::None &&
	    xTaskCreate(adc_task, "AD7771", 2048, NULL, 3,
			&adc_task_handle) != pdPASS)
		return -1;

	vTaskStartScheduler();

	while (1)
		;

	return 0;
}
