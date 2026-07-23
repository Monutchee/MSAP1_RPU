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
#include "xparameters.h"

#include "ad7771.hpp"
#include "control_service.hpp"
#include "metering.hpp"
#include "r5c0_service.hpp"

#ifndef XPAR_XSPI_0_BASEADDR
#error "The Vitis platform does not define the AD7771 AXI Quad SPI address"
#endif
#ifndef XPAR_METERCORE_WRAPPER_0_BASEADDR
#error "The Vitis platform does not define the MeterCore address"
#endif

/*
 * SDT describes MeterCore as one node with three reg regions. Vitis emits the
 * canonical xparameters macro for the first region only, so derive the other
 * two from the preserved TopDesign address layout:
 *
 *   capture     0xB0020000
 *   conversion  0xB0040000
 *   processing  0xB0050000
 */
static constexpr std::uintptr_t meter_core_capture_base =
	XPAR_METERCORE_WRAPPER_0_BASEADDR;
static constexpr std::uintptr_t adc_conversion_base =
	meter_core_capture_base + 0x00020000u;
static constexpr std::uintptr_t meter_processing_base =
	meter_core_capture_base + 0x00030000u;

static constexpr msap1::adc::Hardware adc_hardware{
	/*spi_base=*/XPAR_XSPI_0_BASEADDR,
	/*capture_base=*/meter_core_capture_base,
};
static constexpr msap1::meter::Hardware meter_hardware{
	/*conversion_base=*/adc_conversion_base,
	/*processing_base=*/meter_processing_base,
};

static msap1::adc::Ad7771 adc(adc_hardware);
static msap1::meter::MeteringPipeline metering(meter_hardware);
static R5c0Service service(msap1::CoreConfig::current(), adc, metering);

static constexpr std::uint16_t adc_packet_frames = 256;

static TaskHandle_t comm_task_handle;
static TaskHandle_t led_task_handle;

static void comm_task(void *)
{
	service.run();
}

static void led_task(void *)
{
	service.run_heartbeat();
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

	if (xTaskCreate(comm_task, "RPMSG", 2048, NULL, 2,
			&comm_task_handle) != pdPASS)
		return -1;

	if (xTaskCreate(led_task, "LED", 1024, NULL, 1,
			&led_task_handle) != pdPASS)
		return -1;

	// Keep RPMsg and heartbeat available even when ADC bring-up fails. The
	// health command reports initialization state to Linux.
	(void)adc_error;

	vTaskStartScheduler();

	while (1)
		;

	return 0;
}
