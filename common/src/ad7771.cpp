#include "ad7771.hpp"

#include "FreeRTOS.h"
#include "sleep.h"
#include "task.h"
#include "xinterrupt_wrap.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xstatus.h"

namespace msap1::adc {
namespace {

constexpr std::uint8_t reg_general_user_config_1 = 0x11;
constexpr std::uint8_t reg_general_user_config_2 = 0x12;
constexpr std::uint8_t reg_general_user_config_3 = 0x13;
constexpr std::uint8_t reg_dout_format = 0x14;
constexpr std::uint8_t reg_status_3 = 0x5f;
constexpr std::uint8_t reg_src_n_msb = 0x60;
constexpr std::uint8_t reg_src_n_lsb = 0x61;
constexpr std::uint8_t reg_src_if_msb = 0x62;
constexpr std::uint8_t reg_src_if_lsb = 0x63;
constexpr std::uint8_t reg_src_update = 0x64;

constexpr std::uint8_t status_init_complete = 1u << 4;
constexpr std::uint8_t config_1_power_mode = 1u << 6;
constexpr std::uint8_t config_2_filter_mode = 1u << 6;
constexpr std::uint8_t config_2_sar_spi_mode = 1u << 5;
constexpr std::uint8_t config_2_spi_sync = 1u << 0;
constexpr std::uint8_t config_3_spi_data_mode = 1u << 4;
constexpr std::uint8_t src_load_update = 1u << 0;

constexpr std::uint32_t capture_version = 0x00;
constexpr std::uint32_t capture_control = 0x04;
constexpr std::uint32_t capture_packet_frames = 0x08;
constexpr std::uint32_t capture_status = 0x0c;
constexpr std::uint32_t capture_frame_count = 0x10;
constexpr std::uint32_t capture_overflow_count = 0x14;
constexpr std::uint32_t capture_header_error_count = 0x18;
constexpr std::uint32_t capture_alert_count = 0x1c;
constexpr std::uint32_t capture_packet_count = 0x20;
constexpr std::uint32_t capture_identifier = 0x28;

constexpr std::uint32_t expected_capture_identifier = 0x41443731u; // "AD71"
constexpr std::uint16_t maximum_dma_frames = 2047;
constexpr std::uintptr_t cache_line_mask = 63u;
constexpr std::uint32_t minimum_high_resolution_mclk_hz = 655000u;
constexpr std::uint32_t maximum_high_resolution_mclk_hz = 8192000u;
constexpr std::uint32_t minimum_low_power_mclk_hz = 1300000u;
constexpr std::uint32_t maximum_low_power_mclk_hz = 4096000u;
constexpr std::uint32_t minimum_decimation = 16u;
constexpr std::uint32_t maximum_sinc3_integer_decimation = 4095u;
constexpr std::uint32_t maximum_sinc5_integer_decimation = 2048u;

std::uint32_t modulator_clock_divisor(const Configuration &configuration)
{
	return configuration.power_mode == PowerMode::HighResolution ? 4u : 8u;
}

std::uint16_t decimation_for(const Configuration &configuration)
{
	const auto rate = sample_rate_hz(configuration.sample_rate);
	const auto denominator = modulator_clock_divisor(configuration) * rate;
	return static_cast<std::uint16_t>(configuration.master_clock_hz /
					  denominator);
}

} // namespace

const char *to_string(Error error)
{
	switch (error) {
	case Error::None: return "none";
	case Error::InvalidConfiguration: return "invalid configuration";
	case Error::CaptureCoreNotFound: return "PL capture core not found";
	case Error::SpiInitialization: return "AXI SPI initialization failed";
	case Error::SpiTransfer: return "AD7771 SPI transfer failed";
	case Error::SpiProtocol: return "AD7771 SPI response header invalid";
	case Error::AdcNotReady: return "AD7771 initialization did not complete";
	case Error::AdcRegisterMismatch: return "AD7771 register readback mismatch";
	case Error::DmaInitialization: return "AXI DMA initialization failed";
	case Error::DmaIsScatterGather: return "AXI DMA is not in simple mode";
	case Error::DmaInterruptInitialization:
		return "AXI DMA interrupt initialization failed";
	case Error::DmaTransfer: return "AXI DMA S2MM transfer failed";
	case Error::CaptureNotInitialized: return "ADC capture is not initialized";
	case Error::CaptureAlreadyActive: return "ADC capture is already active";
	case Error::CaptureBufferInvalid: return "ADC DMA buffer is invalid";
	}
	return "unknown";
}

std::uint32_t sample_rate_hz(SampleRate rate)
{
	return static_cast<std::uint32_t>(rate);
}

Ad7771::Ad7771(Hardware hardware) : hardware_(hardware) {}

std::uint32_t Ad7771::capture_read(std::uint32_t offset) const
{
	return Xil_In32(hardware_.capture_base + offset);
}

void Ad7771::capture_write(std::uint32_t offset, std::uint32_t value)
{
	Xil_Out32(hardware_.capture_base + offset, value);
}

void Ad7771::set_capture_control(std::uint32_t value)
{
	control_shadow_ = value;
	capture_write(capture_control, value);
}

Error Ad7771::initialize_spi()
{
	// Vitis 2025.2 currently flattens the AXI Quad SPI inside AdcSubSystem's
	// block-design container to the HPM aperture base in the generated SDT.
	// Use that generated address only to find the driver's static capabilities,
	// then initialize it with the address decoded by the implemented hardware.
	// This remains valid if a later tool release emits the correct address.
	auto *spi_config = XSpi_LookupConfig(XPAR_XSPI_0_BASEADDR);
	if (spi_config == nullptr ||
	    XSpi_CfgInitialize(&spi_, spi_config, hardware_.spi_base) !=
		XST_SUCCESS)
		return Error::SpiInitialization;
	if (XSpi_SetOptions(&spi_, XSP_MASTER_OPTION |
				      XSP_MANUAL_SSELECT_OPTION) != XST_SUCCESS)
		return Error::SpiInitialization;
	if (XSpi_SetSlaveSelect(&spi_, 0x01u) != XST_SUCCESS)
		return Error::SpiInitialization;
	if (XSpi_Start(&spi_) != XST_SUCCESS)
		return Error::SpiInitialization;
	XSpi_IntrGlobalDisable(&spi_);
	return Error::None;
}

Error Ad7771::initialize_dma()
{
	auto *dma_config = XAxiDma_LookupConfig(hardware_.dma_base);
	if (dma_config == nullptr ||
	    XAxiDma_CfgInitialize(&dma_, dma_config) != XST_SUCCESS)
		return Error::DmaInitialization;
	if (XAxiDma_HasSg(&dma_))
		return Error::DmaIsScatterGather;
	XAxiDma_IntrDisable(&dma_, XAXIDMA_IRQ_ALL_MASK,
			     XAXIDMA_DEVICE_TO_DMA);
	return Error::None;
}

Error Ad7771::write_adc_register(std::uint8_t address, std::uint8_t value)
{
	std::uint8_t transmit[2] = {
		static_cast<std::uint8_t>(address & 0x7fu), value
	};
	std::uint8_t receive[2] = {};
	if (XSpi_Transfer(&spi_, transmit, receive, sizeof(transmit)) !=
	    XST_SUCCESS)
		return Error::SpiTransfer;
	if (receive[0] != 0x20u)
		return Error::SpiProtocol;
	return Error::None;
}

Error Ad7771::read_adc_register(std::uint8_t address, std::uint8_t &value)
{
	std::uint8_t transmit[2] = {
		static_cast<std::uint8_t>(0x80u | (address & 0x7fu)), 0u
	};
	std::uint8_t receive[2] = {};
	if (XSpi_Transfer(&spi_, transmit, receive, sizeof(transmit)) !=
	    XST_SUCCESS)
		return Error::SpiTransfer;
	if (receive[0] != 0x20u)
		return Error::SpiProtocol;
	value = receive[1];
	return Error::None;
}

Error Ad7771::update_adc_register(std::uint8_t address, std::uint8_t mask,
				 std::uint8_t value)
{
	std::uint8_t current = 0;
	auto error = read_adc_register(address, current);
	if (error != Error::None)
		return error;
	const auto updated = static_cast<std::uint8_t>(
		(current & ~mask) | (value & mask));
	error = write_adc_register(address, updated);
	if (error != Error::None)
		return error;
	std::uint8_t readback = 0;
	error = read_adc_register(address, readback);
	if (error != Error::None)
		return error;
	return ((readback & mask) == (updated & mask)) ?
		Error::None : Error::AdcRegisterMismatch;
}

Error Ad7771::configure_sample_rate()
{
	const std::uint16_t decimation = decimation_for(configuration_);
	Error error = write_adc_register(reg_src_n_msb,
		static_cast<std::uint8_t>((decimation >> 8) & 0x0fu));
	if (error != Error::None) return error;
	error = write_adc_register(reg_src_n_lsb,
		static_cast<std::uint8_t>(decimation & 0xffu));
	if (error != Error::None) return error;
	error = write_adc_register(reg_src_if_msb, 0u);
	if (error != Error::None) return error;
	error = write_adc_register(reg_src_if_lsb, 0u);
	if (error != Error::None) return error;

	// Software-load the new SRC value. One microsecond exceeds two periods of
	// the sensor board's 8.192 MHz MCLK.
	error = update_adc_register(reg_src_update, src_load_update,
				    src_load_update);
	if (error != Error::None) return error;
	usleep(1);
	return update_adc_register(reg_src_update, src_load_update, 0u);
}

Error Ad7771::synchronize_adc()
{
	// SYNC_OUT is wired to SYNC_IN on the sensor board. Toggling SPI_SYNC
	// therefore resets the digital filters without changing programmed data.
	Error error = update_adc_register(reg_general_user_config_2,
					  config_2_spi_sync, 0u);
	if (error != Error::None) return error;
	usleep(1);
	return update_adc_register(reg_general_user_config_2,
				   config_2_spi_sync, config_2_spi_sync);
}

Error Ad7771::reset_and_configure_adc()
{
	// Hold RESET low, keep the positive-pulse START input inactive/low, and
	// reset the PL FIFO. Synchronization is performed through SPI_SYNC below.
	set_capture_control(control_fifo_reset);
	usleep(10);
	set_capture_control(control_fifo_reset | control_adc_reset_n);
	usleep(2500);

	std::uint8_t status = 0;
	for (unsigned int attempt = 0; attempt < 20; ++attempt) {
		const auto error = read_adc_register(reg_status_3, status);
		if (error != Error::None)
			return error;
		if ((status & status_init_complete) != 0)
			break;
		usleep(500);
	}
	if ((status & status_init_complete) == 0)
		return Error::AdcNotReady;

	Error error = update_adc_register(reg_general_user_config_1,
		config_1_power_mode,
		configuration_.power_mode == PowerMode::HighResolution ?
			config_1_power_mode : 0u);
	if (error != Error::None) return error;
	error = update_adc_register(reg_general_user_config_2,
		config_2_filter_mode | config_2_sar_spi_mode,
		configuration_.filter == Filter::Sinc5 ?
			config_2_filter_mode : 0u);
	if (error != Error::None) return error;
	error = update_adc_register(reg_general_user_config_3,
		config_3_spi_data_mode, 0u);
	if (error != Error::None) return error;

	// Four DOUT lanes, status headers, DCLK=MCLK. A constant 8.192 MHz DCLK
	// supports every declared ODR and keeps the PL timing contract unchanged.
	error = write_adc_register(reg_dout_format, 0x00u);
	if (error != Error::None) return error;
	error = configure_sample_rate();
	if (error != Error::None) return error;
	return synchronize_adc();
}

Error Ad7771::initialize(const Configuration &configuration)
{
	const auto rate = sample_rate_hz(configuration.sample_rate);
	const auto maximum_mclk =
		configuration.power_mode == PowerMode::HighResolution ?
		maximum_high_resolution_mclk_hz : maximum_low_power_mclk_hz;
	const auto minimum_mclk =
		configuration.power_mode == PowerMode::HighResolution ?
		minimum_high_resolution_mclk_hz : minimum_low_power_mclk_hz;
	const auto maximum_decimation = configuration.filter == Filter::Sinc5 ?
		maximum_sinc5_integer_decimation :
		maximum_sinc3_integer_decimation;
	const auto denominator = modulator_clock_divisor(configuration) * rate;
	const auto decimation = denominator == 0u ? 0u :
		configuration.master_clock_hz / denominator;
	if (configuration.frames_per_packet == 0 ||
	    configuration.frames_per_packet > maximum_dma_frames ||
	    rate < 1000u || rate > 128000u ||
	    configuration.master_clock_hz < minimum_mclk ||
	    configuration.master_clock_hz > maximum_mclk ||
	    (configuration.master_clock_hz % denominator) != 0u ||
	    decimation < minimum_decimation ||
	    decimation > maximum_decimation)
		return Error::InvalidConfiguration;

	configuration_ = configuration;
	spi_initialized_ = false;
	initialized_ = false;
	capture_active_ = false;

	if (capture_read(capture_identifier) != expected_capture_identifier ||
	    (capture_read(capture_version) >> 16) != 1u)
		return Error::CaptureCoreNotFound;

	Error error = initialize_spi();
	if (error != Error::None) return error;
	spi_initialized_ = true;
	error = initialize_dma();
	if (error != Error::None) return error;
	error = reset_and_configure_adc();
	if (error != Error::None) return error;

	capture_write(capture_packet_frames,
		      configuration_.frames_per_packet);
	initialized_ = true;
	return Error::None;
}

void Ad7771::dma_interrupt_handler(void *reference)
{
	auto *adc = static_cast<Ad7771 *>(reference);
	const auto irq_status = XAxiDma_IntrGetIrq(
		&adc->dma_, XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrAckIrq(&adc->dma_, irq_status, XAXIDMA_DEVICE_TO_DMA);
	if ((irq_status & XAXIDMA_IRQ_ALL_MASK) == 0u)
		return;

	if ((irq_status & XAXIDMA_IRQ_ERROR_MASK) != 0u)
		adc->dma_interrupt_error_ = true;

	BaseType_t higher_priority_task_woken = pdFALSE;
	if (adc->capture_waiter_ != nullptr)
		vTaskNotifyGiveFromISR(
			static_cast<TaskHandle_t>(adc->capture_waiter_),
			&higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}

Error Ad7771::enable_capture_interrupt()
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	if (dma_interrupt_enabled_)
		return Error::None;

	capture_waiter_ = xTaskGetCurrentTaskHandle();
	dma_interrupt_error_ = false;
	XAxiDma_IntrDisable(&dma_, XAXIDMA_IRQ_ALL_MASK,
			     XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrAckIrq(&dma_, XAXIDMA_IRQ_ALL_MASK,
			   XAXIDMA_DEVICE_TO_DMA);
	if (xPortInstallInterruptHandler(XPAR_FABRIC_XAXIDMA_0_INTR,
					 static_cast<XInterruptHandler>(dma_interrupt_handler),
					 this) != pdPASS)
		return Error::DmaInterruptInitialization;

	XSetPriorityTriggerType(
		XPAR_XAXIDMA_0_INTERRUPTS,
		static_cast<u8>(portLOWEST_USABLE_INTERRUPT_PRIORITY <<
				portPRIORITY_SHIFT),
		XPAR_XAXIDMA_0_INTERRUPT_PARENT);
	vPortEnableInterrupt(XPAR_FABRIC_XAXIDMA_0_INTR);
	XAxiDma_IntrEnable(&dma_, XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK,
			    XAXIDMA_DEVICE_TO_DMA);
	dma_interrupt_enabled_ = true;
	return Error::None;
}

Error Ad7771::wait_for_capture()
{
	if (!initialized_ || !capture_active_ || !dma_interrupt_enabled_)
		return Error::CaptureNotInitialized;

	while (!capture_complete()) {
		(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (dma_interrupt_error_)
			return Error::DmaTransfer;
	}
	return dma_interrupt_error_ ? Error::DmaTransfer : Error::None;
}

std::size_t Ad7771::packet_bytes() const
{
	return static_cast<std::size_t>(configuration_.frames_per_packet) *
	       sizeof(SampleFrame);
}

Error Ad7771::arm_dma(SampleFrame *buffer, std::size_t capacity_frames,
			     bool first_packet)
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	if (buffer == nullptr ||
	    capacity_frames < configuration_.frames_per_packet ||
	    (reinterpret_cast<std::uintptr_t>(buffer) & cache_line_mask) != 0)
		return Error::CaptureBufferInvalid;

	const auto bytes = packet_bytes();
	Xil_DCacheFlushRange(reinterpret_cast<INTPTR>(buffer),
			     static_cast<u32>(bytes));
	if (first_packet) {
		set_capture_control(control_fifo_reset | control_adc_reset_n);
		usleep(1);
	}

	if (XAxiDma_SimpleTransfer(&dma_,
			reinterpret_cast<UINTPTR>(buffer),
			static_cast<u32>(bytes),
			XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS)
		return Error::DmaTransfer;

	if (first_packet) {
		set_capture_control(control_capture_enable | control_adc_reset_n);
		capture_active_ = true;
	}
	return Error::None;
}

Error Ad7771::start_capture(SampleFrame *buffer,
			   std::size_t capacity_frames)
{
	if (capture_active_)
		return Error::CaptureAlreadyActive;
	return arm_dma(buffer, capacity_frames, true);
}

bool Ad7771::capture_complete() const
{
	return capture_active_ &&
	       !XAxiDma_Busy(const_cast<XAxiDma *>(&dma_),
			      XAXIDMA_DEVICE_TO_DMA);
}

Error Ad7771::finish_capture(SampleFrame *buffer)
{
	if (!initialized_ || !capture_active_)
		return Error::CaptureNotInitialized;
	if (buffer == nullptr)
		return Error::CaptureBufferInvalid;
	if (!capture_complete())
		return Error::DmaTransfer;
	Xil_DCacheInvalidateRange(reinterpret_cast<INTPTR>(buffer),
				  static_cast<u32>(packet_bytes()));
	return Error::None;
}

Error Ad7771::rearm_capture(SampleFrame *buffer,
			   std::size_t capacity_frames)
{
	if (!capture_active_)
		return Error::CaptureNotInitialized;
	if (!capture_complete())
		return Error::DmaTransfer;
	return arm_dma(buffer, capacity_frames, false);
}

void Ad7771::stop_capture()
{
	if (!initialized_)
		return;
	set_capture_control(control_fifo_reset | control_adc_reset_n);
	capture_active_ = false;
}

CaptureStatus Ad7771::status() const
{
	return {
		capture_read(capture_status),
		capture_read(capture_frame_count),
		capture_read(capture_overflow_count),
		capture_read(capture_header_error_count),
		capture_read(capture_alert_count),
		capture_read(capture_packet_count),
	};
}

Error Ad7771::read_register_health(RegisterHealth &health)
{
	health = {};
	health.expected_decimation = decimation_for(configuration_);
	if (!spi_initialized_)
		return Error::SpiInitialization;

	Error error = Error::None;
	auto read = [&](std::uint8_t address, std::uint8_t &value) {
		if (error == Error::None)
			error = read_adc_register(address, value);
	};
	read(reg_status_3, health.status_3);
	read(reg_general_user_config_1, health.general_user_config_1);
	read(reg_general_user_config_2, health.general_user_config_2);
	read(reg_general_user_config_3, health.general_user_config_3);
	read(reg_dout_format, health.dout_format);
	read(reg_src_n_msb, health.src_n_msb);
	read(reg_src_n_lsb, health.src_n_lsb);
	read(reg_src_if_msb, health.src_if_msb);
	read(reg_src_if_lsb, health.src_if_lsb);
	read(reg_src_update, health.src_update);
	if (error != Error::None)
		return error;

	const auto expected_config_1 =
		configuration_.power_mode == PowerMode::HighResolution ?
			config_1_power_mode : 0u;
	const auto expected_config_2 = static_cast<std::uint8_t>(
		(configuration_.filter == Filter::Sinc5 ? config_2_filter_mode : 0u) |
		config_2_spi_sync);
	const auto expected_decimation = health.expected_decimation;
	health.configuration_matches =
		(health.general_user_config_1 & config_1_power_mode) ==
			expected_config_1 &&
		(health.general_user_config_2 &
		 (config_2_filter_mode | config_2_sar_spi_mode | config_2_spi_sync)) ==
			expected_config_2 &&
		(health.general_user_config_3 & config_3_spi_data_mode) == 0u &&
		health.dout_format == 0u &&
		health.src_n_msb == ((expected_decimation >> 8) & 0x0fu) &&
		health.src_n_lsb == (expected_decimation & 0xffu) &&
		health.src_if_msb == 0u && health.src_if_lsb == 0u &&
		(health.src_update & src_load_update) == 0u;
	return Error::None;
}

} // namespace msap1::adc
