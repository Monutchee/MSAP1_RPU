#ifndef MSAP1_AD7771_HPP
#define MSAP1_AD7771_HPP

#include <cstddef>
#include <cstdint>

#include "xaxidma.h"
#include "xspi.h"

namespace msap1::adc {

constexpr std::size_t channel_count = 8;

enum class SampleRate : std::uint32_t {
	Sps1000 = 1000,
	Sps2000 = 2000,
	Sps4000 = 4000,
	Sps8000 = 8000,
	Sps16000 = 16000,
	Sps32000 = 32000,
	Sps64000 = 64000,
	Sps128000 = 128000,
};

enum class Filter {
	Sinc3,
	Sinc5,
};

enum class PowerMode {
	HighResolution,
	LowPower,
};

struct Configuration {
	SampleRate sample_rate = SampleRate::Sps32000;
	Filter filter = Filter::Sinc5;
	PowerMode power_mode = PowerMode::HighResolution;
	std::uint32_t master_clock_hz = 8192000;
	std::uint16_t frames_per_packet = 256;
};

struct Hardware {
	std::uintptr_t spi_base;
	std::uintptr_t capture_base;
	std::uintptr_t dma_base;
};

// One simultaneous AD7771 conversion. The PL sign-extends every 24-bit ADC
// result so consumers can work with ordinary signed 32-bit values.
struct SampleFrame {
	std::int32_t channel[channel_count];
};

static_assert(sizeof(SampleFrame) == 32, "AD7771 DMA frame must be 32 bytes");

struct CaptureStatus {
	std::uint32_t flags;
	std::uint32_t frames;
	std::uint32_t overflows;
	std::uint32_t header_errors;
	std::uint32_t alerts;
	std::uint32_t packets;
};

struct RegisterHealth {
	std::uint16_t expected_decimation = 0;
	std::uint8_t status_3 = 0;
	std::uint8_t general_user_config_1 = 0;
	std::uint8_t general_user_config_2 = 0;
	std::uint8_t general_user_config_3 = 0;
	std::uint8_t dout_format = 0;
	std::uint8_t src_n_msb = 0;
	std::uint8_t src_n_lsb = 0;
	std::uint8_t src_if_msb = 0;
	std::uint8_t src_if_lsb = 0;
	std::uint8_t src_update = 0;
	bool configuration_matches = false;
};

enum class Error {
	None,
	InvalidConfiguration,
	CaptureCoreNotFound,
	SpiInitialization,
	SpiTransfer,
	SpiProtocol,
	AdcNotReady,
	AdcRegisterMismatch,
	DmaInitialization,
	DmaIsScatterGather,
	DmaInterruptInitialization,
	DmaTransfer,
	CaptureNotInitialized,
	CaptureAlreadyActive,
	CaptureBufferInvalid,
};

const char *to_string(Error error);
std::uint32_t sample_rate_hz(SampleRate rate);

class Ad7771 {
public:
	// The application supplies addresses from its generated platform
	// definitions. Keeping them outside this reusable library prevents an XSA
	// address-map change from silently leaving stale literal addresses here.
	explicit Ad7771(Hardware hardware);

	// Reset the ADC, configure its SPI registers, and prepare the S2MM DMA.
	// No samples flow until start_capture() is called.
	Error initialize(const Configuration &configuration = Configuration{});

	// Start the first DMA packet. Capacity must be at least
	// configuration.frames_per_packet and the buffer must be cache-line aligned.
	Error start_capture(SampleFrame *buffer, std::size_t capacity_frames);

	// Use the AXI DMA S2MM interrupt to wake the calling FreeRTOS task. Call
	// once from the task that owns the capture loop before start_capture().
	Error enable_capture_interrupt();
	Error wait_for_capture();

	// DMA completion is intentionally exposed as a pollable operation. This
	// keeps the first integration independent of a board-specific GIC setup.
	bool capture_complete() const;

	// Make the just-completed packet visible to the RPU data cache.
	Error finish_capture(SampleFrame *buffer);

	// Arm another packet without stopping the ADC or flushing the PL FIFO.
	Error rearm_capture(SampleFrame *buffer, std::size_t capacity_frames);

	void stop_capture();
	CaptureStatus status() const;
	Error read_register_health(RegisterHealth &health);

	const Configuration &configuration() const { return configuration_; }
	bool initialized() const { return initialized_; }
	bool capture_active() const { return capture_active_; }

	Ad7771(const Ad7771 &) = delete;
	Ad7771 &operator=(const Ad7771 &) = delete;

private:
	static constexpr std::uint32_t control_capture_enable = 1u << 0;
	static constexpr std::uint32_t control_fifo_reset = 1u << 1;
	static constexpr std::uint32_t control_adc_reset_n = 1u << 2;
	static constexpr std::uint32_t control_adc_start = 1u << 3;

	Error initialize_spi();
	Error initialize_dma();
	Error reset_and_configure_adc();
	Error configure_sample_rate();
	Error synchronize_adc();
	static void dma_interrupt_handler(void *reference);
	Error arm_dma(SampleFrame *buffer, std::size_t capacity_frames,
		      bool first_packet);

	Error write_adc_register(std::uint8_t address, std::uint8_t value);
	Error read_adc_register(std::uint8_t address, std::uint8_t &value);
	Error update_adc_register(std::uint8_t address, std::uint8_t mask,
			  std::uint8_t value);

	std::uint32_t capture_read(std::uint32_t offset) const;
	void capture_write(std::uint32_t offset, std::uint32_t value);
	void set_capture_control(std::uint32_t value);
	std::size_t packet_bytes() const;

	Hardware hardware_;
	Configuration configuration_{};
	XSpi spi_{};
	XAxiDma dma_{};
	std::uint32_t control_shadow_ = control_fifo_reset;
	void *capture_waiter_ = nullptr;
	volatile bool dma_interrupt_error_ = false;
	bool dma_interrupt_enabled_ = false;
	bool spi_initialized_ = false;
	bool initialized_ = false;
	bool capture_active_ = false;
};

} // namespace msap1::adc

#endif // MSAP1_AD7771_HPP
