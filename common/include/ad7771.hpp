#ifndef MSAP1_AD7771_HPP
#define MSAP1_AD7771_HPP

#include <cstdint>

#include "xspi.h"

namespace msap1::adc {

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
};

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
	CaptureNotInitialized,
	CaptureAlreadyActive,
};

const char *to_string(Error error);
std::uint32_t sample_rate_hz(SampleRate rate);

class Ad7771 {
public:
	// The application supplies addresses from its generated platform
	// definitions. Keeping them outside this reusable library prevents an XSA
	// address-map change from silently leaving stale literal addresses here.
	explicit Ad7771(Hardware hardware);

	// Reset the ADC, configure its SPI registers, and leave PL capture stopped.
	// Linux owns AXI DMA and must arm it before requesting start_capture().
	Error initialize(const Configuration &configuration = Configuration{});

	// Reset the PL FIFO and enable the stream. The caller must ensure Linux has
	// armed the IIO DMA channel before invoking this operation.
	Error start_capture();
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
	Error reset_and_configure_adc();
	Error configure_sample_rate();
	Error synchronize_adc();

	Error write_adc_register(std::uint8_t address, std::uint8_t value);
	Error read_adc_register(std::uint8_t address, std::uint8_t &value);
	Error update_adc_register(std::uint8_t address, std::uint8_t mask,
			  std::uint8_t value);

	std::uint32_t capture_read(std::uint32_t offset) const;
	void capture_write(std::uint32_t offset, std::uint32_t value);
	void set_capture_control(std::uint32_t value);

	Hardware hardware_;
	Configuration configuration_{};
	XSpi spi_{};
	std::uint32_t control_shadow_ = control_fifo_reset;
	bool spi_initialized_ = false;
	bool initialized_ = false;
	bool capture_active_ = false;
};

} // namespace msap1::adc

#endif // MSAP1_AD7771_HPP
