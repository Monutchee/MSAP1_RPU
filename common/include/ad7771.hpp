#ifndef MSAP1_AD7771_HPP
#define MSAP1_AD7771_HPP

#include <array>
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

enum class PgaGain : std::uint8_t {
	X1 = 1,
	X2 = 2,
	X4 = 4,
	X8 = 8,
};

inline constexpr std::size_t channel_count = 8;

struct Configuration {
	SampleRate sample_rate = SampleRate::Sps32000;
	Filter filter = Filter::Sinc5;
	PowerMode power_mode = PowerMode::HighResolution;
	std::uint32_t master_clock_hz = 8192000;
	std::uint16_t frames_per_packet = 256;
	std::array<PgaGain, channel_count> pga_gains{
		PgaGain::X1, PgaGain::X1, PgaGain::X1, PgaGain::X1,
		PgaGain::X1, PgaGain::X1, PgaGain::X1, PgaGain::X1,
	};
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
	std::uint32_t dclk_frequency_hz;
	std::uint32_t drdy_frequency_hz;
};

struct RegisterHealth {
	std::uint16_t expected_decimation = 0;
	std::array<std::uint8_t, channel_count> channel_config{};
	std::uint8_t channel_disable = 0;
	std::array<std::uint8_t, channel_count> channel_sync_offset{};
	std::uint8_t status_3 = 0;
	std::uint8_t general_user_config_1 = 0;
	std::uint8_t general_user_config_2 = 0;
	std::uint8_t general_user_config_3 = 0;
	std::uint8_t dout_format = 0;
	std::uint8_t adc_mux_config = 0;
	std::uint8_t global_mux_config = 0;
	std::uint8_t gpio_config = 0;
	std::uint8_t gpio_data = 0;
	std::uint8_t buffer_config_1 = 0;
	std::uint8_t buffer_config_2 = 0;
	std::array<std::array<std::uint8_t, 3>, channel_count>
		channel_offset{};
	std::array<std::array<std::uint8_t, 3>, channel_count>
		channel_gain{};
	std::uint8_t src_n_msb = 0;
	std::uint8_t src_n_lsb = 0;
	std::uint8_t src_if_msb = 0;
	std::uint8_t src_if_lsb = 0;
	std::uint8_t src_update = 0;
	std::array<std::uint8_t, channel_count> channel_error{};
	std::array<std::uint8_t, 4> saturation_error{};
	std::uint8_t channel_error_enable = 0;
	std::uint8_t general_error_1 = 0;
	std::uint8_t general_error_1_enable = 0;
	std::uint8_t general_error_2 = 0;
	std::uint8_t general_error_2_enable = 0;
	std::uint8_t status_1 = 0;
	std::uint8_t status_2 = 0;
	bool configuration_matches = false;
};

struct DiagnosticSnapshot {
	std::uint32_t capture_flags = 0;
	std::uint32_t frame_count = 0;
	std::uint32_t packet_count = 0;
	std::uint32_t dclk_frequency_hz = 0;
	std::uint32_t drdy_frequency_hz = 0;
	std::uint8_t status_1 = 0;
	std::uint8_t status_2 = 0;
	std::uint8_t status_3 = 0;
	std::uint8_t general_user_config_1 = 0;
	std::uint8_t general_user_config_2 = 0;
	std::uint8_t general_user_config_3 = 0;
	std::uint8_t dout_format = 0;
	std::uint8_t channel_disable = 0;
	std::uint8_t buffer_config_1 = 0;
	std::uint8_t buffer_config_2 = 0;
	std::uint8_t src_n_msb = 0;
	std::uint8_t src_n_lsb = 0;
	std::uint8_t src_if_msb = 0;
	std::uint8_t src_if_lsb = 0;
	std::uint8_t src_update = 0;
	bool spi_valid = false;
	bool configuration_matches = false;
};

struct DiagnosticResult {
	std::uint32_t requested_sample_rate_hz = 0;
	std::uint32_t flags = 0;
	std::uint32_t failure_stage = 0;
	std::uint32_t reset_hold_ms = 0;
	std::uint8_t src_update_high_readback = 0;
	std::uint8_t src_update_low_readback = 0;
	DiagnosticSnapshot before{};
	DiagnosticSnapshot reset_asserted{};
	DiagnosticSnapshot reset_defaults{};
	DiagnosticSnapshot after{};
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
bool sample_rate_from_hz(std::uint32_t rate_hz, SampleRate &rate);

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
	Error configure_operating_point(
		SampleRate sample_rate,
		const std::array<PgaGain, channel_count> &channel_gains);
	Error configure_pga(
		const std::array<PgaGain, channel_count> &channel_gains);
	/*
	 * Run the destructive Flow-1 diagnostic while capture is stopped. This
	 * pulses the sensor-board ADC RESET_N output driven by PL; it does not
	 * power-cycle or reset Linux/the FPGA. The active Configuration is restored
	 * before returning.
	 */
	Error run_diagnostic_flow1(DiagnosticResult &result);
	CaptureStatus status() const;
	Error read_register_health(RegisterHealth &health);

	const Configuration &configuration() const { return configuration_; }
	bool initialized() const { return initialized_; }
	bool capture_active() const { return capture_active_; }

	Ad7771(const Ad7771 &) = delete;
	Ad7771 &operator=(const Ad7771 &) = delete;

private:
	struct SrcLoadTrace {
		std::uint8_t high_readback = 0;
		std::uint8_t low_readback = 0;
	};

	static constexpr std::uint32_t control_capture_enable = 1u << 0;
	static constexpr std::uint32_t control_fifo_reset = 1u << 1;
	static constexpr std::uint32_t control_adc_reset_n = 1u << 2;
	static constexpr std::uint32_t control_adc_start = 1u << 3;

	Error initialize_spi();
	Error reset_and_configure_adc();
	Error wait_for_initialization();
	Error configure_adc_registers(SrcLoadTrace *trace = nullptr,
				      unsigned long src_update_hold_us = 10u);
	Error configure_sample_rate(SampleRate sample_rate,
				    SrcLoadTrace *trace = nullptr,
				    unsigned long update_hold_us = 10u);
	Error program_channel_gains(
		const std::array<PgaGain, channel_count> &channel_gains);
	Error synchronize_adc();
	Error take_diagnostic_snapshot(DiagnosticSnapshot &snapshot,
				       bool include_spi);

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
