#ifndef MSAP1_ADC_DEVICE_HPP
#define MSAP1_ADC_DEVICE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace msap1::adc {

enum class Source : std::uint32_t { Physical = 0, Simulator = 1 };

enum class SampleRate : std::uint32_t {
	Sps1000 = 1000, Sps2000 = 2000, Sps4000 = 4000,
	Sps8000 = 8000, Sps16000 = 16000, Sps32000 = 32000,
	Sps64000 = 64000, Sps128000 = 128000,
};

enum class Filter { Sinc3, Sinc5 };
enum class PowerMode { HighResolution, LowPower };
enum class PgaGain : std::uint8_t { X1 = 1, X2 = 2, X4 = 4, X8 = 8 };

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

struct SimulatorConfiguration {
	std::uint32_t generation = 0;
	std::uint32_t frequency_millihz = 60000;
	std::uint32_t valid_mask = 0x7fu;
	std::array<std::int32_t, channel_count> peak_counts{};
	std::array<std::uint32_t, channel_count> phase_q32{};
	std::uint32_t phase_step_q32 = 0;
};

struct CaptureStatus {
	std::uint32_t flags = 0, frames = 0, overflows = 0;
	std::uint32_t header_errors = 0, alerts = 0, packets = 0;
	std::uint32_t dclk_frequency_hz = 0, drdy_frequency_hz = 0;
};

struct DeviceStatus {
	Source source = Source::Physical;
	bool initialized = false;
	bool capture_active = false;
	bool configuration_matches = false;
	std::uint32_t active_generation = 0;
	std::uint32_t simulator_status = 0;
	std::uint32_t saturation_count = 0;
	std::uint32_t missed_sample_count = 0;
	CaptureStatus capture{};
};

struct RegisterHealth {
	std::uint16_t expected_decimation = 0;
	std::array<std::uint8_t, channel_count> channel_config{};
	std::uint8_t channel_disable = 0;
	std::array<std::uint8_t, channel_count> channel_sync_offset{};
	std::uint8_t status_3 = 0, general_user_config_1 = 0;
	std::uint8_t general_user_config_2 = 0, general_user_config_3 = 0;
	std::uint8_t dout_format = 0, adc_mux_config = 0, global_mux_config = 0;
	std::uint8_t gpio_config = 0, gpio_data = 0;
	std::uint8_t buffer_config_1 = 0, buffer_config_2 = 0;
	std::array<std::array<std::uint8_t, 3>, channel_count> channel_offset{};
	std::array<std::array<std::uint8_t, 3>, channel_count> channel_gain{};
	std::uint8_t src_n_msb = 0, src_n_lsb = 0;
	std::uint8_t src_if_msb = 0, src_if_lsb = 0, src_update = 0;
	std::array<std::uint8_t, channel_count> channel_error{};
	std::array<std::uint8_t, 4> saturation_error{};
	std::uint8_t channel_error_enable = 0;
	std::uint8_t general_error_1 = 0, general_error_1_enable = 0;
	std::uint8_t general_error_2 = 0, general_error_2_enable = 0;
	std::uint8_t status_1 = 0, status_2 = 0;
	bool configuration_matches = false;
};

struct SpiHealthDiagnostics {
	std::uint32_t protocol_error_count = 0, retry_recovery_count = 0;
	std::uint8_t last_failed_register = 0, last_received_header = 0;
	/* Malformed reply headers bucketed by high nibble (saturating).
	 * The expected header is 0x20, so a healthy bus leaves this all
	 * zero; the shape of a non-zero histogram says whether the
	 * corruption is systematic or random. */
	std::uint16_t header_histogram[16] = {};
};

struct DiagnosticSnapshot {
	std::uint32_t capture_flags = 0, frame_count = 0, packet_count = 0;
	std::uint32_t dclk_frequency_hz = 0, drdy_frequency_hz = 0;
	std::uint8_t status_1 = 0, status_2 = 0, status_3 = 0;
	std::uint8_t general_user_config_1 = 0, general_user_config_2 = 0;
	std::uint8_t general_user_config_3 = 0, dout_format = 0;
	std::uint8_t channel_disable = 0, buffer_config_1 = 0, buffer_config_2 = 0;
	std::uint8_t src_n_msb = 0, src_n_lsb = 0;
	std::uint8_t src_if_msb = 0, src_if_lsb = 0, src_update = 0;
	bool spi_valid = false, configuration_matches = false;
};

struct DiagnosticResult {
	std::uint32_t requested_sample_rate_hz = 0, flags = 0;
	std::uint32_t failure_stage = 0, reset_hold_ms = 0;
	std::uint8_t src_update_high_readback = 0, src_update_low_readback = 0;
	DiagnosticSnapshot before{}, reset_asserted{}, reset_defaults{}, after{};
};

enum class Error {
	None, InvalidConfiguration, CaptureCoreNotFound, SpiInitialization,
	SpiTransfer, SpiProtocol, AdcNotReady, AdcRegisterMismatch,
	CaptureNotInitialized, CaptureAlreadyActive, UnsupportedOperation,
};

const char *to_string(Error error);
std::uint32_t sample_rate_hz(SampleRate rate);
bool sample_rate_from_hz(std::uint32_t rate_hz, SampleRate &rate);

/*
 * Source-independent lifecycle consumed by AdcController.  Physical SPI
 * diagnostics deliberately do not appear here: callers that need an AD7771
 * register audit must use the concrete Ad7771 object owned by the controller.
 */
class AdcDevice {
public:
	virtual ~AdcDevice() = default;
	virtual Source source() const = 0;
	virtual Error initialize(const Configuration &configuration) = 0;
	virtual Error start_capture() = 0;
	virtual void stop_capture() = 0;
	virtual Error configure_operating_point(
		SampleRate, const std::array<PgaGain, channel_count> &) = 0;
	virtual DeviceStatus status() const = 0;
	virtual const Configuration &configuration() const = 0;
	virtual bool initialized() const = 0;
	virtual bool capture_active() const = 0;
};

} // namespace msap1::adc

#endif
