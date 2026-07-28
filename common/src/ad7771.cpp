#include "ad7771.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "sleep.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xstatus.h"

namespace msap1::adc {
namespace {

constexpr std::uint8_t reg_channel_config_base = 0x00;
constexpr std::uint8_t reg_channel_disable = 0x08;
constexpr std::uint8_t reg_channel_sync_offset_base = 0x09;
constexpr std::uint8_t reg_general_user_config_1 = 0x11;
constexpr std::uint8_t reg_general_user_config_2 = 0x12;
constexpr std::uint8_t reg_general_user_config_3 = 0x13;
constexpr std::uint8_t reg_dout_format = 0x14;
constexpr std::uint8_t reg_adc_mux_config = 0x15;
constexpr std::uint8_t reg_global_mux_config = 0x16;
constexpr std::uint8_t reg_gpio_config = 0x17;
constexpr std::uint8_t reg_gpio_data = 0x18;
constexpr std::uint8_t reg_buffer_config_1 = 0x19;
constexpr std::uint8_t reg_buffer_config_2 = 0x1a;
constexpr std::uint8_t reg_channel_offset_base = 0x1c;
constexpr std::uint8_t reg_channel_calibration_stride = 6;
constexpr std::uint8_t reg_channel_gain_offset = 3;
constexpr std::uint8_t reg_channel_error_base = 0x4c;
constexpr std::uint8_t reg_saturation_error_base = 0x54;
constexpr std::uint8_t reg_channel_error_enable = 0x58;
constexpr std::uint8_t reg_general_error_1 = 0x59;
constexpr std::uint8_t reg_general_error_1_enable = 0x5a;
constexpr std::uint8_t reg_general_error_2 = 0x5b;
constexpr std::uint8_t reg_general_error_2_enable = 0x5c;
constexpr std::uint8_t reg_status_1 = 0x5d;
constexpr std::uint8_t reg_status_2 = 0x5e;
constexpr std::uint8_t reg_status_3 = 0x5f;
constexpr std::uint8_t reg_src_n_msb = 0x60;
constexpr std::uint8_t reg_src_n_lsb = 0x61;
constexpr std::uint8_t reg_src_if_msb = 0x62;
constexpr std::uint8_t reg_src_if_lsb = 0x63;
constexpr std::uint8_t reg_src_update = 0x64;

constexpr std::uint8_t status_init_complete = 1u << 4;
constexpr std::uint8_t config_1_power_mode = 1u << 6;
constexpr std::uint8_t config_1_refout_buffer = 1u << 4;
constexpr std::uint8_t config_2_filter_mode = 1u << 6;
constexpr std::uint8_t config_2_sar_spi_mode = 1u << 5;
constexpr std::uint8_t config_2_spi_sync = 1u << 0;
constexpr std::uint8_t config_3_spi_data_mode = 1u << 4;
constexpr std::uint8_t channel_config_pga_mask = 0xc0u;
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
constexpr std::uint32_t capture_dclk_frequency_hz = 0x2c;
constexpr std::uint32_t capture_drdy_frequency_hz = 0x30;

constexpr std::uint32_t expected_capture_identifier = 0x41443731u; // "AD71"
constexpr std::uint16_t maximum_packet_frames = 2047;
constexpr std::uint32_t minimum_high_resolution_mclk_hz = 655000u;
constexpr std::uint32_t maximum_high_resolution_mclk_hz = 8192000u;
constexpr std::uint32_t minimum_low_power_mclk_hz = 1300000u;
constexpr std::uint32_t maximum_low_power_mclk_hz = 4096000u;
constexpr std::uint32_t minimum_decimation = 16u;
constexpr std::uint32_t maximum_sinc3_integer_decimation = 4095u;
constexpr std::uint32_t maximum_sinc5_integer_decimation = 2048u;
constexpr unsigned long reference_output_settling_us = 2000u;
constexpr unsigned long adc_start_sync_pulse_us = 10u;
constexpr unsigned int spi_register_read_attempts = 3u;
constexpr unsigned long spi_register_retry_delay_us = 10u;
constexpr std::uint32_t diagnostic_reset_hold_ms = 2200u;
constexpr std::uint32_t diagnostic_measurement_settle_ms = 2200u;
constexpr unsigned long diagnostic_src_update_hold_us = 1000u;

constexpr std::uint32_t diagnostic_reset_asserted = 1u << 0;
constexpr std::uint32_t diagnostic_reset_drdy_stopped = 1u << 1;
constexpr std::uint32_t diagnostic_reset_defaults_read = 1u << 2;
constexpr std::uint32_t diagnostic_src_update_high_read = 1u << 3;
constexpr std::uint32_t diagnostic_src_update_low_read = 1u << 4;
constexpr std::uint32_t diagnostic_src_holding_match = 1u << 5;
constexpr std::uint32_t diagnostic_final_config_match = 1u << 6;
constexpr std::uint32_t diagnostic_final_drdy_match = 1u << 7;

constexpr std::uint32_t diagnostic_stage_preflight = 1u;
constexpr std::uint32_t diagnostic_stage_before = 2u;
constexpr std::uint32_t diagnostic_stage_reset_assert = 3u;
constexpr std::uint32_t diagnostic_stage_reset_release = 4u;
constexpr std::uint32_t diagnostic_stage_reset_defaults = 5u;
constexpr std::uint32_t diagnostic_stage_reconfigure = 6u;
constexpr std::uint32_t diagnostic_stage_after = 7u;

bool valid_sample_rate(SampleRate rate)
{
	switch (rate) {
	case SampleRate::Sps1000:
	case SampleRate::Sps2000:
	case SampleRate::Sps4000:
	case SampleRate::Sps8000:
	case SampleRate::Sps16000:
	case SampleRate::Sps32000:
	case SampleRate::Sps64000:
	case SampleRate::Sps128000:
		return true;
	}
	return false;
}

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

bool valid_pga_gain(PgaGain gain)
{
	switch (gain) {
	case PgaGain::X1:
	case PgaGain::X2:
	case PgaGain::X4:
	case PgaGain::X8:
		return true;
	}
	return false;
}

std::uint8_t pga_register_value(PgaGain gain)
{
	switch (gain) {
	case PgaGain::X1: return 0u << 6;
	case PgaGain::X2: return 1u << 6;
	case PgaGain::X4: return 2u << 6;
	case PgaGain::X8: return 3u << 6;
	}
	return 0u;
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
	case Error::CaptureNotInitialized: return "ADC capture is not initialized";
	case Error::CaptureAlreadyActive: return "ADC capture is already active";
	}
	return "unknown";
}

std::uint32_t sample_rate_hz(SampleRate rate)
{
	return static_cast<std::uint32_t>(rate);
}

bool sample_rate_from_hz(std::uint32_t rate_hz, SampleRate &rate)
{
	const auto candidate = static_cast<SampleRate>(rate_hz);
	if (!valid_sample_rate(candidate))
		return false;
	rate = candidate;
	return true;
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
	for (unsigned int attempt = 0; attempt < spi_register_read_attempts;
	     ++attempt) {
		std::uint8_t transmit[2] = {
			static_cast<std::uint8_t>(0x80u | (address & 0x7fu)), 0u
		};
		std::uint8_t receive[2] = {};
		if (XSpi_Transfer(&spi_, transmit, receive, sizeof(transmit)) !=
		    XST_SUCCESS)
			return Error::SpiTransfer;
		if (receive[0] == 0x20u) {
			if (attempt != 0u)
				++spi_health_diagnostics_.retry_recovery_count;
			value = receive[1];
			return Error::None;
		}

		++spi_health_diagnostics_.protocol_error_count;
		spi_health_diagnostics_.last_failed_register = address;
		spi_health_diagnostics_.last_received_header = receive[0];
		if (attempt + 1u < spi_register_read_attempts)
			usleep(spi_register_retry_delay_us);
	}
	return Error::SpiProtocol;
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

Error Ad7771::configure_sample_rate(SampleRate sample_rate,
				    SrcLoadTrace *trace,
				    unsigned long update_hold_us)
{
	Configuration staged = configuration_;
	staged.sample_rate = sample_rate;
	if (!valid_sample_rate(sample_rate))
		return Error::InvalidConfiguration;
	const auto rate_hz = sample_rate_hz(sample_rate);
	const auto denominator = modulator_clock_divisor(staged) * rate_hz;
	if (denominator == 0u ||
	    (staged.master_clock_hz % denominator) != 0u)
		return Error::InvalidConfiguration;
	const std::uint16_t decimation = decimation_for(staged);
	const auto maximum_decimation =
		staged.filter == Filter::Sinc5 ?
			maximum_sinc5_integer_decimation :
			maximum_sinc3_integer_decimation;
	if (decimation < minimum_decimation ||
	    decimation > maximum_decimation)
		return Error::InvalidConfiguration;

	// SRC_N/SRC_IF are holding registers. Force software load mode and an
	// inactive update level before changing them so a previous partial
	// transaction cannot suppress the required low-to-high load transition.
	Error error = write_adc_register(reg_src_update, 0u);
	if (error != Error::None) return error;
	error = write_adc_register(reg_src_n_msb,
		static_cast<std::uint8_t>((decimation >> 8) & 0x0fu));
	if (error != Error::None) return error;
	error = write_adc_register(reg_src_n_lsb,
		static_cast<std::uint8_t>(decimation & 0xffu));
	if (error != Error::None) return error;
	error = write_adc_register(reg_src_if_msb, 0u);
	if (error != Error::None) return error;
	error = write_adc_register(reg_src_if_lsb, 0u);
	if (error != Error::None) return error;

	// Transfer the holding registers to the active DSP decimator. Use direct
	// writes so the rate switch exercises an unambiguous 0 -> 1 -> 0 sequence.
	// Normal configuration holds for 10 us; Flow 1 deliberately holds for
	// 1 ms and samples the bit high and low to make the load observable.
	error = write_adc_register(reg_src_update, src_load_update);
	if (error != Error::None) return error;
	if (trace != nullptr) {
		error = read_adc_register(reg_src_update, trace->high_readback);
		if (error != Error::None) return error;
	}
	usleep(update_hold_us);
	error = write_adc_register(reg_src_update, 0u);
	if (error != Error::None) return error;
	if (trace != nullptr) {
		error = read_adc_register(reg_src_update, trace->low_readback);
		if (error != Error::None) return error;
	}

	std::uint8_t readback = 0;
	error = read_adc_register(reg_src_n_msb, readback);
	if (error != Error::None ||
	    readback != static_cast<std::uint8_t>((decimation >> 8) & 0x0fu))
		return error == Error::None ? Error::AdcRegisterMismatch : error;
	error = read_adc_register(reg_src_n_lsb, readback);
	if (error != Error::None ||
	    readback != static_cast<std::uint8_t>(decimation & 0xffu))
		return error == Error::None ? Error::AdcRegisterMismatch : error;
	error = read_adc_register(reg_src_if_msb, readback);
	if (error != Error::None || readback != 0u)
		return error == Error::None ? Error::AdcRegisterMismatch : error;
	error = read_adc_register(reg_src_if_lsb, readback);
	if (error != Error::None || readback != 0u)
		return error == Error::None ? Error::AdcRegisterMismatch : error;
	error = read_adc_register(reg_src_update, readback);
	if (error != Error::None || readback != 0u)
		return error == Error::None ? Error::AdcRegisterMismatch : error;

	// The loaded ODR becomes effective within three conversion cycles. Keep
	// synchronization separate from that transition and wait four cycles so
	// the old and new rate cannot straddle the subsequent filter reset.
	const auto transition_us =
		(4u * 1000000u + rate_hz - 1u) / rate_hz;
	usleep(transition_us);
	return Error::None;
}

Error Ad7771::program_channel_gains(
	const std::array<PgaGain, channel_count> &channel_gains)
{
	for (std::size_t channel = 0; channel < channel_gains.size(); ++channel) {
		if (!valid_pga_gain(channel_gains[channel]))
			return Error::InvalidConfiguration;
		const auto error = update_adc_register(
			static_cast<std::uint8_t>(reg_channel_config_base + channel),
			channel_config_pga_mask,
			pga_register_value(channel_gains[channel]));
		if (error != Error::None)
			return error;
	}
	return Error::None;
}

Error Ad7771::synchronize_adc()
{
	/*
	 * The sensor board loops SYNC_OUT back to SYNC_IN. Drive the AD7771 START
	 * pin high long enough for its MCLK-domain synchronizer to emit a clean
	 * SYNC_OUT pulse, then return it low. This documented hardware path avoids
	 * the ambiguous START gating described for GENERAL_USER_CONFIG_2.SPI_SYNC.
	 *
	 * The PL port is retained as adc_start_n for compatibility with the board
	 * net name, but AD7771 START is a positive synchronization pulse.
	 */
	const auto inactive_control = control_shadow_ & ~control_adc_start;
	set_capture_control(inactive_control);
	usleep(1);
	set_capture_control(inactive_control | control_adc_start);
	usleep(adc_start_sync_pulse_us);
	set_capture_control(inactive_control);
	return Error::None;
}

Error Ad7771::reset_and_configure_adc()
{
	// Hold RESET low, keep the positive-pulse START input inactive/low, and
	// reset the PL FIFO. Configuration later emits a hardware START pulse.
	set_capture_control(control_fifo_reset);
	usleep(10);
	set_capture_control(control_fifo_reset | control_adc_reset_n);
	usleep(2500);

	auto error = wait_for_initialization();
	if (error != Error::None)
		return error;
	return configure_adc_registers();
}

Error Ad7771::wait_for_initialization()
{
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
	return Error::None;
}

Error Ad7771::configure_adc_registers(SrcLoadTrace *trace,
				      unsigned long src_update_hold_us)
{
	// The sensor board derives its buffered REF1+/REF2+ voltage from REFOUT.
	// PDB_REFOUT_BUF is active low and resets to zero, so explicitly deassert
	// power-down and allow REFOUT plus the external reference buffer to settle
	// before the filters are configured and synchronized.
	const auto config_1_value = static_cast<std::uint8_t>(
		config_1_refout_buffer |
		(configuration_.power_mode == PowerMode::HighResolution ?
			config_1_power_mode : 0u));
	Error error = update_adc_register(reg_general_user_config_1,
		config_1_power_mode | config_1_refout_buffer, config_1_value);
	if (error != Error::None) return error;
	usleep(reference_output_settling_us);
	error = update_adc_register(reg_general_user_config_2,
		config_2_filter_mode | config_2_sar_spi_mode | config_2_spi_sync,
		configuration_.filter == Filter::Sinc5 ?
			config_2_filter_mode | config_2_spi_sync :
			config_2_spi_sync);
	if (error != Error::None) return error;
	error = update_adc_register(reg_general_user_config_3,
		config_3_spi_data_mode, 0u);
	if (error != Error::None) return error;

	// Four DOUT lanes, status headers, DCLK=MCLK. A constant 8.192 MHz DCLK
	// supports every declared ODR and keeps the PL timing contract unchanged.
	error = write_adc_register(reg_dout_format, 0x00u);
	if (error != Error::None) return error;
	error = program_channel_gains(configuration_.pga_gains);
	if (error != Error::None) return error;
	error = configure_sample_rate(configuration_.sample_rate, trace,
				      src_update_hold_us);
	if (error != Error::None) return error;
	error = synchronize_adc();
	if (error != Error::None) return error;
	const auto rate_hz = sample_rate_hz(configuration_.sample_rate);
	const auto settling_cycles =
		configuration_.filter == Filter::Sinc5 ? 6u : 4u;
	usleep((settling_cycles * 1000000u + rate_hz - 1u) / rate_hz);
	return Error::None;
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
	bool gains_valid = true;
	for (const auto gain : configuration.pga_gains)
		gains_valid = gains_valid && valid_pga_gain(gain);
	if (!valid_sample_rate(configuration.sample_rate) ||
	    configuration.frames_per_packet == 0 ||
	    configuration.frames_per_packet > maximum_packet_frames ||
	    rate < 1000u || rate > 128000u ||
	    configuration.master_clock_hz < minimum_mclk ||
	    configuration.master_clock_hz > maximum_mclk ||
	    (configuration.master_clock_hz % denominator) != 0u ||
	    decimation < minimum_decimation ||
	    decimation > maximum_decimation || !gains_valid)
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
	error = reset_and_configure_adc();
	if (error != Error::None) return error;

	capture_write(capture_packet_frames,
		      configuration_.frames_per_packet);
	initialized_ = true;
	return Error::None;
}

Error Ad7771::start_capture()
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	if (capture_active_)
		return Error::CaptureAlreadyActive;
	set_capture_control(control_fifo_reset | control_adc_reset_n);
	usleep(1);
	set_capture_control(control_capture_enable | control_adc_reset_n);
	capture_active_ = true;
	return Error::None;
}

void Ad7771::stop_capture()
{
	if (!initialized_)
		return;
	set_capture_control(control_fifo_reset | control_adc_reset_n);
	capture_active_ = false;
}

Error Ad7771::configure_pga(
	const std::array<PgaGain, channel_count> &channel_gains)
{
	return configure_operating_point(configuration_.sample_rate,
					 channel_gains);
}

Error Ad7771::configure_operating_point(
	SampleRate sample_rate,
	const std::array<PgaGain, channel_count> &channel_gains)
{
	if (!initialized_)
		return Error::CaptureNotInitialized;
	if (capture_active_)
		return Error::CaptureAlreadyActive;
	if (!valid_sample_rate(sample_rate))
		return Error::InvalidConfiguration;
	for (const auto gain : channel_gains)
		if (!valid_pga_gain(gain))
			return Error::InvalidConfiguration;

	auto error = program_channel_gains(channel_gains);
	if (error != Error::None)
		return error;
	error = configure_sample_rate(sample_rate);
	if (error != Error::None)
		return error;
	error = synchronize_adc();
	if (error != Error::None)
		return error;
	const auto rate_hz = sample_rate_hz(sample_rate);
	const auto settling_cycles =
		configuration_.filter == Filter::Sinc5 ? 6u : 4u;
	usleep((settling_cycles * 1000000u + rate_hz - 1u) / rate_hz);

	configuration_.sample_rate = sample_rate;
	configuration_.pga_gains = channel_gains;
	return Error::None;
}

Error Ad7771::take_diagnostic_snapshot(DiagnosticSnapshot &snapshot,
				       bool include_spi)
{
	snapshot = {};
	const auto capture = status();
	snapshot.capture_flags = capture.flags;
	snapshot.frame_count = capture.frames;
	snapshot.packet_count = capture.packets;
	snapshot.dclk_frequency_hz = capture.dclk_frequency_hz;
	snapshot.drdy_frequency_hz = capture.drdy_frequency_hz;
	if (!include_spi)
		return Error::None;

	RegisterHealth health;
	const auto error = read_register_health(health);
	if (error != Error::None)
		return error;
	snapshot.status_1 = health.status_1;
	snapshot.status_2 = health.status_2;
	snapshot.status_3 = health.status_3;
	snapshot.general_user_config_1 = health.general_user_config_1;
	snapshot.general_user_config_2 = health.general_user_config_2;
	snapshot.general_user_config_3 = health.general_user_config_3;
	snapshot.dout_format = health.dout_format;
	snapshot.channel_disable = health.channel_disable;
	snapshot.buffer_config_1 = health.buffer_config_1;
	snapshot.buffer_config_2 = health.buffer_config_2;
	snapshot.src_n_msb = health.src_n_msb;
	snapshot.src_n_lsb = health.src_n_lsb;
	snapshot.src_if_msb = health.src_if_msb;
	snapshot.src_if_lsb = health.src_if_lsb;
	snapshot.src_update = health.src_update;
	snapshot.spi_valid = true;
	snapshot.configuration_matches = health.configuration_matches;
	return Error::None;
}

Error Ad7771::run_diagnostic_flow1(DiagnosticResult &result)
{
	result = {};
	result.requested_sample_rate_hz =
		sample_rate_hz(configuration_.sample_rate);
	result.reset_hold_ms = diagnostic_reset_hold_ms;
	result.failure_stage = diagnostic_stage_preflight;
	if (!initialized_)
		return Error::CaptureNotInitialized;
	if (capture_active_)
		return Error::CaptureAlreadyActive;

	result.failure_stage = diagnostic_stage_before;
	auto error = take_diagnostic_snapshot(result.before, true);
	if (error != Error::None)
		return error;

	/*
	 * This is a warm pin reset. PL drives the sensor-board ADC RESET_N low
	 * while Linux, the FPGA fabric, and the RPU remain alive. Holding it over
	 * two PL measurement windows guarantees the DCLK/DRDY snapshot no longer
	 * contains pre-reset edges.
	 */
	result.failure_stage = diagnostic_stage_reset_assert;
	set_capture_control(control_fifo_reset);
	result.flags |= diagnostic_reset_asserted;
	vTaskDelay(pdMS_TO_TICKS(diagnostic_reset_hold_ms));
	error = take_diagnostic_snapshot(result.reset_asserted, false);
	if (error != Error::None)
		return error;
	if (result.reset_asserted.drdy_frequency_hz == 0u)
		result.flags |= diagnostic_reset_drdy_stopped;

	result.failure_stage = diagnostic_stage_reset_release;
	set_capture_control(control_fifo_reset | control_adc_reset_n);
	vTaskDelay(pdMS_TO_TICKS(3u));
	error = wait_for_initialization();
	if (error != Error::None) {
		(void)reset_and_configure_adc();
		return error;
	}

	result.failure_stage = diagnostic_stage_reset_defaults;
	error = take_diagnostic_snapshot(result.reset_defaults, true);
	if (error != Error::None) {
		(void)reset_and_configure_adc();
		return error;
	}
	result.flags |= diagnostic_reset_defaults_read;

	result.failure_stage = diagnostic_stage_reconfigure;
	SrcLoadTrace trace{};
	error = configure_adc_registers(&trace,
					diagnostic_src_update_hold_us);
	result.src_update_high_readback = trace.high_readback;
	result.src_update_low_readback = trace.low_readback;
	if ((trace.high_readback & src_load_update) != 0u)
		result.flags |= diagnostic_src_update_high_read;
	if ((trace.low_readback & src_load_update) == 0u)
		result.flags |= diagnostic_src_update_low_read;
	if (error != Error::None) {
		(void)reset_and_configure_adc();
		return error;
	}

	// Yield instead of busy-waiting so the lower-priority heartbeat task stays
	// responsive throughout this deliberately multi-second diagnostic.
	vTaskDelay(pdMS_TO_TICKS(diagnostic_measurement_settle_ms));
	result.failure_stage = diagnostic_stage_after;
	error = take_diagnostic_snapshot(result.after, true);
	if (error != Error::None)
		return error;

	const auto expected_decimation = decimation_for(configuration_);
	const auto actual_decimation = static_cast<std::uint16_t>(
		(static_cast<std::uint16_t>(result.after.src_n_msb & 0x0fu) << 8) |
		result.after.src_n_lsb);
	if (actual_decimation == expected_decimation &&
	    result.after.src_if_msb == 0u && result.after.src_if_lsb == 0u)
		result.flags |= diagnostic_src_holding_match;
	if (result.after.configuration_matches)
		result.flags |= diagnostic_final_config_match;

	const auto measured = result.after.drdy_frequency_hz;
	const auto requested = result.requested_sample_rate_hz;
	const auto difference = measured > requested ?
		measured - requested : requested - measured;
	if (measured != 0u &&
	    difference <= (requested / 100u + 2u))
		result.flags |= diagnostic_final_drdy_match;

	result.failure_stage = 0u;
	return Error::None;
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
		capture_read(capture_dclk_frequency_hz),
		capture_read(capture_drdy_frequency_hz),
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
	for (std::size_t channel = 0; channel < channel_count; ++channel)
		read(static_cast<std::uint8_t>(reg_channel_config_base + channel),
		     health.channel_config[channel]);
	read(reg_channel_disable, health.channel_disable);
	for (std::size_t channel = 0; channel < channel_count; ++channel)
		read(static_cast<std::uint8_t>(
			     reg_channel_sync_offset_base + channel),
		     health.channel_sync_offset[channel]);
	read(reg_general_user_config_1, health.general_user_config_1);
	read(reg_general_user_config_2, health.general_user_config_2);
	read(reg_general_user_config_3, health.general_user_config_3);
	read(reg_dout_format, health.dout_format);
	read(reg_adc_mux_config, health.adc_mux_config);
	read(reg_global_mux_config, health.global_mux_config);
	read(reg_gpio_config, health.gpio_config);
	read(reg_gpio_data, health.gpio_data);
	read(reg_buffer_config_1, health.buffer_config_1);
	read(reg_buffer_config_2, health.buffer_config_2);
	for (std::size_t channel = 0; channel < channel_count; ++channel) {
		const auto offset_base = static_cast<std::uint8_t>(
			reg_channel_offset_base +
			channel * reg_channel_calibration_stride);
		for (std::size_t byte = 0; byte < 3; ++byte) {
			read(static_cast<std::uint8_t>(offset_base + byte),
			     health.channel_offset[channel][byte]);
			read(static_cast<std::uint8_t>(
				     offset_base + reg_channel_gain_offset + byte),
			     health.channel_gain[channel][byte]);
		}
	}
	for (std::size_t channel = 0; channel < channel_count; ++channel)
		read(static_cast<std::uint8_t>(reg_channel_error_base + channel),
		     health.channel_error[channel]);
	for (std::size_t pair = 0; pair < health.saturation_error.size(); ++pair)
		read(static_cast<std::uint8_t>(reg_saturation_error_base + pair),
		     health.saturation_error[pair]);
	read(reg_channel_error_enable, health.channel_error_enable);
	read(reg_general_error_1, health.general_error_1);
	read(reg_general_error_1_enable, health.general_error_1_enable);
	read(reg_general_error_2, health.general_error_2);
	read(reg_general_error_2_enable, health.general_error_2_enable);
	read(reg_status_1, health.status_1);
	read(reg_status_2, health.status_2);
	read(reg_status_3, health.status_3);
	read(reg_src_n_msb, health.src_n_msb);
	read(reg_src_n_lsb, health.src_n_lsb);
	read(reg_src_if_msb, health.src_if_msb);
	read(reg_src_if_lsb, health.src_if_lsb);
	read(reg_src_update, health.src_update);
	if (error != Error::None)
		return error;

	const auto expected_config_1 = static_cast<std::uint8_t>(
		config_1_refout_buffer |
		(configuration_.power_mode == PowerMode::HighResolution ?
			config_1_power_mode : 0u));
	const auto expected_config_2 = static_cast<std::uint8_t>(
		(configuration_.filter == Filter::Sinc5 ? config_2_filter_mode : 0u) |
		config_2_spi_sync);
	const auto expected_decimation = health.expected_decimation;
	bool gains_match = true;
	for (std::size_t channel = 0; channel < channel_count; ++channel)
		gains_match = gains_match &&
			(health.channel_config[channel] & channel_config_pga_mask) ==
			pga_register_value(configuration_.pga_gains[channel]);
	health.configuration_matches =
		(health.general_user_config_1 &
		 (config_1_power_mode | config_1_refout_buffer)) ==
			expected_config_1 &&
		(health.general_user_config_2 &
		 (config_2_filter_mode | config_2_sar_spi_mode | config_2_spi_sync)) ==
			expected_config_2 &&
		(health.general_user_config_3 & config_3_spi_data_mode) == 0u &&
		health.dout_format == 0u &&
		health.src_n_msb == ((expected_decimation >> 8) & 0x0fu) &&
		health.src_n_lsb == (expected_decimation & 0xffu) &&
		health.src_if_msb == 0u && health.src_if_lsb == 0u &&
		(health.src_update & src_load_update) == 0u && gains_match;
	return Error::None;
}

} // namespace msap1::adc
