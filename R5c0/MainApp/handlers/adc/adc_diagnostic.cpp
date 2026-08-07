#include "adc_diagnostic.hpp"

namespace msap1::r5c0 {

namespace {

/** Map an ADC library error onto the wire diagnostic error class. */
std::uint32_t diagnostic_error(msap1::adc::Error error)
{
	switch (error) {
	case msap1::adc::Error::None:
		return MSAP1_ADC_DIAGNOSTIC_ERROR_NONE;
	case msap1::adc::Error::CaptureNotInitialized:
		return MSAP1_ADC_DIAGNOSTIC_ERROR_NOT_INITIALIZED;
	case msap1::adc::Error::CaptureAlreadyActive:
		return MSAP1_ADC_DIAGNOSTIC_ERROR_CAPTURE_ACTIVE;
	case msap1::adc::Error::SpiInitialization:
	case msap1::adc::Error::SpiTransfer:
	case msap1::adc::Error::SpiProtocol:
		return MSAP1_ADC_DIAGNOSTIC_ERROR_SPI;
	case msap1::adc::Error::AdcNotReady:
		return MSAP1_ADC_DIAGNOSTIC_ERROR_ADC_NOT_READY;
	case msap1::adc::Error::AdcRegisterMismatch:
		return MSAP1_ADC_DIAGNOSTIC_ERROR_REGISTER_MISMATCH;
	default:
		return MSAP1_ADC_DIAGNOSTIC_ERROR_INTERNAL;
	}
}

/** Copy one capture/register snapshot into its wire representation. */
void copy_snapshot(msap1_adc_diagnostic_snapshot &wire,
		   const msap1::adc::DiagnosticSnapshot &snapshot)
{
	wire.snapshot_flags = snapshot.spi_valid ?
		static_cast<std::uint32_t>(
			MSAP1_ADC_DIAGNOSTIC_SNAPSHOT_SPI_VALID) : 0u;
	wire.capture_flags = snapshot.capture_flags;
	wire.frame_count = snapshot.frame_count;
	wire.packet_count = snapshot.packet_count;
	wire.dclk_frequency_hz = snapshot.dclk_frequency_hz;
	wire.drdy_frequency_hz = snapshot.drdy_frequency_hz;
	wire.status_1 = snapshot.status_1;
	wire.status_2 = snapshot.status_2;
	wire.status_3 = snapshot.status_3;
	wire.general_user_config_1 = snapshot.general_user_config_1;
	wire.general_user_config_2 = snapshot.general_user_config_2;
	wire.general_user_config_3 = snapshot.general_user_config_3;
	wire.dout_format = snapshot.dout_format;
	wire.channel_disable = snapshot.channel_disable;
	wire.buffer_config_1 = snapshot.buffer_config_1;
	wire.buffer_config_2 = snapshot.buffer_config_2;
	wire.src_n_msb = snapshot.src_n_msb;
	wire.src_n_lsb = snapshot.src_n_lsb;
	wire.src_if_msb = snapshot.src_if_msb;
	wire.src_if_lsb = snapshot.src_if_lsb;
	wire.src_update = snapshot.src_update;
}

} // namespace

std::uint32_t run_adc_diagnostic(msap1::adc::AdcController &adc,
				 const msap1_adc_diagnostic_request &request,
				 msap1_adc_diagnostic_payload &wire)
{
	if (request.flow != 1u)
		return MSAP1_RPU_STATUS_BAD_PAYLOAD;
	if (adc.source() != msap1::adc::Source::Physical)
		return MSAP1_RPU_STATUS_ADC_STATE;

	msap1::adc::DiagnosticResult result;
	const auto error = adc.physical().run_diagnostic_flow1(result);
	wire = {};
	wire.flow = request.flow;
	wire.requested_sample_rate_hz = result.requested_sample_rate_hz;
	wire.diagnostic_flags = result.flags;
	wire.diagnostic_error = diagnostic_error(error);
	wire.failure_stage = result.failure_stage;
	wire.reset_hold_ms = result.reset_hold_ms;
	wire.src_update_high_readback = result.src_update_high_readback;
	wire.src_update_low_readback = result.src_update_low_readback;
	copy_snapshot(wire.before, result.before);
	copy_snapshot(wire.reset_asserted, result.reset_asserted);
	copy_snapshot(wire.reset_defaults, result.reset_defaults);
	copy_snapshot(wire.after, result.after);
	return MSAP1_RPU_STATUS_OK;
}

} // namespace msap1::r5c0
