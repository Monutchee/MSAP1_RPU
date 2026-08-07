#include "adc_capture.hpp"

namespace msap1::r5c0 {

std::uint32_t start_capture(msap1::adc::AdcController &adc)
{
	const auto error = adc.start_capture();
	if (error == msap1::adc::Error::CaptureNotInitialized)
		return MSAP1_RPU_STATUS_ADC_UNAVAILABLE;
	if (error == msap1::adc::Error::CaptureAlreadyActive)
		return MSAP1_RPU_STATUS_ADC_STATE;
	if (error != msap1::adc::Error::None)
		return MSAP1_RPU_STATUS_INTERNAL_ERROR;
	return MSAP1_RPU_STATUS_OK;
}

void stop_capture(msap1::adc::AdcController &adc)
{
	adc.stop_capture();
}

} // namespace msap1::r5c0
