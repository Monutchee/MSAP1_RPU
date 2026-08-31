#ifndef MSAP1_R5C1_VOLTAGE_SAMPLE_FRAME_DECODER_HPP
#define MSAP1_R5C1_VOLTAGE_SAMPLE_FRAME_DECODER_HPP

#include "power_quality_protocol.hpp"

namespace msap1::aggregation {

/** Strict decoder for one CRC32C-protected VSB1 raw-voltage batch. */
class VoltageSampleFrameDecoder final {
public:
	[[nodiscard]] FrameValidationError decode(const AggregationFrame &frame,
		VoltageSampleInputView &output) const noexcept;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_VOLTAGE_SAMPLE_FRAME_DECODER_HPP
