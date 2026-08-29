#ifndef MSAP1_R5C1_FLICKER_FRAME_DECODER_HPP
#define MSAP1_R5C1_FLICKER_FRAME_DECODER_HPP

#include "m18_protocol.hpp"

namespace msap1::aggregation {

/** Strict decoder for one CRC32C-protected FLK1 sufficient-statistic packet. */
class FlickerFrameDecoder final {
public:
	[[nodiscard]] FrameValidationError decode(const AggregationFrame &frame,
		FlickerInputView &output) const noexcept;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_FLICKER_FRAME_DECODER_HPP
