#ifndef MSAP1_R5C1_FREQUENCY_10S_FRAME_DECODER_HPP
#define MSAP1_R5C1_FREQUENCY_10S_FRAME_DECODER_HPP

#include "frequency_10s_protocol.hpp"

namespace msap1::aggregation {

/** Pure parser for the fixed FRQ1 observation packet. */
class Frequency10sFrameDecoder final {
public:
	[[nodiscard]] FrameValidationError decode(const AggregationFrame &frame,
		Frequency10sInputView &output) const noexcept;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_FREQUENCY_10S_FRAME_DECODER_HPP
