#ifndef MSAP1_R5C1_AGGREGATION_FRAME_DECODER_HPP
#define MSAP1_R5C1_AGGREGATION_FRAME_DECODER_HPP

#include "aggregation_protocol.hpp"

namespace msap1::aggregation {

/** Pure parser for the exact PL-to-R5C1 co-release transport frame. */
class AggregationFrameDecoder final {
public:
	[[nodiscard]] FrameValidationError decode(const AggregationFrame &frame,
		AggregationInputView &output) const noexcept;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_FRAME_DECODER_HPP
