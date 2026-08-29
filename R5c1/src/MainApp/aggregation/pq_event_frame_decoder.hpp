#ifndef MSAP1_R5C1_PQ_EVENT_FRAME_DECODER_HPP
#define MSAP1_R5C1_PQ_EVENT_FRAME_DECODER_HPP

#include "m18_protocol.hpp"

namespace msap1::aggregation {

class PqEventFrameDecoder final {
public:
	[[nodiscard]] FrameValidationError decode(const AggregationFrame &frame,
		PqEventInputView &output) const noexcept;
};

} // namespace msap1::aggregation

#endif
