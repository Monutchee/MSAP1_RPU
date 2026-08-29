#ifndef MSAP1_R5C1_MAINS_SIGNAL_FRAME_DECODER_HPP
#define MSAP1_R5C1_MAINS_SIGNAL_FRAME_DECODER_HPP

#include "power_quality_protocol.hpp"

namespace msap1::aggregation {

/** Strict decoder for one CRC32C-protected MCS1 observation packet. */
class MainsSignalFrameDecoder final {
public:
	[[nodiscard]] FrameValidationError decode(const AggregationFrame &frame,
		MainsSignalInputView &output) const noexcept;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_MAINS_SIGNAL_FRAME_DECODER_HPP
