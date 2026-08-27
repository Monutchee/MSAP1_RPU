#ifndef MSAP1_R5C1_HARMONIC_FRAME_DECODER_HPP
#define MSAP1_R5C1_HARMONIC_FRAME_DECODER_HPP

#include "harmonic_protocol.hpp"

namespace msap1::aggregation {

/** Pure CRC, record-geometry, validity, and family-provenance validator. */
class HarmonicFrameDecoder final {
public:
	[[nodiscard]] FrameValidationError decode(const AggregationFrame &frame,
		HarmonicInputView &output) const noexcept;
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_HARMONIC_FRAME_DECODER_HPP
