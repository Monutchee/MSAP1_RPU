#ifndef MSAP1_R5C1_AGGREGATION_ENGINE_HPP
#define MSAP1_R5C1_AGGREGATION_ENGINE_HPP

#include "aggregation_health.hpp"
#include "aggregation_protocol.hpp"
#include "aggregation_record_sink.hpp"

#include "aggregation_engine.hpp"

#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/**
 * Controls whether completed R5C1 records are observational or are returned
 * to PL. Production uses emit; shadow remains available only for focused
 * differential tests of the shared fixed-point implementation.
 */
enum class AggregationOutputMode : std::uint8_t {
	shadow,
	emit,
};

/**
 * R5C1 owner of the fixed-point aggregation algorithm.
 *
 * The arithmetic source is owned beside this wrapper. This class owns the
 * firmware lifecycle, fixed-width transport adaptation, record framing,
 * health counters, and optional FIFO publication. It deliberately exposes no
 * HLS types to the rest of the R5 application.
 */
class R5AggregationEngine final {
public:
	R5AggregationEngine(AggregationRecordSink &sink,
		AggregationHealth &health, AggregationOutputMode mode) noexcept;

	bool initialize() noexcept;
	void process(const AggregationInputView &input) noexcept;
	/**
	 * Marks the next accepted single-cycle input as the first cycle after a
	 * transport discontinuity.  The shadow service calls this for failures
	 * that do not carry a usable transport sequence (CRC, FIFO, or ring loss).
	 */
	void note_transport_discontinuity() noexcept;

	[[nodiscard]] AggregationOutputMode output_mode() const noexcept
	{
		return mode_;
	}

private:
	static constexpr std::size_t deferred_pass_count = 8U;

	/**
	 * Run one engine scheduling pass and return the number of complete
	 * records produced by that pass.  A zero result may be either an ordinary
	 * non-boundary cycle or an internal deferred-finalizer preparation pass;
	 * callers must not treat it as a general pending-work indication.
	 */
	[[nodiscard]] std::size_t run_one_pass() noexcept;
	void drain(record_axis_stream_t &stream) noexcept;
	void accept_beat(const record_axis_t &beat) noexcept;
	void complete_record() noexcept;
	void record_completed_family(std::uint32_t format) noexcept;
	void fail_engine() noexcept;
	[[nodiscard]] bool accept_transport_sequence(std::uint32_t sequence,
		bool &discontinuity) noexcept;

	AggregationRecordSink &sink_;
	AggregationHealth &health_;
	AggregationOutputMode mode_;
	hls::stream<single_cycle_word_t> input_{};
	record_axis_stream_t basic_output_{};
	record_axis_stream_t aggregate_output_{};
	AggregationMeterRecord assembling_{};
	std::size_t assembling_words_{};
	std::size_t pass_records_completed_{};
	std::uint32_t last_transport_sequence_{};
	std::uint32_t discontinuity_pending_{};
	bool have_transport_sequence_{};
	bool ready_{};
};

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_ENGINE_HPP
