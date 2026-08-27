#include "aggregation_frame_decoder.hpp"
#include "aggregation_frame_ring.hpp"
#include "aggregation_health.hpp"
#include "aggregation_record_ring.hpp"
#include "aggregation_scheduler_policy.hpp"
#include "crc32c.hpp"
#include "harmonic_aggregation_engine.hpp"
#include "harmonic_frame_decoder.hpp"
#include "r5_aggregation_engine.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string_view>

namespace aggregation = msap1::aggregation;

namespace {

class CapturingRecordSink final : public aggregation::AggregationRecordSink {
public:
	[[nodiscard]] bool publish(
		const aggregation::AggregationMeterRecord &record) noexcept override
	{
		if (count >= records.size())
			return false;
		records[count++] = record;
		return true;
	}

	// Thirty-one Basic families plus two deferred 150/180-cycle families need
	// 132 records. Keep enough headroom for additional low-cadence interval
	// assertions so a test of the engine scheduler cannot fail merely because
	// this capture-only sink filled first.
	std::array<aggregation::AggregationMeterRecord, 256U> records{};
	std::size_t count{};
};

class RejectingRecordSink final : public aggregation::AggregationRecordSink {
public:
	[[nodiscard]] bool publish(
		const aggregation::AggregationMeterRecord &) noexcept override
	{
		++attempts;
		return false;
	}

	std::size_t attempts{};
};

class CountingHarmonicSink final : public aggregation::AggregationRecordSink {
public:
	[[nodiscard]] bool publish(
		const aggregation::AggregationMeterRecord &record) noexcept override
	{
		const auto period = static_cast<std::size_t>(record.words[14U] & 0x3U);
		if (period >= records_by_period.size())
			return false;
		++records_by_period[period];
		if ((record.words[8U] & (1U << 3U)) != 0U)
			++valid_records_by_period[period];
		last_record_by_period[period] = record;
		return true;
	}

	std::array<std::size_t, 4U> records_by_period{};
	std::array<std::size_t, 4U> valid_records_by_period{};
	std::array<aggregation::AggregationMeterRecord, 4U> last_record_by_period{};
};

[[noreturn]] void fail(std::string_view message)
{
	std::cerr << "aggregation shadow test failed: " << message << '\n';
	std::exit(EXIT_FAILURE);
}

void expect(bool condition, std::string_view message)
{
	if (!condition)
		fail(message);
}

aggregation::AggregationFrame make_frame(std::uint32_t sequence = 42U)
{
	aggregation::AggregationFrame frame{};
	frame.word_count = aggregation::AggregationProtocol::frame_words;
	for (std::size_t index = aggregation::AggregationProtocol::payload_index;
		index < aggregation::AggregationProtocol::crc_index; ++index)
		frame.words[index] = static_cast<std::uint32_t>(index * 0x10203U);

	frame.words[aggregation::AggregationProtocol::magic_index] =
		aggregation::AggregationProtocol::magic;
	frame.words[aggregation::AggregationProtocol::contract_revision_index] =
		aggregation::AggregationProtocol::contract_revision;
	frame.words[aggregation::AggregationProtocol::payload_count_index] =
		aggregation::AggregationProtocol::payload_words;
	frame.words[aggregation::AggregationProtocol::transport_sequence_index] =
		sequence;
	frame.words[aggregation::AggregationProtocol::payload_index] = sequence;

	const auto context = aggregation::AggregationProtocol::context_index;
	frame.words[context + 0U] = 0x11223344U;
	frame.words[context + 1U] = 32000U;
	frame.words[context + 2U] = 0x00001FFFU;
	frame.words[context + 3U] = 0x55667788U;
	frame.words[context + 4U] = 0x10203040U;
	frame.words[context + 5U] = 99U;
	frame.words[context + 6U] = 100U;
	frame.words[context + 7U] = 101U;
	frame.words[context + 8U] = 102U;
	frame.words[context + 9U] = 103U;
	frame.words[context + 10U] = 0x89ABCDEFU;
	frame.words[context + 11U] = 0x01234567U;
	frame.words[context + 12U] = 0x00000003U;
	frame.words[aggregation::AggregationProtocol::crc_index] =
		aggregation::crc32c_words(frame.words.data(),
			aggregation::AggregationProtocol::crc_index);
	return frame;
}

void refresh_crc(aggregation::AggregationFrame &frame)
{
	frame.words[aggregation::AggregationProtocol::crc_index] =
		aggregation::crc32c_words(frame.words.data(),
			aggregation::AggregationProtocol::crc_index);
}

void test_crc32c()
{
	constexpr std::string_view check = "123456789";
	expect(aggregation::crc32c_bytes(
		reinterpret_cast<const std::uint8_t *>(check.data()), check.size()) ==
		0xE3069283U, "CRC-32C check vector");
}

void test_valid_frame()
{
	const auto frame = make_frame();
	aggregation::AggregationFrameDecoder decoder;
	aggregation::AggregationInputView input{};
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::none, "valid frame rejected");
	expect(input.sequence == 42U, "sequence decode");
	expect(input.single_cycle_word_count == 221U, "single-cycle extent");
	expect(input.single_cycle_words[0] == 42U, "single-cycle pointer");
	expect(input.context.configuration_generation == 0x11223344U,
		"generation decode");
	expect(input.context.sample_rate_hz == 32000U, "sample rate decode");
	expect(input.context.utc_target_sample == 0x0123456789ABCDEFULL,
		"64-bit UTC target decode");
}

void test_invalid_frames()
{
	aggregation::AggregationFrameDecoder decoder;
	aggregation::AggregationInputView input{};

	auto frame = make_frame();
	frame.words[aggregation::AggregationProtocol::magic_index] ^= 1U;
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::invalid_magic, "magic guard");

	frame = make_frame();
	frame.words[aggregation::AggregationProtocol::contract_revision_index] += 1U;
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::contract_mismatch,
		"fixed co-release contract guard");

	frame = make_frame();
	frame.words[aggregation::AggregationProtocol::payload_count_index] -= 1U;
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::invalid_payload_count,
		"payload count guard");

	frame = make_frame();
	frame.words[aggregation::AggregationProtocol::transport_sequence_index] += 1U;
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::sequence_mismatch,
		"sequence mirror guard");

	frame = make_frame();
	frame.words[20U] ^= 0x80000000U;
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::crc_mismatch, "CRC guard");

	frame = make_frame();
	frame.words[aggregation::AggregationProtocol::context_index + 2U] |=
		0x80000000U;
	refresh_crc(frame);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::reserved_bits_nonzero,
		"control reserved-bit guard");

	frame = make_frame();
	frame.words[aggregation::AggregationProtocol::context_index + 12U] |=
		0x00000004U;
	refresh_crc(frame);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::reserved_bits_nonzero,
		"target reserved-bit guard");
}

aggregation::AggregationFrame make_harmonic_frame(
	std::uint32_t sequence = 1U, std::uint64_t first_sample = 0U,
	std::uint32_t sample_rate_hz = 32000U,
	std::uint32_t sample_count = 6400U)
{
	aggregation::AggregationFrame frame{};
	frame.word_count = aggregation::HarmonicProtocol::frame_words;
	frame.words[aggregation::HarmonicProtocol::magic_index] =
		aggregation::HarmonicProtocol::magic;
	frame.words[aggregation::HarmonicProtocol::contract_revision_index] =
		aggregation::HarmonicProtocol::contract_revision;
	frame.words[aggregation::HarmonicProtocol::payload_count_index] =
		aggregation::HarmonicProtocol::payload_words;
	frame.words[aggregation::HarmonicProtocol::transport_sequence_index] =
		sequence;

	for (std::size_t channel = 0U;
	     channel < aggregation::HarmonicProtocol::channels; ++channel) {
		for (std::size_t chunk = 0U;
		     chunk < aggregation::HarmonicProtocol::chunks_per_channel;
		     ++chunk) {
			const auto record_index = channel *
				aggregation::HarmonicProtocol::chunks_per_channel + chunk;
			auto *record = frame.words.data() +
				aggregation::HarmonicProtocol::payload_index + record_index *
					aggregation::HarmonicProtocol::record_words;
			record[0U] = aggregation::HarmonicProtocol::record_magic;
			record[1U] = aggregation::HarmonicProtocol::base_record_format;
			record[2U] = aggregation::HarmonicProtocol::record_bytes;
			record[3U] = sequence;
			record[4U] = 0x12345678U;
			record[5U] = sample_rate_hz;
			record[6U] = sample_count;
			record[7U] = 0x7FU;
			record[8U] = 0x3EU;
			record[9U] = static_cast<std::uint32_t>(first_sample);
			record[10U] = static_cast<std::uint32_t>(first_sample >> 32U);
			const auto first_order = chunk *
				aggregation::HarmonicProtocol::orders_per_chunk + 1U;
			const auto count = std::min(
				aggregation::HarmonicProtocol::orders_per_chunk,
				aggregation::HarmonicProtocol::maximum_order - first_order + 1U);
			record[13U] = static_cast<std::uint32_t>(channel) |
				(static_cast<std::uint32_t>(chunk) << 3U) |
				(static_cast<std::uint32_t>(first_order) << 7U) |
				(static_cast<std::uint32_t>(count) << 15U) |
				(static_cast<std::uint32_t>(
					aggregation::HarmonicProtocol::chunks_per_channel) << 20U) |
				(static_cast<std::uint32_t>(
					aggregation::HarmonicProtocol::maximum_order) << 24U);
			record[14U] = 50000U;
			record[15U] = 127U | (50U << 8U) | (10U << 16U) |
				(3U << 24U);
			for (std::size_t entry = 0U; entry < count; ++entry) {
				const auto order = first_order + entry;
				const std::uint64_t magnitude =
					channel * 1000000U + order;
				const std::uint64_t angle = order * 1000U;
				const auto packed = magnitude | (angle << 40U) |
					(std::uint64_t{1} << 60U) |
					(std::uint64_t{1} << 61U);
				record[16U + entry * 2U] =
					static_cast<std::uint32_t>(packed);
				record[17U + entry * 2U] =
					static_cast<std::uint32_t>(packed >> 32U);
			}
		}
	}
	frame.words[aggregation::HarmonicProtocol::crc_index] =
		aggregation::crc32c_words(frame.words.data(),
			aggregation::HarmonicProtocol::crc_index);
	return frame;
}

void refresh_harmonic_crc(aggregation::AggregationFrame &frame)
{
	frame.words[aggregation::HarmonicProtocol::crc_index] =
		aggregation::crc32c_words(frame.words.data(),
			aggregation::HarmonicProtocol::crc_index);
}

void set_last_harmonic_magnitude(aggregation::AggregationFrame &frame,
	std::uint64_t magnitude)
{
	const auto last_record = aggregation::HarmonicProtocol::payload_index +
		(aggregation::HarmonicProtocol::records_per_family - 1U) *
			aggregation::HarmonicProtocol::record_words;
	const auto last_entry_word = last_record + 16U + 6U * 2U;
	const auto packed = magnitude | (std::uint64_t{1} << 60U);
	frame.words[last_entry_word] = static_cast<std::uint32_t>(packed);
	frame.words[last_entry_word + 1U] =
		static_cast<std::uint32_t>(packed >> 32U);
	refresh_harmonic_crc(frame);
}

void test_harmonic_frame_decoder()
{
	aggregation::HarmonicFrameDecoder decoder;
	aggregation::HarmonicInputView input{};
	auto frame = make_harmonic_frame(7U, 0x1000U);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::none,
		"valid HRM1 family rejected");
	expect(input.sequence == 7U && input.first_sample == 0x1000U &&
		input.records != nullptr,
		"HRM1 family provenance decode");

	frame.words[20U] ^= 1U;
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::crc_mismatch,
		"HRM1 CRC guard");

	frame = make_harmonic_frame();
	const auto second_record = aggregation::HarmonicProtocol::payload_index +
		aggregation::HarmonicProtocol::record_words;
	frame.words[second_record + 5U] = 64000U;
	refresh_harmonic_crc(frame);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::provenance_mismatch,
		"HRM1 cross-record provenance guard");
}

void test_harmonic_engine_emits_complete_three_second_family()
{
	constexpr std::uint64_t large_magnitude = 0xFEDCBA9876ULL;
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::HarmonicFrameDecoder decoder;
	aggregation::HarmonicAggregationEngine engine(sink, health);
	expect(engine.initialize(), "harmonic aggregation engine initialization");

	for (std::uint32_t index = 0U; index < 15U; ++index) {
		auto frame = make_harmonic_frame(index + 1U,
			static_cast<std::uint64_t>(index) * 6400U);
		set_last_harmonic_magnitude(frame, large_magnitude);
		aggregation::HarmonicInputView input{};
		expect(decoder.decode(frame, input) ==
			aggregation::FrameValidationError::none,
			"three-second source family decode");
		engine.process(input);
	}

	expect(engine.ready(), "harmonic aggregation engine remains ready");
	expect(sink.count == aggregation::HarmonicProtocol::records_per_family,
		"fifteen base spectra emit one 42-record three-second family");
	for (std::size_t index = 0U; index < sink.count; ++index) {
		const auto &record = sink.records[index];
		expect(record.words[0U] == aggregation::HarmonicProtocol::record_magic &&
			record.words[1U] ==
				aggregation::HarmonicProtocol::aggregate_record_format &&
			record.words[2U] == aggregation::HarmonicProtocol::record_bytes,
			"aggregate harmonic record envelope");
		expect((record.words[14U] & 0x3U) == 1U &&
			((record.words[14U] >> 2U) & 0xFFFU) == 15U,
			"aggregate harmonic period and contributor count");
		expect(record.words[62U] == 1U && record.words[63U] == 15U,
			"aggregate harmonic base-family provenance");
	}
	const auto &last = sink.records[sink.count - 1U];
	const auto packed = static_cast<std::uint64_t>(last.words[38U]) |
		(static_cast<std::uint64_t>(last.words[39U]) << 32U);
	expect((packed & ((std::uint64_t{1} << 40U) - 1U)) == large_magnitude &&
		(packed & (std::uint64_t{1} << 60U)) != 0U,
		"aggregate harmonic RMS preserves a full-width 40-bit magnitude");
}

void test_harmonic_engine_resets_partial_tiers_in_place()
{
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::HarmonicFrameDecoder decoder;
	aggregation::HarmonicAggregationEngine engine(sink, health);
	expect(engine.initialize(), "harmonic reset test initialization");

	/* Leave dirty state in every active three-second array, then verify that
	 * reinitialization removes it without contributing an early family. */
	for (std::uint32_t index = 0U; index < 7U; ++index) {
		const auto frame = make_harmonic_frame(index + 1U,
			static_cast<std::uint64_t>(index) * 6400U);
		aggregation::HarmonicInputView input{};
		expect(decoder.decode(frame, input) ==
			aggregation::FrameValidationError::none,
			"pre-reinitialization harmonic decode");
		engine.process(input);
	}
	expect(engine.initialize(), "harmonic engine reinitialization");

	/* Dirty a second partial tier, then force the normal discontinuity reset.
	 * Only the fifteen records after that boundary may form the output. */
	for (std::uint32_t index = 0U; index < 7U; ++index) {
		const auto frame = make_harmonic_frame(100U + index,
			static_cast<std::uint64_t>(index) * 6400U);
		aggregation::HarmonicInputView input{};
		expect(decoder.decode(frame, input) ==
			aggregation::FrameValidationError::none,
			"pre-discontinuity harmonic decode");
		engine.process(input);
	}
	engine.note_transport_discontinuity();
	for (std::uint32_t index = 0U; index < 15U; ++index) {
		const auto frame = make_harmonic_frame(200U + index,
			static_cast<std::uint64_t>(index) * 6400U);
		aggregation::HarmonicInputView input{};
		expect(decoder.decode(frame, input) ==
			aggregation::FrameValidationError::none,
			"post-discontinuity harmonic decode");
		engine.process(input);
	}

	expect(sink.count == aggregation::HarmonicProtocol::records_per_family,
		"partial harmonic tiers survived an in-place reset");
	expect(sink.records.front().words[62U] == 200U &&
		sink.records[sink.count - 1U].words[63U] == 214U,
		"reset harmonic family retained stale provenance");
	expect((sink.records.front().words[8U] & (1U << 6U)) != 0U,
		"post-discontinuity harmonic family lost its boundary marker");
}

void test_harmonic_engine_emits_clean_ten_minute_and_two_hour_families()
{
	CountingHarmonicSink sink;
	aggregation::AggregationHealth health;
	aggregation::HarmonicFrameDecoder decoder;
	aggregation::HarmonicAggregationEngine engine(sink, health);
	expect(engine.initialize(), "long-interval harmonic engine initialization");

	/* Accelerate the exact interval state machine without changing it: each
	 * synthetic base family spans 40 samples at 1 sample/s, so fifteen inputs
	 * reach one 600-second boundary. The first interval is deliberately
	 * contaminated; twelve subsequent clean intervals form one 2-hour result. */
	aggregation::AggregationContext timing{};
	timing.utc_target_sample = 600U;
	timing.utc_target_status = 1U;
	engine.observe_timing_context(timing);

	for (std::uint32_t index = 0U; index < 195U; ++index) {
		const auto frame = make_harmonic_frame(index + 1U,
			static_cast<std::uint64_t>(index) * 40U, 1U, 40U);
		aggregation::HarmonicInputView input{};
		expect(decoder.decode(frame, input) ==
			aggregation::FrameValidationError::none,
			"long-interval source family decode");
		engine.process(input);
	}

	constexpr auto family_records =
		aggregation::HarmonicProtocol::records_per_family;
	expect(engine.ready(), "long-interval harmonic engine remains ready");
	expect(sink.records_by_period[1U] == 13U * family_records,
		"long test emits every 15-family spectrum");
	expect(sink.records_by_period[2U] == 13U * family_records &&
		sink.valid_records_by_period[2U] == 12U * family_records,
		"first 10-minute family is contaminated and twelve are clean");
	expect(sink.records_by_period[3U] == family_records &&
		sink.valid_records_by_period[3U] == family_records,
		"twelve clean 10-minute families emit one valid 2-hour family");

	const auto &two_hour = sink.last_record_by_period[3U];
	expect(((two_hour.words[14U] >> 2U) & 0xFFFU) == 12U &&
		two_hour.words[11U] == 7800U && two_hour.words[12U] == 0U,
		"2-hour contributor count and aligned target");
	expect(two_hour.words[62U] == 16U && two_hour.words[63U] == 195U,
		"2-hour family preserves exact base-family provenance");
}

void test_ring()
{
	aggregation::AggregationFrameRing ring;
	expect(ring.available_capacity() ==
		aggregation::AggregationFrameRing::capacity,
		"empty ring capacity");
	for (std::uint32_t index = 0U;
		index < aggregation::AggregationFrameRing::capacity; ++index)
		expect(ring.try_push(make_frame(index)), "ring accepts capacity");
	expect(!ring.try_push(make_frame(999U)), "ring rejects overflow");
	expect(ring.size() == aggregation::AggregationFrameRing::capacity,
		"ring full size");
	expect(ring.available_capacity() == 0U, "full ring capacity");

	aggregation::AggregationFrame frame{};
	expect(ring.try_pop(frame), "ring first pop");
	expect(frame.words[aggregation::AggregationProtocol::payload_index] == 0U,
		"ring first FIFO value");
	expect(ring.available_capacity() == 1U, "released ring capacity");
	for (std::uint32_t index = 1U;
		index < aggregation::AggregationFrameRing::capacity; ++index) {
		expect(ring.try_pop(frame), "ring pop");
		expect(frame.words[aggregation::AggregationProtocol::payload_index] ==
			index, "ring preserves FIFO order");
	}
	expect(!ring.try_pop(frame), "ring empty");
	expect(ring.available_capacity() ==
		aggregation::AggregationFrameRing::capacity,
		"restored ring capacity");
}

void test_output_ring()
{
	aggregation::AggregationRecordRing ring;
	for (std::uint32_t index = 0U;
		index < aggregation::AggregationRecordRing::capacity; ++index) {
		aggregation::AggregationMeterRecord record{};
		record.sequence = index;
		record.words[0] = 0x3152544DU;
		record.words[3] = index;
		expect(ring.try_push(record), "output ring accepts capacity");
	}
	aggregation::AggregationMeterRecord overflow{};
	expect(!ring.try_push(overflow), "output ring rejects overflow");

	for (std::uint32_t index = 0U;
		index < aggregation::AggregationRecordRing::capacity; ++index) {
		aggregation::AggregationMeterRecord record{};
		expect(ring.try_pop(record), "output ring pop");
		expect(record.sequence == index && record.words[3] == index,
			"output ring preserves record and metadata");
	}
	expect(!ring.try_pop(overflow), "output ring empty");

	// Exercise more than one complete index wrap.  The production TX task now
	// drains records concurrently with the validator, so the SPSC ring must be
	// reusable indefinitely rather than only across its first 64 slots.
	for (std::uint32_t index = 0U;
		index < aggregation::AggregationRecordRing::capacity * 4U; ++index) {
		aggregation::AggregationMeterRecord pushed{};
		pushed.sequence = index + 1000U;
		expect(ring.try_push(pushed), "output ring accepts wrapped push");
		aggregation::AggregationMeterRecord popped{};
		expect(ring.try_pop(popped), "output ring accepts wrapped pop");
		expect(popped.sequence == pushed.sequence,
			"output ring preserves wrapped record order");
	}
}

void test_health()
{
	aggregation::AggregationHealth health;
	health.set_transport_available(true);
	health.set_transport_initialized(true);
	health.record_received();
	health.record_sequence(0xFFFFFFFEU);
	health.record_sequence(0xFFFFFFFFU);
	health.record_sequence(1U); // Missing sequence zero across wrap.
	health.record_sequence(1U); // Repeated frame.
	health.record_sequence(0U); // Older than the expected sequence.
	health.record_valid(1U);
	health.record_invalid(
		aggregation::FrameValidationError::reserved_bits_nonzero);
	health.record_ring_overflow();
	health.record_fifo_error(0x55AAU);
	health.record_length_error(12U);

	const auto value = health.snapshot();
	expect(value.transport_available && value.transport_initialized,
		"transport health");
	expect(value.frames_received == 1U && value.frames_valid == 1U &&
		value.frames_invalid == 1U, "frame counters");
	expect(value.sequence_gaps == 1U, "wrap-aware gap counter");
	expect(value.repeated_frames == 1U, "repeat counter");
	expect(value.out_of_order_frames == 1U, "out-of-order counter");
	expect(value.format_errors == 1U, "format counter");
	expect(value.ring_overflows == 1U && value.fifo_errors == 1U &&
		value.length_errors == 1U, "transport counters");
	expect(value.software_ring_push_failures == 1U,
		"software-ring push failure counter");
	expect(value.input_records_dropped == 1U &&
		value.first_dropped_sequence == 0U &&
		value.last_dropped_sequence == 0U,
		"deterministic dropped-sequence telemetry");
	expect(value.last_fifo_error == 0x55AAU && value.last_frame_length == 12U,
		"last-error context");
}

void test_ring_pressure_telemetry()
{
	using aggregation::RingPressureLevel;

	expect(aggregation::classify_ring_pressure(0U, 64U) ==
		RingPressureLevel::normal, "empty ring pressure");
	expect(aggregation::classify_ring_pressure(31U, 64U) ==
		RingPressureLevel::normal, "below-warning ring pressure");
	expect(aggregation::classify_ring_pressure(32U, 64U) ==
		RingPressureLevel::warning, "warning ring pressure boundary");
	expect(aggregation::classify_ring_pressure(48U, 64U) ==
		RingPressureLevel::high, "high ring pressure boundary");
	expect(aggregation::classify_ring_pressure(57U, 64U) ==
		RingPressureLevel::high, "below-critical ring pressure");
	expect(aggregation::classify_ring_pressure(58U, 64U) ==
		RingPressureLevel::critical, "critical ring pressure boundary");
	expect(aggregation::classify_ring_pressure(63U, 64U) ==
		RingPressureLevel::critical, "below-full ring pressure");
	expect(aggregation::classify_ring_pressure(64U, 64U) ==
		RingPressureLevel::full, "full ring pressure boundary");

	aggregation::AggregationHealth health;
	health.observe_software_ring(31U, 64U);
	health.observe_software_ring(32U, 64U);
	health.observe_software_ring(48U, 64U);
	health.observe_software_ring(58U, 64U);
	health.observe_software_ring(64U, 64U);
	// Returning below a threshold rearms that threshold's edge counter.
	health.observe_software_ring(1U, 64U);
	health.observe_software_ring(32U, 64U);
	health.observe_hardware_fifo(12U);
	health.observe_hardware_fifo(7U);
	health.record_hardware_fifo_full_events(2U);
	health.record_input_activation(4U, 10U);
	health.record_input_activation(2U, 20U);
	health.record_validator_activation(4U, 7U, 100U);
	health.record_validator_activation(2U, 11U, 80U);

	const auto value = health.snapshot();
	expect(value.software_ring_current == 32U &&
		value.software_ring_capacity == 64U &&
		value.software_ring_high_water == 64U,
		"software-ring occupancy telemetry");
	expect(value.software_ring_pressure == RingPressureLevel::warning,
		"software-ring current pressure");
	expect(value.software_ring_warning_entries == 2U &&
		value.software_ring_high_entries == 1U &&
		value.software_ring_critical_entries == 1U &&
		value.software_ring_full_entries == 1U,
		"software-ring edge counters");
	expect(value.hardware_fifo_current_words == 7U &&
		value.hardware_fifo_high_water_words == 12U,
		"hardware FIFO occupancy telemetry");
	expect(value.hardware_fifo_full_events == 2U,
		"hardware FIFO programmable-full telemetry");
	expect(value.input_wake_count == 2U &&
		value.input_records_processed == 6U &&
		value.input_max_batch == 4U &&
		value.input_max_runtime_us == 20U,
		"input-task activation telemetry");
	expect(value.validator_wake_count == 2U &&
		value.validator_records_processed == 6U &&
		value.validator_max_runtime_us == 11U &&
		value.validator_max_schedule_gap_us == 100U,
		"validator-task activation telemetry");
}

void test_bounded_input_handoff_preserves_validator_progress()
{
	constexpr std::uint32_t backlog = 256U;
	constexpr std::uint32_t ring_capacity = 64U;
	static_assert(
		aggregation::scheduler_policy::maximum_input_batch == 4U,
		"production input drain must remain bounded to four packets");

	std::uint32_t hardware_pending = backlog;
	std::uint32_t software_pending = 0U;
	std::uint32_t validated = 0U;
	std::uint32_t input_activations = 0U;
	std::uint32_t validator_activations = 0U;
	std::uint32_t ring_high_water = 0U;

	while (validated < backlog) {
		const auto available = ring_capacity - software_pending;
		const auto queued = hardware_pending <
			aggregation::scheduler_policy::maximum_input_batch ?
			hardware_pending :
			aggregation::scheduler_policy::maximum_input_batch;
		const auto accepted = queued < available ? queued : available;
		hardware_pending -= accepted;
		software_pending += accepted;
		if (software_pending > ring_high_water)
			ring_high_water = software_pending;
		++input_activations;

		// The production one-tick block after each bounded input batch gives
		// the validator a real scheduling point even though it has lower
		// priority than the FIFO task.
		validated += software_pending;
		software_pending = 0U;
		++validator_activations;
	}

	expect(hardware_pending == 0U && software_pending == 0U &&
		validated == backlog, "bounded scheduler drains the complete backlog");
	expect(input_activations == validator_activations,
		"validator runs after every bounded input activation");
	expect(ring_high_water <=
		aggregation::scheduler_policy::maximum_input_batch,
		"bounded input handoff prevents software-ring accumulation");
}

std::array<std::uint32_t, aggregation::AggregationProtocol::single_cycle_words>
make_zero_cycle(std::uint32_t sequence, std::uint32_t generation,
	std::uint64_t first_sample, std::uint32_t sample_count)
{
	std::array<std::uint32_t,
		aggregation::AggregationProtocol::single_cycle_words> words{};
	words[0] = sequence;
	words[1] = generation;
	words[2] = static_cast<std::uint32_t>(first_sample);
	words[3] = static_cast<std::uint32_t>(first_sample >> 32U);
	const auto last_sample = first_sample + sample_count - 1U;
	words[4] = static_cast<std::uint32_t>(last_sample);
	words[5] = static_cast<std::uint32_t>(last_sample >> 32U);
	words[6] = sample_count;
	words[7] = sequence;
	// 60 Hz, channels 0..6 valid, and grid timing locked.
	words[8] = 60U | (0x7FU << 8U) | (1U << 16U);
	words[10] = 60000U;
	words[11] = 1U | (1U << 1U); // Frequency valid and APPLY level one.
	words[12] = sequence * 1000U;
	return words;
}

void test_r5_engine_emits_complete_basic_family()
{
	constexpr std::uint32_t generation = 0x12345678U;
	constexpr std::uint32_t samples_per_cycle = 533U;
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::R5AggregationEngine engine(sink, health,
		aggregation::AggregationOutputMode::emit);
	expect(engine.initialize(), "R5 aggregation engine initialization");

	std::uint64_t first_sample = 0U;
	for (std::uint32_t cycle = 1U; cycle <= 12U; ++cycle) {
		auto words = make_zero_cycle(cycle, generation, first_sample,
			samples_per_cycle);
		aggregation::AggregationInputView input{};
		input.sequence = cycle;
		input.single_cycle_words = words.data();
		input.single_cycle_word_count = words.size();
		input.context.configuration_generation = generation;
		input.context.sample_rate_hz = 32000U;
		// Channels 0..6, enable, DC removal, APPLY=1, timing locked.
		input.context.control_status = 0x00000F7FU;
		input.context.frequency_status = 0x2U;
		input.context.frequency_period_q16 = 0x00010000U;
		input.context.frequency_sequence = cycle;
		engine.process(input);
		first_sample += samples_per_cycle;
	}

	expect(sink.count == 4U,
		"12 cycles must emit one four-record Basic family");
	constexpr std::array<std::uint32_t, 4U> expected_formats = {
		MREC_FORMAT_BASIC_V4,
		MREC_FORMAT_POWER_V1,
		MREC_FORMAT_PHASOR_V2,
		MREC_FORMAT_UNBAL_V2,
	};
	for (std::size_t index = 0U; index < sink.count; ++index) {
		const auto &record = sink.records[index];
		expect(record.words[MREC_MAGIC_WORD] == MREC_MAGIC,
			"R5 record magic");
		expect(record.words[MREC_SIZE_WORD] ==
			aggregation::AggregationMeterRecord::byte_count,
			"R5 record byte count");
		expect(record.words[MREC_FORMAT_WORD] == expected_formats[index],
			"R5 Basic-family record order and format");
		expect(record.words[MREC_SEQUENCE_WORD] == 1U,
			"R5 Basic-family sequence");
	}

	const auto status = health.snapshot();
	expect(status.engine_ready, "R5 aggregation engine remains ready");
	expect(status.authoritative,
		"emit-mode R5 aggregation engine reports authoritative");
	expect(status.basic_completed == 1U,
		"R5 health counts the completed Basic measurement record");
}

void test_r5_engine_fails_closed_when_output_rejects_record()
{
	constexpr std::uint32_t generation = 0x22345678U;
	constexpr std::uint32_t samples_per_cycle = 533U;
	RejectingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::R5AggregationEngine engine(sink, health,
		aggregation::AggregationOutputMode::emit);
	expect(engine.initialize(), "rejecting engine initialization");

	std::uint64_t first_sample = 0U;
	for (std::uint32_t cycle = 1U; cycle <= 12U; ++cycle) {
		auto words = make_zero_cycle(cycle, generation, first_sample,
			samples_per_cycle);
		aggregation::AggregationInputView input{};
		input.sequence = cycle;
		input.single_cycle_words = words.data();
		input.single_cycle_word_count = words.size();
		input.context.configuration_generation = generation;
		input.context.sample_rate_hz = 32000U;
		// Toggle APPLY low for this independent engine instance.  The host-side
		// HLS model intentionally retains its static state between calls, just as
		// the synthesized engine does; changing only the generation is not an
		// APPLY event.
		input.context.control_status =
			0x00000F7FU & ~(1U << AGG_CONTEXT_APPLY_BIT);
		input.context.frequency_status = 0x2U;
		engine.process(input);
		first_sample += samples_per_cycle;
	}

	expect(sink.attempts == 1U,
		"first rejected authoritative record stops publication");
	const auto status = health.snapshot();
	expect(status.authoritative,
		"failed emit-mode engine remains identified as authoritative");
	expect(!status.engine_ready,
		"authoritative output rejection fails the engine closed");
}

void test_r5_engine_drains_deferred_aggregate_family()
{
	constexpr std::uint32_t generation = 0x32345678U;
	constexpr std::uint32_t samples_per_cycle = 533U;
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::R5AggregationEngine engine(sink, health,
		aggregation::AggregationOutputMode::emit);
	expect(engine.initialize(), "deferred aggregate engine initialization");

	std::uint64_t first_sample = 0U;
	// The first Basic block after initialization carries FIRST_BLOCK and is
	// intentionally ineligible for the normative 150/180-cycle interval. Feed
	// one startup block plus two groups of fifteen consecutive eligible blocks.
	// Requiring the second aggregate prevents a scheduler regression where the
	// first deferred family drains but later Basic boundaries stop advancing.
	for (std::uint32_t cycle = 1U; cycle <= 372U; ++cycle) {
		auto words = make_zero_cycle(cycle, generation, first_sample,
			samples_per_cycle);
		aggregation::AggregationInputView input{};
		input.sequence = cycle;
		input.single_cycle_words = words.data();
		input.single_cycle_word_count = words.size();
		input.context.configuration_generation = generation;
		input.context.sample_rate_hz = 32000U;
		input.context.control_status = 0x00000F7FU;
		input.context.frequency_status = 0x2U;
		engine.process(input);
		first_sample += samples_per_cycle;
	}

	const auto status = health.snapshot();
	expect(status.engine_ready,
		"deferred aggregate processing keeps the engine ready");
	expect(status.basic_completed == 31U,
		"372 cycles must complete one startup and thirty eligible Basic measurements");
	expect(status.aggregate_completed == 2U,
		"each group of fifteen eligible Basic blocks must drain an aggregate");
}

void test_r5_shadow_mode_is_non_authoritative()
{
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::R5AggregationEngine engine(sink, health,
		aggregation::AggregationOutputMode::shadow);
	expect(engine.initialize(), "shadow engine initialization");
	const auto status = health.snapshot();
	expect(status.engine_ready && !status.authoritative,
		"shadow engine is ready but non-authoritative");
}

} // namespace

int main()
{
	test_crc32c();
	test_valid_frame();
	test_invalid_frames();
	test_harmonic_frame_decoder();
	test_harmonic_engine_emits_complete_three_second_family();
	test_harmonic_engine_resets_partial_tiers_in_place();
	test_harmonic_engine_emits_clean_ten_minute_and_two_hour_families();
	test_ring();
	test_output_ring();
	test_health();
	test_ring_pressure_telemetry();
	test_bounded_input_handoff_preserves_validator_progress();
	test_r5_engine_emits_complete_basic_family();
	test_r5_engine_fails_closed_when_output_rejects_record();
	test_r5_engine_drains_deferred_aggregate_family();
	test_r5_shadow_mode_is_non_authoritative();
	std::cout << "aggregation shadow tests passed\n";
	return EXIT_SUCCESS;
}
