#include "aggregation_frame_decoder.hpp"
#include "aggregation_frame_ring.hpp"
#include "aggregation_health.hpp"
#include "aggregation_record_ring.hpp"
#include "crc32c.hpp"
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

	std::array<aggregation::AggregationMeterRecord, 16U> records{};
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

void test_ring()
{
	aggregation::AggregationFrameRing ring;
	for (std::uint32_t index = 0U;
		index < aggregation::AggregationFrameRing::capacity; ++index)
		expect(ring.try_push(make_frame(index)), "ring accepts capacity");
	expect(!ring.try_push(make_frame(999U)), "ring rejects overflow");
	expect(ring.size() == aggregation::AggregationFrameRing::capacity,
		"ring full size");

	aggregation::AggregationFrame frame{};
	for (std::uint32_t index = 0U;
		index < aggregation::AggregationFrameRing::capacity; ++index) {
		expect(ring.try_pop(frame), "ring pop");
		expect(frame.words[aggregation::AggregationProtocol::payload_index] ==
			index, "ring preserves FIFO order");
	}
	expect(!ring.try_pop(frame), "ring empty");
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
	expect(value.last_fifo_error == 0x55AAU && value.last_frame_length == 12U,
		"last-error context");
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
	constexpr std::uint32_t generation = 0x12345678U;
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
		input.context.control_status = 0x00000F7FU;
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
	test_ring();
	test_output_ring();
	test_health();
	test_r5_engine_emits_complete_basic_family();
	test_r5_engine_fails_closed_when_output_rejects_record();
	test_r5_shadow_mode_is_non_authoritative();
	std::cout << "aggregation shadow tests passed\n";
	return EXIT_SUCCESS;
}
