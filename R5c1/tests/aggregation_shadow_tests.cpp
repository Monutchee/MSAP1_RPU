#include "aggregation_frame_decoder.hpp"
#include "aggregation_frame_ring.hpp"
#include "aggregation_health.hpp"
#include "aggregation_record_ring.hpp"
#include "aggregation_scheduler_policy.hpp"
#include "crc32c.hpp"
#include "flicker_engine.hpp"
#include "flicker_frame_decoder.hpp"
#include "harmonic_aggregation_engine.hpp"
#include "harmonic_frame_decoder.hpp"
#include "pq_event_frame_decoder.hpp"
#include "pq_event_lifecycle_engine.hpp"
#include "r5_aggregation_engine.hpp"
#include "r5_session_id.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <type_traits>

namespace aggregation = msap1::aggregation;

static_assert(!std::is_copy_constructible_v<aggregation::EnergyDemandEngine>);
static_assert(!std::is_copy_assignable_v<aggregation::EnergyDemandEngine>);
static_assert(!std::is_move_constructible_v<aggregation::EnergyDemandEngine>);
static_assert(!std::is_move_assignable_v<aggregation::EnergyDemandEngine>);

namespace {

void expect(bool condition, std::string_view message);

void test_session_id_uses_boot_varying_counter()
{
	constexpr aggregation::R5SessionEntropy first{
		.shared_system_counter = 0x0123456789abcdefULL,
		.local_cycle_counter_before = 0x1234U,
		.local_cycle_counter_after = 0x5678U,
	};
	constexpr auto second = aggregation::R5SessionEntropy{
		.shared_system_counter = first.shared_system_counter + 1U,
		.local_cycle_counter_before = first.local_cycle_counter_before,
		.local_cycle_counter_after = first.local_cycle_counter_after,
	};
	constexpr auto first_id = aggregation::derive_r5_session_id(first);
	constexpr auto second_id = aggregation::derive_r5_session_id(second);
	static_assert(first_id != 0U && second_id != 0U);
	static_assert(first_id != second_id);
	expect(first_id != second_id,
		"SoC-wide boot counter must change the R5C1 session ID");
}

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

	// Thirty-one six-record Basic/Energy families plus two deferred
	// 150/180-cycle families fit below 256 records. Keep enough headroom for
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

class LatestEnergyRecordSink final : public aggregation::AggregationRecordSink {
public:
	[[nodiscard]] bool publish(
		const aggregation::AggregationMeterRecord &record) noexcept override
	{
		if (record.words[MREC_FORMAT_WORD] != MREC_FORMAT_ENERGY_V1)
			return false;
		if ((record.words[MREC_FORMAT_HEADER_WORD] & 0x3U) ==
		    ENERGY_PART_SUMMARY)
			summary = record;
		else
			quadrants = record;
		++count;
		return true;
	}

	aggregation::AggregationMeterRecord summary{};
	aggregation::AggregationMeterRecord quadrants{};
	std::size_t count{};
};

class CountingHarmonicSink final : public aggregation::AggregationRecordSink {
public:
	[[nodiscard]] bool publish(
		const aggregation::AggregationMeterRecord &record) noexcept override
	{
		const auto period = static_cast<std::size_t>(record.words[14U] & 0x3U);
		if (period >= records_by_period.size())
			return false;
		if (records_by_period[period] == 0U)
			first_record_by_period[period] = record;
		++records_by_period[period];
		if ((record.words[8U] & (1U << 3U)) != 0U)
			++valid_records_by_period[period];
		last_record_by_period[period] = record;
		return true;
	}

	std::array<std::size_t, 4U> records_by_period{};
	std::array<std::size_t, 4U> valid_records_by_period{};
	std::array<aggregation::AggregationMeterRecord, 4U> first_record_by_period{};
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

std::size_t harmonic_entry_word(std::size_t channel, std::size_t order)
{
	const auto order_index = order - 1U;
	const auto chunk = order_index /
		aggregation::HarmonicProtocol::orders_per_chunk;
	const auto entry = order_index %
		aggregation::HarmonicProtocol::orders_per_chunk;
	const auto record = aggregation::HarmonicProtocol::payload_index +
		(channel * aggregation::HarmonicProtocol::chunks_per_channel + chunk) *
			aggregation::HarmonicProtocol::record_words;
	return record + 16U + entry * 2U;
}

std::uint64_t harmonic_entry(const aggregation::AggregationFrame &frame,
	std::size_t channel, std::size_t order)
{
	const auto word = harmonic_entry_word(channel, order);
	return static_cast<std::uint64_t>(frame.words[word]) |
		(static_cast<std::uint64_t>(frame.words[word + 1U]) << 32U);
}

void set_harmonic_angle(aggregation::AggregationFrame &frame,
	std::size_t channel, std::size_t order, std::uint32_t angle_millidegrees,
	bool valid)
{
	constexpr auto angle_field_mask =
		((std::uint64_t{1} << 20U) - 1U) << 40U;
	const auto word = harmonic_entry_word(channel, order);
	auto packed = harmonic_entry(frame, channel, order);
	packed &= ~(angle_field_mask | (std::uint64_t{1} << 61U));
	if (valid) {
		packed |= static_cast<std::uint64_t>(angle_millidegrees) << 40U;
		packed |= std::uint64_t{1} << 61U;
	}
	frame.words[word] = static_cast<std::uint32_t>(packed);
	frame.words[word + 1U] = static_cast<std::uint32_t>(packed >> 32U);
	refresh_harmonic_crc(frame);
}

std::uint64_t aggregate_entry(
	const aggregation::AggregationMeterRecord &record, std::size_t entry)
{
	const auto word = 16U + entry * 2U;
	return static_cast<std::uint64_t>(record.words[word]) |
		(static_cast<std::uint64_t>(record.words[word + 1U]) << 32U);
}

bool angle_near(std::uint32_t actual, std::uint32_t expected,
	std::uint32_t tolerance)
{
	const auto direct = actual > expected ? actual - expected : expected - actual;
	const auto circular = 360000U - direct;
	return std::min(direct, circular) <= tolerance;
}

void set_last_harmonic_magnitude(aggregation::AggregationFrame &frame,
	std::uint64_t magnitude)
{
	constexpr auto magnitude_mask = (std::uint64_t{1} << 40U) - 1U;
	const auto last_entry_word = harmonic_entry_word(6U, 127U);
	const auto packed =
		(harmonic_entry(frame, 6U, 127U) & ~magnitude_mask) |
		magnitude | (std::uint64_t{1} << 60U);
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
		(packed & (std::uint64_t{1} << 60U)) != 0U &&
		(packed & (std::uint64_t{1} << 61U)) != 0U &&
		angle_near(static_cast<std::uint32_t>((packed >> 40U) & 0xFFFFFU),
			127000U, 2U),
		"aggregate harmonic preserves full-width magnitude and circular angle");
}

void test_harmonic_engine_uses_circular_angles_and_propagates_validity()
{
	constexpr std::array<std::uint32_t, 3U> wrap_angles{
		359000U, 1000U, 0U};
	{
		CapturingRecordSink sink;
		aggregation::AggregationHealth health;
		aggregation::HarmonicFrameDecoder decoder;
		aggregation::HarmonicAggregationEngine engine(sink, health);
		expect(engine.initialize(), "circular harmonic test initialization");
		for (std::uint32_t index = 0U; index < 15U; ++index) {
			auto frame = make_harmonic_frame(index + 1U,
				static_cast<std::uint64_t>(index) * 6400U);
			set_harmonic_angle(frame, 0U, 2U,
				wrap_angles[index % wrap_angles.size()], true);
			aggregation::HarmonicInputView input{};
			expect(decoder.decode(frame, input) ==
				aggregation::FrameValidationError::none,
				"circular harmonic source decode");
			engine.process(input);
		}
		const auto packed = aggregate_entry(sink.records.front(), 1U);
		expect((packed & (std::uint64_t{1} << 61U)) != 0U &&
			angle_near(static_cast<std::uint32_t>(
				(packed >> 40U) & 0xFFFFFU), 0U, 2U),
			"359/1-degree inputs did not aggregate across the circular wrap");
	}

	{
		CapturingRecordSink sink;
		aggregation::AggregationHealth health;
		aggregation::HarmonicFrameDecoder decoder;
		aggregation::HarmonicAggregationEngine engine(sink, health);
		expect(engine.initialize(), "invalid-angle harmonic test initialization");
		for (std::uint32_t index = 0U; index < 15U; ++index) {
			auto frame = make_harmonic_frame(index + 1U,
				static_cast<std::uint64_t>(index) * 6400U);
			if (index == 7U)
				set_harmonic_angle(frame, 0U, 2U, 0U, false);
			aggregation::HarmonicInputView input{};
			expect(decoder.decode(frame, input) ==
				aggregation::FrameValidationError::none,
				"invalid-angle harmonic source decode");
			engine.process(input);
		}
		const auto packed = aggregate_entry(sink.records.front(), 1U);
		expect((packed & (std::uint64_t{1} << 60U)) != 0U &&
			(packed & (std::uint64_t{1} << 61U)) == 0U &&
			((packed >> 40U) & 0xFFFFFU) == 0U,
			"invalid source angle did not preserve magnitude-only validity");
	}
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

void test_harmonic_engine_accepts_one_sample_endpoint_quantization()
{
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::HarmonicFrameDecoder decoder;
	aggregation::HarmonicAggregationEngine engine(sink, health);
	expect(engine.initialize(), "harmonic endpoint test initialization");

	std::uint64_t first_sample = 0U;
	for (std::uint32_t index = 0U; index < 15U; ++index) {
		const auto frame = make_harmonic_frame(index + 1U, first_sample,
			32000U, 6400U);
		aggregation::HarmonicInputView input{};
		expect(decoder.decode(frame, input) ==
			aggregation::FrameValidationError::none,
			"endpoint-quantized harmonic family decode");
		engine.process(input);
		/* The record keeps the exact nominal 6,400-sample lattice while
		 * successive detected cycle boundaries alternate by one frame. */
		first_sample += (index & 1U) == 0U ? 6399U : 6401U;
	}

	expect(sink.count == aggregation::HarmonicProtocol::records_per_family,
		"one-sample endpoint quantization reset the three-second tier");
	expect((sink.records.front().words[8U] & (1U << 6U)) != 0U,
		"endpoint-quantized startup family lost its boundary marker");
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

	const auto &contaminated = sink.first_record_by_period[2U];
	expect((contaminated.words[8U] & ((1U << 3U) | (1U << 4U))) == 0U,
		"startup 10-minute family unexpectedly claims valid magnitudes");
	for (std::size_t word = 16U; word < 62U; ++word)
		expect(contaminated.words[word] == 0U,
			"contaminated 10-minute family retained an invalid payload");

	const auto &two_hour = sink.last_record_by_period[3U];
	expect(((two_hour.words[14U] >> 2U) & 0xFFFU) == 12U &&
		two_hour.words[11U] == 7800U && two_hour.words[12U] == 0U,
		"2-hour contributor count and aligned target");
	expect(two_hour.words[62U] == 16U && two_hour.words[63U] == 195U,
		"2-hour family preserves exact base-family provenance");
	const auto ten_minute_packed = aggregate_entry(
		sink.last_record_by_period[2U], 0U);
	const auto two_hour_packed = aggregate_entry(two_hour, 0U);
	for (const auto packed : {ten_minute_packed, two_hour_packed})
		expect((packed & (std::uint64_t{1} << 61U)) != 0U &&
			angle_near(static_cast<std::uint32_t>(
				(packed >> 40U) & 0xFFFFFU), 116000U, 2U),
			"long harmonic interval lost its circular angle");
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

void test_scheduler_timing_survives_pmu_counter_wrap()
{
	constexpr std::uint32_t counter_hz = 8333333U;
	constexpr std::uint32_t start = 0xFFFFF000U;
	constexpr std::uint32_t elapsed_ticks = 12500U;
	constexpr std::uint32_t finish = start + elapsed_ticks;

	expect(aggregation::scheduler_policy::elapsed_counter_ticks(
		start, finish) == elapsed_ticks,
		"PMU tick delta across 32-bit wrap");
	expect(aggregation::scheduler_policy::elapsed_microseconds(
		start, finish, counter_hz) == 1500U,
		"PMU tick conversion after 32-bit wrap");
	expect(aggregation::scheduler_policy::elapsed_microseconds(
		start, finish, 0U) == 0U,
		"zero-frequency timing fallback");
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

void write_record_u64(aggregation::AggregationMeterRecord &record,
	std::size_t low_word, std::uint64_t value)
{
	record.words[low_word] = static_cast<std::uint32_t>(value);
	record.words[low_word + 1U] = static_cast<std::uint32_t>(value >> 32U);
}

void write_record_s64(aggregation::AggregationMeterRecord &record,
	std::size_t low_word, std::int64_t value)
{
	write_record_u64(record, low_word, std::bit_cast<std::uint64_t>(value));
}

std::uint64_t read_record_u64(
	const aggregation::AggregationMeterRecord &record, std::size_t low_word)
{
	return static_cast<std::uint64_t>(record.words[low_word]) |
		(static_cast<std::uint64_t>(record.words[low_word + 1U]) << 32U);
}

std::int64_t read_record_s64(
	const aggregation::AggregationMeterRecord &record, std::size_t low_word)
{
	return std::bit_cast<std::int64_t>(read_record_u64(record, low_word));
}

aggregation::AggregationMeterRecord make_typed_record(std::uint32_t format,
	std::uint32_t sequence, std::uint32_t status = 0U,
	std::uint32_t sample_rate_hz = 1000U,
	std::uint32_t sample_count = 1000U)
{
	aggregation::AggregationMeterRecord record{};
	record.sequence = sequence;
	record.words[MREC_MAGIC_WORD] = MREC_MAGIC;
	record.words[MREC_FORMAT_WORD] = format;
	record.words[MREC_SIZE_WORD] =
		aggregation::AggregationMeterRecord::byte_count;
	record.words[MREC_SEQUENCE_WORD] = sequence;
	record.words[MREC_GENERATION_WORD] = 0x4d313700U;
	record.words[MREC_SAMPLE_RATE_WORD] = sample_rate_hz;
	record.words[MREC_SAMPLE_COUNT_WORD] = sample_count;
	record.words[MREC_VALID_MASK_WORD] = 0x7fU;
	record.words[MREC_STATUS_WORD] = status;
	write_record_u64(record, MREC_FIRST_SAMPLE_LOW_WORD,
		static_cast<std::uint64_t>(sequence - 1U) * sample_count);
	return record;
}

void emit_basic_energy(aggregation::EnergyDemandEngine &engine,
	aggregation::AggregationRecordSink &sink, std::uint32_t sequence,
	const std::array<std::int64_t, 4U> &active,
	const std::array<std::int64_t, 4U> &reactive,
	std::uint32_t status = 0U, std::uint32_t sample_rate_hz = 1000U,
	std::uint32_t sample_count = 1000U)
{
	auto basic = make_typed_record(MREC_FORMAT_BASIC_V4, sequence, status,
		sample_rate_hz, sample_count);
	write_record_u64(basic, BASIC_LAST_SAMPLE_LOW_WORD,
		static_cast<std::uint64_t>(sequence) * sample_count - 1U);
	auto power = make_typed_record(MREC_FORMAT_POWER_V1, sequence, status,
		sample_rate_hz, sample_count);
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		const auto base = static_cast<std::size_t>(POWER_PHASE_BASE_WORD) +
			phase * static_cast<std::size_t>(POWER_PHASE_STRIDE);
		write_record_s64(power, base + POWER_PHASE_P_LOW, active[phase]);
		write_record_u64(power, base + POWER_PHASE_S_LOW, 3600000000ULL);
	}
	write_record_s64(power, POWER_TOTAL_P_LOW_WORD, active[3U]);
	write_record_u64(power, POWER_TOTAL_S_LOW_WORD, 3600000000ULL);
	auto phasor = make_typed_record(MREC_FORMAT_PHASOR_V2, sequence, status,
		sample_rate_hz, sample_count);
	for (std::size_t phase = 0U; phase < 3U; ++phase)
		write_record_s64(phasor,
			static_cast<std::size_t>(PHASOR_Q1_BASE_WORD) + phase * 2U,
			reactive[phase]);
	write_record_s64(phasor, PHASOR_Q1_TOTAL_LOW_WORD, reactive[3U]);
	const auto unbalance = make_typed_record(MREC_FORMAT_UNBAL_V2, sequence,
		status, sample_rate_hz, sample_count);

	expect(engine.observe(basic, sink, true), "ENERGY accepts BASIC");
	expect(engine.observe(power, sink, true), "ENERGY accepts POWER");
	expect(engine.observe(phasor, sink, true), "ENERGY accepts PHASOR");
	expect(engine.observe(unbalance, sink, true), "ENERGY accepts UNBALANCE");
}

void test_energy_engine_discards_startup_priming()
{
	constexpr std::int64_t one_micro_hour_per_second = 3600000000LL;
	CapturingRecordSink sink;
	aggregation::EnergyDemandEngine engine(0x4d313700U);
	engine.initialize(0x4d313700U);

	// A complete but ineligible startup family is outside the energy session.
	emit_basic_energy(engine, sink, 1U,
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		1U << MREC_STATUS_ARITHMETIC_BIT);
	expect(sink.count == 0U,
		"ineligible startup family emits no authoritative ENERGY pair");

	// Nor does a typed family which is incomplete while the pipeline starts.
	auto basic = make_typed_record(MREC_FORMAT_BASIC_V4, 2U);
	write_record_u64(basic, BASIC_LAST_SAMPLE_LOW_WORD, 1999U);
	auto power = make_typed_record(MREC_FORMAT_POWER_V1, 2U);
	const auto unbalance = make_typed_record(MREC_FORMAT_UNBAL_V2, 2U);
	expect(engine.observe(basic, sink, true), "startup priming accepts BASIC");
	expect(engine.observe(power, sink, true), "startup priming accepts POWER");
	expect(engine.observe(unbalance, sink, true),
		"startup priming discards a family missing PHASOR");
	expect(sink.count == 0U,
		"incomplete startup family remains outside session provenance");

	// The first fully valid family defines the session baseline and begins at a
	// clean zero-skipped provenance point.
	emit_basic_energy(engine, sink, 3U,
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second});
	expect(sink.count == 2U, "first eligible family emits one ENERGY pair");
	const auto &first = sink.records[0U];
	expect(read_record_u64(first, ENERGY_SUMMARY_IMPORT_BASE_WORD) == 1U &&
		first.words[ENERGY_ACCEPTED_BLOCKS_WORD] == 1U &&
		first.words[ENERGY_SKIPPED_BLOCKS_WORD] == 0U &&
		read_record_u64(first, ENERGY_SKIPPED_SAMPLES_LOW_WORD) == 0U &&
		(first.words[MREC_STATUS_WORD] &
			((1U << ENERGY_STATUS_INCOMPLETE_INPUT_BIT) |
			 (1U << ENERGY_STATUS_DISCONTINUITY_BIT))) == 0U,
		"first ENERGY checkpoint is complete with no startup skip provenance");

	// Once the session has started, the same invalidity remains sticky and is
	// never hidden as startup behavior.
	emit_basic_energy(engine, sink, 4U,
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		1U << MREC_STATUS_ARITHMETIC_BIT);
	expect(sink.count == 4U &&
		sink.records[2U].words[ENERGY_SKIPPED_BLOCKS_WORD] == 1U &&
		read_record_u64(sink.records[2U], ENERGY_SKIPPED_SAMPLES_LOW_WORD) == 1000U &&
		(sink.records[2U].words[MREC_STATUS_WORD] &
			(1U << ENERGY_STATUS_INCOMPLETE_INPUT_BIT)) != 0U,
		"post-baseline invalid family remains sticky and observable");
}

void test_energy_engine_four_quadrants_and_axes()
{
	static_assert(met_energy_quadrant(1, 1) == EnergyQuadrant::quadrant_i);
	static_assert(met_energy_quadrant(-1, 1) == EnergyQuadrant::quadrant_ii);
	static_assert(met_energy_quadrant(-1, -1) == EnergyQuadrant::quadrant_iii);
	static_assert(met_energy_quadrant(1, -1) == EnergyQuadrant::quadrant_iv);
	static_assert(met_energy_quadrant(0, 1) == EnergyQuadrant::quadrant_i);
	static_assert(met_energy_quadrant(0, -1) == EnergyQuadrant::quadrant_iv);
	static_assert(met_energy_quadrant(1, 0) == EnergyQuadrant::none);

	constexpr std::uint64_t session_id = 0xfedcba9876543210ULL;
	constexpr std::int64_t one_micro_hour_per_second = 3600000000LL;
	CapturingRecordSink sink;
	aggregation::EnergyDemandEngine engine(session_id);
	engine.initialize(session_id);
	emit_basic_energy(engine, sink, 1U,
		{one_micro_hour_per_second, -one_micro_hour_per_second,
			-one_micro_hour_per_second, one_micro_hour_per_second},
		{one_micro_hour_per_second, one_micro_hour_per_second,
			-one_micro_hour_per_second, -one_micro_hour_per_second});
	expect(sink.count == 2U, "one Basic family emits two ENERGY parts");
	const auto &summary = sink.records[0U];
	const auto &quadrants = sink.records[1U];
	expect(read_record_u64(summary, ENERGY_SUMMARY_IMPORT_BASE_WORD) == 1U &&
		read_record_u64(summary,
			ENERGY_SUMMARY_IMPORT_BASE_WORD + 3U * ENERGY_VALUE_STRIDE) == 1U,
		"positive active power integrates import for phase and algebraic total");
	expect(read_record_u64(summary,
		ENERGY_SUMMARY_EXPORT_BASE_WORD + ENERGY_VALUE_STRIDE) == 1U &&
		read_record_u64(summary,
			ENERGY_SUMMARY_EXPORT_BASE_WORD + 2U * ENERGY_VALUE_STRIDE) == 1U,
		"negative active power integrates export by magnitude");
	for (std::size_t index = 0U; index < 4U; ++index)
		expect(read_record_u64(summary,
			ENERGY_SUMMARY_APPARENT_BASE_WORD + index * ENERGY_VALUE_STRIDE) == 1U,
			"apparent energy integrates phase/total values");
	constexpr std::array<std::size_t, 4U> quadrant_bases = {
		ENERGY_QUADRANT_I_BASE_WORD, ENERGY_QUADRANT_II_BASE_WORD,
		ENERGY_QUADRANT_III_BASE_WORD, ENERGY_QUADRANT_IV_BASE_WORD};
	for (std::size_t index = 0U; index < 4U; ++index)
		expect(read_record_u64(quadrants,
			quadrant_bases[index] + index * ENERGY_VALUE_STRIDE) == 1U,
			"each P/Q sign pair selects its quadrant independently");
	expect(read_record_u64(quadrants, ENERGY_SESSION_ID_LOW_WORD) == session_id,
		"ENERGY preserves the 64-bit R5C1 session identity");

	// P == 0 remains on the import side; Q == 0 accumulates no quadrant.
	emit_basic_energy(engine, sink, 2U, {0, 0, 0, 0},
		{one_micro_hour_per_second, -one_micro_hour_per_second, 0, 0});
	expect(sink.count == 4U, "second Basic family emits a cumulative ENERGY pair");
	const auto &axis_quadrants = sink.records[3U];
	expect(read_record_u64(axis_quadrants, ENERGY_QUADRANT_I_BASE_WORD) == 2U,
		"P-axis positive reactive energy selects quadrant I");
	expect(read_record_u64(axis_quadrants,
		ENERGY_QUADRANT_IV_BASE_WORD + ENERGY_VALUE_STRIDE) == 1U,
		"P-axis negative reactive energy selects quadrant IV");
	expect(read_record_u64(axis_quadrants,
		ENERGY_QUADRANT_III_BASE_WORD + 2U * ENERGY_VALUE_STRIDE) == 1U,
		"Q-axis zero adds no reactive energy");

	// A transport/source discontinuity skips the affected block and remains
	// visible for the lifetime of the volatile R5C1 session.
	emit_basic_energy(engine, sink, 3U, {0, 0, 0, 0}, {0, 0, 0, 0},
		1U << 2U);
	expect((sink.records[4U].words[MREC_STATUS_WORD] &
		(1U << ENERGY_STATUS_DISCONTINUITY_BIT)) != 0U,
		"ENERGY exposes the sticky source discontinuity flag");
	emit_basic_energy(engine, sink, 4U, {0, 0, 0, 0}, {0, 0, 0, 0});
	expect((sink.records[6U].words[MREC_STATUS_WORD] &
		(1U << ENERGY_STATUS_DISCONTINUITY_BIT)) != 0U,
		"ENERGY discontinuity flag remains sticky within the session");
}

void test_energy_engine_rates_remainders_invalidity_and_saturation()
{
	constexpr std::array<std::uint32_t, 8U> supported_rates{
		1000U, 2000U, 4000U, 8000U, 16000U, 32000U, 64000U,
		128000U};
	constexpr std::int64_t one_micro_hour_per_second = 3600000000LL;
	for (const auto rate : supported_rates) {
		CapturingRecordSink sink;
		aggregation::EnergyDemandEngine engine(0x1000U + rate);
		engine.initialize(0x1000U + rate);
		emit_basic_energy(engine, sink, 1U,
			{one_micro_hour_per_second, one_micro_hour_per_second,
				one_micro_hour_per_second, one_micro_hour_per_second},
			{one_micro_hour_per_second, one_micro_hour_per_second,
				one_micro_hour_per_second, one_micro_hour_per_second},
			0U, rate, rate);
		expect(read_record_u64(sink.records[0U],
			ENERGY_SUMMARY_IMPORT_BASE_WORD) == 1U,
			"every supported sample rate integrates the same elapsed second");
	}

	CapturingRecordSink sink;
	aggregation::EnergyDemandEngine engine(0x55aaU);
	engine.initialize(0x55aaU);
	constexpr std::int64_t half_micro_hour_per_second = 1800000000LL;
	emit_basic_energy(engine, sink, 1U,
		{half_micro_hour_per_second, 0, 0, half_micro_hour_per_second},
		{half_micro_hour_per_second, 0, 0, half_micro_hour_per_second});
	expect(read_record_u64(sink.records[0U],
		ENERGY_SUMMARY_IMPORT_BASE_WORD) == 0U,
		"fractional energy is retained rather than rounded per block");
	emit_basic_energy(engine, sink, 2U,
		{half_micro_hour_per_second, 0, 0, half_micro_hour_per_second},
		{half_micro_hour_per_second, 0, 0, half_micro_hour_per_second});
	expect(read_record_u64(sink.records[2U],
		ENERGY_SUMMARY_IMPORT_BASE_WORD) == 1U,
		"retained fixed-point remainder carries into the next block");

	// A sign transition starts the opposite directional counters without
	// subtracting the already accumulated import/quadrant-I energy.
	emit_basic_energy(engine, sink, 3U,
		{-one_micro_hour_per_second, 0, 0, -one_micro_hour_per_second},
		{-one_micro_hour_per_second, 0, 0, -one_micro_hour_per_second});
	expect(read_record_u64(sink.records[4U],
		ENERGY_SUMMARY_IMPORT_BASE_WORD) == 1U &&
		read_record_u64(sink.records[4U],
			ENERGY_SUMMARY_EXPORT_BASE_WORD) == 1U &&
		read_record_u64(sink.records[5U],
			ENERGY_QUADRANT_III_BASE_WORD) == 1U,
		"sign transition preserves import and accumulates export/quadrant III");

	const auto before_invalid = read_record_u64(sink.records[4U],
		ENERGY_SUMMARY_IMPORT_BASE_WORD);
	emit_basic_energy(engine, sink, 4U,
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		1U << MREC_STATUS_ARITHMETIC_BIT);
	const auto &invalid = sink.records[6U];
	expect(read_record_u64(invalid, ENERGY_SUMMARY_IMPORT_BASE_WORD) ==
		before_invalid &&
		read_record_u64(invalid, ENERGY_SKIPPED_SAMPLES_LOW_WORD) == 1000U &&
		invalid.words[ENERGY_SKIPPED_BLOCKS_WORD] == 1U &&
		(invalid.words[MREC_STATUS_WORD] &
			(1U << ENERGY_STATUS_INCOMPLETE_INPUT_BIT)) != 0U,
		"invalid Basic block is skipped with sticky sample/block provenance");

	// Missing typed siblings are rejected as one skipped source interval.
	auto missing_basic = make_typed_record(MREC_FORMAT_BASIC_V4, 5U);
	write_record_u64(missing_basic, BASIC_LAST_SAMPLE_LOW_WORD, 4999U);
	auto missing_power = make_typed_record(MREC_FORMAT_POWER_V1, 5U);
	const auto missing_unbalance = make_typed_record(MREC_FORMAT_UNBAL_V2, 5U);
	expect(engine.observe(missing_basic, sink, true),
		"ENERGY accepts incomplete-family BASIC");
	expect(engine.observe(missing_power, sink, true),
		"ENERGY accepts incomplete-family POWER");
	expect(engine.observe(missing_unbalance, sink, true),
		"ENERGY rejects a family missing PHASOR without emitting");
	expect(sink.count == 8U,
		"incomplete Basic sibling set emitted no ENERGY part");

	emit_basic_energy(engine, sink, 6U,
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second});
	expect(read_record_u64(sink.records[8U],
		ENERGY_SKIPPED_SAMPLES_LOW_WORD) == 2000U &&
		sink.records[8U].words[ENERGY_SKIPPED_BLOCKS_WORD] == 2U,
		"missing sibling family advances skipped provenance exactly once");
	const auto before_duplicate = read_record_u64(sink.records[8U],
		ENERGY_SUMMARY_IMPORT_BASE_WORD);

	// A repeated source sequence is idempotent and remains visible as a
	// rejected block on the next genuinely new cumulative family.
	emit_basic_energy(engine, sink, 6U,
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second});
	expect(sink.count == 10U, "duplicate Basic family emits no ENERGY pair");
	emit_basic_energy(engine, sink, 7U,
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second});
	expect(read_record_u64(sink.records[10U],
		ENERGY_SUMMARY_IMPORT_BASE_WORD) == before_duplicate + 1U &&
		sink.records[10U].words[ENERGY_SKIPPED_BLOCKS_WORD] == 3U,
		"duplicate family neither double-counts energy nor hides provenance");

	// A forward sequence/sample-anchor gap is not interpolated. The missing
	// interval is counted while the next coherent family still integrates.
	emit_basic_energy(engine, sink, 9U,
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second},
		{one_micro_hour_per_second, 0, 0, one_micro_hour_per_second});
	expect(read_record_u64(sink.records[12U],
		ENERGY_SKIPPED_SAMPLES_LOW_WORD) == 3000U &&
		sink.records[12U].words[ENERGY_SKIPPED_BLOCKS_WORD] == 4U &&
		(read_record_u64(sink.records[12U],
			ENERGY_SUMMARY_IMPORT_BASE_WORD) == before_duplicate + 2U) &&
		(sink.records[12U].words[MREC_STATUS_WORD] &
			(1U << ENERGY_STATUS_DISCONTINUITY_BIT)) != 0U,
		"forward gap records missing samples/blocks and resumes accumulation");

	LatestEnergyRecordSink saturation_sink;
	aggregation::EnergyDemandEngine saturation_engine(0x7788U);
	saturation_engine.initialize(0x7788U);
	for (std::uint32_t sequence = 1U; sequence <= 900U; ++sequence)
		emit_basic_energy(saturation_engine, saturation_sink, sequence,
			{INT64_MAX, INT64_MAX, INT64_MAX, INT64_MAX},
			{INT64_MAX, INT64_MAX, INT64_MAX, INT64_MAX}, 0U, 1000U,
			UINT32_MAX);
	expect(read_record_u64(saturation_sink.summary,
		ENERGY_SUMMARY_IMPORT_BASE_WORD) ==
		static_cast<std::uint64_t>(INT64_MAX) &&
		read_record_u64(saturation_sink.quadrants,
			ENERGY_QUADRANT_I_BASE_WORD) ==
			static_cast<std::uint64_t>(INT64_MAX) &&
		(saturation_sink.summary.words[MREC_STATUS_WORD] &
			(1U << ENERGY_STATUS_SATURATED_BIT)) != 0U,
		"energy counters saturate at INT64_MAX and latch overflow");
}

void emit_demand(aggregation::EnergyDemandEngine &engine,
	CapturingRecordSink &sink, std::uint32_t sequence,
	const std::array<std::int64_t, 4U> &active, bool contaminated)
{
	const std::uint32_t status =
		(1U << TEN_MINUTE_STATUS_COMPLETE_BIT) |
		(1U << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT) |
		(1U << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT) |
		(static_cast<std::uint32_t>(contaminated)
			<< TEN_MINUTE_STATUS_CONTAMINATED_BIT);
	auto fundamental = make_typed_record(MREC_FORMAT_TEN_MINUTE_V1,
		sequence, status);
	write_record_u64(fundamental, AGG_LAST_SAMPLE_LOW_WORD,
		static_cast<std::uint64_t>(sequence) * 1000U - 1U);
	write_record_u64(fundamental, TEN_MINUTE_TARGET_SAMPLE_LOW_WORD,
		static_cast<std::uint64_t>(sequence) * 1000U);
	fundamental.words[MTR2_SHAPE_WORD] = 1U;
	auto power = make_typed_record(MREC_FORMAT_TEN_MINUTE_POWER_V1,
		sequence, status);
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		const auto base = static_cast<std::size_t>(POWER_PHASE_BASE_WORD) +
			phase * static_cast<std::size_t>(POWER_PHASE_STRIDE);
		write_record_s64(power, base + POWER_PHASE_P_LOW, active[phase]);
	}
	write_record_s64(power, POWER_TOTAL_P_LOW_WORD, active[3U]);
	const auto phasor = make_typed_record(MREC_FORMAT_TEN_MINUTE_PHASOR_V2,
		sequence, status);
	const auto unbalance = make_typed_record(MREC_FORMAT_TEN_MINUTE_UNBAL_V2,
		sequence, status);
	expect(engine.observe(fundamental, sink, true), "DEMAND accepts Min10");
	expect(engine.observe(power, sink, true), "DEMAND accepts Min10 POWER");
	expect(engine.observe(phasor, sink, true), "DEMAND accepts Min10 PHASOR");
	expect(engine.observe(unbalance, sink, true), "DEMAND accepts Min10 UNBALANCE");
}

void test_demand_engine_peaks_and_contamination()
{
	constexpr std::uint64_t session_id = 0x123456789abcdef0ULL;
	CapturingRecordSink sink;
	aggregation::EnergyDemandEngine engine(session_id);
	engine.initialize(session_id);
	expect(engine.configure_demand(aggregation::DemandMethod::fixed_block,
		DEMAND_FIXED_INTERVAL_SECONDS, DEMAND_FIXED_INTERVAL_SECONDS, 2U),
		"fixed ten-minute demand profile is accepted");
	emit_demand(engine, sink, 1U,
		{5000000LL, -7000000LL, 3000000LL, -9000000LL}, false);
	expect(sink.count == 1U, "one Min10 family emits one DEMAND record");
	const auto &valid = sink.records[0U];
	expect(read_record_s64(valid, DEMAND_CURRENT_BASE_WORD) == 5 &&
		read_record_s64(valid,
			DEMAND_CURRENT_BASE_WORD + DEMAND_VALUE_STRIDE) == -7 &&
		read_record_s64(valid,
			DEMAND_CURRENT_BASE_WORD + 3U * DEMAND_VALUE_STRIDE) == -9,
		"DEMAND publishes signed current micro-watts");
	expect(read_record_u64(valid, DEMAND_IMPORT_PEAK_BASE_WORD) == 5U &&
		read_record_u64(valid,
			DEMAND_EXPORT_PEAK_BASE_WORD + DEMAND_VALUE_STRIDE) == 7U &&
		read_record_u64(valid,
			DEMAND_EXPORT_PEAK_BASE_WORD + 3U * DEMAND_VALUE_STRIDE) == 9U,
		"valid Min10 interval updates directional session peaks");
	expect(read_record_u64(valid, DEMAND_SESSION_ID_LOW_WORD) == session_id,
		"DEMAND preserves the R5C1 session identity");

	emit_demand(engine, sink, 2U,
		{50000000LL, -70000000LL, 30000000LL, -90000000LL}, true);
	expect(sink.count == 2U, "contaminated Min10 family still emits DEMAND quality");
	const auto &invalid = sink.records[1U];
	expect(((invalid.words[MREC_FORMAT_HEADER_WORD] >>
		DEMAND_HEADER_VALID_LSB) & 0x0fU) == 0U,
		"contaminated interval invalidates current demand");
	expect(read_record_u64(invalid, DEMAND_IMPORT_PEAK_BASE_WORD) == 5U &&
		read_record_u64(invalid,
			DEMAND_EXPORT_PEAK_BASE_WORD + DEMAND_VALUE_STRIDE) == 7U,
		"contaminated interval does not update session peaks");
}

void emit_sliding_demand(aggregation::EnergyDemandEngine &engine,
	CapturingRecordSink &sink, std::uint32_t sequence,
	const std::array<std::int64_t, 4U> &active, bool valid = true)
{
	constexpr std::uint32_t sample_rate_hz = 1000U;
	constexpr std::uint32_t sample_count =
		sample_rate_hz * DEMAND_SLIDING_UPDATE_SECONDS;
	const std::uint32_t status = valid
		? 1U << MTR2_STATUS_COMPLETE_BIT
		: (1U << MTR2_STATUS_COMPLETE_BIT) |
			(1U << MREC_STATUS_ARITHMETIC_BIT);
	auto fundamental = make_typed_record(MREC_FORMAT_AGG_V3, sequence,
		status, sample_rate_hz, sample_count);
	write_record_u64(fundamental, AGG_LAST_SAMPLE_LOW_WORD,
		static_cast<std::uint64_t>(sequence) * sample_count - 1U);
	const std::uint32_t shape = 15U | (50U << MTR2_SHAPE_NOMINAL_LSB);
	fundamental.words[MTR2_SHAPE_WORD] = shape;
	auto power = make_typed_record(MREC_FORMAT_AGG_POWER_V1, sequence,
		status, sample_rate_hz, sample_count);
	power.words[MTR2_SHAPE_WORD] = shape;
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		const auto base = static_cast<std::size_t>(POWER_PHASE_BASE_WORD) +
			phase * static_cast<std::size_t>(POWER_PHASE_STRIDE);
		write_record_s64(power, base + POWER_PHASE_P_LOW, active[phase]);
	}
	write_record_s64(power, POWER_TOTAL_P_LOW_WORD, active[3U]);
	auto phasor = make_typed_record(MREC_FORMAT_AGG_PHASOR_V2,
		sequence, status, sample_rate_hz, sample_count);
	phasor.words[MTR2_SHAPE_WORD] = shape;
	auto unbalance = make_typed_record(MREC_FORMAT_AGG_UNBAL_V2,
		sequence, status, sample_rate_hz, sample_count);
	unbalance.words[MTR2_SHAPE_WORD] = shape;
	expect(engine.observe(fundamental, sink, true),
		"sliding DEMAND accepts aggregate fundamental");
	expect(engine.observe(power, sink, true),
		"sliding DEMAND accepts aggregate POWER");
	expect(engine.observe(phasor, sink, true),
		"sliding DEMAND accepts aggregate PHASOR");
	expect(engine.observe(unbalance, sink, true),
		"sliding DEMAND accepts aggregate UNBALANCE");
}

void test_sliding_demand_cadence_recovery_and_profile()
{
	constexpr std::int64_t five_micro_watts = 5000000LL;
	CapturingRecordSink sink;
	aggregation::EnergyDemandEngine engine(0x445566778899aabbULL);
	engine.initialize(0x445566778899aabbULL);

	expect(aggregation::EnergyDemandEngine::valid_demand_configuration(
		aggregation::DemandMethod::sliding, 60U, 3U),
		"default sliding demand profile is valid");
	for (const auto window : {300U, 600U, 900U, 1800U})
		expect(aggregation::EnergyDemandEngine::valid_demand_configuration(
			aggregation::DemandMethod::sliding, window, 3U),
			"every selectable sliding demand profile is valid");
	expect(!aggregation::EnergyDemandEngine::valid_demand_configuration(
		aggregation::DemandMethod::sliding, 61U, 3U),
		"unsupported sliding window is rejected");
	expect(!aggregation::EnergyDemandEngine::valid_demand_configuration(
		aggregation::DemandMethod::sliding, 60U, 1U),
		"sliding update cadence remains tied to completed aggregates");

	for (std::uint32_t sequence = 1U; sequence <= 19U; ++sequence)
		emit_sliding_demand(engine, sink, sequence,
			{five_micro_watts, -five_micro_watts,
				five_micro_watts, -five_micro_watts});
	expect(sink.count == 0U,
		"sliding demand waits for one complete 60-second window");
	emit_sliding_demand(engine, sink, 20U,
		{five_micro_watts, -five_micro_watts,
			five_micro_watts, -five_micro_watts});
	expect(sink.count == 1U,
		"full sliding window emits at the three-second aggregate cadence");
	const auto &first = sink.records[0U];
	expect(read_record_s64(first, DEMAND_CURRENT_BASE_WORD) == 5 &&
		read_record_s64(first,
			DEMAND_CURRENT_BASE_WORD + DEMAND_VALUE_STRIDE) == -5,
		"sliding demand publishes the sample-weighted signed average");
	expect(((first.words[MREC_FORMAT_HEADER_WORD] >>
		DEMAND_HEADER_METHOD_LSB) & 0x03U) == DEMAND_METHOD_SLIDING &&
		(first.words[MREC_FORMAT_HEADER_WORD] & 0xffffU) == 60U &&
		((first.words[MREC_FORMAT_HEADER_WORD] >>
			DEMAND_HEADER_UPDATE_SECONDS_LSB) & 0x03ffU) == 3U &&
		first.words[DEMAND_PROFILE_GENERATION_WORD] == 1U,
		"DEMAND-v1 carries the active method, window, cadence, and profile");

	// Once warm, each new aggregate advances the window and emits a fresh
	// value. Replacing one 5-uW bucket with a 25-uW bucket yields 6 uW.
	emit_sliding_demand(engine, sink, 21U,
		{25000000LL, -25000000LL, 25000000LL, -25000000LL});
	expect(sink.count == 2U &&
		read_record_s64(sink.records[1U], DEMAND_CURRENT_BASE_WORD) == 6,
		"warm sliding demand advances every three seconds");

	// A rejected aggregate publishes invalid current immediately and clears
	// the window. The quality is current-window state, not a sticky lifetime
	// latch: twenty subsequent clean buckets recover it automatically.
	emit_sliding_demand(engine, sink, 22U,
		{five_micro_watts, -five_micro_watts,
			five_micro_watts, -five_micro_watts}, false);
	expect(sink.count == 3U &&
		((sink.records[2U].words[MREC_FORMAT_HEADER_WORD] >>
			DEMAND_HEADER_VALID_LSB) & 0x0fU) == 0U &&
		(sink.records[2U].words[MREC_STATUS_WORD] &
			(1U << DEMAND_STATUS_INCOMPLETE_INPUT_BIT)) != 0U,
		"bad aggregate invalidates demand and starts a clean refill");
	for (std::uint32_t sequence = 23U; sequence <= 42U; ++sequence)
		emit_sliding_demand(engine, sink, sequence,
			{five_micro_watts, -five_micro_watts,
				five_micro_watts, -five_micro_watts});
	expect(sink.count == 4U &&
		((sink.records[3U].words[MREC_FORMAT_HEADER_WORD] >>
			DEMAND_HEADER_VALID_LSB) & 0x0fU) == 0x0fU &&
		(sink.records[3U].words[MREC_STATUS_WORD] &
			(1U << DEMAND_STATUS_INCOMPLETE_INPUT_BIT)) == 0U,
		"clean refill clears incomplete demand quality without a restart");

	// Exercise the maximum static ring, not just its validation branch. It
	// must warm for all 600 three-second buckets and then publish one coherent
	// 30-minute result without heap or task-stack storage.
	CapturingRecordSink maximum_sink;
	aggregation::EnergyDemandEngine maximum_engine(0xaabbccddU);
	maximum_engine.initialize(0xaabbccddU);
	expect(maximum_engine.configure_demand(
		aggregation::DemandMethod::sliding, 1800U, 3U, 2U),
		"maximum sliding demand profile is accepted");
	for (std::uint32_t sequence = 1U; sequence <= 599U; ++sequence)
		emit_sliding_demand(maximum_engine, maximum_sink, sequence,
			{five_micro_watts, -five_micro_watts,
				five_micro_watts, -five_micro_watts});
	expect(maximum_sink.count == 0U,
		"30-minute profile waits for the complete static ring");
	emit_sliding_demand(maximum_engine, maximum_sink, 600U,
		{five_micro_watts, -five_micro_watts,
			five_micro_watts, -five_micro_watts});
	expect(maximum_sink.count == 1U &&
		maximum_sink.records[0U].words[DEMAND_SOURCE_INTERVAL_COUNT_WORD] ==
			600U &&
		read_record_s64(maximum_sink.records[0U],
			DEMAND_CURRENT_BASE_WORD) == 5,
		"30-minute profile publishes after exactly 600 clean buckets");
}

void test_r5_engine_primes_energy_before_emission()
{
	constexpr std::uint32_t generation = 0x12345678U;
	constexpr std::uint32_t samples_per_cycle = 533U;
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::R5AggregationEngine engine(sink, health,
		aggregation::AggregationOutputMode::emit);
	expect(engine.initialize(), "R5 aggregation engine initialization");

	std::uint64_t first_sample = 0U;
	for (std::uint32_t cycle = 1U; cycle <= 24U; ++cycle) {
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

	expect(sink.count == 10U,
		"startup Basic primes energy; the next Basic emits the first ENERGY pair");
	constexpr std::array<std::uint32_t, 10U> expected_formats = {
		MREC_FORMAT_BASIC_V4,
		MREC_FORMAT_POWER_V1,
		MREC_FORMAT_PHASOR_V2,
		MREC_FORMAT_UNBAL_V2,
		MREC_FORMAT_BASIC_V4,
		MREC_FORMAT_POWER_V1,
		MREC_FORMAT_PHASOR_V2,
		MREC_FORMAT_UNBAL_V2,
		MREC_FORMAT_ENERGY_V1,
		MREC_FORMAT_ENERGY_V1,
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
		const auto expected_sequence = index < 4U ? 1U : 2U;
		expect(record.words[MREC_SEQUENCE_WORD] == expected_sequence,
			"R5 Basic-family sequence");
	}
	expect((sink.records[8U].words[MREC_FORMAT_HEADER_WORD] & 0x3U) ==
		ENERGY_PART_SUMMARY &&
		(sink.records[9U].words[MREC_FORMAT_HEADER_WORD] & 0x3U) ==
			ENERGY_PART_QUADRANTS,
		"ENERGY summary and quadrant part ordering");
	expect(((sink.records[8U].words[MREC_FORMAT_HEADER_WORD] >>
		ENERGY_HEADER_PART_COUNT_LSB) & 0x3U) == ENERGY_PART_COUNT &&
		((sink.records[9U].words[MREC_FORMAT_HEADER_WORD] >>
		ENERGY_HEADER_PART_COUNT_LSB) & 0x3U) == ENERGY_PART_COUNT,
		"ENERGY atomic family part count");
	expect(sink.records[8U].words[ENERGY_ACCEPTED_BLOCKS_WORD] == 1U &&
		sink.records[8U].words[ENERGY_SKIPPED_BLOCKS_WORD] == 0U &&
		read_record_u64(sink.records[8U], ENERGY_SKIPPED_SAMPLES_LOW_WORD) == 0U,
		"startup priming is outside accepted/skipped session provenance");
	expect((sink.records[8U].words[MREC_STATUS_WORD] &
		((1U << ENERGY_STATUS_INCOMPLETE_INPUT_BIT) |
		 (1U << ENERGY_STATUS_DISCONTINUITY_BIT))) == 0U,
		"first authoritative ENERGY family starts complete and continuous");

	const auto status = health.snapshot();
	expect(status.engine_ready, "R5 aggregation engine remains ready");
	expect(status.authoritative,
		"emit-mode R5 aggregation engine reports authoritative");
	expect(status.basic_completed == 2U,
		"R5 health counts both completed Basic measurement records");
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

void write_u64(std::array<std::uint32_t,
	aggregation::maximum_transport_frame_words> &words,
	std::size_t index, std::uint64_t value)
{
	words[index] = static_cast<std::uint32_t>(value);
	words[index + 1U] = static_cast<std::uint32_t>(value >> 32U);
}

aggregation::AggregationFrame make_pqe_frame(std::uint32_t sequence = 1U)
{
	aggregation::AggregationFrame frame{};
	frame.word_count = aggregation::PqEventProtocol::frame_words;
	auto &words = frame.words;
	words[0U] = aggregation::PqEventProtocol::magic;
	words[1U] = aggregation::PqEventProtocol::contract_revision;
	words[2U] = aggregation::PqEventProtocol::payload_words;
	words[3U] = sequence;
	const auto payload = aggregation::PqEventProtocol::payload_index;
	words[payload + 0U] = sequence;
	words[payload + 1U] = 7U;
	words[payload + 2U] = 32000U;
	words[payload + 3U] = 0x1U;
	words[payload + 4U] = 0x707U;
	words[payload + 5U] = 640U;
	write_u64(words, payload + 6U, 1000U);
	write_u64(words, payload + 8U, 1639U);
	write_u64(words, payload + 10U, 0x0123456789abcdefULL);
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		write_u64(words, payload + 12U + phase * 2U,
			std::uint64_t{120000000U + static_cast<std::uint32_t>(phase)} << 16U);
		write_u64(words, payload + 18U + phase * 2U,
			std::uint64_t{5000000U + static_cast<std::uint32_t>(phase)} << 16U);
	}
	words[payload + 24U] = 120000000U;
	words[payload + 25U] = 9000U;
	words[payload + 26U] = 11000U;
	words[payload + 27U] = 1000U;
	words[payload + 28U] = 200U;
	words[payload + 29U] = 1U;
	words[aggregation::PqEventProtocol::crc_index] =
		aggregation::crc32c_words(words.data(),
			aggregation::PqEventProtocol::crc_index);
	return frame;
}

void test_pq_event_frame_decoder()
{
	aggregation::PqEventFrameDecoder decoder;
	aggregation::PqEventInputView input{};
	auto frame = make_pqe_frame(9U);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::none,
		"valid PQE1 frame decodes");
	expect(input.sequence == 9U && input.configuration_generation == 7U &&
		input.first_sample == 1000U && input.last_sample == 1639U,
		"PQE1 provenance decodes exactly");
	expect(input.voltage_valid_mask == 0x7U &&
		input.current_valid_mask == 0x7U,
		"PQE1 phase validity is split by quantity");
	expect((input.urms_q16[2U] >> 16U) == 120000002U &&
		(input.irms_q16[1U] >> 16U) == 5000001U,
		"PQE1 Q16 RMS values decode without loss");

	auto corrupted = frame;
	corrupted.words[aggregation::PqEventProtocol::payload_index + 30U] = 1U;
	corrupted.words[aggregation::PqEventProtocol::crc_index] =
		aggregation::crc32c_words(corrupted.words.data(),
			aggregation::PqEventProtocol::crc_index);
	expect(decoder.decode(corrupted, input) ==
		aggregation::FrameValidationError::reserved_bits_nonzero,
		"PQE1 rejects nonzero reserved words after CRC validation");
	corrupted = frame;
	corrupted.words[aggregation::PqEventProtocol::crc_index] ^= 1U;
	expect(decoder.decode(corrupted, input) ==
		aggregation::FrameValidationError::crc_mismatch,
		"PQE1 rejects CRC corruption");
}

aggregation::AggregationFrame make_flicker_frame(std::uint32_t sequence,
	std::uint32_t kind = aggregation::FlickerProtocol::kind_live,
	std::uint32_t histogram_base = 0U)
{
	aggregation::AggregationFrame frame{};
	frame.word_count = aggregation::FlickerProtocol::frame_words;
	auto &words = frame.words;
	words[0U] = aggregation::FlickerProtocol::magic;
	words[1U] = aggregation::FlickerProtocol::contract_revision;
	words[2U] = aggregation::FlickerProtocol::payload_words;
	words[3U] = sequence;
	const auto payload = aggregation::FlickerProtocol::payload_index;
	words[payload + aggregation::FlickerProtocol::sequence_word] = sequence;
	words[payload + aggregation::FlickerProtocol::generation_word] = 7U;
	words[payload + aggregation::FlickerProtocol::sample_rate_word] = 32000U;
	words[payload + aggregation::FlickerProtocol::status_word] = 0x3U;
	words[payload + aggregation::FlickerProtocol::phase_mask_word] = 0x7U;
	words[payload + aggregation::FlickerProtocol::kind_word] = kind;
	words[payload + aggregation::FlickerProtocol::model_word] =
		120U | (60U << 16U);
	words[payload + aggregation::FlickerProtocol::timing_word] =
		1000U | (600U << 16U);
	words[payload + aggregation::FlickerProtocol::histogram_base_word] =
		histogram_base;
	const auto seconds = kind == aggregation::FlickerProtocol::kind_live
		? 1U : 600U;
	const auto ticks = seconds * 2000U;
	for (std::size_t phase = 0U;
		phase < aggregation::FlickerProtocol::phases; ++phase) {
		words[payload + aggregation::FlickerProtocol::valid_count_word + phase] =
			ticks;
		words[payload + aggregation::FlickerProtocol::pinst_word + phase] =
			static_cast<std::uint32_t>((phase + 1U) << 16U);
	}
	write_u64(words,
		payload + aggregation::FlickerProtocol::first_sample_word, 1000U);
	write_u64(words,
		payload + aggregation::FlickerProtocol::last_sample_word,
		1000U + std::uint64_t{seconds} * 32000U - 1U);
	if (kind == aggregation::FlickerProtocol::kind_histogram &&
		histogram_base <= 255U && histogram_base +
			aggregation::FlickerProtocol::histogram_bins > 255U) {
		const auto offset = 255U - histogram_base;
		for (std::size_t phase = 0U;
			phase < aggregation::FlickerProtocol::phases; ++phase)
			words[payload + aggregation::FlickerProtocol::histogram_word +
				phase * aggregation::FlickerProtocol::histogram_bins + offset] =
				1200000U;
	}
	words[aggregation::FlickerProtocol::crc_index] =
		aggregation::crc32c_words(words.data(),
			aggregation::FlickerProtocol::crc_index);
	return frame;
}

void refresh_flicker_crc(aggregation::AggregationFrame &frame)
{
	frame.words[aggregation::FlickerProtocol::crc_index] =
		aggregation::crc32c_words(frame.words.data(),
			aggregation::FlickerProtocol::crc_index);
}

void test_flicker_frame_decoder()
{
	aggregation::FlickerFrameDecoder decoder;
	aggregation::FlickerInputView input{};
	auto frame = make_flicker_frame(11U);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::none,
		"valid one-second FLK1 frame decodes");
	expect(input.sequence == 11U && input.configuration_generation == 7U &&
		input.lamp_voltage == 120U && input.nominal_hz == 60U &&
		input.valid_count[2U] == 2000U,
		"FLK1 model, timing, and phase counts decode exactly");

	frame = make_flicker_frame(12U,
		aggregation::FlickerProtocol::kind_histogram, 510U);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::none,
		"final lossless classifier chunk accepts zero padding");
	const auto payload = aggregation::FlickerProtocol::payload_index;
	frame.words[payload + aggregation::FlickerProtocol::histogram_word + 2U] =
		1U;
	refresh_flicker_crc(frame);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::reserved_bits_nonzero,
		"final FLK1 classifier chunk rejects nonzero padding");

	frame = make_flicker_frame(13U);
	frame.words[payload + aggregation::FlickerProtocol::last_sample_word] -= 1U;
	refresh_flicker_crc(frame);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::invalid_record_geometry,
		"FLK1 rejects a shortened source-sample interval");
	frame = make_flicker_frame(14U);
	frame.words[payload + aggregation::FlickerProtocol::valid_count_word] -= 1U;
	refresh_flicker_crc(frame);
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::invalid_record_geometry,
		"FLK1 rejects a short count marked phase-valid");
	frame = make_flicker_frame(15U);
	frame.words[aggregation::FlickerProtocol::crc_index] ^= 1U;
	expect(decoder.decode(frame, input) ==
		aggregation::FrameValidationError::crc_mismatch,
		"FLK1 rejects CRC corruption");
}

msap1_m18_config_payload make_event_configuration(std::uint32_t generation)
{
	msap1_m18_config_payload configuration{};
	configuration.generation = generation;
	configuration.event_profile_count = MSAP1_M18_EVENT_TYPE_COUNT;
	configuration.reference_voltage_microvolts = 120000000U;
	configuration.reference_current_microamperes = 5000000U;
	for (std::size_t type = 0U; type < MSAP1_M18_EVENT_TYPE_COUNT; ++type) {
		auto &profile = configuration.event[type];
		profile.flags = (type <= MSAP1_M18_EVENT_RAPID_VOLTAGE_CHANGE ||
			type == MSAP1_M18_EVENT_TRANSIENT_VOLTAGE)
			? static_cast<std::uint32_t>(
				MSAP1_M18_EVENT_IEC_CLASSIFICATION) : 0U;
		profile.threshold_e4 = 1000U;
		profile.hysteresis_e4 = 100U;
		profile.phase_mask = 0x7U;
		profile.waveform_decimation = 1U;
	}
	configuration.event[MSAP1_M18_EVENT_VOLTAGE_SAG].flags |=
		MSAP1_M18_EVENT_ENABLED | MSAP1_M18_EVENT_PER_PHASE |
		MSAP1_M18_EVENT_WAVEFORM_ENABLED;
	configuration.event[MSAP1_M18_EVENT_VOLTAGE_SAG].threshold_e4 = 9000U;
	configuration.event[MSAP1_M18_EVENT_VOLTAGE_SAG].hysteresis_e4 = 200U;
	configuration.event[MSAP1_M18_EVENT_VOLTAGE_SAG].waveform_pretrigger_ms =
		3000U;
	configuration.event[MSAP1_M18_EVENT_VOLTAGE_SAG].waveform_posttrigger_ms =
		4000U;
	configuration.event[MSAP1_M18_EVENT_VOLTAGE_SWELL].flags |=
		MSAP1_M18_EVENT_ENABLED | MSAP1_M18_EVENT_PER_PHASE;
	configuration.event[MSAP1_M18_EVENT_VOLTAGE_SWELL].threshold_e4 = 11000U;
	configuration.event[MSAP1_M18_EVENT_VOLTAGE_SWELL].hysteresis_e4 = 200U;
	configuration.flicker_phase_mask = 0x7U;
	configuration.flicker_lamp_voltage = 120U;
	configuration.flicker_live_cadence_ms = 1000U;
	configuration.flicker_pst_interval_seconds = 600U;
	configuration.flicker_plt_pst_count = 12U;
	configuration.mains_carrier_millihz = 1000000U;
	configuration.mains_bandwidth_millihz = 20000U;
	configuration.mains_observation_ms = 200U;
	configuration.mains_phase_mask = 0x7U;
	return configuration;
}

aggregation::FlickerInputView make_flicker_histogram_input(
	std::uint32_t sequence, std::uint32_t interval,
	std::uint16_t histogram_base)
{
	aggregation::FlickerInputView input{};
	input.sequence = sequence;
	input.configuration_generation = 7U;
	input.sample_rate_hz = 32000U;
	input.status = 0x3U;
	input.phase_mask = 0x7U;
	input.kind = static_cast<std::uint8_t>(
		aggregation::FlickerProtocol::kind_histogram);
	input.lamp_voltage = 120U;
	input.nominal_hz = 60U;
	input.live_cadence_ms = 1000U;
	input.pst_interval_seconds = 600U;
	input.histogram_base = histogram_base;
	input.valid_count.fill(1200000U);
	input.pinst_q16 = {2U << 16U, 3U << 16U, 4U << 16U};
	input.first_sample = std::uint64_t{interval} * 600U * 32000U;
	input.last_sample = input.first_sample + 600U * 32000U - 1U;
	if (histogram_base <= 255U && histogram_base +
			aggregation::FlickerProtocol::histogram_bins > 255U) {
		const auto offset = 255U - histogram_base;
		for (auto &phase : input.histogram)
			phase[offset] = 1200000U;
	}
	return input;
}

void test_flicker_engine_pst_and_plt()
{
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::FlickerEngine engine(sink, health);
	expect(engine.initialize(), "flicker aggregation engine initializes");
	auto configuration = make_event_configuration(7U);
	configuration.flicker_flags = MSAP1_M18_ENGINE_ENABLED;
	expect(engine.configure(configuration), "valid flicker profile stages");

	std::uint32_t sequence = 0U;
	for (std::uint32_t interval = 0U; interval < 12U; ++interval) {
		for (std::uint32_t chunk = 0U;
			chunk < aggregation::FlickerProtocol::classifier_chunks; ++chunk) {
			engine.process(make_flicker_histogram_input(++sequence, interval,
				static_cast<std::uint16_t>(chunk *
					aggregation::FlickerProtocol::histogram_bins)));
		}
	}
	expect(sink.count == 13U,
		"twelve complete Pst intervals emit twelve Pst and one Plt record");
	for (std::size_t interval = 0U; interval < 12U; ++interval) {
		const auto &record = sink.records[interval];
		expect((record.words[13U] & 0xffU) == 1U &&
			((record.words[13U] >> 8U) & 0x7U) == 0x7U,
			"each lossless classifier family emits one valid Pst record");
		expect(record.words[19U] != 0U && record.words[20U] != 0U &&
			record.words[21U] != 0U && record.words[28U] == 600U,
			"Pst percentiles retain all configured phases and duration");
	}
	const auto &plt = sink.records[12U];
	expect((plt.words[13U] & 0xffU) == 2U &&
		((plt.words[13U] >> 8U) & 0x7U) == 0x7U &&
		plt.words[22U] != 0U && plt.words[24U] != 0U &&
		plt.words[28U] == 7200U,
		"twelfth consecutive valid Pst emits the cubic-mean Plt");
	expect(plt.words[6U] == 7200U * 32000U &&
		plt.words[25U] == 12U * 1200000U,
		"Plt sample and valid-count spans cover all twelve Pst intervals");
	expect(plt.words[9U] == 0U && plt.words[10U] == 0U,
		"Plt provenance starts at the oldest retained Pst interval");

	const auto before_gap = sink.count;
	engine.process(make_flicker_histogram_input(sequence + 1U, 12U, 0U));
	for (std::uint32_t chunk = 1U;
		chunk < aggregation::FlickerProtocol::classifier_chunks; ++chunk)
		engine.process(make_flicker_histogram_input(sequence + 2U + chunk,
			12U, static_cast<std::uint16_t>(chunk *
				aggregation::FlickerProtocol::histogram_bins)));
	expect(sink.count == before_gap,
		"a sequence gap invalidates the whole in-flight classifier family");
}

aggregation::PqEventInputView make_pq_input(std::uint32_t sequence,
	std::uint32_t generation, std::uint64_t last_sample,
	std::array<std::uint32_t, 3U> voltage)
{
	aggregation::PqEventInputView input{};
	input.sequence = sequence;
	input.configuration_generation = generation;
	input.sample_rate_hz = 32000U;
	input.voltage_valid_mask = 0x7U;
	input.current_valid_mask = 0x7U;
	input.window_samples = 640U;
	input.first_sample = last_sample - 639U;
	input.last_sample = last_sample;
	for (std::size_t phase = 0U; phase < 3U; ++phase) {
		input.urms_q16[phase] = std::uint64_t{voltage[phase]} << 16U;
		input.irms_q16[phase] = std::uint64_t{5000000U} << 16U;
	}
	return input;
}

void test_pq_event_lifecycle_engine()
{
	constexpr std::uint32_t start_lifecycle = 0U;
	constexpr std::uint32_t update_lifecycle = 1U;
	constexpr std::uint32_t end_lifecycle = 2U;
	constexpr std::uint32_t abort_lifecycle = 3U;
	CapturingRecordSink sink;
	aggregation::AggregationHealth health;
	aggregation::PqEventLifecycleEngine engine(sink, health);
	expect(engine.configure_session_id(0x1122334455667788ULL),
		"PQ event session ID configures before initialization");
	expect(engine.initialize(), "PQ lifecycle engine initializes");
	auto configuration = make_event_configuration(7U);
	expect(engine.configure(configuration), "valid M18 event profile stages");

	auto input = make_pq_input(1U, 7U, 1639U,
		{120000000U, 120000000U, 120000000U});
	input.status = 1U << 2U;
	engine.process(input);
	expect(sink.count == 0U, "nominal first half-cycle opens no event");

	input = make_pq_input(2U, 7U, 1959U,
		{80000000U, 120000000U, 120000000U});
	engine.process(input);
	expect(sink.count == 1U, "phase-A sag emits START");
	const auto start = sink.records[0U];
	expect((start.words[13U] & 0x3U) == start_lifecycle &&
		((start.words[13U] >> 4U) & 0xfU) ==
			MSAP1_M18_EVENT_VOLTAGE_SAG &&
		((start.words[13U] >> 8U) & 0x7U) == 0x1U,
		"START identifies sag and exact affected phase");
	expect(start.words[20U] == 7U && start.words[21U] == 9000U &&
		start.words[22U] == 200U && start.words[26U] == 120000000U,
		"START snapshots the evaluated generation, thresholds, and reference");
	expect(start.words[24U] == 3000U && start.words[25U] == 4000U &&
		(start.words[23U] & 1U) != 0U,
		"START snapshots its waveform policy");
	expect((start.words[8U] & (1U << 2U)) != 0U &&
		(start.words[8U] & (1U << 3U)) != 0U && start.words[45U] == 0U,
		"first lifecycle record exposes discontinuity and unresolved time");
	expect(start.words[48U] != 0U && start.words[51U] != 0U,
		"settings fingerprint is populated");

	input = make_pq_input(3U, 7U, 33959U,
		{80000000U, 120000000U, 120000000U});
	engine.process(input);
	expect(sink.count == 2U &&
		(sink.records[1U].words[13U] & 0x3U) == update_lifecycle,
		"active sag emits bounded one-second UPDATE");
	input = make_pq_input(4U, 7U, 34279U,
		{120000000U, 120000000U, 120000000U});
	engine.process(input);
	expect(sink.count == 3U &&
		(sink.records[2U].words[13U] & 0x3U) == end_lifecycle,
		"recovery past hysteresis emits END");
	for (std::size_t word = 16U; word <= 19U; ++word)
		expect(sink.records[0U].words[word] == sink.records[1U].words[word] &&
			sink.records[1U].words[word] == sink.records[2U].words[word],
			"START/UPDATE/END retain one stable event ID");

	input = make_pq_input(5U, 7U, 34599U,
		{80000000U, 140000000U, 120000000U});
	engine.process(input);
	expect(sink.count == 5U,
		"independent sag and swell phase states overlap");
	expect(sink.records[3U].words[18U] != sink.records[4U].words[18U],
		"overlapping events receive distinct stable IDs");

	configuration = make_event_configuration(8U);
	expect(engine.configure(configuration), "replacement event profile stages");
	input = make_pq_input(6U, 8U, 34919U,
		{120000000U, 120000000U, 120000000U});
	engine.process(input);
	expect(sink.count == 7U &&
		(sink.records[5U].words[13U] & 0x3U) == abort_lifecycle &&
		(sink.records[6U].words[13U] & 0x3U) == abort_lifecycle,
		"configuration APPLY aborts every overlapping old-profile event");
	expect(sink.records[5U].words[20U] == 7U &&
		sink.records[6U].words[20U] == 7U,
		"ABORT retains the exact historical profile generation");
}

} // namespace

int main()
{
	test_session_id_uses_boot_varying_counter();
	test_crc32c();
	test_valid_frame();
	test_invalid_frames();
	test_harmonic_frame_decoder();
	test_harmonic_engine_emits_complete_three_second_family();
	test_harmonic_engine_uses_circular_angles_and_propagates_validity();
	test_harmonic_engine_resets_partial_tiers_in_place();
	test_harmonic_engine_accepts_one_sample_endpoint_quantization();
	test_harmonic_engine_emits_clean_ten_minute_and_two_hour_families();
	test_ring();
	test_output_ring();
	test_health();
	test_ring_pressure_telemetry();
	test_scheduler_timing_survives_pmu_counter_wrap();
	test_bounded_input_handoff_preserves_validator_progress();
	test_energy_engine_discards_startup_priming();
	test_energy_engine_four_quadrants_and_axes();
	test_energy_engine_rates_remainders_invalidity_and_saturation();
	test_demand_engine_peaks_and_contamination();
	test_sliding_demand_cadence_recovery_and_profile();
	test_r5_engine_primes_energy_before_emission();
	test_r5_engine_fails_closed_when_output_rejects_record();
	test_r5_engine_drains_deferred_aggregate_family();
	test_r5_shadow_mode_is_non_authoritative();
	test_pq_event_frame_decoder();
	test_pq_event_lifecycle_engine();
	test_flicker_frame_decoder();
	test_flicker_engine_pst_and_plt();
	std::cout << "aggregation shadow tests passed\n";
	return EXIT_SUCCESS;
}
