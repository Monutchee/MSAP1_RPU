#include "mains_signal_engine.hpp"

#include <cstring>

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t meter_record_magic = 0x3152544DU;
constexpr std::uint32_t mains_signal_record_format = 0x000F0001U;
constexpr std::uint32_t record_bytes = 256U;

std::uint32_t low(std::uint64_t value) noexcept
{
	return static_cast<std::uint32_t>(value);
}

std::uint32_t high(std::uint64_t value) noexcept
{
	return static_cast<std::uint32_t>(value >> 32U);
}

} // namespace

MainsSignalEngine::MainsSignalEngine(AggregationRecordSink &sink,
	AggregationHealth &health) noexcept : sink_(sink), health_(health)
{
}

bool MainsSignalEngine::configure(
	const msap1_m18_config_payload &configuration) noexcept
{
	if (!msap1::power_quality::valid_configuration(configuration))
		return false;
	(void)__atomic_add_fetch(&staged_revision_, 1U, __ATOMIC_ACQ_REL);
	for (std::size_t word = 0U; word < configuration_words; ++word) {
		std::uint32_t value{};
		std::memcpy(&value,
			reinterpret_cast<const std::uint8_t *>(&configuration) +
				word * sizeof(value), sizeof(value));
		__atomic_store_n(&staged_words_[word], value, __ATOMIC_RELAXED);
	}
	(void)__atomic_add_fetch(&staged_revision_, 1U, __ATOMIC_RELEASE);
	return true;
}

bool MainsSignalEngine::load_staged(
	msap1_m18_config_payload &configuration) const noexcept
{
	for (std::size_t attempt = 0U; attempt < 3U; ++attempt) {
		const auto before = __atomic_load_n(&staged_revision_, __ATOMIC_ACQUIRE);
		if (before == 0U || (before & 1U) != 0U)
			return false;
		for (std::size_t word = 0U; word < configuration_words; ++word) {
			const auto value = __atomic_load_n(&staged_words_[word],
				__ATOMIC_RELAXED);
			std::memcpy(reinterpret_cast<std::uint8_t *>(&configuration) +
				word * sizeof(value), &value, sizeof(value));
		}
		if (__atomic_load_n(&staged_revision_, __ATOMIC_ACQUIRE) == before)
			return true;
	}
	return false;
}

bool MainsSignalEngine::initialize() noexcept
{
	output_sequence_ = 0U;
	last_input_sequence_ = 0U;
	have_active_configuration_ = false;
	have_input_sequence_ = false;
	external_discontinuity_ = false;
	first_after_discontinuity_ = true;
	ready_ = true;
	return true;
}

void MainsSignalEngine::note_transport_discontinuity() noexcept
{
	external_discontinuity_ = true;
}

void MainsSignalEngine::fail() noexcept
{
	ready_ = false;
	health_.set_engine_ready(false);
}

bool MainsSignalEngine::apply_matching_configuration(
	const MainsSignalInputView &input) noexcept
{
	if (have_active_configuration_ &&
		active_configuration_.generation == input.configuration_generation)
		return active_configuration_.mains_carrier_millihz ==
				input.configured_millihz &&
			active_configuration_.mains_bandwidth_millihz ==
				input.bandwidth_millihz &&
			active_configuration_.mains_observation_ms == input.observation_ms &&
			active_configuration_.mains_threshold_e4 == input.threshold_e4;

	if (!load_staged(candidate_configuration_) ||
		candidate_configuration_.generation != input.configuration_generation ||
		(candidate_configuration_.mains_flags & MSAP1_M18_ENGINE_ENABLED) == 0U ||
		candidate_configuration_.mains_carrier_millihz !=
			input.configured_millihz ||
		candidate_configuration_.mains_bandwidth_millihz !=
			input.bandwidth_millihz ||
		candidate_configuration_.mains_observation_ms != input.observation_ms ||
		candidate_configuration_.mains_threshold_e4 != input.threshold_e4 ||
		(input.valid_phase_mask & ~candidate_configuration_.mains_phase_mask) !=
			0U)
		return false;
	active_configuration_ = candidate_configuration_;
	have_active_configuration_ = true;
	return true;
}

void MainsSignalEngine::emit(const MainsSignalInputView &input,
	bool sequence_discontinuity) noexcept
{
	if (!ready_)
		return;
	AggregationMeterRecord record{};
	record.sequence = ++output_sequence_;
	auto &words = record.words;
	words[0U] = meter_record_magic;
	words[1U] = mains_signal_record_format;
	words[2U] = record_bytes;
	words[3U] = record.sequence;
	words[4U] = input.configuration_generation;
	words[5U] = input.sample_rate_hz;
	words[6U] = input.sample_rate_hz / 5U;
	words[7U] = static_cast<std::uint32_t>(input.valid_phase_mask) << 4U;
	if ((input.status & (1U << 4U)) != 0U)
		words[8U] |= 1U;
	if (sequence_discontinuity || (input.status & (1U << 3U)) != 0U ||
		first_after_discontinuity_) {
		words[8U] |= 1U << 2U;
		first_after_discontinuity_ = false;
	}
	words[9U] = low(input.first_sample);
	words[10U] = high(input.first_sample);
	words[13U] = static_cast<std::uint32_t>(input.valid_phase_mask) |
		(static_cast<std::uint32_t>(input.detected_phase_mask) << 8U);
	words[14U] = low(input.last_sample);
	words[15U] = high(input.last_sample);
	words[16U] = input.configured_millihz;
	words[17U] = input.measured_millihz;
	for (std::size_t phase = 0U;
		phase < MainsSignalProtocol::phases; ++phase) {
		words[18U + phase] = input.magnitude_microvolts[phase];
		words[21U + phase] = input.background_microvolts[phase];
	}
	words[24U] = input.bandwidth_millihz;
	words[25U] = input.observation_ms;
	words[26U] = input.configuration_generation;
	words[27U] = input.status;
	words[28U] = input.threshold_e4;
	words[29U] = active_configuration_.reference_voltage_microvolts;
	if (!sink_.publish(record))
		fail();
}

void MainsSignalEngine::process(const MainsSignalInputView &input) noexcept
{
	if (!ready_)
		return;
	if (!apply_matching_configuration(input)) {
		external_discontinuity_ = true;
		return;
	}
	if (have_input_sequence_ && input.sequence == last_input_sequence_)
		return;
	const bool sequence_discontinuity = external_discontinuity_ ||
		(have_input_sequence_ && input.sequence != last_input_sequence_ + 1U);
	last_input_sequence_ = input.sequence;
	have_input_sequence_ = true;
	external_discontinuity_ = false;
	if (sequence_discontinuity)
		first_after_discontinuity_ = true;
	emit(input, sequence_discontinuity);
}

} // namespace msap1::aggregation
