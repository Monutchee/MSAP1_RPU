#include "mains_signal_engine.hpp"

#include "metrology_sine_lut.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t meter_record_magic = 0x3152544DU;
constexpr std::uint32_t mains_signal_record_format = 0x000F0001U;
constexpr std::uint32_t record_bytes = 256U;
constexpr std::uint32_t observation_ms = 200U;
/*
 * The analogue frontend is band-limited below 12.5 kHz.  A 32 kSPS analysis
 * rate therefore retains every supported mains-signalling frequency while
 * avoiding seven-probe correlation at the 128 kSPS transport rate.
 */
constexpr std::uint32_t maximum_analysis_rate_hz = 32000U;
constexpr std::uint32_t maximum_analysis_window_samples =
	maximum_analysis_rate_hz / 5U;
constexpr std::uint64_t maximum_correlation_term =
	std::uint64_t{1U} << 48U;
static_assert(maximum_correlation_term * maximum_analysis_window_samples <
	static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()));
constexpr std::array<int, 7U> probe_offset_quarters{
	-4, -2, -1, 0, 1, 2, 4};
constexpr std::array<std::uint32_t, 3U> valid_bits{
	VoltageSampleProtocol::sample_valid_a,
	VoltageSampleProtocol::sample_valid_b,
	VoltageSampleProtocol::sample_valid_c};

std::uint32_t low(std::uint64_t value) noexcept
{
	return static_cast<std::uint32_t>(value);
}

std::uint32_t high(std::uint64_t value) noexcept
{
	return static_cast<std::uint32_t>(value >> 32U);
}

bool supported_sample_rate(std::uint32_t rate) noexcept
{
	switch (rate) {
	case 2000U:
	case 4000U:
	case 8000U:
	case 16000U:
	case 32000U:
	case 64000U:
	case 128000U:
		return true;
	default:
		return false;
	}
}

std::uint32_t phase_step_q32(std::uint32_t carrier_millihz,
	std::uint32_t bandwidth_millihz, int offset_quarters,
	std::uint32_t sample_rate_hz) noexcept
{
	const auto frequency_quarter_millihz =
		static_cast<std::int64_t>(carrier_millihz) * 4LL +
		static_cast<std::int64_t>(bandwidth_millihz) * offset_quarters;
	if (frequency_quarter_millihz <= 0 || sample_rate_hz == 0U)
		return 0U;
	const auto numerator =
		static_cast<std::uint64_t>(frequency_quarter_millihz) << 32U;
	const auto denominator =
		static_cast<std::uint64_t>(sample_rate_hz) * 4000U;
	return static_cast<std::uint32_t>(
		(numerator + denominator / 2U) / denominator);
}

std::int32_t sine_point(std::uint16_t point) noexcept
{
	const auto quadrant = static_cast<std::uint16_t>(point >> 10U);
	const auto q = static_cast<std::uint16_t>(point & 0x3ffU);
	const auto index = (quadrant & 1U) == 0U ? q : 1024U - q;
	const auto value = MET_SINE_QLUT[index];
	return (quadrant & 2U) == 0U ? value : -value;
}

std::int32_t sine_q17(std::uint32_t phase) noexcept
{
	const auto point = static_cast<std::uint16_t>(phase >> 20U);
	const auto fraction = phase & 0xfffffU;
	const auto first = sine_point(point);
	const auto second = sine_point(static_cast<std::uint16_t>(point + 1U));
	const auto interpolated =
		static_cast<std::int64_t>(first) * (1LL << 20U) +
		static_cast<std::int64_t>(second - first) * fraction;
	return static_cast<std::int32_t>(interpolated / (1LL << 20U));
}

std::int32_t cosine_q17(std::uint32_t phase) noexcept
{
	return sine_q17(phase + 0x40000000U);
}

bool add_saturating(std::int64_t &accumulator, std::int64_t addend) noexcept
{
	if (addend > 0 && accumulator >
		std::numeric_limits<std::int64_t>::max() - addend) {
		accumulator = std::numeric_limits<std::int64_t>::max();
		return false;
	}
	if (addend < 0 && accumulator <
		std::numeric_limits<std::int64_t>::min() - addend) {
		accumulator = std::numeric_limits<std::int64_t>::min();
		return false;
	}
	accumulator += addend;
	return true;
}

std::uint64_t magnitude(std::int64_t value) noexcept
{
	return value < 0
		? static_cast<std::uint64_t>(-(value + 1)) + 1U
		: static_cast<std::uint64_t>(value);
}

std::int64_t mean_component(std::int64_t sum, std::uint32_t count) noexcept
{
	if (count == 0U)
		return 0;
	const auto scaled = magnitude(sum) / count / (1U << 17U);
	return sum < 0 ? -static_cast<std::int64_t>(scaled)
		: static_cast<std::int64_t>(scaled);
}

std::uint64_t integer_sqrt(std::uint64_t value) noexcept
{
	std::uint64_t result = 0U;
	std::uint64_t bit = std::uint64_t{1U} << 62U;
	while (bit > value)
		bit >>= 2U;
	while (bit != 0U) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1U) + bit;
		} else {
			result >>= 1U;
		}
		bit >>= 2U;
	}
	return result;
}

std::uint32_t phasor_rms_microvolts(std::int64_t real,
	std::int64_t imaginary, bool &overflow) noexcept
{
	const auto real_magnitude = magnitude(real);
	const auto imaginary_magnitude = magnitude(imaginary);
	const auto square = real_magnitude * real_magnitude +
		imaginary_magnitude * imaginary_magnitude;
	const auto root = integer_sqrt(square);
	const auto result = (root * 92682U) >> 16U;
	if (result > std::numeric_limits<std::uint32_t>::max()) {
		overflow = true;
		return std::numeric_limits<std::uint32_t>::max();
	}
	return static_cast<std::uint32_t>(result);
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
	have_active_configuration_ = false;
	have_input_sequence_ = false;
	have_last_input_sample_ = false;
	external_discontinuity_ = false;
	pending_discontinuity_ = true;
	output_sequence_ = 0U;
	last_input_sequence_ = 0U;
	sample_rate_hz_ = 0U;
	analysis_rate_hz_ = 0U;
	observation_samples_ = 0U;
	decimation_divisor_ = 1U;
	decimation_phase_ = 0U;
	clear_window();
	probe_phase_.fill(0U);
	probe_step_.fill(0U);
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

void MainsSignalEngine::clear_window() noexcept
{
	for (auto &phase : real_sum_)
		phase.fill(0);
	for (auto &phase : imaginary_sum_)
		phase.fill(0);
	window_span_count_ = 0U;
	window_count_ = 0U;
	window_valid_mask_ = have_active_configuration_
		? static_cast<std::uint8_t>(
			active_configuration_.mains_phase_mask & 0x7U)
		: 0U;
	window_locked_ = true;
	window_fallback_ = false;
	arithmetic_overflow_ = false;
}

void MainsSignalEngine::reset_signal_path(bool contaminated) noexcept
{
	clear_window();
	probe_phase_.fill(0U);
	decimation_phase_ = 0U;
	have_last_input_sample_ = false;
	if (contaminated)
		pending_discontinuity_ = true;
}

bool MainsSignalEngine::apply_matching_configuration(
	const VoltageSampleInputView &input) noexcept
{
	if (have_active_configuration_ &&
		active_configuration_.generation == input.configuration_generation) {
		if ((active_configuration_.mains_flags &
				MSAP1_M18_ENGINE_ENABLED) == 0U ||
			(active_configuration_.mains_phase_mask &
				~static_cast<std::uint32_t>(input.phase_mask)) != 0U ||
			active_configuration_.reference_voltage_microvolts !=
				input.reference_microvolts)
			return false;
		if (sample_rate_hz_ == input.sample_rate_hz)
			return true;
	}

	if (!load_staged(candidate_configuration_) ||
		candidate_configuration_.generation != input.configuration_generation ||
		(candidate_configuration_.mains_flags &
			MSAP1_M18_ENGINE_ENABLED) == 0U ||
		candidate_configuration_.mains_observation_ms != observation_ms ||
		candidate_configuration_.mains_phase_mask == 0U ||
		(candidate_configuration_.mains_phase_mask &
			~static_cast<std::uint32_t>(input.phase_mask)) != 0U ||
		candidate_configuration_.reference_voltage_microvolts !=
			input.reference_microvolts ||
		!supported_sample_rate(input.sample_rate_hz))
		return false;

	const auto carrier = candidate_configuration_.mains_carrier_millihz;
	const auto bandwidth = candidate_configuration_.mains_bandwidth_millihz;
	const auto nyquist_millihz =
		static_cast<std::uint64_t>(input.sample_rate_hz) * 500U;
	if (carrier == 0U || bandwidth < 4U || bandwidth >= carrier ||
		static_cast<std::uint64_t>(carrier) + bandwidth >=
			nyquist_millihz ||
		static_cast<std::uint64_t>(carrier) + bandwidth >= 12500000U ||
		candidate_configuration_.mains_threshold_e4 > 0xffffU)
		return false;

	active_configuration_ = candidate_configuration_;
	have_active_configuration_ = true;
	sample_rate_hz_ = input.sample_rate_hz;
	analysis_rate_hz_ = std::min(sample_rate_hz_, maximum_analysis_rate_hz);
	decimation_divisor_ = static_cast<std::uint16_t>(
		sample_rate_hz_ / analysis_rate_hz_);
	observation_samples_ = sample_rate_hz_ / 5U;
	for (std::size_t probe = 0U; probe < probes; ++probe)
		probe_step_[probe] = phase_step_q32(carrier, bandwidth,
			probe_offset_quarters[probe], analysis_rate_hz_);
	have_input_sequence_ = false;
	reset_signal_path(true);
	return true;
}

void MainsSignalEngine::process_sample(const VoltageSampleInputView &input,
	std::size_t offset, std::uint64_t sample_index) noexcept
{
	const auto *words = input.packed_sample_words +
		offset * VoltageSampleProtocol::words_per_sample;
	const std::array<std::int32_t, phases> voltage{
		static_cast<std::int32_t>(words[0U]),
		static_cast<std::int32_t>(words[1U]),
		static_cast<std::int32_t>(words[2U])};
	const auto flags = words[3U];
	const bool sequence_gap = have_last_input_sample_ &&
		sample_index != last_input_sample_ + 1U;
	if (sequence_gap)
		reset_signal_path(true);
	last_input_sample_ = sample_index;
	have_last_input_sample_ = true;
	if ((flags & VoltageSampleProtocol::sample_malformed) != 0U) {
		reset_signal_path(true);
		last_input_sample_ = sample_index;
		have_last_input_sample_ = true;
		return;
	}
	if ((flags & VoltageSampleProtocol::sample_saturated) != 0U)
		arithmetic_overflow_ = true;
	if (window_span_count_ == 0U)
		window_first_sample_ = sample_index;
	window_locked_ = window_locked_ &&
		(flags & VoltageSampleProtocol::sample_locked) != 0U;
	window_fallback_ = window_fallback_ ||
		(flags & VoltageSampleProtocol::sample_fallback) != 0U;

	for (std::size_t phase = 0U; phase < phases; ++phase) {
		const bool valid =
			(active_configuration_.mains_phase_mask & (1U << phase)) != 0U &&
			(flags & valid_bits[phase]) != 0U;
		if (!valid)
			window_valid_mask_ &= static_cast<std::uint8_t>(~(1U << phase));
	}

	const bool analyse = decimation_phase_ == 0U;
	if (++decimation_phase_ >= decimation_divisor_)
		decimation_phase_ = 0U;
	if (analyse) {
		for (std::size_t probe = 0U; probe < probes; ++probe) {
			const auto cosine = cosine_q17(probe_phase_[probe]);
			const auto sine = sine_q17(probe_phase_[probe]);
			for (std::size_t phase = 0U; phase < phases; ++phase) {
				if ((window_valid_mask_ & (1U << phase)) == 0U)
					continue;
				/*
				 * A 200 ms window contains at most 6,400 analysed samples.
				 * Signed 32-bit microvolts multiplied by a Q17 oscillator are
				 * bounded by 2^48 per term, so the complete correlation is
				 * provably inside int64_t. Avoid two saturating branch trees per
				 * phase/probe/sample on this dominant 32 kSPS target path.
				 */
				real_sum_[phase][probe] +=
					static_cast<std::int64_t>(voltage[phase]) * cosine;
				imaginary_sum_[phase][probe] -=
					static_cast<std::int64_t>(voltage[phase]) * sine;
			}
			probe_phase_[probe] += probe_step_[probe];
		}
		++window_count_;
	}

	++window_span_count_;
	if (window_span_count_ >= observation_samples_)
		complete_window(sample_index);
}

void MainsSignalEngine::complete_window(std::uint64_t last_sample) noexcept
{
	std::array<std::array<std::uint32_t, probes>, phases> magnitude_uv{};
	for (std::size_t phase = 0U; phase < phases; ++phase)
		for (std::size_t probe = 0U; probe < probes; ++probe)
			magnitude_uv[phase][probe] = phasor_rms_microvolts(
				mean_component(real_sum_[phase][probe], window_count_),
				mean_component(imaginary_sum_[phase][probe], window_count_),
				arithmetic_overflow_);

	std::array<std::uint32_t, phases> carrier_magnitude{};
	std::array<std::uint32_t, phases> background_magnitude{};
	std::array<std::uint64_t, 5U> probe_weight{};
	bool background_dominant = false;
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		for (std::size_t inner = 0U; inner < probe_weight.size(); ++inner) {
			carrier_magnitude[phase] = std::max(carrier_magnitude[phase],
				magnitude_uv[phase][inner + 1U]);
			if ((window_valid_mask_ & (1U << phase)) != 0U)
				probe_weight[inner] += magnitude_uv[phase][inner + 1U];
		}
		background_magnitude[phase] = std::max(
			magnitude_uv[phase][0U], magnitude_uv[phase][6U]);
		if ((window_valid_mask_ & (1U << phase)) != 0U &&
			background_magnitude[phase] > carrier_magnitude[phase])
			background_dominant = true;
	}

	const auto threshold_uv =
		(static_cast<std::uint64_t>(
			active_configuration_.reference_voltage_microvolts) *
			active_configuration_.mains_threshold_e4 + 9999U) / 10000U;
	std::uint8_t detected_mask = 0U;
	for (std::size_t phase = 0U; phase < phases; ++phase)
		if ((window_valid_mask_ & (1U << phase)) != 0U &&
			carrier_magnitude[phase] >= threshold_uv)
			detected_mask |= static_cast<std::uint8_t>(1U << phase);

	auto measured_millihz = active_configuration_.mains_carrier_millihz;
	std::uint64_t weight_total = 0U;
	std::int64_t weighted_offset = 0;
	for (std::size_t inner = 0U; inner < probe_weight.size(); ++inner) {
		weight_total += probe_weight[inner];
		const auto term = static_cast<std::int64_t>(probe_weight[inner]) *
			(static_cast<std::int64_t>(inner) - 2LL) *
			active_configuration_.mains_bandwidth_millihz;
		if (!add_saturating(weighted_offset, term))
			arithmetic_overflow_ = true;
	}
	if (detected_mask != 0U && weight_total != 0U) {
		const auto offset = weighted_offset /
			static_cast<std::int64_t>(weight_total * 4U);
		const auto measured = static_cast<std::int64_t>(
			active_configuration_.mains_carrier_millihz) + offset;
		if (measured < 0) {
			measured_millihz = 0U;
			arithmetic_overflow_ = true;
		} else if (static_cast<std::uint64_t>(measured) >
			std::numeric_limits<std::uint32_t>::max()) {
			measured_millihz = std::numeric_limits<std::uint32_t>::max();
			arithmetic_overflow_ = true;
		} else {
			measured_millihz = static_cast<std::uint32_t>(measured);
		}
	}

	const auto status = 1U |
		(static_cast<std::uint32_t>(window_locked_) << 1U) |
		(static_cast<std::uint32_t>(window_fallback_) << 2U) |
		(static_cast<std::uint32_t>(pending_discontinuity_) << 3U) |
		(static_cast<std::uint32_t>(arithmetic_overflow_) << 4U) |
		(static_cast<std::uint32_t>(background_dominant) << 5U);
	emit(status, window_valid_mask_, detected_mask, measured_millihz,
		carrier_magnitude, background_magnitude, last_sample);
	pending_discontinuity_ = false;
	clear_window();
}

void MainsSignalEngine::emit(std::uint32_t status, std::uint8_t valid_mask,
	std::uint8_t detected_mask, std::uint32_t measured_millihz,
	const std::array<std::uint32_t, phases> &magnitude_microvolts,
	const std::array<std::uint32_t, phases> &background_microvolts,
	std::uint64_t last_sample) noexcept
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
	words[4U] = active_configuration_.generation;
	words[5U] = sample_rate_hz_;
	words[6U] = observation_samples_;
	words[7U] = static_cast<std::uint32_t>(valid_mask) << 4U;
	if ((status & (1U << 4U)) != 0U)
		words[8U] |= 1U;
	if ((status & (1U << 3U)) != 0U)
		words[8U] |= 1U << 2U;
	words[9U] = low(window_first_sample_);
	words[10U] = high(window_first_sample_);
	words[13U] = static_cast<std::uint32_t>(valid_mask) |
		(static_cast<std::uint32_t>(detected_mask) << 8U);
	words[14U] = low(last_sample);
	words[15U] = high(last_sample);
	words[16U] = active_configuration_.mains_carrier_millihz;
	words[17U] = measured_millihz;
	for (std::size_t phase = 0U; phase < phases; ++phase) {
		words[18U + phase] = magnitude_microvolts[phase];
		words[21U + phase] = background_microvolts[phase];
	}
	words[24U] = active_configuration_.mains_bandwidth_millihz;
	words[25U] = observation_ms;
	words[26U] = active_configuration_.generation;
	words[27U] = status;
	words[28U] = active_configuration_.mains_threshold_e4;
	words[29U] = active_configuration_.reference_voltage_microvolts;
	if (!sink_.publish(record))
		fail();
}

void MainsSignalEngine::process(const VoltageSampleInputView &input) noexcept
{
	if (!ready_ || !apply_matching_configuration(input)) {
		note_transport_discontinuity();
		return;
	}
	if (have_input_sequence_ && input.sequence == last_input_sequence_)
		return;
	const bool packet_sequence_gap = have_input_sequence_ &&
		input.sequence != last_input_sequence_ + 1U;
	last_input_sequence_ = input.sequence;
	have_input_sequence_ = true;
	const bool batch_discontinuity =
		(input.batch_status & VoltageSampleProtocol::batch_discontinuity) != 0U;
	const bool source_drop =
		(input.batch_status & VoltageSampleProtocol::batch_source_drop) != 0U;
	if (external_discontinuity_ || packet_sequence_gap || batch_discontinuity)
		reset_signal_path(external_discontinuity_ || packet_sequence_gap ||
			source_drop);
	external_discontinuity_ = false;
	for (std::size_t offset = 0U; offset < input.actual_count; ++offset)
		process_sample(input, offset, input.first_sample + offset);
	if (source_drop ||
		input.actual_count != VoltageSampleProtocol::batch_frames)
		reset_signal_path(true);
}

} // namespace msap1::aggregation
