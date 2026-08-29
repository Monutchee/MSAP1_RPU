#ifndef MSAP1_METERING_HPP
#define MSAP1_METERING_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "rpu_control_protocol.h"

namespace msap1::meter {

inline constexpr std::size_t channel_count = 8;

struct Hardware {
	std::uintptr_t conversion_base;
	std::uintptr_t processing_base;
};

struct Configuration {
	struct Frequency {
		bool enable = true;
		std::uint32_t mode = 1;
		std::uint32_t reference_channel = 6;
		std::uint32_t averaging_cycles = 10;
		std::uint32_t window_samples = 128000;
		std::uint32_t minimum_millihz = 40000;
		std::uint32_t maximum_millihz = 70000;
		std::uint32_t hysteresis_microvolts = 1000000;
	};

	// IEC 61000-4-30 Urms(1/2) event detection (metrology M12).
	// reference_microvolts is the declared reference Udin and ZERO
	// DISABLES DETECTION: the PL keeps publishing Urms(1/2) snapshots
	// but never declares an event, so an unconfigured reference cannot
	// invent dips. Thresholds are fractions of that reference in units
	// of 1e-4 (9000 = 90.00 %), matching the PL's pq_event_pkg.
	struct PowerQuality {
		std::uint32_t reference_microvolts = 0;
		std::uint32_t sag_threshold_e4 = 9000;
		std::uint32_t swell_threshold_e4 = 11000;
		std::uint32_t interruption_threshold_e4 = 1000;
		std::uint32_t hysteresis_e4 = 200;
	};

	std::uint32_t generation = 0;
	std::uint32_t sample_rate_hz = 128000;
	std::uint32_t rms_window_samples = 25600;
	// Declared nominal grid frequency (50 or 60 Hz). Configuration, not the
	// measured frequency: it selects the cycle count of the PL basic
	// measurement block, while measurement stays with the estimator.
	std::uint32_t nominal_frequency_hz = 60;
	std::uint8_t valid_mask = 0;
	std::array<std::uint32_t, channel_count> scale_micro_units_q16{};
	bool enable = true;
	bool remove_dc = true;
	Frequency frequency{};
	PowerQuality power_quality{};
};

struct Status {
	bool cores_present = false;
	bool configured = false;
	bool generation_matches = false;
	bool enabled = false;
	bool remove_dc = false;
	std::uint32_t generation = 0;
	std::uint32_t conversion_active_generation = 0;
	std::uint32_t processing_active_generation = 0;
	std::uint32_t conversion_status = 0;
	std::uint32_t processing_status = 0;
	std::uint32_t frequency_status = 0;
	std::uint32_t grid_status = 0;
	// 150/180-cycle aggregation health (read-only PL counters). The RPU
	// never consumes Basic results or aggregate data; these are health
	// readbacks only and aggregate measurements stay on the DMA path.
	std::uint32_t aggregate_status = 0;
	std::uint32_t aggregate_records = 0;
	std::uint32_t aggregate_resets = 0;
	std::uint32_t aggregate_ineligible = 0;
	std::uint32_t aggregate_continuity_errors = 0;
	std::uint32_t aggregate_drops = 0;
};

enum class Error {
	None,
	InvalidConfiguration,
	CoreNotFound,
	ReadbackMismatch,
};

const char *to_string(Error error);

class MeteringPipeline {
public:
	explicit MeteringPipeline(Hardware hardware);

	Error configure(const Configuration &configuration);
	Error stage_m18_configuration(
		const msap1_m18_config_payload &configuration);
	Status status() const;

	const Configuration &configuration() const { return configuration_; }
	bool configured() const { return configured_; }

	MeteringPipeline(const MeteringPipeline &) = delete;
	MeteringPipeline &operator=(const MeteringPipeline &) = delete;

private:
	std::uint32_t conversion_read(std::uint32_t offset) const;
	void conversion_write(std::uint32_t offset, std::uint32_t value) const;
	std::uint32_t processing_read(std::uint32_t offset) const;
	void processing_write(std::uint32_t offset, std::uint32_t value) const;
	bool cores_present() const;

	Hardware hardware_;
	Configuration configuration_{};
	bool configured_ = false;
	msap1_m18_config_payload m18_configuration_{};
	bool m18_staged_ = false;
};

} // namespace msap1::meter

#endif // MSAP1_METERING_HPP
