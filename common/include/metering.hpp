#ifndef MSAP1_METERING_HPP
#define MSAP1_METERING_HPP

#include <array>
#include <cstddef>
#include <cstdint>

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
		std::uint32_t window_samples = 32000;
		std::uint32_t minimum_millihz = 40000;
		std::uint32_t maximum_millihz = 70000;
		std::uint32_t hysteresis_microvolts = 1000000;
	};

	std::uint32_t generation = 0;
	std::uint32_t sample_rate_hz = 32000;
	std::uint32_t rms_window_samples = 6400;
	std::uint8_t valid_mask = 0;
	std::array<std::uint32_t, channel_count> scale_micro_units_q16{};
	bool enable = true;
	bool remove_dc = true;
	Frequency frequency{};
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
};

} // namespace msap1::meter

#endif // MSAP1_METERING_HPP
