#ifndef MSAP1_AD7771_HPP
#define MSAP1_AD7771_HPP

#include "adc_device.hpp"
#include "xspi.h"

namespace msap1::adc {

struct Hardware {
	std::uintptr_t spi_base;
	std::uintptr_t capture_base;
};

class Ad7771 final : public AdcDevice {
public:
	// The application supplies addresses from its generated platform
	// definitions. Keeping them outside this reusable library prevents an XSA
	// address-map change from silently leaving stale literal addresses here.
	explicit Ad7771(Hardware hardware);

	// Reset the ADC, configure its SPI registers, and leave PL capture stopped.
	// Linux owns AXI DMA and must arm it before requesting start_capture().
	Source source() const override { return Source::Physical; }
	Error initialize(const Configuration &configuration = Configuration{}) override;

	// Reset the PL FIFO and enable the stream. The caller must ensure Linux has
	// armed the IIO DMA channel before invoking this operation.
	Error start_capture() override;
	void stop_capture() override;
	Error configure_operating_point(
		SampleRate sample_rate,
		const std::array<PgaGain, channel_count> &channel_gains) override;
	Error configure_pga(
		const std::array<PgaGain, channel_count> &channel_gains);
	/*
	 * Run the destructive Flow-1 diagnostic while capture is stopped. This
	 * pulses the sensor-board ADC RESET_N output driven by PL; it does not
	 * power-cycle or reset Linux/the FPGA. The active Configuration is restored
	 * before returning.
	 */
	Error run_diagnostic_flow1(DiagnosticResult &result);
	DeviceStatus status() const override;
	Error read_register_health(RegisterHealth &health);

	const Configuration &configuration() const override { return configuration_; }
	const SpiHealthDiagnostics &spi_health_diagnostics() const
	{
		return spi_health_diagnostics_;
	}
	bool initialized() const override { return initialized_; }
	bool capture_active() const override { return capture_active_; }

	Ad7771(const Ad7771 &) = delete;
	Ad7771 &operator=(const Ad7771 &) = delete;

private:
	struct SrcLoadTrace {
		std::uint8_t high_readback = 0;
		std::uint8_t low_readback = 0;
	};

	static constexpr std::uint32_t control_capture_enable = 1u << 0;
	static constexpr std::uint32_t control_fifo_reset = 1u << 1;
	static constexpr std::uint32_t control_adc_reset_n = 1u << 2;
	static constexpr std::uint32_t control_adc_start = 1u << 3;

	Error initialize_spi();
	Error reset_and_configure_adc();
	Error wait_for_initialization();
	Error configure_adc_registers(SrcLoadTrace *trace = nullptr,
				      unsigned long src_update_hold_us = 10u);
	Error configure_sample_rate(SampleRate sample_rate,
				    SrcLoadTrace *trace = nullptr,
				    unsigned long update_hold_us = 10u);
	Error program_channel_gains(
		const std::array<PgaGain, channel_count> &channel_gains);
	Error synchronize_adc();
	Error take_diagnostic_snapshot(DiagnosticSnapshot &snapshot,
				       bool include_spi);

	Error write_adc_register(std::uint8_t address, std::uint8_t value);
	Error read_adc_register(std::uint8_t address, std::uint8_t &value);
	/* Read-twice-compare for health/status reads; see the definition. */
	Error read_adc_register_confirmed(std::uint8_t address,
					  std::uint8_t &value);
	Error update_adc_register(std::uint8_t address, std::uint8_t mask,
			  std::uint8_t value);

	std::uint32_t capture_read(std::uint32_t offset) const;
	void capture_write(std::uint32_t offset, std::uint32_t value);
	void set_capture_control(std::uint32_t value);

	Hardware hardware_;
	Configuration configuration_{};
	XSpi spi_{};
	std::uint32_t control_shadow_ = control_fifo_reset;
	bool spi_initialized_ = false;
	bool initialized_ = false;
	bool capture_active_ = false;
	SpiHealthDiagnostics spi_health_diagnostics_{};
};

} // namespace msap1::adc

#endif // MSAP1_AD7771_HPP
