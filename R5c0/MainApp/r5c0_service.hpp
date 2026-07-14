#ifndef MSAP1_R5C0_SERVICE_HPP
#define MSAP1_R5C0_SERVICE_HPP

/*
 * R5 core 0 control service.
 *
 * R5c0 owns the board's single LED ("UF2_LED", bit 0x01 of AXI_GPIO_0) and runs
 * the heartbeat. It extends the comm-only ControlService with LED behaviour by
 * composing a LedController and overriding the LED hooks.
 */

#include "xparameters.h"

#include "control_service.hpp"
#include "led_controller.hpp"

class R5c0Service : public msap1::ControlService {
public:
	explicit R5c0Service(const msap1::CoreConfig &config)
		: msap1::ControlService(config),
		  led_(XPAR_XGPIO_0_BASEADDR, /*led_mask=*/0x01u,
		       /*heartbeat_period_ms=*/200u)
	{
	}

	/* Initialise the LED GPIO. Call before starting the scheduler. */
	bool init_led() { return led_.init(); }

	/* Heartbeat task body. */
	void run_heartbeat() { led_.run_heartbeat(); }

protected:
	std::uint32_t on_set_led(std::uint8_t mode) override
	{
		return led_.set_mode(mode);
	}

	void on_fill_status(msap1_rpu_status_payload &status) override
	{
		led_.fill_status(status);
	}

private:
	msap1::LedController led_;
};

#endif /* MSAP1_R5C0_SERVICE_HPP */
