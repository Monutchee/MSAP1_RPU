#ifndef MSAP1_LED_CONTROLLER_HPP
#define MSAP1_LED_CONTROLLER_HPP

/*
 * Reusable LED helper for the MSAP1 RPU firmware. This is a separate, optional
 * building block: it owns an AXI GPIO LED and the heartbeat behaviour, and is
 * intentionally kept OUT of the rpmsg communication core (ControlService).
 *
 * A core that has an LED (KR260 has one, "UF2_LED", driven by R5c0) composes a
 * LedController, runs run_heartbeat() as a FreeRTOS task body, and forwards
 * set_mode()/fill_status() from its ControlService subclass.
 */

#include <cstdint>

#include "xgpio.h"

#include "rpu_control_protocol.h"

namespace msap1 {

class LedController {
public:
	/*
	 * gpio_base   : AXI GPIO base address (e.g. XPAR_XGPIO_0_BASEADDR)
	 * led_mask    : bit(s) of GPIO channel 1 driving the LED
	 * heartbeat_period_ms : half-period of the heartbeat blink
	 */
	LedController(std::uint32_t gpio_base, std::uint32_t led_mask,
		      std::uint32_t heartbeat_period_ms);

	/* Initialise the GPIO and drive the LED off. Returns true on success. */
	bool init();

	/* Apply an MSAP1_RPU_LED_* mode. Returns an MSAP1_RPU_STATUS_* code. */
	std::uint32_t set_mode(std::uint8_t mode);

	/* Blocking heartbeat loop; use as a FreeRTOS task body. */
	void run_heartbeat();

	/* Fill the LED-related status fields. */
	void fill_status(msap1_rpu_status_payload &status) const;

private:
	void apply_state(std::uint8_t on);

	std::uint32_t gpio_base_;
	std::uint32_t led_mask_;
	std::uint32_t heartbeat_period_ms_;

	XGpio gpio_{};

	volatile std::uint32_t heartbeat_count_ = 0;
	volatile std::uint8_t led_mode_ = MSAP1_RPU_LED_HEARTBEAT;
	volatile std::uint8_t led_on_ = 0;
};

} // namespace msap1

#endif /* MSAP1_LED_CONTROLLER_HPP */
