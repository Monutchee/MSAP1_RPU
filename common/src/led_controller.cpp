/*
 * LED helper implementation. See led_controller.hpp.
 */

#include "led_controller.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "xstatus.h"

namespace msap1 {

LedController::LedController(std::uint32_t gpio_base, std::uint32_t led_mask,
			    std::uint32_t heartbeat_period_ms)
	: gpio_base_(gpio_base), led_mask_(led_mask),
	  heartbeat_period_ms_(heartbeat_period_ms)
{
}

void LedController::apply_state(std::uint8_t on)
{
	std::uint32_t value = XGpio_DiscreteRead(&gpio_, 1);

	if (on)
		value |= led_mask_;
	else
		value &= ~led_mask_;

	XGpio_DiscreteWrite(&gpio_, 1, value);
}

bool LedController::init()
{
	if (XGpio_Initialize(&gpio_, gpio_base_) != XST_SUCCESS) {
		return false;
	}

	XGpio_SetDataDirection(&gpio_, 1, 0x00);
	apply_state(0);
	return true;
}

std::uint32_t LedController::set_mode(std::uint8_t mode)
{
	switch (mode) {
	case MSAP1_RPU_LED_OFF:
		led_mode_ = mode;
		led_on_ = 0;
		apply_state(led_on_);
		return MSAP1_RPU_STATUS_OK;
	case MSAP1_RPU_LED_ON:
		led_mode_ = mode;
		led_on_ = 1;
		apply_state(led_on_);
		return MSAP1_RPU_STATUS_OK;
	case MSAP1_RPU_LED_TOGGLE:
		led_mode_ = mode;
		led_on_ = !led_on_;
		apply_state(led_on_);
		return MSAP1_RPU_STATUS_OK;
	case MSAP1_RPU_LED_HEARTBEAT:
		led_mode_ = mode;
		return MSAP1_RPU_STATUS_OK;
	default:
		return MSAP1_RPU_STATUS_BAD_PAYLOAD;
	}
}

void LedController::run_heartbeat()
{
	while (1) {
		if (led_mode_ == MSAP1_RPU_LED_HEARTBEAT) {
			led_on_ = !led_on_;
			apply_state(led_on_);
			heartbeat_count_++;
			vTaskDelay(pdMS_TO_TICKS(heartbeat_period_ms_));
		} else {
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}

void LedController::fill_status(msap1_rpu_status_payload &status) const
{
	status.led_mode = led_mode_;
	status.led_on = led_on_;
	status.heartbeat_count = heartbeat_count_;
}

} // namespace msap1
