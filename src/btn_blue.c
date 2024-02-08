#include "stm32f4xx_hal.h"
#include "btn_blue.h"

bool btn_blue_is_pressed() {
	GPIO_PinState state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	return state == GPIO_PIN_SET;
}
