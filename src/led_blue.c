#include "led_blue.h"
#include "stm32f4xx_hal.h"

#define LED_PORT  GPIOD
#define LED_PIN   GPIO_PIN_15


void led_blue_on() {
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void led_blue_off() {
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void led_blue_toggle() {
	HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}
