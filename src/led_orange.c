#include "led_orange.h"
#include "stm32f4xx_hal.h"

#define LED_PORT  GPIOD
#define LED_PIN   GPIO_PIN_13


void led_orange_on() {
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void led_orange_off() {
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void led_orange_toggle() {
	HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}
