#include "led_red.h"
#include "stm32f4xx_hal.h"

#define LED_PORT  GPIOD
#define LED_PIN   GPIO_PIN_14


void led_red_on() {
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void led_red_off() {
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void led_red_toggle() {
	HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}
