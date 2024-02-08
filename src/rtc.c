#include "stm32f4xx_hal.h"

void sleep_ms(size_t time) {
	HAL_Delay(time);
}
