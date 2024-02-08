#include <string.h>

#include "uart.h"
#include "rtc.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

size_t uart_write(const char* buffer, const size_t size) {
    uint8_t tentativas = 0;
	size_t retval = 0;
	do {
		if ( CDC_Transmit_FS(buffer, size) == USBD_OK ) {
			retval = size;
			break;
		} else {
			sleep_ms(50);
			tentativas += 1;
		}
	} while (tentativas<3);

	return retval;
}

size_t uart_puts(const char* buffer) {
	const size_t size = strlen(buffer);
	return uart_write(buffer, size);
}

size_t uart_read(char* buffer, const size_t maxsize) {
	return 0;
}
