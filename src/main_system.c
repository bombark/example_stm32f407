// =============================================================================
//  Header
// =============================================================================

#include <stdio.h>

// Led Devices
#include "led_green.h"
#include "led_red.h"
#include "led_orange.h"
#include "led_blue.h"

// Other Devices
#include "btn_blue.h"
#include "uart.h"
#include "rtc.h"
#include "cs43l22.h"
#include "audio.h"

// Sinal
#include "klaxon.h"

void error_state() {
    while(1) {
        led_red_toggle();
        sleep_ms(500);
    }
}

// =============================================================================
//  Main
// =============================================================================

int main_system() {
    if (0 != audio_out_init(OUTPUT_DEVICE_HEADPHONE, 100, 16000)) {
        error_state();
    }

	while(1) {
        // uart_puts("Opa\n");
        led_green_toggle();
		sleep_ms(500);
        if ( btn_blue_is_pressed() ) {
            audio_out_play(klaxon_array, sizeof(klaxon_array));
        }
	}

	return 0;
}
