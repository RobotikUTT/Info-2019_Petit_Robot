#include <Arduino.h>

#define PIN_TIRETTE A6


void wait_for_tirette() {
	

	while(analogRead(PIN_TIRETTE) < 512) {
		delay(10);
	}
}

