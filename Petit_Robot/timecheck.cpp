#include "timecheck.h"
#include "moteurs.h"
#include <Arduino.h>

#define MATCH_DURATION 200000

unsigned long match_start;


void startingMatch() {
	match_start = millis();
}

void time_loop() {
	if(millis() - match_start > MATCH_DURATION) {
		endless_stop();
	}
}

void endless_stop() {
	moteurs_fast_stop();
	while(true) {
		delay(100);
	}
}

