#include "moteurs.h"
#include "asservissement.h"
#include "sensors.h"
#include "servos.cpp"
#include "tirette.cpp"
#include "timecheck.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// #define DEBUG	// Uncomment to enable Serial debuging


bool sideIsYellow;
// LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

const uint8_t len_sequence_1 = 5;
const position_t sequence_1[] = {
	{2, 0,		450,	30,	1},
	{2, 180,	350,	0,	1},
	{2, 0,	-150,	0,	1},
	{0, -90,	-350,	30,	1},
	{1, 90,	-350,	30,	1}
};

const uint8_t len_sequence_2 = 5;
const position_t sequence_2[] = {
	{2, 0, 200, 30, 1},
	{0, -90, 1150, 10, 1 },
	{1, 90, 1150, 10, 1 },
	{0, 90, 0, 10, 1 },
	{1, -90, 0, 10, 1 }
};

const uint8_t len_sequence_3 = 6;
const position_t sequence_3[] = {
	{0, 0, 140, 30, 1},
	{1, 0, 110, 30, 1},
	{0, -90, 400, 10, 1 },
	{1, 90, 400, 10, 1 },
	{0, 90, 0, 10, 1 },
	{1, -90, 0, 10, 1 }
};

void setup() {
	#ifdef DEBUG
	Serial.begin(115200);
	Serial.println(F("Setup ..."));
	#endif

	moteur_init();
	asserv_init();

	sensor_init();
	servo_init();


	lcd.begin();
	lcd.noCursor();
	lcd.noBlink();
	lcd.clear();
	lcd.print("Tire la tirette,");
	lcd.setCursor(0, 1);
	lcd.print("Shlagito bougera");
	delay(300);

	#ifdef DEBUG
	Serial.println(F("Setup done. Waiting for tirette ..."));
	#endif
	wait_for_tirette();
	sideIsYellow = (analogRead(A7) < 512);
	startingMatch();
	lcd.clear();
	lcd.print("Score =  0  :-)");
	#ifdef DEBUG
	Serial.println(F("Tirette out ! Match starting !"));
	#endif
	// deploy_right_arm(true);
	// deploy_left_arm(true);
	// delay(500);

	apply_sequence_preprog(sequence_1, len_sequence_1);
	bool flag = false;
	while(!isSequenceCompleted()) {
		asserv_loop();
		// if(asserv_loop()) {
		// 	if(sensor_loop()) {
		// 		#ifdef DEBUG
		// 		Serial.println(F("ROBOT STOPPED"));
		// 		#endif

		// 		// SIMPLE WAIT
		// 		moteurs_fast_stop();
		// 		do {
		// 			delay(100);
		// 			time_loop();
		// 		} while(sensor_loop());

		// 		restart_after_stop();
		// 		// FULLSTOP
		// 		// endeless_stop()
		// 	}
		// }
		time_loop();
		if(!flag && nbMoveCompleted() >= 4) {
			flag = true;
			lcd.clear();
			lcd.print("Score =  6  :-)");
		}
	}
	// lcd.setCursor(0, 1);
	// lcd.print("Score =  2  :-)");
	unsigned long debut_timeout = millis();
	action_moteur_1(-180);
	action_moteur_2(180);
	while(getDistanceLaser() > 100 && millis() - debut_timeout < 1500) {
		delay(1);
	}
	action_moteur_1(-130);
	action_moteur_2(130);
	while(getDistanceLaser() > 10 && millis() - debut_timeout < 3000) {
		delay(1);
	}
	action_moteur_1(0);
	action_moteur_2(0);

	apply_sequence_preprog(sequence_2, len_sequence_2);
	while(!isSequenceCompleted()) {
		if(asserv_loop()) {
			if(sensor_loop()) {
				#ifdef DEBUG
				Serial.println(F("ROBOT STOPPED"));
				#endif

				// SIMPLE WAIT
				moteurs_fast_stop();
				do {
					delay(100);
					time_loop();
				} while(sensor_loop());

				restart_after_stop();
				// FULLSTOP
				// endeless_stop()
			}
		}
		time_loop();
	}

	debut_timeout = millis();
	action_moteur_1(-180);
	action_moteur_2(180);
	while(getDistanceLaser() > 100 && millis() - debut_timeout < 1500) {
		delay(1);
	}
	action_moteur_1(-130);
	action_moteur_2(130);
	while(getDistanceLaser() > 10 && millis() - debut_timeout < 3000) {
		delay(1);
	}
	action_moteur_1(0);
	action_moteur_2(0);

	if(sideIsYellow) {
		deploy_left_arm(true);
	}
	else {
		deploy_right_arm(true);
	}
	delay(500);
	apply_sequence_preprog(sequence_3, len_sequence_3);
	while(!isSequenceCompleted()) {
		if(asserv_loop()) {
			if(sensor_loop()) {
				#ifdef DEBUG
				Serial.println(F("ROBOT STOPPED"));
				#endif

				// SIMPLE WAIT
				moteurs_fast_stop();
				do {
					delay(100);
					time_loop();
				} while(sensor_loop());

				restart_after_stop();
				// FULLSTOP
				// endeless_stop()
			}
		}
		time_loop();
	}

	action_moteur_1(0);
	action_moteur_2(0);

	lcd.home();
	lcd.print("Score = 26  >;)");
	lcd.setCursor(0, 1);
	lcd.print("Finish !");
	deploy_right_arm(false);
	deploy_left_arm(false);
	delay(1000);
	deploy_right_arm(true);
	deploy_left_arm(true);
	delay(300);
	deploy_right_arm(false);
	deploy_left_arm(false);
	delay(300);
	deploy_right_arm(true);
	deploy_left_arm(true);
	delay(300);
	deploy_right_arm(false);
	deploy_left_arm(false);

	delay(500);
	while(true) {
		delay(100);
	}


	/* Test continue */
	// action_moteur_1(-50);
	// action_moteur_2(50);
	// delay(300);
	// action_moteur_1(-125);
	// action_moteur_2(125);
	// delay(1000);
	// action_moteur_1(-200);
	// action_moteur_2(200);
	// delay(1000);
	// action_moteur_1(-255);
	// action_moteur_2(255);
	// while(true) {
	// 	delay(100);
	// }
	// deploy_right_arm(true);
}

void loop() {
	if(asserv_loop()) {
		if(sensor_loop()) {
			#ifdef DEBUG
			Serial.println(F("ROBOT STOPPED"));
			#endif

			// SIMPLE WAIT
			moteurs_fast_stop();
			do {
				delay(100);
			} while(sensor_loop());

			restart_after_stop();
			// FULLSTOP
			// endeless_stop()
		}
	}


	time_loop();

	#ifdef DEBUG
	static unsigned long last_print;
	if(millis() - last_print > 2000) {
		last_print = millis();
		Serial.println(F("Still running"));
	}
	#endif
}




// const uint8_t len_seq = 1;
// const position_t seq_preprog[len_seq] = {
// 	{0, -500, 30, 1}
// };

// const uint8_t len_seq = 6;
// const position_t seq_preprog[len_seq] = {
// 	{0, 500},
// 	{-90, 500},
// 	{90, 500},
// 	{90, 500},
// 	{90, 1000},
// 	{180, 0}
// };

void apply_sequence_preprog(position_t * sequence, uint8_t len) {
	for(byte i=0; i<len; i++) {
		if(sideIsYellow && sequence[i].color == 0) {
			continue;
		}
		if(!sideIsYellow && sequence[i].color == 1) {
			continue;
		}
		#ifdef DEBUG
		Serial.print(sequence[i].angle);
		Serial.print(" ");
		Serial.println(sequence[i].radius);
		#endif
		enqueuePosition(sequence[i]);
	}
}


