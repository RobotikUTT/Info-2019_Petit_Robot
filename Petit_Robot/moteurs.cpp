#include "moteurs.h"
#include <Arduino.h>

//sens moteur 1 et 2
#define M1IN1 8
#define M1IN2 9
#define M2IN1 10
#define M2IN2 11

// Commande de vitesse moteurs

#define M1D2PWM 5
#define M2D2PWM 6

float powerrates[2];

void moteur_init() {
	pinMode(M1D2PWM, OUTPUT);
	pinMode(M2D2PWM, OUTPUT);
	pinMode(M1IN1, OUTPUT);
	pinMode(M1IN2, OUTPUT);
	pinMode(M2IN1, OUTPUT);
	pinMode(M2IN2, OUTPUT);

	analogWrite(M1D2PWM, 0);	// Initialisation sortie moteur Ã  0
	analogWrite(M2D2PWM, 0);
}

void action_moteur_1(float powerrate) {
	powerrates[0] = powerrate;
	if (powerrate > 0) {
		digitalWrite( M1IN1, HIGH );
		digitalWrite( M1IN2, LOW);
		analogWrite( M1D2PWM, powerrate );
	}
	else {
		digitalWrite( M1IN1, LOW );
		digitalWrite( M1IN2, HIGH);
		analogWrite( M1D2PWM, -powerrate );
	}
}

void action_moteur_2(float powerrate) {
	powerrates[0] = powerrate;
	if (powerrate > 0) {
		digitalWrite( M2IN1, HIGH );
		digitalWrite( M2IN2, LOW);
		analogWrite( M2D2PWM, powerrate );
	}
	else {
		digitalWrite( M2IN1, LOW );
		digitalWrite( M2IN2, HIGH);
		analogWrite( M2D2PWM, -powerrate );
	}
}

void moteurs_fast_stop() {
	uint8_t short_delay_1 = abs(powerrates[0]) / 4;
	uint8_t short_delay_2 = abs(powerrates[1]) / 4;
	action_moteur_1(!powerrates[0] ? 0 : -255 * powerrates[0] / abs(powerrates[0]));
	action_moteur_2(!powerrates[1] ? 0 : -255 * powerrates[1] / abs(powerrates[1]));
	delay(min(short_delay_1, short_delay_2));
	if(short_delay_1 < short_delay_2) {
		action_moteur_1(0);
	} else {
		action_moteur_2(0);
	}
	delay(abs(short_delay_1 - short_delay_2));
	if(short_delay_1 < short_delay_2) {
		action_moteur_2(0);
	} else {
		action_moteur_1(0);
	}
}