#include "asservissement.h"
#include "moteurs.h"
#include "sensors.h"
#include <Arduino.h>
#include <SimpleFIFO.h>
// #include <SimpleTimer.h>

// #define DEBUG	// Uncomment to enable Serial debuging
// #define DEBUG_TRACE

#define periode_echantillonnage 10	// Fréquence d'exécution de l'asservissement en ms
#define rapport_reducteur 47.0					
#define tick_par_tour_codeuse 24.0
#define diametre_roue 61.0
#define entraxe_roues 226.0

// codeur
#define voieA1 2
#define voieB1 4
#define voieA2 3
#define voieB2 7

const float perimetre_roue = diametre_roue * PI;
const float perimetre_robot = entraxe_roues * PI;

float degToDistance(float angle) {
	return (angle * perimetre_robot / 360.0);
}

float calculPosition(long tick_codeuse) {
	return perimetre_roue * (float)tick_codeuse / (float)(rapport_reducteur * tick_par_tour_codeuse);
}

SimpleFIFO<position_t, 25> next_positions; //store 25 pos
// SimpleTimer timer;					// Timer pour échantillonnage

volatile long tick_codeuse_1 = 0;	// Compteur de tick de la codeuse
volatile long tick_codeuse_2 = 0;

bool move_completed = true;
uint8_t move_type = 1; // 0 = rotative move; 1 = straight move

float position_1 = 0;
float position_2 = 0;

float T, T0, Tv1, Tv2, Ta1, Ta2, Tv, Ta, pf1, p01, pf2, p02;

float Vmax = 600;
float Amax = 300;

bool loop_fired;


// init calculs asservissement PID
float
erreur_precedente_1,
erreur_precedente_2,
somme_erreur_1,
somme_erreur_2;

//Definition des constantes du correcteur PID
#define default_kp 40
#define default_ki 0
#define default_kd 0
float
kp1 = default_kp,	// Coefficient proportionnel choisis par tatonnement sur le moniteur. Ce sont les valeurs qui donnaient les meilleures performances
ki1 = default_ki,	// Coefficient intégrateur
kd1 = default_kd,	// Coefficient dérivateur

kp2 = default_kp,	// Coefficient proportionnel choisis par tatonnement sur le moniteur. Ce sont les valeurs qui donnaient les meilleures performances
ki2 = default_ki,	// Coefficient intégrateur
kd2 = default_kd;	// Coefficient dérivateur

float max_somme = 2000;


/* Interruption sur tick de la codeuse */
void ISR_encoder_1() {
	if (digitalRead(voieA1)==digitalRead(voieB1))
		tick_codeuse_1--;
	else
		tick_codeuse_1++;
	// On incrèmente le nombre de tick de la codeuse. 
}

void ISR_encoder_2() {
	if (digitalRead(voieA2)==digitalRead(voieB2))
		tick_codeuse_2--;
	else
		tick_codeuse_2++;
}

void asserv_init() {
	#ifdef DEBUG_TRACE
	Serial.begin(115200);
	#endif
	#ifdef DEBUG
	Serial.println(F("Init asserv ..."));
	#endif
	attachInterrupt(digitalPinToInterrupt(2), ISR_encoder_1, CHANGE);	// Interruption sur tick de la codeuse	(interruption 0 = pin2 arduino)
	attachInterrupt(digitalPinToInterrupt(3), ISR_encoder_2, CHANGE); 

	// timer.setInterval(periode_echantillonnage, asservissement);	// Interruption pour calcul du PID et asservissement; toutes les 10ms, on recommence la routine

}

bool isSequenceCompleted() {
	return (! next_positions.count() && move_completed);
}

uint8_t cpt_tmp = 0;
uint8_t nbMoveCompleted() {
	return cpt_tmp;
}

bool asserv_loop() {
	loop_fired = false;
	// timer.run();	//on fait tourner l'horloge
	static unsigned long last_fired;
	if(millis() - last_fired >= 10) {
		last_fired = millis();
		asservissement();
		loop_fired = true;
	}

	if(move_completed) {
		#ifdef DEBUG
		Serial.println(F("move_completed"));
		#endif
		if(next_positions.count()) {
			#ifdef DEBUG
			Serial.print(F("Count OK: "));
			Serial.print(move_type ? "Angle :" : "Radius: ");
			#endif
			move_completed = false;
			move_type = !move_type;

			if(!move_type) {
				position_t pos = next_positions.peek();
				if(pos.sensor_range > 0) {
					change_sensor_range(pos.sensor_range);
				}
				// kp1 = kp2 = default_kp + 2.6 * pos.load;
				// ki1 = ki2 = default_ki + pos.load;
				// kd1 = kd2 = default_kd + 2.6 * pos.load;
				#ifdef DEBUG
				Serial.println(pos.angle);
				#endif
				setNextStepRelative(degToDistance(pos.angle), degToDistance(pos.angle));
			} else {
				position_t pos = next_positions.dequeue();
				#ifdef DEBUG
				Serial.println(pos.radius);
				#endif
				setNextStepRelative(pos.radius, -pos.radius);
			}
		}
	}

	return loop_fired;
}

void enqueuePosition(position_t pos) {
	next_positions.enqueue(pos);
}

void setNextStepRelative(float relative_position_1, float relative_position_2) {
	setNextStepAbsolute(position_1 + relative_position_1, position_2 + relative_position_2);
}

void setNextStepAbsolute(float absolute_position_1, float absolute_position_2) {

	somme_erreur_1 = 0;
	somme_erreur_2 = 0;
	erreur_precedente_1 = 0;
	erreur_precedente_2 = 0;

	p01 = calculPosition(tick_codeuse_1);
	p02 = calculPosition(tick_codeuse_2);
	pf1 = absolute_position_1;
	pf2 = absolute_position_2;

	Tv1 = abs(pf1-p01) * 1000.0 * 30 / (Vmax * 16);
	Tv2 = abs(pf2-p02) * 1000.0 * 30 / (Vmax * 16);
	Ta1 = sqrt(abs(pf1-p01) * 10 / (Amax * 1.732)) * 1000.0;
	Ta2 = sqrt(abs(pf2-p02) * 10 / (Amax * 1.732)) * 1000.0;
	Tv = max(Tv1,Tv2);
	Ta = max(Ta1,Ta2);
	T = max(Tv,Ta) * 1.3;

	T0 = (float) millis();
}

void restart_after_stop() {
	setNextStepAbsolute(pf1, pf2);
}

void asservissement() {
	position_1 = calculPosition(tick_codeuse_1);
	position_2 = calculPosition(tick_codeuse_2);

	float consigne_moteur_1, consigne_moteur_2;

	float elapsed_time = (float)millis() - T0;
	if (elapsed_time < T) {
		float polynome = (10*pow(elapsed_time/T,3) - 15*pow(elapsed_time/T,4) + 6*pow(elapsed_time/T,5));
		consigne_moteur_1 = p01 + (pf1-p01) * polynome;	
		consigne_moteur_2 = p02 + (pf2-p02) * polynome;
	}
	else {
		consigne_moteur_1 = pf1;
		consigne_moteur_2 = pf2; 
		if(abs(position_1 - pf1) < 5 && abs(position_2 - pf2) < 5 ) {
			move_completed = true;
			cpt_tmp++;
		}
	}

	float erreur_1 = consigne_moteur_1 - position_1; // pour le proportionnel
	float erreur_2 = consigne_moteur_2 - position_2;


	somme_erreur_1 += erreur_1; // pour l'intégrateur
	somme_erreur_2 += erreur_2;
	// if (somme_erreur_1 > max_somme) {
	// 		somme_erreur_1 = max_somme;}
	// else if (somme_erreur_1 < -max_somme) {
	// 		somme_erreur_1 = -max_somme;}
	// if (somme_erreur_2 > max_somme) {
	// 		somme_erreur_2 = max_somme;}
	// else if (somme_erreur_2 < -max_somme) {
	// 		somme_erreur_2 = -max_somme;}
	
	float delta_erreur_1 = erreur_1 - erreur_precedente_1;	// pour le dérivateur
	float delta_erreur_2 = erreur_2 - erreur_precedente_2;
	
	erreur_precedente_1 = erreur_1;
	erreur_precedente_2 = erreur_2;


	// P : calcul de la commande
	int vitMoteur_1 = kp1*erreur_1 + ki1*somme_erreur_1 + kd1*delta_erreur_1;	//somme des trois erreurs
	int vitMoteur_2 = kp2*erreur_2 + ki2*somme_erreur_2 + kd2*delta_erreur_2;
	
	// Normalisation et contrÃ´le du moteur
	if (vitMoteur_1 > 255) {
		vitMoteur_1 = 255;	
	}
	else if (vitMoteur_1 < -255) {
		vitMoteur_1 = -255;	
	}
	if (vitMoteur_2 > 255) {
		vitMoteur_2 = 255;	
	}
	else if (vitMoteur_2 < -255) {
		vitMoteur_2 = -255;
	}

	action_moteur_1(vitMoteur_1);
	action_moteur_2(vitMoteur_2);

	#ifdef DEBUG_TRACE
	static unsigned long last_loop;
	Serial.print(vitMoteur_1);
	Serial.write('\t');
	Serial.print(vitMoteur_2);
	Serial.write('\t');
	Serial.print(erreur_1);
	Serial.write('\t');
	Serial.print(erreur_2);
	Serial.write('\t');
	Serial.print(somme_erreur_1);
	Serial.write('\t');
	Serial.print(somme_erreur_2);
	Serial.write('\t');
	// Serial.print(delta_erreur_1);
	// Serial.write('\t');
	// Serial.print(delta_erreur_2);
	// Serial.write('\t');
	Serial.println(min((millis() - last_loop) * 4, 200));
	last_loop = millis();
	#endif

	#ifdef DEBUG
	static unsigned long last_print;
	if(millis() - last_print > 200) {
		last_print = millis();
		// Serial.print((float)consigne_moteur_1,2);
		// Serial.print(" ");//
		// Serial.print((float)position_1,2);
		// Serial.print(" ");//
		// Serial.print((float)consigne_moteur_2,2);
		// Serial.print(" ");//
		// Serial.println((float)position_2,2);

		// Serial.print((float)vitMoteur_1,2);
		// Serial.print(" ");//
		// Serial.println((float)vitMoteur_2,2);

		Serial.print((float)consigne_moteur_1,2);
		Serial.print(" ");//
		Serial.print((float)position_1,2);
		Serial.print(" ");
		Serial.print((float)tick_codeuse_2/((float)(rapport_reducteur*tick_par_tour_codeuse)));
		Serial.print(" ");
		Serial.print((float)vitMoteur_1,2);
		Serial.print(" ");//
		Serial.print((float)consigne_moteur_2,2);
		Serial.print(" ");//
		Serial.println((float)position_2,2);
		Serial.print(" ");
		Serial.print((float)tick_codeuse_2/((float)(rapport_reducteur*tick_par_tour_codeuse)));
		Serial.print(" ");
		Serial.println((float)vitMoteur_2,2);

	}
	#endif
}