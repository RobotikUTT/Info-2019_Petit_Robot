#include <Wire.h>
#include <VL53L0X.h>
#include <Ultrasonic.h>

#include <SimpleTimer.h>

#define DEBUG

typedef struct {
	float		rayon;
	int16_t	charge_utile;
	uint32_t	_millis;
	int16_t		cle;
} position_pol;

SimpleTimer timer;								 // Timer pour échantillonnage

VL53L0X laser;
Ultrasonic ultraleft(14,15);   // (Trig PIN,Echo PIN)
Ultrasonic ultraright(16,17);  // (Trig PIN,Echo PIN)
#define PIN_TIRETTE 12

volatile long tick_codeuse1 = 0;		 // Compteur de tick de la codeuse
volatile long tick_codeuse2 = 0;
volatile long tick_codeuse_av1 = 0;		 
volatile long tick_codeuse_av2 = 0;
volatile float position1 =0;
volatile float position2 =0;

int nb_pas = 1;
int inc_pas = 0;
float pf_roue1[] = {1000, -56.5*PI, 230-56.5*PI, -56.5*PI, 700, -56.5*PI, 230, -56.5*PI, 700};
float pf_roue2[] = {-1000, -56.5*PI, -230-56.5*PI, -56.5*PI, -700, -56.5*PI, 230, -56.5*PI, 700};
// float pf_roue1[] = {200.0, 100.0, 0, 120.0, 720.0,200.0, 400.0, -200.0,
//	-100.0, 600.0,200.0, 400.0, -200.0, -100.0, 600.0,200.0, 400.0, -200.0,
//	 -100.0, 600.0,200.0, 400.0, -200.0, -100.0, 600.0,200.0, 400.0, -200.0,
//		-100.0, 600.0,200.0, 400.0, -200.0, -100.0, 600.0,200.0, 400.0, -200.0,
//		 -100.0, 600.0,200.0, 400.0, -200.0, -100.0, 600.0,200.0, 400.0, -200.0,
//			-100.0, 600.0 };
// float pf_roue2[] = {-200.0, -100.0, 0, 120.0, -480.0, 200.0, -200.0, 200.0,
//	300.0, 600.0, 200.0, -200.0, 200.0, 300.0, 600.0, 200.0, -200.0, 200.0, 300.0,
//	 600.0, 200.0, -200.0, 200.0, 300.0, 600.0, 200.0, -200.0, 200.0, 300.0, 600.0,
//		200.0, -200.0, 200.0, 300.0, 600.0, 200.0, -200.0, 200.0, 300.0, 600.0, 200.0,
//		 -200.0, 200.0, 300.0, 600.0, 200.0, -200.0, 200.0, 300.0, 600.0 };

bool pas_suivant = true;
bool arret = false;

float T = 2000;
float T1 = 2000;

float T0 = 0;
float Tv1 = 0;
float Tv2 = 0;
float Ta1 = 0;
float Ta2 = 0;
float Tv = 0;
float Ta = 0;


float pf1 = 200;
float p01 = 0;
float pf2 = -200;
float p02 = 0;


float Vmax = 600;
float Amax = 300;

//tour entier 240 240

int vitMoteur1 = 0;		 
int vitMoteur2 = 0;	 // Commande du moteur

int inc_saisie_donnee=0;
float donnee1=0;
float donnee2=0;

#define periode_echantillonnage 10	// FrÃ©quence d'exÃ©cution de l'asservissement
#define rapport_reducteur 47.0					
#define tick_par_tour_codeuse 24.0
const float perimetre_roue = 61.0 * PI;
const float perimetre_robot = 226.0 * PI;
// const float coeff_

//definition des entrÃ©es

//sens moteur 1 et 2
#define M1IN1 8
#define M1IN2 9
#define M2IN1 10
#define M2IN2 11

// codeur
#define voieA1 2
#define voieB1 4
#define voieA2 3
#define voieB2 7

// Commande de vitesse moteurs

#define M1D2PWM 5
#define M2D2PWM 6


// init calculs asservissement PID
float erreur_precedente1 = 0; 
float erreur_precedente2 = 0;
float erreur1 = 0; 
float erreur2 = 0;

float consigne_moteur2 = 0;
float consigne_moteur1 = 0;

float somme_erreur1 = 0;
float somme_erreur2 = 0;
float delta_erreur1=0;
float delta_erreur2=0;

//Definition des constantes du correcteur PID
float kp1 = 38;					 // Coefficient proportionnel		choisis par tatonnement sur le moniteur. Ce sont les valeurs qui donnaient les meilleures performances
float ki1 = 0.02;						// Coefficient intÃ©grateur
float kd1 = 70.0;					// Coefficient dÃ©rivateur

float kp2 = 38;					 // Coefficient proportionnel		choisis par tatonnement sur le moniteur. Ce sont les valeurs qui donnaient les meilleures performances
float ki2 = .02;						// Coefficient intÃ©grateur
float kd2 = 70.0;					// Coefficient dÃ©rivateur

/* Routine d'initialisation */
void setup() {
	Serial.begin(115200);				 // Initialisation port COM
	pinMode(M1D2PWM, OUTPUT);
	pinMode(M2D2PWM, OUTPUT);
	pinMode(M1IN1, OUTPUT);
	pinMode(M1IN2, OUTPUT);
	pinMode(M2IN1, OUTPUT);
	pinMode(M2IN2, OUTPUT);

	analogWrite(M1D2PWM, 0);	// Initialisation sortie moteur Ã  0
	analogWrite(M2D2PWM, 0);

	// Sensors:
	pinMode(PIN_TIRETTE, INPUT_PULLUP);
	Wire.begin();

	laser.init();
	laser.setTimeout(100);
	laser.setMeasurementTimingBudget(20000);


	delay(300);								// Pause de 0,3 sec pour laisser le temps au moteur de s'arrÃ©ter si celui-ci est en marche

	attachInterrupt(digitalPinToInterrupt(2), compteur1, CHANGE);		// Interruption sur tick de la codeuse	(interruption 0 = pin2 arduino)
	attachInterrupt(digitalPinToInterrupt(3), compteur2, CHANGE); 
	timer.setInterval(periode_echantillonnage, asservissement);	// Interruption pour calcul du PID et asservissement; toutes les 10ms, on recommence la routine
	Serial.setTimeout(10);
	Tv1=abs(pf1-p01)*1000.0*30/(Vmax*16);
	Tv2=abs(pf2-p02)*1000.0*30/(Vmax*16);
	Ta1 = sqrt(abs(pf1-p01)*10/(Amax * 1.732))*1000.0;
	Ta2 = sqrt(abs(pf2-p02)*10/(Amax * 1.732))*1000.0;
	T0=(float)millis();
	Tv = max(Tv1,Tv2);
	Ta = max(Ta1,Ta2);
	T = max(Tv,Ta);

	while(!digitalRead(PIN_TIRETTE)) {
		delay(10);
	}

}

byte cpt = 0;
/* Fonction principale */
void loop() {
	if(sensor_loop()) {
		Serial.println("ROBOT STOPPED");
		tourne_moteur1(0);
		tourne_moteur2(0);
		while(true) {
			delay(100);
		}
	}
	timer.run();	//on fait tourner l'horloge
	if (Serial.available()) {
		if (inc_saisie_donnee==0) {
			donnee1 = Serial.parseFloat();
			inc_saisie_donnee=inc_saisie_donnee+1;
		}
		else {
			donnee2 = Serial.parseFloat();
			inc_saisie_donnee=0;
			p01=consigne_moteur1;
			p02=consigne_moteur2;
			pf1=donnee1;
			pf2=donnee2;
			Tv1=abs(pf1-p01)*1000.0*30/(Vmax*16);
			Tv2=abs(pf2-p02)*1000.0*30/(Vmax*16);
			Ta1 = sqrt(abs(pf1-p01)*10/(Amax * 1.732))*1000.0;
			Ta2 = sqrt(abs(pf2-p02)*10/(Amax * 1.732))*1000.0;
			T0=(float)millis();
			Tv = max(Tv1,Tv2);
			Ta = max(Ta1,Ta2);
			T = max(Tv,Ta);
		}
		Serial.parseFloat();
	}
	if (pas_suivant)
	{
		pas_suivant = false;
		p01=consigne_moteur1;
		p02=consigne_moteur2;
		pf1=pf_roue1[inc_pas];
		pf2=pf_roue2[inc_pas];
		Tv1=abs(pf1-p01)*1000.0*30/(Vmax*16);
		Tv2=abs(pf2-p02)*1000.0*30/(Vmax*16);
		Ta1 = sqrt(abs(pf1-p01)*10/(Amax * 1.732))*1000.0;
		Ta2 = sqrt(abs(pf2-p02)*10/(Amax * 1.732))*1000.0;
		T0=(float)millis();
		Tv = max(Tv1,Tv2);
		Ta = max(Ta1,Ta2);
		T = max(Tv,Ta);
		inc_pas++;
		if (inc_pas>=nb_pas)
		{
			if (false) {
				cpt++;
				inc_pas=0;
			}
			else
				arret=true;
		}

	}
}

bool sensor_loop() {
	static byte cpt_low_dist = 0;
	float U_L = ultraleft.Ranging(CM);
	float U_R = ultraright.Ranging(CM);
	float L = laser.readRangeSingleMillimeters();

	if(L < 150.0 || U_L < 15.0 || U_R < 15.0) {
		cpt_low_dist++;
		if(cpt_low_dist > 5) {
			return true;
		}
	} else {
		cpt_low_dist = 0;
	}
	return false;
}


/* Interruption sur tick de la codeuse */
void compteur1() {
	if (digitalRead(voieA1)==digitalRead(voieB1))
		tick_codeuse1--;
	else
		tick_codeuse1++;
	// On incrÃ©mente le nombre de tick de la codeuse. 
}

void compteur2() {
	if (digitalRead(voieA2)==digitalRead(voieB2))
		tick_codeuse2--;
	else
		tick_codeuse2++;
}


/* Interruption pour calcul du P */
void asservissement() {
	static unsigned long last_print;
	/*
	Serial.print(tick_codeuse1);
	Serial.print(" ");
	Serial.print((float)tick_codeuse1/((float)(rapport_reducteur*tick_par_tour_codeuse)));
	Serial.print(" ");
	Serial.print(tick_codeuse2);
	Serial.print(" ");
	Serial.println((float)tick_codeuse2/((float)(rapport_reducteur*tick_par_tour_codeuse)));
	*/
	position1=(float)perimetre_roue*(float)tick_codeuse1/((float)(rapport_reducteur*tick_par_tour_codeuse));
	position2=(float)perimetre_roue*(float)tick_codeuse2/((float)(rapport_reducteur*tick_par_tour_codeuse));

	float elapsed_time = (float)millis() - T0;
	if (elapsed_time < T) {
		float polynome = (10*pow(elapsed_time/T,3) - 15*pow(elapsed_time/T,4) + 6*pow(elapsed_time/T,5));
		consigne_moteur1 = p01 + (pf1-p01) * polynome;	
		consigne_moteur2 = p02 + (pf2-p02) * polynome;
	}
	else{
		consigne_moteur1 = pf1;
		consigne_moteur2 = pf2; 
		if (!arret)
			pas_suivant=true;
	}


	erreur1 = consigne_moteur1 - position1; // pour le proportionnel
	erreur2 = consigne_moteur2 - position2;


	somme_erreur1 += erreur1; // pour l'intÃ©grateur
	somme_erreur2 += erreur2;
	if (somme_erreur1 > 50){
			somme_erreur1 = 50;}
	else if (somme_erreur1 < -50){
			somme_erreur1 = -50;}
	if (somme_erreur2 > 50){
			somme_erreur2 = 50;}
	else if (somme_erreur2 < -50){
			somme_erreur2 = -50;}
	
	delta_erreur1 = erreur1 - erreur_precedente1;	// pour le dÃ©rivateur
	delta_erreur2 = erreur2 - erreur_precedente2;
	
	erreur_precedente1 = erreur1;
	erreur_precedente2 = erreur2;


	// P : calcul de la commande
	vitMoteur1 = kp1*erreur1 + ki1*somme_erreur1 + kd1*delta_erreur1;	//somme des trois erreurs
	vitMoteur2 = kp2*erreur2 + ki2*somme_erreur2 + kd2*delta_erreur2;
	
	// Normalisation et contrÃ´le du moteur
	if (vitMoteur1 > 255) {
		vitMoteur1 = 255;	
	}
	else if (vitMoteur1 < -255) {
		vitMoteur1 = -255;	
	}
	if (vitMoteur2 > 255) {
		vitMoteur2 = 255;	
	}
	else if (vitMoteur2 < -255) {
		vitMoteur2 = -255;
	}
	tourne_moteur1(vitMoteur1);
	tourne_moteur2(vitMoteur2);

	
	// DEBUG
	#ifdef DEBUG
	if(millis() - last_print > 200) {
		last_print = millis();

		//	Serial.print(" ");//
		//	Serial.print(p01,2);
		//	Serial.print(" ");//
		//	Serial.print(pf1,2);
		//	Serial.print(" ");//
		//	Serial.print(p02,2);
		//	Serial.print(" ");//
		//	Serial.print(pf2,2);
		//	Serial.print(" ");//
		//	Serial.print(T0,2);
		//	Serial.print(" ");//
		//	Serial.print(millis());
		//	Serial.print("		");//
		
		Serial.print((float)consigne_moteur1,2);
		Serial.print(" ");//
		Serial.print((float)position1,2);
		//Serial.print(" ");
		//Serial.print((float)tick_codeuse2/((float)(rapport_reducteur*tick_par_tour_codeuse)));
		//Serial.print(" ");
		//Serial.print((float)vitMoteur1,2);
		Serial.print(" ");//
		Serial.print((float)consigne_moteur2,2);
		Serial.print(" ");//
		Serial.println((float)position2,2);
		//Serial.print(" ");
		//Serial.print((float)tick_codeuse2/((float)(rapport_reducteur*tick_par_tour_codeuse)));
		//Serial.print(" ");
		//Serial.println((float)vitMoteur2,2);
	}
	#endif
	
}

void tourne_moteur1(float powerrate) {
	if (powerrate>0) {
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

void tourne_moteur2(float powerrate) {
	if (powerrate>0) {
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
