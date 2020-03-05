#include "sensors.h"
#include "asservissement.h"
#include "timecheck.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <Ultrasonic.h>

#define U_LEFT_TRIG 14
#define U_LEFT_ECHO 15
#define U_RIGHT_TRIG 16
#define U_RIGHT_ECHO 17

#define LASER_OFFSET 0

VL53L0X laser;
Ultrasonic ultra_right(U_LEFT_TRIG, U_LEFT_ECHO);   // (Trig PIN,Echo PIN)
Ultrasonic ultra_left(U_RIGHT_TRIG, U_RIGHT_ECHO);  // (Trig PIN,Echo PIN)

uint8_t next_sensor = 0;
int8_t priority_flag = -1;
uint8_t sensors_values[] = {0, 0};
bool sensors_status[] = {false, false};

float distance_detection = 30.0;

void change_sensor_range(float distance) {
	distance_detection = distance;
}

void sensor_init() {
	Wire.begin();

	laser.init();
	laser.setTimeout(100);
	laser.setMeasurementTimingBudget(20000);
}

// bool sensor_loop(bool previous_state) {
// 	static byte cpt_low_dist = 0;
// 	float U_R = ultra_right.Ranging(CM);
// 	float U_L = ultra_left.Ranging(CM);
// 	float L = laser.readRangeSingleMillimeters();

// 	if(L < 200.0 || U_R < 20.0 || U_L < 20.0) {
		
// 		if(cpt_low_dist < 2) {
// 			cpt_low_dist++;
// 		} else {
// 			return true;
// 		}
// 	} else {
		
// 		if(cpt_low_dist > 0) {
// 			cpt_low_dist--;
// 		} else {
// 			return false;
// 		}
// 	}
// 	return previous_state;
// }


bool sensor_loop() {
	int8_t sensor_number;

	if(priority_flag >= 0) {
		sensor_number = priority_flag;
	} else {
		next_sensor = (next_sensor + 1) % 2;
		sensor_number = next_sensor;
	}
	
	switch(sensor_number) {
		case 0:
			sensors_values[0] = ultra_left.Ranging(CM); //laser.readRangeSingleMillimeters() / 10;
			break;
		case 1:
			sensors_values[1] = ultra_right.Ranging(CM);
			break;
		// case 2:
		// 	sensors_values[2] = ultra_left.Ranging(CM);
		// 	break;
	}
	if(sensors_values[sensor_number] < distance_detection ) {
		if(!sensors_status[sensor_number]) {
			if(priority_flag == sensor_number) {
				sensors_status[sensor_number] = true;
				priority_flag = -1;
			} else {
				priority_flag = sensor_number;
			}
		}
	} else {
		if(sensors_status[sensor_number]) {
			if(priority_flag == sensor_number) {
				sensors_status[sensor_number] = false;
				priority_flag = -1;
			} else {
				priority_flag = sensor_number;
			}
		}
	}

	return (sensors_status[0] || sensors_status[1]); // || sensors_status[2]);
}

float getDistanceLaser() {
	return laser.readRangeSingleMillimeters();
}

// void laser_correction_distance(float distance) {
// 	float value;
// 	while(true) {
// 		value = 0;
// 		for(uint8_t i=0; i<5; i++) {
// 			value += laser.readRangeSingleMillimeters();
// 			delay(50);
// 		}
// 		value /= 5;
// 		value -= LASER_OFFSET;
// 		if(abs(value - distance) < 5) {
// 			break;
// 		}
// 		position_t p = {2, 0, value - distance, 0, 1};
// 		enqueuePosition(p);
// 		do {
// 			asserv_loop();
// 			time_loop();
// 		} while(!isMoveCompleted());
// 	}
	

// }
