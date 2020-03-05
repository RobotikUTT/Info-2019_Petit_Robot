#ifndef _ASSERVISSEMENT
#define _ASSERVISSEMENT

#include <inttypes.h>

typedef struct {
	// bool 	isAbsolute;
	uint8_t	color;	// 0 = purple, 1 = yellow, 2 = both
	float	angle;	// degree
	float	radius;	// millimeter
	float	sensor_range; // sensor distance detection
	float	load;
} position_t;


void ISR_encoder_1();
void ISR_encoder_2();
void asserv_init();
bool isSequenceCompleted();
uint8_t nbMoveCompleted();
bool asserv_loop();
void enqueuePosition(position_t pos);
void setNextStepRelative(float relative_position_1, float relative_position_2);
void setNextStepAbsolute(float absolute_position_1, float absolute_position_2);
void restart_after_stop();
void asservissement();

#endif
