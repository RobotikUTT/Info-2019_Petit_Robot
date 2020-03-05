#ifndef _SENSORS
#define _SENSORS

void change_sensor_range(float distance);
void sensor_init();
bool sensor_loop();
float getDistanceLaser();
void laser_correction_distance(float distance);

#endif
