#include <Servo.h>

#define SERVO_RIGHT 13
#define SERVO_LEFT 12

Servo servo_right;
Servo servo_left;


void deploy_right_arm(bool state) {
	// servo_right.write(state ? 150 : 60);

	servo_right.write(state ? 130 : 50);
}

void deploy_left_arm(bool state) {
	servo_left.write(state ? 78 : 150);
}

void servo_init() {
	servo_right.attach(SERVO_RIGHT);
	servo_left.attach(SERVO_LEFT);
	deploy_right_arm(false);
	deploy_left_arm(false);
}
