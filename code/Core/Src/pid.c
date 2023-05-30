/*
 * pid.c
 *
 *  Created on: 29 maj 2023
 *      Author: Hubert
 */

#include "pid.h"
#include "madgwick.h"

void updatePID(float setpoint, float measurement) {

	float err = measurement - setpoint;

	controller.integral += err * delta;

	controller.derivative = err - controller.prevErr;

	controller.out = (controller.Kp * err) + (controller.Ki * controller.integral) + (controller.Kd * controller.derivative);

	if (controller.out > OUT_LIM_MAX)
		controller.out = OUT_LIM_MAX;
	else if (controller.out < OUT_LIM_MIN)
		controller.out = OUT_LIM_MIN;

	controller.prevErr = err;

}
