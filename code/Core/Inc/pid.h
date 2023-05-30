/*
 * pid.h
 *
 *  Created on: 29 maj 2023
 *      Author: Hubert
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define OUT_LIM_MAX 500.0f
#define OUT_LIM_MIN -500.0f
#define INT_LIM_MAX 10.0f
#define INT_LIM_MIN 10.0f
#define _KP 6.0f
#define _KI 0.05f
#define _KD 0.5f

extern const float delta;

typedef struct PID {
	float Kp, Ki, Kd;

	float integral;
	float derivative;
	float prevErr;

	float out;

} PID;

extern PID controller;

// update PID - single step
void updatePID(float setpoint, float measurement);


#endif /* INC_PID_H_ */
