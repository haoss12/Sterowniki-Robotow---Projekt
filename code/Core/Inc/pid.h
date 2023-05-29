/*
 * pid.h
 *
 *  Created on: 29 maj 2023
 *      Author: Hubert
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct PID {
	float Kp, Ki, Kd;

	float outLimMax, outLimMin;

	float intLimMax, intLimMin;

	float tau;

	float integrator;
	float prevErr;
	float differentiator;
	float prevMeasure;

	float out;

} PID;

//PID controller = {_KP, _KI, _KD};

//void initPID();

float updatePID(float setpoint, float measurement);


#endif /* INC_PID_H_ */
