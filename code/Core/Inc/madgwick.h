/*
 * madgwick.h
 *
 *  Created on: 14 maj 2023
 *      Author: Hubert
 */

#ifndef INC_MADGWICK_H_
#define INC_MADGWICK_H_

#include <math.h>
#include <inttypes.h>

//defined constants
#define delta 0.01f 							//sampling time
#define gyroError 15 //10.19957562	//M_PI * (6.0f / 180.0f) 	//gyroscope error in rad/s (5 deg/s)
#define gyroDrift 0.09804329484f	//M_PI * (1.0f / 180.0f)	//gyroscope drift in rad/s^2 (0.2 deg/s^2) 0,09804329484
#define beta sqrt(3.0f / 4.0f) * gyroError
#define zeta sqrt(3.0f / 4.0f) * gyroDrift

//highpass filter maybe k

typedef struct RPY {
	float roll;
	float pitch;
	float yaw;
} RPY;

typedef struct Quaternion {
	float w;
	float x, y, z;
} Quaternion;

extern Quaternion q;

// one tick of imu fusion filter
void updateQuat(float *gyro, float *acc, float *mag);

// recalculate quaternion to euler angles
void quat2rpy(Quaternion *Q, RPY *E);

#endif /* INC_MADGWICK_H_ */
