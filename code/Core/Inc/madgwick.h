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
#define gyroError 14.19957562		// gyroscope error in rad/s
#define gyroDrift 0.09804329484f	//gyroscope drift in rad/s^2
#define beta sqrt(3.0f / 4.0f) * gyroError
#define zeta sqrt(3.0f / 4.0f) * gyroDrift

extern const float delta;

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
