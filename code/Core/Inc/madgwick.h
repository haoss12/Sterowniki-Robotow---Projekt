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
#define delta 0.05f 							//sampling time
#define gyroError M_PI * (6.0f / 180.0f) 	//gyroscope error in rad/s (5 deg/s)
#define gyroDrift M_PI * (1.0f / 180.0f)	//gyroscope drift in rad/s^2 (0.2 deg/s^2)
#define beta sqrt(3.0f / 4.0f) * gyroError
#define zeta sqrt(3.0f / 4.0f) * gyroDrift

//TODO: Quaternion structure, also think about imu structure
//highpass filter maybe k
//float estQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float b_x, b_z; //reference direction of flux in earth frame
float gyroBias[3];

typedef struct RPY {
	float roll;
	float pitch;
	float yaw;
} RPY;

typedef struct Quaternion {
	float w;
	float x, y, z;
} Quaternion;

// one tick of imu fusion filter
void updateQuat(float *gyro, float *acc, float *mag, float *estQuat);

// recalculate quaternion to euler angles
void quat2rpy(float *Q, RPY *E);

#endif /* INC_MADGWICK_H_ */
