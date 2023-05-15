/*
 * madgwick.h
 *
 *  Created on: 14 maj 2023
 *      Author: Hubert
 */

#ifndef INC_MADGWICK_H_
#define INC_MADGWICK_H_

#include <math.h>

//defined constants
#define delta 0.1f 							//sampling time
#define gyroError M_PI * (5.0f * 180.0f) 	//gyroscope error in rad/s (5 deg/s)
#define gyroDrift M_PI * (0.2f * 180.0f)	//gyroscope drift in rad/s^2 (0.2 deg/s^2)
#define beta sqrt(3.0f / 4.0f) * gyroError
#define zeta sqrt(3.0f / 4.0f) * gyroDrift


//float estQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float b_x = 1.0f, b_z = 0.0f; //reference direction of flux in earth frame
float gyroBias[3] = {0.0f, 0.0f, 0.0f};

void updateQuat(float *gyro, float *acc, float *mag, float *estQuat);


#endif /* INC_MADGWICK_H_ */
