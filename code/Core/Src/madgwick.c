/*
 * madgwick.c
 *
 *  Created on: 14 maj 2023
 *      Author: Hubert
 */

#include "madgwick.h"

void updateQuat(float *gyro, float *acc, float *mag, float *estQuat) {

	float norm;	// variable to normalize vectors
	float QuatDotOmega[4];	// quaternion rate from gyroscope elements (angular - omega)
	float f[6];	// objective function
	float 	J11_24, J12_23, J13_22, J14_21, J32, J33, J41, J42,
			J43, J44, J51, J52, J53, J54, J61, J62, J63, J64;	// jacobian elements, some are the same but with opposite sign, so to save memory it get squashed
	float QuatHatDot[4];	// estimated direction of gyroscope error
	float w_err[3];	// estimated angular direction of gyroscope error
	float h[3];	// flux in the earth frame

	float twob_x = 2.0f * b_x;
	float twob_z = 2.0f * b_z;
	float halfEstQuat[4];
	float twoEstQuat[4];
	float twob_xEstQuat[4];
	float twob_zEstQuat[4];
	float twomag[4];


	for (uint8_t i = 0; i < 4; ++i) { //i guess that compiler is going to unroll this loop
		halfEstQuat[i] = 0.5f * estQuat[i];
		twoEstQuat[i] = 2.0f * estQuat[i];
		twob_xEstQuat[i] = twob_x * estQuat[i];
		twob_zEstQuat[i] = twob_z * estQuat[i];
		twomag[i] = 2.0f * mag[i];
	}

	// accelerometer and magnetometer normalisation

	norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
	acc[0] /= norm;
	acc[1] /= norm;
	acc[2] /= norm;

	norm = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	mag[0] /= norm;
	mag[1] /= norm;
	mag[2] /= norm;

	// calculation of objective function
	f[0] = twoEstQuat[1] * estQuat[3] - twoEstQuat[0] * estQuat[2] - acc[0];
	f[1] = twoEstQuat[0] * estQuat[1] - twoEstQuat[2] * estQuat[3] - acc[1];
	f[2] = 1.0f - twoEstQuat[1] * estQuat[1] - twoEstQuat[2] * estQuat[2] - acc[2];
	f[3] = twob_x * (0.5f - estQuat[2] * estQuat[2] - estQuat[3] * estQuat[3]) + twob_z * (estQuat[1] * estQuat[3] - estQuat[0] * estQuat[2]) - mag[0];
	f[4] = twob_x * (estQuat[1] * estQuat[2] - estQuat[0] * estQuat3) + twob_z * (estQuat[0] * estQuat[1] + estQuat[2] * estQuat[3]) - mag[1];
	f[5] = twob_x * (estQuat[1] * estQuat[3] + estQuat[0] * estQuat[2]) + twob_z * (0.5f - estQuat[1] * estQuat[1] - estQuat[2] - estQuat[2]) - mag[2];


}
