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
	float gyroErr[3];	// estimated angular direction of gyroscope error
	float h[3];	// flux in the earth frame

	float twob_x = 2.0f * b_x;
	float twob_z = 2.0f * b_z;
	float halfEstQuat[4];
	float twoEstQuat[4];
	float twob_xEstQuat[4];
	float twob_zEstQuat[4];
	float twomag[3];


	for (uint8_t i = 0; i < 4; ++i) { //i guess that compiler is going to unroll this loop
		halfEstQuat[i] = 0.5f * estQuat[i];
		twoEstQuat[i] = 2.0f * estQuat[i];
		twob_xEstQuat[i] = twob_x * estQuat[i];
		twob_zEstQuat[i] = twob_z * estQuat[i];
	}

	twomag[0] = 2.0f * mag[0];
	twomag[1] = 2.0f * mag[1];
	twomag[2] = 2.0f * mag[2];

	// accelerometer and magnetometer normalisation

	norm = 1.0f / sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
	acc[0] *= norm;
	acc[1] *= norm;
	acc[2] *= norm;

	norm = 1.0f / sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	mag[0] *= norm;
	mag[1] *= norm;
	mag[2] *= norm;

	// calculation of objective function
	f[0] = twoEstQuat[1] * estQuat[3] - twoEstQuat[0] * estQuat[2] - acc[0];
	f[1] = twoEstQuat[0] * estQuat[1] - twoEstQuat[2] * estQuat[3] - acc[1];
	f[2] = 1.0f - twoEstQuat[1] * estQuat[1] - twoEstQuat[2] * estQuat[2] - acc[2];
	f[3] = twob_x * (0.5f - estQuat[2] * estQuat[2] - estQuat[3] * estQuat[3]) + twob_z * (estQuat[1] * estQuat[3] - estQuat[0] * estQuat[2]) - mag[0];
	f[4] = twob_x * (estQuat[1] * estQuat[2] - estQuat[0] * estQuat[3]) + twob_z * (estQuat[0] * estQuat[1] + estQuat[2] * estQuat[3]) - mag[1];
	f[5] = twob_x * (estQuat[1] * estQuat[3] + estQuat[0] * estQuat[2]) + twob_z * (0.5f - estQuat[1] * estQuat[1] - estQuat[2] - estQuat[2]) - mag[2];

	J11_24 = twoEstQuat[2];
	J12_23 = twoEstQuat[3];
	J13_22 = twoEstQuat[0];
	J14_21 = twoEstQuat[1];
	J32 = 2.0f * twoEstQuat[1];
	J33 = 2.0f * twoEstQuat[2];
	J41 = twob_zEstQuat[2];
	J42 = twob_zEstQuat[3];
	J43 = 2.0f * twob_xEstQuat[2] + twob_xEstQuat[0];
	J44 = 2.0f * twob_xEstQuat[3] - twob_xEstQuat[1];
	J51 = twob_xEstQuat[3] - twob_xEstQuat[1];
	J52 = twob_xEstQuat[2] + twob_xEstQuat[0];
	J53 = twob_xEstQuat[1] + twob_xEstQuat[3];
	J54 = twob_xEstQuat[0] - twob_xEstQuat[2];
	J61 = twob_xEstQuat[2];
	J62 = twob_xEstQuat[3] - 2.0f * twob_xEstQuat[1];
	J63 = twob_xEstQuat[0] - 2.0f * twob_xEstQuat[2];
	J64 = twob_xEstQuat[1];

	// compute and normalise the gradient
	QuatHatDot[0] = J14_21 * f[1] - J11_24 * f[0] - J41 * f[3] - J51 * f[4] + J61 * f[5];
	QuatHatDot[1] = J12_23 * f[0] + J13_22 * f[1] - J32 * f[2] + J42 * f[3] + J52 * f[4] + J62 * f[5];
	QuatHatDot[2] = J12_23 * f[1] - J33 * f[2] - J13_22 * f[0] - J43 * f[3] + J53 * f[4] + J63 * f[5];
	QuatHatDot[3] = J14_21 * f[0] + J11_24 * f[1] - J44 * f[3] - J54 * f[4] + J64 * f[5];

	norm = 1.0f / sqrtf(QuatHatDot[0] * QuatHatDot[0] + QuatHatDot[1] * QuatHatDot[1] + QuatHatDot[2] * QuatHatDot[2] + QuatHatDot[3] * QuatHatDot[3]);
	QuatHatDot[0] *= norm;
	QuatHatDot[1] *= norm;
	QuatHatDot[2] *= norm;
	QuatHatDot[3] *= norm;

	gyroErr[0] = twoEstQuat[0] * QuatHatDot[1] - twoEstQuat[1] * QuatHatDot[0] - twoEstQuat[2] * QuatHatDot[3] + twoEstQuat[3] * QuatHatDot[2];
	gyroErr[1] = twoEstQuat[0] * QuatHatDot[2] + twoEstQuat[1] * QuatHatDot[3] - twoEstQuat[2] * QuatHatDot[0] - twoEstQuat[3] * QuatHatDot[1];
	gyroErr[2] = twoEstQuat[0] * QuatHatDot[3] - twoEstQuat[1] * QuatHatDot[2] + twoEstQuat[2] * QuatHatDot[1] - twoEstQuat[3] * QuatHatDot[0];

	gyroBias[0] = gyroErr[0] * delta * zeta;
	gyroBias[1] = gyroErr[1] * delta * zeta;
	gyroBias[2] = gyroErr[2] * delta * zeta;
	gyro[0] -= gyroBias[0];
	gyro[1] -= gyroBias[1];
	gyro[2] -= gyroBias[2];

	QuatDotOmega[0] = - halfEstQuat[1] * gyro[0] - halfEstQuat[2] * gyro[1] - halfEstQuat[3] * gyro[2];
	QuatDotOmega[1] =   halfEstQuat[0] * gyro[0] + halfEstQuat[2] * gyro[2] - halfEstQuat[3] * gyro[1];
	QuatDotOmega[2] =   halfEstQuat[0] * gyro[1] - halfEstQuat[1] * gyro[2] + halfEstQuat[3] * gyro[0];
	QuatDotOmega[3] =   halfEstQuat[1] * gyro[2] + halfEstQuat[2] * gyro[1] - halfEstQuat[2] * gyro[0];

	estQuat[0] += (QuatDotOmega[0] - (beta * QuatHatDot[0])) * delta;
	estQuat[1] += (QuatDotOmega[1] - (beta * QuatHatDot[1])) * delta;
	estQuat[2] += (QuatDotOmega[2] - (beta * QuatHatDot[2])) * delta;
	estQuat[3] += (QuatDotOmega[3] - (beta * QuatHatDot[3])) * delta;

	norm = 1.0f / sqrtf(estQuat[0] * estQuat[0] + estQuat[1] * estQuat[1] + estQuat[2] * estQuat[2] + estQuat[3] * estQuat[3]);
	estQuat[0] *= norm;
	estQuat[1] *= norm;
	estQuat[2] *= norm;
	estQuat[3] *= norm;

	h[0] = twomag[0] * (0.5f - estQuat[2] * estQuat[2] - estQuat[3] * estQuat[3]) + twomag[1] * (estQuat[1] * estQuat[2] - estQuat[0] * estQuat[3]) + twomag[2] * (estQuat[1] * estQuat[3] + estQuat[0] * estQuat[2]);
	h[1] = twomag[0] * (estQuat[1] * estQuat[2] + estQuat[0] * estQuat[3]) + twomag[1] * (0.5f - estQuat[1] * estQuat[1] - estQuat[3] * estQuat[3]) + twomag[2] * (estQuat[2] * estQuat[3] - estQuat[0] * estQuat[1]);
	h[2] = twomag[0] * (estQuat[1] * estQuat[3] - estQuat[0] * estQuat[2]) + twomag[1] * (estQuat[2] * estQuat[3] + estQuat[0] * estQuat[1]) + twomag[2] * (0.5f - estQuat[1] * estQuat[1] - estQuat[2] * estQuat[2]);

	b_x = sqrtf(h[0] * h[0] + h[1] * h[1]);
	b_z = h[2];
}

//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
void quat2rpy(float *Q, RPY *E) {

	E->roll = atan2f( (2.0f * (Q[0] * Q[1] + Q[2] * Q[3])), (1.0f - 2.0f * (Q[1] * Q[1] + Q[2] * Q[2])) );
	E->pitch = 2.0f * atan2f( sqrtf(1.0f + 2.0f * (Q[0] * Q[2] - Q[1] * Q[3])), sqrtf(1.0f - 2.0f * (Q[0] * Q[2] - Q[1] * Q[3])) ) - M_PI/2;
	E->yaw = atan2f( (2.0f * (Q[0] * Q[3] + Q[1] * Q[2])), (1.0f - 2.0f * (Q[2] * Q[2] + Q[3] * Q[3])) );

}
