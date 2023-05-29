/*
 * madgwick.c
 *
 *  Created on: 14 maj 2023
 *      Author: Hubert
 */

#include "madgwick.h"

static float invSqrt(float x) // Quake :)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void updateQuat(float *gyro, float *acc, float *mag /*, float *estQuat*/) {

    float recipNorm;
    float s0, s1, s2, s3;
    Quaternion qDot;
    float hx, hy;
    float _2qwmx, _2qwmy, _2qwmz, _2qxmx, _2bx, _2bz, _4bx, _4bz, _2qw, _2qx, _2qy, _2qz, _2qwqy, _2qyqz, qwqw, qwqx, qwqy, qwqz, qxqx, qxqy, qxqz, qyqy, qyqz, qzqz;



    // Rate of change of quaternion from gyroscope
    qDot.w = 0.5f * (-q.x * gyro[0] - q.y * gyro[1] - q.z * gyro[2]);
    qDot.x = 0.5f * (q.w * gyro[0] + q.y * gyro[2] - q.z * gyro[1]);
    qDot.y = 0.5f * (q.w * gyro[1] - q.x * gyro[2] + q.z * gyro[0]);
    qDot.z = 0.5f * (q.w * gyro[2] + q.x * gyro[1] - q.y * gyro[0]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((acc[0] == 0.0f) && (acc[1] == 0.0f) && (acc[2] == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        acc[0] *= recipNorm;
        acc[1] *= recipNorm;
        acc[2] *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
        mag[0] *= recipNorm;
        mag[1] *= recipNorm;
        mag[2] *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2qwmx = 2.0f * q.w * mag[0];
        _2qwmy = 2.0f * q.w * mag[1];
        _2qwmz = 2.0f * q.w * mag[2];
        _2qxmx = 2.0f * q.x * mag[0];
        _2qw = 2.0f * q.w;
        _2qx = 2.0f * q.x;
        _2qy = 2.0f * q.y;
        _2qz = 2.0f * q.z;
        _2qwqy = 2.0f * q.w * q.y;
        _2qyqz = 2.0f * q.y * q.z;
        qwqw = q.w * q.w;
        qwqx = q.w * q.x;
        qwqy = q.w * q.y;
        qwqz = q.w * q.z;
        qxqx = q.x * q.x;
        qxqy = q.x * q.y;
        qxqz = q.x * q.z;
        qyqy = q.y * q.y;
        qyqz = q.y * q.z;
        qzqz = q.z * q.z;

        // Reference direction of Earth's magnetic field
        hx = mag[0] * qwqw - _2qwmy * q.z + _2qwmz * q.y + mag[0] * qxqx + _2qx * mag[1] * q.y + _2qx * mag[2] * q.z - mag[0] * qyqy - mag[0] * qzqz;
        hy = _2qwmx * q.z + mag[1] * qwqw - _2qwmz * q.x + _2qxmx * q.y - mag[1] * qxqx + mag[1] * qyqy + _2qy * mag[2] * q.z - mag[1] * qzqz;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2qwmx * q.y + _2qwmy * q.x + mag[2] * qwqw + _2qxmx * q.z - mag[2] * qxqx + _2qy * mag[1] * q.z - mag[2] * qyqy + mag[2] * qzqz;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2qy * (2.0f * qxqz - _2qwqy - acc[0]) + _2qx * (2.0f * qwqx + _2qyqz - acc[1]) - _2bz * q.y * (_2bx * (0.5f - qyqy - qzqz) + _2bz * (qxqz - qwqy) - mag[0]) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (qxqy - qwqz) + _2bz * (qwqx + qyqz) - mag[1]) + _2bx * q.y * (_2bx * (qwqy + qxqz) + _2bz * (0.5f - qxqx - qyqy) - mag[2]);
        s1 = _2qz * (2.0f * qxqz - _2qwqy - acc[0]) + _2qw * (2.0f * qwqx + _2qyqz - acc[1]) - 4.0f * q.x * (1 - 2.0f * qxqx - 2.0f * qyqy - acc[2]) + _2bz * q.z * (_2bx * (0.5f - qyqy - qzqz) + _2bz * (qxqz - qwqy) - mag[0]) + (_2bx * q.y + _2bz * q.w) * (_2bx * (qxqy - qwqz) + _2bz * (qwqx + qyqz) - mag[1]) + (_2bx * q.z - _4bz * q.x) * (_2bx * (qwqy + qxqz) + _2bz * (0.5f - qxqx - qyqy) - mag[2]);
        s2 = -_2qw * (2.0f * qxqz - _2qwqy - acc[0]) + _2qz * (2.0f * qwqx + _2qyqz - acc[1]) - 4.0f * q.y * (1 - 2.0f * qxqx - 2.0f * qyqy - acc[2]) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - qyqy - qzqz) + _2bz * (qxqz - qwqy) - mag[0]) + (_2bx * q.x + _2bz * q.z) * (_2bx * (qxqy - qwqz) + _2bz * (qwqx + qyqz) - mag[1]) + (_2bx * q.w - _4bz * q.y) * (_2bx * (qwqy + qxqz) + _2bz * (0.5f - qxqx - qyqy) - mag[2]);
        s3 = _2qx * (2.0f * qxqz - _2qwqy - acc[0]) + _2qy * (2.0f * qwqx + _2qyqz - acc[1]) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - qyqy - qzqz) + _2bz * (qxqz - qwqy) - mag[0]) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (qxqy - qwqz) + _2bz * (qwqx + qyqz) - mag[1]) + _2bx * q.x * (_2bx * (qwqy + qxqz) + _2bz * (0.5f - qxqx - qyqy) - mag[2]);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot.w -= beta * s0;
        qDot.x -= beta * s1;
        qDot.y -= beta * s2;
        qDot.z -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot.w * delta;
    q.x += qDot.x * delta;
    q.y += qDot.y * delta;
    q.z += qDot.z * delta;

    // Normalise quaternion
    recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

//    estQuat[0] = q.w;
//    estQuat[1] = q.x;
//    estQuat[2] = q.y;
//    estQuat[3] = q.z;


}

//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
void quat2rpy(Quaternion *Q, RPY *E) {

	E->roll = atan2f( (2.0f * (Q->w * Q->x + Q->y * Q->z)), (1.0f - 2.0f * (Q->x * Q->x + Q->y * Q->y)) );
	E->pitch = 2.0f * atan2f( sqrtf(1.0f + 2.0f * (Q->w * Q->y - Q->x * Q->z)), sqrtf(1.0f - 2.0f * (Q->w * Q->y - Q->x * Q->z)) ) - M_PI/2;
	E->yaw = atan2f( (2.0f * (Q->w * Q->z + Q->x * Q->y)), (1.0f - 2.0f * (Q->y * Q->y + Q->z * Q->z)) );

}
