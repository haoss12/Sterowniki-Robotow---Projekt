#include "imu.h"

void IMU(float *angX, float *angY, float *angZ) {
	int16_t accRaw[3] = {0, 0, 0};
	float gyroRaw[3]  = {0.0f, 0.0f, 0.0f};

	BSP_GYRO_GetXYZ(gyroRaw);
	BSP_COMPASS_AccGetXYZ(accRaw);
	//raw to m/s^2
	float accX = (float)accRaw[0] * ACC_SENSITIVITY;
	float accY = (float)accRaw[1] * ACC_SENSITIVITY;
	float accZ = (float)accRaw[2] * ACC_SENSITIVITY;

	//gyro values to degrees
	float gyroX = gyroRaw[0] * GYRO_SENSITIVITY * DT;
	float gyroY = gyroRaw[1] * GYRO_SENSITIVITY * DT;
	float gyroZ = gyroRaw[2] * GYRO_SENSITIVITY * DT;

	//acc values to degrees
	float accAngX = (float)((atan2(accY, accZ)) + M_PI ) * (180 / M_PI);
	float accAngY = (float)((atan2(accZ, accX)) + M_PI ) * (180 / M_PI);

	//complementary filtration and position calculation
	*angX = ALPHA * (gyroX * DT + *angX) + (1.0f - ALPHA) * accAngX;
	*angY = ALPHA * (gyroY * DT + *angY) + (1.0f - ALPHA) * accAngY;
	//roll filtration and calculation
	*angZ = (gyroZ * DT + *angZ);
}
