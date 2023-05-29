#include "imu.h"


//Roll -> Y
//Pitch -> X
//Yaw -> Z


void IMU(float *gyroX, float *gyroY, float *gyroZ, float *accX, float *accY, float *accZ, float *magX, float *magY, float *magZ)
{
	float gyroRaw[3]  = {0.0f, 0.0f, 0.0f};
	int16_t accRaw[3] = {0, 0, 0};
	float acc[3] = {0.0f, 0.0f, 0.0f};
	int16_t magRaw[3] = {0, 0, 0};
	float mag[3] = {0.0f, 0.0f, 0.0f};

	BSP_GYRO_GetXYZ(gyroRaw);
	BSP_COMPASS_AccGetXYZ(accRaw);
	BSP_COMPASS_MagGetXYZ(magRaw);

	//GYRO
	//gyro values to degrees
	*gyroX = (float)gyroRaw[0] * GYRO_SENSITIVITY * DT;
	*gyroY = (float)gyroRaw[1] * GYRO_SENSITIVITY * DT;
	*gyroZ = (float)gyroRaw[2] * GYRO_SENSITIVITY * DT;

	//ACC
	//raw data to m/s^2 with soft iron
	acc[0] = (float)accRaw[0] * ACC_SENSITIVITY - ACCSI1;
	acc[1] = (float)accRaw[1] * ACC_SENSITIVITY - ACCSI2;
	acc[2] = (float)accRaw[2] * ACC_SENSITIVITY - ACCSI3;
	//hard iron
	*accX = ACCHI11 * acc[0] + ACCHI12 * acc[1] + ACCHI13 * acc[2];
	*accY = ACCHI21 * acc[0] + ACCHI22 * acc[1] + ACCHI23 * acc[2];
	*accZ = ACCHI31 * acc[0] + ACCHI32 * acc[1] + ACCHI33 * acc[2];

	//MAG
	//Calculate raw data to nanotesla with soft iron
	mag[0] = magRaw[0] * MAG_SENSITIVITY - MAGSI1;
	mag[1] = magRaw[1] * MAG_SENSITIVITY - MAGSI2;
	mag[2] = magRaw[2] * MAG_SENSITIVITY - MAGSI3;
	//hard iron
	*magX = MAGHI11 * mag[0] + MAGHI12 * mag[1] + MAGHI13 * mag[2];
	*magY = MAGHI21 * mag[0] + MAGHI22 * mag[1] + MAGHI23 * mag[2];
	*magZ = MAGHI31 * mag[0] + MAGHI32 * mag[1] + MAGHI33 * mag[2];


//	//acc values to degrees
//	float accAngX = (float)((atan2(accY, accZ)) + M_PI ) * (180 / M_PI);
//	float accAngY = (float)((atan2(accZ, accX)) + M_PI ) * (180 / M_PI);
//
//	accAngY -= 270.0f;
//    if (accAngY < -180.0f)
//    	accAngY += 360.0f;
//
//    accAngX -= 180.0f;
//
//
//	//complementary filtration and position calculation
//	*angX = ALPHA * (gyroX * DT + *angX) + (1.0f - ALPHA) * accAngX;
//	*angY = ALPHA * (gyroY * DT + *angY) + (1.0f - ALPHA) * accAngY;
//	//roll filtration and calculation
//	*angZ = (gyroZ * DT + *angZ);
}
