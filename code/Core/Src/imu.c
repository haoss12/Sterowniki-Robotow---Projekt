#include "imu.h"


//Roll -> Y
//Pitch -> X
//Yaw -> Z


void IMU(float **a, float **g, float **m)
{
	int16_t magRaw[3] = {0, 0, 0};
	int16_t accRaw[3] = {0, 0, 0};
	float gyroRaw[3]  = {0.0f, 0.0f, 0.0f};
	float acc[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};

	BSP_GYRO_GetXYZ(gyroRaw);
	BSP_COMPASS_AccGetXYZ(accRaw);
	BSP_COMPASS_MagGetXYZ(magRaw);

	//GYRO
	//gyro values to degrees
	*g[0] = (float)gyroRaw[0] * GYRO_SENSITIVITY;
	*g[1] = (float)gyroRaw[1] * GYRO_SENSITIVITY;
	*g[2] = (float)gyroRaw[2] * GYRO_SENSITIVITY;

	//ACC
	//raw data to m/s^2 with soft iron
	acc[0] = (float)accRaw[0] * ACC_SENSITIVITY - ACCSI1;
	acc[1] = (float)accRaw[1] * ACC_SENSITIVITY - ACCSI2;
	acc[2] = (float)accRaw[2] * ACC_SENSITIVITY - ACCSI3;
	//hard iron
	*a[0] = ACCHI11 * acc[0] + ACCHI12 * acc[1] + ACCHI13 * acc[2];
	*a[1] = -(ACCHI21 * acc[0] + ACCHI22 * acc[1] + ACCHI23 * acc[2]);	//Axis swap
	*a[2] = ACCHI31 * acc[0] + ACCHI32 * acc[1] + ACCHI33 * acc[2];

	//MAG
	//Calculate raw data to nanotesla with soft iron
	mag[0] = magRaw[0] * MAG_SENSITIVITY - MAGSI1;
	mag[1] = magRaw[1] * MAG_SENSITIVITY - MAGSI2;
	mag[2] = magRaw[2] * MAG_SENSITIVITY - MAGSI3;
	//hard iron
	*m[0] = MAGHI11 * mag[0] + MAGHI12 * mag[1] + MAGHI13 * mag[2];
	*m[1] = -(MAGHI21 * mag[0] + MAGHI22 * mag[1] + MAGHI23 * mag[2]);	//Axis swap
	*m[2] = MAGHI31 * mag[0] + MAGHI32 * mag[1] + MAGHI33 * mag[2];

}
