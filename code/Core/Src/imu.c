#include "imu.h"


//Roll -> Y
//Pitch -> X
//Yaw -> Z


void updateIMU(int16_t *mr, int16_t *ar, float *gr, float *a, float *g, float *m)
{
//	int16_t magRaw[3] = {0, 0, 0};
//	int16_t accRaw[3] = {0, 0, 0};
//	float gyroRaw[3]  = {0.0f, 0.0f, 0.0f};
	float acc[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};

	BSP_GYRO_GetXYZ(gr);
	BSP_COMPASS_AccGetXYZ(ar);
	BSP_COMPASS_MagGetXYZ(mr);

	//GYRO
	//gyro values to degrees
	g[0] = gr[0] * GYRO_SENSITIVITY;
	g[1] = gr[1] * GYRO_SENSITIVITY;
	g[2] = gr[2] * GYRO_SENSITIVITY;

	//ACC
	//raw data to m/s^2 with soft iron
	acc[0] = (float)ar[0] * ACC_SENSITIVITY - ACCSI1;
	acc[1] = (float)ar[1] * ACC_SENSITIVITY - ACCSI2;
	acc[2] = (float)ar[2] * ACC_SENSITIVITY - ACCSI3;
	//hard iron
	a[0] = ACCHI11 * acc[0] + ACCHI12 * acc[1] + ACCHI13 * acc[2];
	a[1] = -(ACCHI21 * acc[0] + ACCHI22 * acc[1] + ACCHI23 * acc[2]);	//Axis swap
	a[2] = ACCHI31 * acc[0] + ACCHI32 * acc[1] + ACCHI33 * acc[2];

	//MAG
	//Calculate raw data to nanotesla with soft iron
	mag[0] = (float)mr[0] * MAG_SENSITIVITY - MAGSI1;
	mag[1] = (float)mr[1] * MAG_SENSITIVITY - MAGSI2;
	mag[2] = (float)mr[2] * MAG_SENSITIVITY - MAGSI3;
	//hard iron
	m[0] = MAGHI11 * mag[0] + MAGHI12 * mag[1] + MAGHI13 * mag[2];
	m[1] = -(MAGHI21 * mag[0] + MAGHI22 * mag[1] + MAGHI23 * mag[2]);	//Axis swap
	m[2] = MAGHI31 * mag[0] + MAGHI32 * mag[1] + MAGHI33 * mag[2];

	g[0] = g[0] * M_PI / 180.0f * delta;
	g[1] = g[1] * M_PI / 180.0f * delta;
	g[2] = g[2] * M_PI / 180.0f * delta;

}
