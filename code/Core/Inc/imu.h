#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_qspi.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_compass.h"
#include <math.h>

#define ACC_SENSITIVITY  0.00061
#define GYRO_SENSITIVITY 0.0175
#define MAG_SENSITIVITY 0.058
#define ALPHA 0.95		//filtration rate
#define DT 0.05			//time tick

#define MAGSI1 -30.416148;
#define MAGSI2 26.435598;
#define MAGSI3 8.019681;
#define MAGHI11 1.039317
#define MAGHI12 0.023616
#define MAGHI13 -0.003230
#define MAGHI21 0.023616
#define MAGHI22 1.075811
#define MAGHI23 -0.000486
#define MAGHI31 -0.003230
#define MAGHI32 -0.000486
#define MAGHI33 1.124874

#define ACCSI1 0.361457;
#define ACCSI2 0.415663;
#define ACCSI3 0.018694;
#define ACCHI11 0.982255
#define ACCHI12 -0.001020
#define ACCHI13 -0.000266
#define ACCHI21 -0.001020
#define ACCHI22 0.989506
#define ACCHI23 -0.001753
#define ACCHI31 -0.000266
#define ACCHI32 -0.001753
#define ACCHI33 0.995164

void IMU(float **a, float **g, float **m);

#endif /* INC_IMU_H_ */
