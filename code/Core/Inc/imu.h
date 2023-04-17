#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_qspi.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_compass.h"
#include <math.h>

#define ACC_SENSITIVITY  0.0061
#define GYRO_SENSITIVITY 0.0175
#define ALPHA 0.98		//filtration rate
#define DT 0.02			//time tick

void IMU(float *angX, float *angY, float *angZ);

#endif /* INC_IMU_H_ */
