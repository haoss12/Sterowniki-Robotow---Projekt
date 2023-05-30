/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "imu.h"
#include "madgwick.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QSPI_FLASH_SIZE                      23
#define QSPI_PAGE_SIZE                       256
#define PRECISION 							 0.000001f
#define REMAP_OK							 0
#define REMAP_ERROR							 1
#define ALPHA 								 0.98
#define USART_DEBUG 			0	// 1 - print to usart
#define TEST_READ_AND_PRINT		0	// 1 - read data and print it
#define TIME_COUNTING			0
#define BUFFER_SIZE 			128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define COUNT(__BUF__)        (sizeof(__BUF__) / sizeof(*(__BUF__)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char DataBuffer[BUFFER_SIZE];
uint8_t ReadBuffer[BUFFER_SIZE];

float angX = 0.0f;
float angY = 0.0f;
float angZ = 0.0f;

float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
RPY euler, eulerPrev;

float r, p, y;		// for STM32CubeMonitor
float pid_out_temp; // for STM32CubeMonitor

//calculated data from IMU
float g[3];
float a[3];
float m[3];

//Raw data for readings
int16_t mr[3] = {0, 0, 0};
int16_t ar[3] = {0, 0, 0};
float gr[3]  = {0.0f, 0.0f, 0.0f};

const float delta = 0.01f;

Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};

PID controller = {.Kp = _KP, .Ki = _KI, .Kd = _KD};

uint8_t flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline void remap(float value, float *output, float min, float max, float r_min, float r_max);

void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim ) {
        if ( htim == &htim6 ) {
        	HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
        	flag = 1;
//    		__HAL_TIM_SET_COUNTER(&htim6, 999);
        }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t address = 0;
	float pid_out_sum = 0.0f;
	uint8_t counter = 0;
	float servo_value = 0.0f;
	uint32_t crc;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint32_t historic = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_QUADSPI_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  // timer for PWM initialization
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

//  __HAL_TIM_SET_COUNTER(&htim6, 999);

  HAL_Delay(200);

  //test of servos from -90 to 90 degrees
for (int var = 160; var <= 800; var += 160) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, var);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, var);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, var);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, var);
	HAL_Delay(500);
}

	// set servos in neutral position
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 480);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 480);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 480);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 480);
}

r = 0.0f;
p = 0.0f;
y = 0.0f;

HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 1);
HAL_Delay(100);
HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 0);
HAL_Delay(900);

  BSP_QSPI_Init();

  // Clear memory
	#if USART_DEBUG
  	  	  printf("Start of memory erasing\r\n");
	#endif
    if (BSP_QSPI_Erase_Chip() != QSPI_OK) {
  	  Error_Handler();
    }
	#if USART_DEBUG
    	printf("End of memory erasing\r\n");
	#endif
    	HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 1);
    	HAL_Delay(100);
    	HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 0);
    	HAL_Delay(200);
    	HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 1);
    	HAL_Delay(100);
    	HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 0);
    	HAL_Delay(200);


	BSP_GYRO_Init();
	BSP_COMPASS_Init();

	HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flag == 1) {

		flag = 0;

		// get new measurements from IMU
		updateIMU(mr, ar, gr, a, g, m);

		// update quaternion by madgwick filter and recalculate it to euler angles
		updateQuat(g, a, m);
		quat2rpy(&q, &euler);

		// low pass filter
		euler.roll = ALPHA * eulerPrev.roll + (1.0f - ALPHA) * euler.roll;
		eulerPrev.roll = euler.roll;
		euler.pitch = ALPHA * eulerPrev.pitch + (1.0f - ALPHA) * euler.pitch;
		eulerPrev.pitch = euler.pitch;
		euler.yaw = ALPHA * eulerPrev.yaw + (1.0f - ALPHA) * euler.yaw;
		eulerPrev.yaw = euler.yaw;

		// rad to deg
		r = euler.roll 	* (180.0 / M_PI);
		p = euler.pitch * (180.0 / M_PI);
		y = euler.yaw 	* (180.0 / M_PI);

		// pid step for one axis
		updatePID(0.0f, y);

		// for STM32CubeMonitor
		pid_out_temp = controller.out;
		pid_out_sum += controller.out;

		  snprintf(DataBuffer, BUFFER_SIZE, "X %f %f %f %f", r, p, y, pid_out_temp);
		  crc = HAL_CRC_Calculate(&hcrc, (void*)DataBuffer, strlen(DataBuffer));
		  snprintf(DataBuffer, BUFFER_SIZE, "%s 0x%02x\r\n", DataBuffer, crc);

		#if USART_DEBUG
		  	  printf("%s", DataBuffer);
		#endif

		BSP_QSPI_Write((uint8_t*)DataBuffer, address, BUFFER_SIZE);	// write data to memory


		#if TEST_READ_AND_PRINT
			BSP_QSPI_Read(ReadBuffer, address, BUFFER_SIZE);
			printf("%s", ReadBuffer);
		#endif

		address += BUFFER_SIZE;
		counter++;

		if (counter == 4) {
			pid_out_sum /= 4.0f;
			remap(pid_out_sum, &servo_value, -500.0f, 500.0f, 320.0, 640.0);

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_value);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo_value);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, servo_value);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, servo_value);

			pid_out_sum = 0.0f;
			counter = 0;
		}

		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, ptr, len, 50);

	return len;
}

/*
min and max - base range
r_min and r_max - range after remap
*/
static inline void remap(float value, float *output, float min, float max, float r_min, float r_max)
{
    if (fabs(max - min) <= PRECISION)
    {
        *output = (r_min + r_min)/2;
    }

    *output = (r_min + (r_max - r_min)*(value - min)/(max - min));
    if (r_max > r_min){
        if (*output > r_max) *output = r_max;
        else if (*output < r_min) *output = r_min;
    }
    if (r_min > r_max){
        if (*output > r_min) *output = r_min;
        else if (*output < r_max) *output = r_max;
    }
}


//void IMU(float *angX, float *angY, float *angZ) {
//	int16_t accRaw[3] = {0, 0, 0};
//	float gyroRaw[3]  = {0.0f, 0.0f, 0.0f};
//
//	BSP_GYRO_GetXYZ(gyroRaw);
//	BSP_COMPASS_AccGetXYZ(accRaw);
//	//raw to m/s^2
//	float accX = (float)accRaw[0] * ACC_SENSITIVITY;
//	float accY = (float)accRaw[1] * ACC_SENSITIVITY;
//	float accZ = (float)accRaw[2] * ACC_SENSITIVITY;
//
//	//gyro values to degrees
//	float gyroX = gyroRaw[0] * GYRO_SENSITIVITY * DT;
//	float gyroY = gyroRaw[1] * GYRO_SENSITIVITY * DT;
//	float gyroZ = gyroRaw[2] * GYRO_SENSITIVITY * DT;
//
//	//acc values to degrees
//	float accAngX = (float)((atan2(accY, accZ)) + M_PI ) * (180 / M_PI);
//	float accAngY = (float)((atan2(accZ, accX)) + M_PI ) * (180 / M_PI);
//
//	//complementary filtration and position calculation
//	*angX = ALPHA * (gyroX * DT + angX) + (1.0f - ALPHA) * accAngX;
//	*angY = ALPHA * (gyroY * DT + angY) + (1.0f - ALPHA) * accAngY;
//	//roll filtration and calculation
//	*angZ = ALPHA * (gyroZ * DT + angZ);
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 1);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 0);
	  HAL_Delay(500);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
