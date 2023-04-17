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
#include "dma.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "imu.h"
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define COUNT(__BUF__)        (sizeof(__BUF__) / sizeof(*(__BUF__)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t TxBuffer[] = "1234567890abcdefgh"; //
uint8_t RxBuffer[COUNT(TxBuffer) - 1];

float angX = 0.0f;
float angY = 0.0f;
float angZ = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t remap(float value, float *output, float min, float max, float r_min, float r_max);
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
	//uint16_t index;
	uint16_t flag = 0;
	float angXremapped = 0.0f;
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
  /* USER CODE BEGIN 2 */

  // write example data to flash memory

//  QSPI_CommandTypeDef s_command;
//
//  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
//  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
//  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
//  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  BSP_QSPI_Init();

  //BSP_QSPI_Erase_Chip();
//soft i hard iron
  BSP_QSPI_Write(TxBuffer, address, COUNT(TxBuffer) - 1);

  BSP_QSPI_Read(RxBuffer, address, COUNT(TxBuffer) - 1);

	for (int i = 0; i < COUNT(TxBuffer) - 1; ++i) {
		  if (RxBuffer[i] == TxBuffer[i]) {
			  flag++;
		  }
	}

	BSP_GYRO_Init();
	BSP_COMPASS_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GetTick() - historic > 20){
		historic = HAL_GetTick();
		IMU(&angX, &angY, &angZ);
		remap(angX, &angXremapped, 0.0f, 360.0f, -180.0f, 180.0f);
		printf("%f -> %f\r\n", angX, angXremapped);
  	  	}

		//		gyroX = GYRO_ALPHA * gyroX + (1.0f - GYRO_ALPHA) * gyroRaw[0] * GYRO_SENSITIVITY;
		//		gyroY = GYRO_ALPHA * gyroY + (1.0f - GYRO_ALPHA) * gyroRaw[1] * GYRO_SENSITIVITY;
		//		gyroZ = GYRO_ALPHA * gyroZ + (1.0f - GYRO_ALPHA) * gyroRaw[2] * GYRO_SENSITIVITY;
		//
		//		accX = ACC_ALPHA * accX + (1.0f - ACC_ALPHA) * (float)accRaw[0] * ACC_SENSITIVITY;
		//		accY = ACC_ALPHA * accY + (1.0f - ACC_ALPHA) * (float)accRaw[1] * ACC_SENSITIVITY;
		//		accZ = ACC_ALPHA * accZ + (1.0f - ACC_ALPHA) * (float)accRaw[2] * ACC_SENSITIVITY;
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
uint8_t remap(float value, float *output, float min, float max, float r_min, float r_max)
{
	if (min > max) return REMAP_ERROR;
	if (r_min > r_max) return REMAP_ERROR;

    if ((max - min) <= PRECISION)
    {
        *output = (r_min + r_min)/2;
        return REMAP_OK;
    }

    *output = (r_min + (r_max - r_min)*((value - min)/(max - min)));
    return REMAP_OK;
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
