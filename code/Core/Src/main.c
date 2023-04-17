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
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_qspi.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_compass.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QSPI_FLASH_SIZE 		23
#define QSPI_PAGE_SIZE			256
#define USART_DEBUG 			1 	// 1 - print to usart
#define TEST_READ_AND_PRINT		1	// 1 - read data and print it
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	uint32_t crc;
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	int16_t acc[3] = {0, 0, 0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

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
  printf("start czyszczenia pamieci\r\n");
  if (BSP_QSPI_Erase_Chip() != QSPI_OK) {
	  Error_Handler();
  }
  printf("koniec czyszczenia pamieci\r\n");
  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 1);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, 0);
  HAL_Delay(900);

	//HAL_Delay(3000);	// time for user to open debug terminal

	BSP_GYRO_Init();
	BSP_COMPASS_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);
	  uint32_t startTime = HAL_GetTick();

	  BSP_GYRO_GetXYZ(gyro);
	  BSP_COMPASS_AccGetXYZ(acc);

	  snprintf(DataBuffer, BUFFER_SIZE, "X %f %f %f %d %d %d", gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);

	  #if USART_DEBUG
	  crc = HAL_CRC_Calculate(&hcrc, (void*)DataBuffer, strlen(DataBuffer));
	  printf("%s 0x%02x\r\n", DataBuffer, crc);
	  //#else
	  BSP_QSPI_Write((uint8_t*)DataBuffer, address, BUFFER_SIZE);	// write data to memory
	  #endif

	  uint32_t endTime = HAL_GetTick();
	  uint32_t elapsedTime = endTime-startTime;

	  #if USART_DEBUG
	  printf("%d \r\n", elapsedTime);
	  #endif

	  #if TEST_READ_AND_PRINT
	  BSP_QSPI_Read(ReadBuffer, address, BUFFER_SIZE);
	  printf("%s\r\n", ReadBuffer);
	  #endif

	  address += BUFFER_SIZE;
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
