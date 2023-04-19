/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bluetooth.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
BluetoothConfig bluetoothConfig = {
		  .uart = &huart3,
};
BluetoothController bluetoothController;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

// IMU related functions
HAL_StatusTypeDef InitIMU();
uint16_t GetAccel();
// File I/O related functions
FRESULT InitFS();

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

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
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  bluetoothConfig.uart->RxCpltCallback = HAL_UART_RxCpltCallback;
  bluetoothConfig.uart->TxCpltCallback = HAL_UART_TxCpltCallback;
  bluetooth_init(&bluetoothConfig, &bluetoothController);

//  FATFS fs;
//  HAL_StatusTypeDef ret;
//  FRESULT fres;
//  FIL f;
//
//  fres = InitFS(&fs);
//  if (fres != FR_OK) {
//	  Error_Handler();
//  }

//  HAL_StatusTypeDef res = InitIMU();
//  if (res != HAL_OK) {
//	  Error_Handler();
//  }

//  //Junk begins here
//  uint16_t count = 0;
//  double val = 0.0;
//  int16_t val_x = 0;
//  int16_t val_y = 0;
//  int16_t val_z = 0;
//  uint8_t buf[6] = {0, 0, 0, 0, 0, 0} ;
//  uint8_t str[20];
//  UINT writeBytes = 0;
//
//  fres = f_open(&f, "log3.txt", FA_CREATE_ALWAYS| FA_WRITE);
//  if (fres != FR_OK){
//	  Error_Handler();
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
//	  //HAL_Delay(100);
//
//
//	  // collect and log a bunch of samples
//	  if (count < 10000) {
//		ret = HAL_I2C_Mem_Read(&hi2c1, ADXL343_ADDR, ADXL343_REG_DATAX0, 1, buf, 6, 10);
//		if (ret != HAL_OK) {
//			Error_Handler();
//		}
//
//		val_x = (buf[1] << 8) + buf[0];
//		val_y = (buf[3] << 8) + buf[2];
//		val_z = (buf[5] << 8) + buf[4];
//		val = ADXL343_SCALE_8G * val_z;
//
//		sprintf(str, "%f, %d\n", val, count);
//		fres = f_write(&f, str, strlen(str), &writeBytes);
//		if (fres != FR_OK){
//			return fres;
//		}
//		/*
//		if (f_printf(&f, "%5f, %d\n", val, count) < 0) {
//			Error_Handler();
//		}
//		*/
//
//		++count;
//	  }
//	  else if (count == 10000) {
//		  fres = f_close(&f);
//		  if (fres != FR_OK) {
//			  Error_Handler();
//		  }
//		  ++count;
//	  }
//	  else {
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
//	  HAL_Delay(100);
//	  }

	  	  bluetooth_run(&bluetoothController);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 224;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

HAL_StatusTypeDef InitIMU() {

	HAL_StatusTypeDef ret;
	uint8_t buf[1] = {0};

	//TODO: ASXL343 has a built in self-test. Use it?
	ret = HAL_I2C_Mem_Read(&hi2c1, ADXL343_ADDR, ADXL343_REG_DEVID, 1, buf, 1, 10);
	if (ret != HAL_OK) {
		return ret;
	}
	if (buf[0] != 0xe5) {	//check device id
		return HAL_ERROR;
	}


	// configure interrupts
	buf[0] = 0b00000000;		// disable all interrupts for now
	ret = HAL_I2C_Mem_Write(&hi2c1, ADXL343_ADDR, ADXL343_REG_INTENABLE, 1, buf, 1, 10);
	if (ret != HAL_OK) {
		return ret;
	}

	// set the data format and range
	buf[0] = 0b00000010;		// set to +/-8g max
	ret = HAL_I2C_Mem_Write(&hi2c1, ADXL343_ADDR, ADXL343_REG_DATAFORMAT, 1, buf, 1, 10);
	if (ret != HAL_OK) {
		return ret;
	}

	// set the data rate and power mode
	buf[0] = 0b00001101;		// set to 800 Hz, normal operation mode
	ret = HAL_I2C_Mem_Write(&hi2c1, ADXL343_ADDR, ADXL343_REG_BWRATE, 1, buf, 1, 10);
	if (ret != HAL_OK) {
		return ret;
	}

	// set the fifo operation mode
	buf[0] = 0b00000000;		// set fifo to bypass
	ret = HAL_I2C_Mem_Write(&hi2c1, ADXL343_ADDR, ADXL343_REG_FIFOCTL, 1, buf, 1, 10);
	if (ret != HAL_OK) {
		return ret;
	}


	// set the power/sleep settings
	buf[0] = 0b00001000;		// sets device in measurement mode
	ret = HAL_I2C_Mem_Write(&hi2c1, ADXL343_ADDR, ADXL343_REG_PWRCTL, 1, buf, 1, 10);
	if (ret != HAL_OK) {
		return ret;
	}
	return HAL_OK;
}


uint16_t GetAccel() {

	return 0;
}


FRESULT InitFS(FATFS *fs){
  FIL f;
  FRESULT fres;
  char buf[6] = "Hello";
  UINT writeBytes;

  // Re-initialize SD
  if ( BSP_SD_Init() != MSD_OK ) {
    return FR_NOT_READY;
  }
  // Re-initialize FATFS
  if ( FATFS_UnLinkDriver(SDPath) != 0 ) {
    return FR_NOT_READY;
  }
  if ( FATFS_LinkDriver(&SD_Driver, SDPath) != 0 ) {
    return FR_NOT_READY;
  }


  fres = f_mount(fs, SDPath, 0);
  if (fres != FR_OK){
	  return fres;
  }
  fres = f_open(&f, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
  if (fres != FR_OK){
	  return fres;
  }
  fres = f_write(&f, buf, 5, &writeBytes);
  if (fres != FR_OK){
	  return fres;
  }
  fres = f_close(&f);		// close also syncs the file
  if (fres != FR_OK){
	  return fres;
  }
  return FR_OK;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == bluetoothConfig.uart) {
		bluetooth_uart_rx(&bluetoothController);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == bluetoothConfig.uart) {
		bluetooth_uart_tx(&bluetoothController);
	}
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
