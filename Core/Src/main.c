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
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
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
 SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t receive_buff[64];
uint8_t RxBuff[12];

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

char buffer[1024];

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void send_uart (char *string)
{
	uint8_t len = strlen (string);
	HAL_UART_Transmit(&huart2, (uint8_t *) string, len, 2000);// transmit in blocking mode
}

int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void clear_buffer (void)
{
	for (int i=0; i<1024; i++)
	{
		buffer[i] = '\0';
	}
}

void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(&huart1);                                                     	//stop DMA transfer
    uint8_t data_length  = 64 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);   	//calculate received data length

    //test - return received data
//    HAL_UART_Transmit(&huart2,receive_buff,data_length,0x200);                   	//Test: transfer received data directly
    memcpy(RxBuff,receive_buff,12);
    if (RxBuff[3] == 0x83)
//    	RxFlag = TRUE;
    	memset(receive_buff,0,data_length);                                            	//clear receive buffer
    data_length = 0;
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, 64);                    	//open DMA again
}

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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_USART1_UART_Init();
  MX_SPI1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, 64);

  //Mounting SD card
  fresult = f_mount(&fs, "/", 1);
  if (fresult != FR_OK)
	  send_uart ("ERROR!!! in mounting SD CARD...\n\n");
  else
	  send_uart("SD CARD mounted successfully...\n\n");

  //Check free space
  f_getfree("", &fre_clust, &pfs);

  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  sprintf (buffer, "SD CARD Total Size: \t%lu\n",total);
  send_uart(buffer);
  clear_buffer();
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
  sprintf (buffer, "SD CARD Free Space: \t%lu\n\n",free_space);
  send_uart(buffer);
  clear_buffer();

/*
  //==========The following operation is using PUTS and GETS===========

  //Open file to write/ create a file if it doesn't exist
  fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

  //Writing text
  f_puts("This data is from the FILE1.txt. And it was written using ...f_puts... ", &fil);

  //Close file
  fresult = f_close(&fil);

  if (fresult == FR_OK)
	  send_uart ("File1.txt created and the data is written \n");

  //Open file to read
  fresult = f_open(&fil, "file1.txt", FA_READ);

  //Read string from the file
  f_gets(buffer, f_size(&fil), &fil);

  send_uart("File1.txt is opened and it contains the data as shown below\n");
  send_uart(buffer);
  send_uart("\n\n");

  //Close file
  f_close(&fil);

  clear_buffer();
*/

/*
  //==========The following operation is using f_write and f_read==========

  //Create second file with read write access and open it
  fresult = f_open(&fil, "file2.txt", FA_CREATE_ALWAYS | FA_WRITE);

  //Writing text
  strcpy (buffer, "This is File2.txt, written using ...f_write... and it says Hello from Controllerstech\n");

  fresult = f_write(&fil, buffer, bufsize(buffer), &bw);

  send_uart ("File2.txt created and data is written\n");

  //Close file
  f_close(&fil);

  // clearing buffer to show that result obtained is from the file
  clear_buffer();

  //Open second file to read
  fresult = f_open(&fil, "file2.txt", FA_READ);
  if (fresult == FR_OK)
	  send_uart ("file2.txt is open and the data is shown below\n");

  //Read data from the file, please see the function details for the arguments
  f_read (&fil, buffer, f_size(&fil), &br);
  send_uart(buffer);
  send_uart("\n\n");

  //Close file
  f_close(&fil);

  clear_buffer();
*/

/*
  //==========UPDATING an existing file==========

  //Open the file with write access
  fresult = f_open(&fil, "file2.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);

  //Move to offset to the end of the file
  fresult = f_lseek(&fil, f_size(&fil));

  if (fresult == FR_OK)
	  send_uart ("About to update the file2.txt\n");

  //write the string to the file
  fresult = f_puts("This is updated data and it should be in the end", &fil);

  f_close (&fil);

  clear_buffer();

  //Open to read the file
  fresult = f_open (&fil, "file2.txt", FA_READ);

  //Read string from the file
  fresult = f_read (&fil, buffer, f_size(&fil), &br);
  if (fresult == FR_OK)
	  send_uart ("Below is the data from updated file2.txt\n");
  send_uart(buffer);
  send_uart("\n\n");

  //Close file
  f_close(&fil);

  clear_buffer();

  //==========REMOVING FILES FROM THE DIRECTORY==========

  fresult = f_unlink("/file1.txt");
  if (fresult == FR_OK)
	  send_uart("file1.txt removed successfully...\n");

  fresult = f_unlink("/file2.txt");
  if (fresult == FR_OK)
	  send_uart("file2.txt removed successfully...\n");

  //Unmount SDCARD
  fresult = f_mount(NULL, "/", 1);
  if (fresult == FR_OK)
	  send_uart ("SD CARD UNMOUNTED successfully...\n");
*/

  //test code
  //Create the file with read write access and open it
  fresult = f_open(&fil, "trip.txt", FA_CREATE_ALWAYS | FA_WRITE);

  //Writing text
  strcpy (buffer, "vehicleNo meterNo startTime endTime waitTime journeyTime paidKM totalKM nonBusiness amount surCharge totalCharge\n");

  fresult = f_write(&fil, buffer, bufsize(buffer), &bw);

  send_uart ("trip.txt created and data is written\n");

  //Close file
  f_close(&fil);

  // clearing buffer to show that result obtained is from the file
  clear_buffer();
/*
  //Open the file to read
  fresult = f_open(&fil, "trip.txt", FA_READ);
  if (fresult == FR_OK)
	  send_uart ("trip.txt is open and the data is shown below\n");

  //Read data from the file, please see the function details for the arguments
  f_read (&fil, buffer, f_size(&fil), &br);
  send_uart(buffer);
  send_uart("\n\n");

  //Close file
  f_close(&fil);

  clear_buffer();
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ((GetPress() == 1))
	  {
		  fresult = f_open(&fil, "trip.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
		  fresult = f_lseek(&fil, f_size(&fil));
		  if (fresult == FR_OK)
			  send_uart ("Update the trip.txt\n");

		  fresult = f_puts("AB1234 A000001 2023-07-18 18:18:30 4.3 17.4 10.3 10.8 5.6 107.8 5 112.8\n", &fil);
		  f_close (&fil);
		  clear_buffer();


/*
		  //Open to read the file
		  fresult = f_open(&fil, "trip.txt", FA_READ);

		  //Read string from the file
		  fresult = f_read(&fil, buffer, f_size(&fil), &br);
		  if (fresult == FR_OK)
			  send_uart ("Below is the data from updated trip.txt\n");
		  send_uart(buffer);
		  send_uart("\n\n");

		  //Close file
		  f_close(&fil);
		  clear_buffer();
*/
		  send_uart("data write\n");
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
_Bool GetPress(void)
{
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0)
	{
		HAL_Delay(10);
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0)
		{
			while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0)
			{;}
			return 1;
		}
		else return 0;
	}
	else return 0;
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)                                   //Determine whether it is serial port 1
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //Judging whether it is idle interruption
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);						//Clear idle interrupt sign (otherwise it will continue to enter interrupt)
            USER_UART_IDLECallback(huart);                          //Call interrupt handler
        }
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
