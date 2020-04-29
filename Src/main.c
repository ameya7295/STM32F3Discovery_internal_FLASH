/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_STORAGE 0x08005000
#define page_size 0x200

void write_to_flash(uint8_t* data)
{
	volatile uint32_t data_to_flash[(strlen((char*)data)/4) + (int)((strlen((char*)data)%4) != 0)];
	memset((uint8_t*)data_to_flash,0,strlen((char*)data_to_flash));
	strcpy((char*)data_to_flash, (char*) data);

	volatile uint32_t data_lenght = (strlen((char*)data)/4) + (int)((strlen((char*)data)%4) != 0);
	//volatile uint16_t pages = (strlen((char*)data)/4) + (int)((strlen((char*)data)%4) != 0);

	HAL_FLASH_Unlock();

	HAL_FLASH_OB_Unlock();


	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	//EraseInitStruct.Banks = 1;
	EraseInitStruct.Sector = FLASH_SECTOR_1;
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	uint32_t sectorError;

	volatile uint32_t write_count = 0, index = 0;
	volatile HAL_StatusTypeDef status;

	//Erase the data before writing
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &sectorError);

	while(index < data_lenght)
	{
		if(status == HAL_OK)
		{
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_STORAGE + write_count,data_to_flash[index]);
			if(status == HAL_OK)
			{
				write_count = write_count + 4;
				index ++;
			}
		}
	}
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}

void read_from_flash(uint8_t* data)
{
	volatile uint32_t read_data;
	volatile uint32_t read_count = 0;

	do
	{
		read_data = *(uint32_t*) (FLASH_STORAGE + read_count);
		if(read_data != 0xFFFFFFFF)
		{
			data[read_count] = (uint8_t)read_data;
			data[read_count + 1]  = (uint8_t)(read_data >> 8);
			data[read_count + 2]  = (uint8_t)(read_data >> 16);
			data[read_count + 3]  = (uint8_t)(read_data >> 24);
			read_count = read_count + 4;
		}
	}while(read_data != 0xFFFFFFFF);
	read_count;
}


uint32_t Readflash(uint32_t addr)
{
 uint32_t* data = (uint32_t*)(addr);
 return *data;
}

void Write_Buff_To_InternalFlash(uint8_t data_in[],uint32_t start_addr,unsigned int len)
{
	unsigned int i;
	uint32_t flash_status = 0;

	if((start_addr - 0x8000000)%0x800==0) //Erase new page if data locate at new page
		{
			//HAL_FLASH_Unlock();
			//FLASH_ErasePage(start_addr);
		}
		for(i = 0;i<len;i++)
		{

			flash_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr+4*i, data_in[i]);

			//FLASH_Program_Word(start_addr+4*i, data_in[i]);
			//flash_status = __HAL_FLASH_GET_FLAG;
			//if(flash_status != HAL_OK)
				//return flash_status;

			/*if(*((uint32_t*)(start_addr+4*i)) != *((uint32_t*)(data_in[i])))
			{
				return false;
			}*/
		}

		if(flash_status)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

		/*flash_status = flash_status flash_get_status_flags();
				if(flash_status != FLASH_SR_EOP)
					return flash_status;
		*/
		/*verify if correct data is programmed*/

		/*if(*((uint32_t*)(current_address+iter)) != *((uint32_t*)(input_data + iter)))
					return FLASH_WRONG_DATA_WRITTEN;
		*/
}
void Read_Buff_From_InternalFlash(uint8_t data_out[],uint32_t start_addr,unsigned int len)
{
	 unsigned int i;
	 for(i = 0;i<len;i++ )
	 {
		 data_out[i] = (unsigned char)(Readflash(start_addr+4*i));
		 printf("%d DataGot is : ", data_out[i]);
	 }


	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //FLASH_Program_Word(Address, Data);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

  //uint8_t WData[] = "HelloSaved";
 // Write_Buff_To_InternalFlash("He", 0x81000000, 3/*strlen(WData)*/);

 // uint8_t RData;
 //Read_Buff_From_InternalFlash(RData, 0x81000000, 3);
  //printf("%d", RData);


  //NEW CODE FORM HERE

  	  	  	  //THIS WILL WRITE
  char write_data[10];
  memset(write_data,0,sizeof(write_data));
  strcpy(write_data, "HelloSaved");
  //write_to_flash((uint8_t*)write_data);


  //THIS WILL READ
  char read_data[10];
  memset(read_data,0,sizeof(read_data));
  read_from_flash((uint8_t*)read_data);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
