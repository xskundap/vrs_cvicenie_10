/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "float.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void proccesDmaData(const uint8_t* sign, uint16_t len);
void print();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t count = 0;
int male = 0, velke = 0;
int start = 0;
int velkost = 256;
letter_count_ letters;

uint8_t string[20];

uint8_t tx_data[] = "Data to send over UART DMA!\n\r";
uint8_t rx_data[10];

uint8_t buffer[4];
uint8_t memory[4];
uint8_t load[4];
uint8_t male_pismena[4];
uint8_t velke_pismena[4];

uint16_t occupiedMem;

int dollar = 0;

char state_manual[6] = "manual";
char state_auto[4] = "auto";
int state = 0;						//Ziadny stav 0, Auto = 1, Man = 2


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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

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
  USART2_RegisterCallback(proccesDmaData);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  print();
	  LL_mDelay(5000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void print(){
	uint8_t s1[] = "Buffer capacity: ";
	uint8_t s2[] = "bytes, occupied memory: ";
	uint8_t s3[] = "bytes, load [in %]: ";
	uint8_t s4[] = "%\n\r";
	uint8_t s5[] = "Pocet malych pismen: ";
	uint8_t s6[] = "	Pocet velkych pismen: ";
	uint8_t s7[] = "\n\r";
	uint16_t buffer_size =  DMA_USART2_BUFFER_SIZE;
	float pom_occupiedMem = occupiedMem;
	float pom_buffer_size = buffer_size;
	float percento = (pom_occupiedMem/pom_buffer_size)*100.0;

	sprintf(buffer, "%d", buffer_size);
	strcat(s1, buffer);

	USART2_PutBuffer(s1, strlen(s1));
	LL_mDelay(50);

	sprintf(memory, "%d", occupiedMem);
	strcat(s2, memory);

	USART2_PutBuffer(s2, strlen(s2));
	LL_mDelay(50);

	gcvt(percento, 6, load);
	strcat(s3, load);

	USART2_PutBuffer(s3, strlen(s3));
	LL_mDelay(50);

	USART2_PutBuffer(s4, strlen(s4));
	LL_mDelay(50);

	sprintf(male_pismena, "%d", letters.small_letter);
	strcat(s5, male_pismena);

	USART2_PutBuffer(s5, strlen(s5));
	LL_mDelay(50);

	sprintf(velke_pismena, "%d", letters.capital_letter);
	strcat(s6, velke_pismena);

	USART2_PutBuffer(s6, strlen(s6));
	LL_mDelay(50);

	USART2_PutBuffer(s7, strlen(s7));
	LL_mDelay(50);
	USART2_PutBuffer(s7, strlen(s7));
	LL_mDelay(50);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
}

/* USER CODE BEGIN 4 */
void proccesDmaData(const uint8_t* sign, uint16_t len)
{
	/* Process received data */

	uint8_t pom = 0;
	int count;
	int count_1 = 0;
		// type your algorithm here:

	while(pom < len){
	uint8_t akt = *(sign+pom);
		if(*(sign+pom) == '$'){
			++dollar;
			start = 1;
			male = 0;
			count = 0;
			velke = 0;

			if(dollar >= 2){
				start = 0;

				letters.capital_letter = letters.capital_letter + velke;
				letters.small_letter = letters.small_letter + male;

				dollar == 0;
			}
		}

		if(start == 1){

			/*
			if(*(sign+pom) >= 'a' && *(sign+pom) <= 'z'){
				male = male+1;
			}
			if(*(sign+pom) >= 'A' && *(sign+pom) <= 'Z'){
				velke = velke + 1;
			}
			*/
			if(*(sign+pom) == state_manual[count_1]){
				count_1++;
				if(count_1 == 6){		//manual
					state = 2;

					count_1 = 0;
				}
			}

			else if((*(sign+pom) != state_manual[count_1]) && (*(sign+pom) == state_auto[count_1])){
				count_1++;
				if(count_1 == 4){		//auto
					state = 1;

					count_1 = 0;
				}
			}

			count++;
		}

		if(dollar >= 2){
			if(state == 1){
				// Pustanie LED auto
				state = 0;
			}
			else if(state == 2){
				//Pustanie LED manual
				state = 0;
			}
		}
/*
		if(*(sign+pom) == '$'){
			start = 0;


			letters.capital_letter = letters.capital_letter + velke;
			letters.small_letter = letters.small_letter + male;

		}
*/
		if(count == 35 && *(sign+pom) != '$'){
			start = 0;
			male = 0;
			count = 0;
			velke = 0;
		}
		pom++;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
