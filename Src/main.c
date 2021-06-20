/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
/* USER CODE BEGIN Includes */
//#include "lcd_hd44780_stm32f0.h"

#define LCD_PORT_A
#define LCD_RS 0
#define LCD_E 1
#define LCD_RW 6
#define LCD_DB4 2
#define LCD_DB5 3
#define LCD_DB6 4
#define LCD_DB7 5
#include "LCDstm32.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t cnt = 0;
volatile char externalInterruptFlag = 0;
volatile char externalInterruptSettingFlag = 0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance==TIM1)
      {

		cnt++;
		HAL_TIM_Base_Start_IT(&htim1);
      }
}

/* USER CODE END 0 */

#define SAMPLE_NUM	10
#define DELAY_FOR_EMPTY_CAP		1000
volatile uint32_t finalsample=0;
static float C0=1;
static float C=0;
static float Er=0;
volatile uint32_t T0=550;
static float distanceValue = 6;

uint32_t sampleing()
{
	uint32_t result=1;
	GPIOB->BSRR = (1<<1);
    HAL_Delay(DELAY_FOR_EMPTY_CAP);
    HAL_TIM_Base_Stop_IT(&htim1);
	GPIOB->BRR = (1<<1);
	__HAL_TIM_SET_COUNTER(&htim1 , 0);
	HAL_TIM_Base_Start_IT(&htim1);
	while(!externalInterruptFlag);
	HAL_TIM_Base_Stop_IT(&htim1);
	externalInterruptFlag=0;
	result = __HAL_TIM_GetCounter(&htim1)+ cnt*65536;
	cnt = 0;
	return result;
}


int main(void)
{
  uint32_t Dint=0;
  uint32_t Dfloat=0;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  GPIOA->MODER = 0x00000000;
  GPIOA->PUPDR = 0x00000000;
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
	lcd_init();
	lcd_clear();
	lcd_gotoxy(1,1);
//////	sprintf(string,"AshkAra Co.");
	lcd_clear();
	lcd_putsf("AshkAra Co.");

//	LCDInit(CURSOR_NONE);

//	LCDWriteStringXY(0,0,"AshkAra !");
/* USER CODE END 2 */
HAL_Delay(1000);

/* USER CODE BEGIN 2 */


/* Infinite loop */
/* USER CODE BEGIN WHILE */
uint32_t value;
char string[16];
volatile int16_t inpPIN = 0;

HAL_TIM_Base_Start_IT(&htim1);
HAL_TIM_Base_Stop_IT(&htim1);
cnt=0;
__HAL_TIM_SET_COUNTER(&htim1 , 0);



while (1)
{
/* USER CODE END WHILE */

	  inpPIN = (GPIOA->IDR & (1<<10));
	  if(inpPIN)
	  {

/*		  GPIOB->BSRR = (1<<1);
		  lcd_clear();
		  lcd_putsf("Please wait...");
		  HAL_Delay(DELAY_FOR_EMPTY_CAP);
		  GPIOB->BRR = (1<<1);
		  __HAL_TIM_SET_COUNTER(&htim1 , 0);
		  HAL_TIM_Base_Start_IT(&htim1);
		  while(!externalInterruptFlag);
		  HAL_TIM_Base_Stop_IT(&htim1);
		  value = __HAL_TIM_GetCounter(&htim1)+ cnt* 65536;
*/
		  lcd_clear();
		  lcd_putsf("Please wait...");
		  value =  sampleing();

/*		  lcd_clear();
		  itoa(value,string,10);
		  lcd_gotoxy(1,1);
		  lcd_putsf("t=");
		  lcd_gotoxy(1,3);
		  lcd_putsf(string);*/
		  C = (float)(abs(value - T0) / 5.75);
		  if(C0 != 0)
			  Er = (float)(( C * distanceValue) / C0);

		  lcd_clear();

		  lcd_gotoxy(1,1);
		  Dint = (uint32_t)C;
		  Dfloat = (uint32_t)((C - Dint) * 100  );
		  lcd_putsf("C=");
		  itoa(Dint,string,10);
		  lcd_putsf(string);
		  lcd_putsf(".");
		  if(Dfloat< 10)
	        	  lcd_putsf("0");
		  itoa(Dfloat,string,10);
    	  lcd_putsf(string);

    	  lcd_gotoxy(2,1);
    	  Dint = (uint32_t)Er ;
    	  Dfloat = (uint32_t)((Er - Dint) * 100  );
    	  lcd_putsf("Er=");
    	  itoa(Dint,string,10);
    	  lcd_putsf(string);
    	  lcd_putsf(".");
    	  if(Dfloat< 10)
    	    	  lcd_putsf("0");
    	  itoa(Dfloat,string,10);
    	  lcd_putsf(string);


		  cnt = 0;
		  externalInterruptFlag = 0;

	  }
	  if(externalInterruptSettingFlag)
	  {
		  //HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
		  HAL_Delay(300);
		  lcd_clear();
		  lcd_gotoxy(1,1);
		  lcd_putsf("Distance Setting");
		  lcd_gotoxy(2,1);
		  Dint = (uint32_t)distanceValue ;
		  Dfloat = (uint32_t)((distanceValue * 100) - (Dint * 100) );
		  lcd_gotoxy(2,1);
		  lcd_putsf("Distance=");
		  itoa(Dint,string,10);
		  lcd_putsf(string);
		  lcd_putsf(".");
		  itoa(Dfloat,string,10);
		  lcd_putsf(string);
		  externalInterruptSettingFlag=0;
		  while(!externalInterruptSettingFlag)
		  {
			  HAL_Delay(300);
			  inpPIN = (GPIOA->IDR & (1<<13));
			  if(inpPIN)  // incress
			  {
				  HAL_Delay(300);
				  distanceValue += 0.05;
				  lcd_gotoxy(2,10);
				  lcd_putsf("      ");
				  lcd_gotoxy(2,10);
				  Dint = (uint32_t)distanceValue ;
				  Dfloat = (uint32_t)((distanceValue - Dint) * 100  );
				  itoa(Dint,string,10);
				  lcd_putsf(string);
				  lcd_putsf(".");
				  if(Dfloat< 10)
					  lcd_putsf("0");
				  itoa(Dfloat,string,10);
				  lcd_putsf(string);
			  }
			  inpPIN = (GPIOA->IDR & (1<<6));
			  if(inpPIN) // decress
			  {
				  HAL_Delay(300);
				  distanceValue -= 0.05;
				  lcd_gotoxy(2,10);
				  lcd_putsf("      ");
				  lcd_gotoxy(2,10);
				  Dint = (uint32_t)distanceValue ;
				  Dfloat = (uint32_t)((distanceValue - Dint )* 100 );
				  itoa(Dint,string,10);
				  lcd_putsf(string);
				  lcd_putsf(".");
				  if(Dfloat< 10)
					  lcd_putsf("0");
				  itoa(Dfloat,string,10);
				  lcd_putsf(string);

			  }

		  }
		  HAL_Delay(300);
		  externalInterruptSettingFlag=0;


		  lcd_clear();
		  lcd_gotoxy(1,1);
		  lcd_putsf("Calibration");
		  lcd_gotoxy(2,1);
		  lcd_putsf("Free Reference");

		  while(!externalInterruptSettingFlag)
		  {
			  HAL_Delay(300);
			  inpPIN = (GPIOA->IDR & (1<<10));
			  if(inpPIN)
			  {
				  lcd_gotoxy(2,1);
				  lcd_putsf("                ");
				  lcd_gotoxy(2,1);
				  lcd_putsf("Please Wait...");


				  T0 = sampleing();
				  lcd_gotoxy(2,1);
				  lcd_putsf("                ");
  				  itoa(T0,string,10);
  				  lcd_gotoxy(2,1);
  				  lcd_putsf("T0=");
  				  lcd_putsf(string);
			  }
		  }
		  HAL_Delay(300);
		  externalInterruptSettingFlag=0;


		  lcd_clear();
		  lcd_gotoxy(1,1);
		  lcd_putsf("Calibration");
		  lcd_gotoxy(2,1);
		  lcd_putsf("Mate. Reference");

		  while(!externalInterruptSettingFlag)
		  {
			  HAL_Delay(300);
			  inpPIN = (GPIOA->IDR & (1<<10));
			  if(inpPIN)
			  {
				  lcd_gotoxy(2,1);
				  lcd_putsf("                ");
				  lcd_gotoxy(2,1);
				  lcd_putsf("Please Wait...");


				  finalsample = sampleing();
				  lcd_gotoxy(2,1);
				  lcd_putsf("                ");
				  C0 = (float)(abs(finalsample - T0) / 5.75);
				  Dint = (uint32_t)C0 ;
  				  Dfloat = (uint32_t)((C0 - Dint) * 100  );
  			      lcd_gotoxy(2,1);
  				  lcd_putsf("C0=");
  				  itoa(Dint,string,10);
  				  lcd_putsf(string);
  				  lcd_putsf(".");
  				  if(Dfloat< 10)
  					  lcd_putsf("0");
  				  itoa(Dfloat,string,10);
  				  lcd_putsf(string);
			  }
		  }



		  HAL_Delay(300);
		  externalInterruptSettingFlag=0;
		  lcd_clear();

	    }
}
		  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}


//		LCDWriteStringXY(0,0,"AshkAra3");
/* USER CODE BEGIN 3 */

  /* USER CODE END 3 */



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{
  GPIOA->MODER = 0x00000000;
  GPIOA->PUPDR = 0x00000000;


  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);


}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
