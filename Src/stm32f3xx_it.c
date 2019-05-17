/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */

	extern int mode;
	extern int c;
	extern int blinkCounter;
	extern RTC_TimeTypeDef t;
	extern RTC_DateTypeDef d;
	extern unsigned char dd;
	extern unsigned char buffer[100];
	extern int pos;
	extern char temp[100];
	

	//TimeAndCalenderMethods
	extern char* zeropadd(int time);
	extern char* toWeekday(int weekday);
	extern char* toMonth(int month);
	extern void showTimeCalender(RTC_TimeTypeDef t,RTC_DateTypeDef d, int counter);
	extern void increaseTime();
	
	/* Sorush and GOD just know it! */
	extern int ClearScreen();
	
	/*TempMethods*/
	extern void showTemp();

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	for(int i = 0; i < 4; i++){
		if(i == 0){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,1);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,0);		
		}
		else if(i == 1){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,1);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,0);
		}
		else if(i == 2){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,1);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,0);	
		}
		else if(i == 3){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,1);
		}
		
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) && i == 0){
			if(mode == 3){ 
				mode = 0;
				blinkCounter = 0; 
			}
			else if (mode == 0){
				mode = 3;
				blinkCounter = 2;
			}
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
			HAL_Delay(20);
		}
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) && i == 1){
			if(mode == 3){
				blinkCounter = blinkCounter + 3 > 20 ? 2 :blinkCounter + 3;
			}
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
			HAL_Delay(20);
		}
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) && i == 2){
			if(mode == 3){
				increaseTime();
			}
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
			HAL_Delay(20);
		}
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) && i == 3){
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
			HAL_Delay(20);
		}
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,1);
		
	}

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 and Touch Sense controller.
*/
void EXTI2_TSC_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_TSC_IRQn 0 */

  /* USER CODE END EXTI2_TSC_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_TSC_IRQn 1 */

  /* USER CODE END EXTI2_TSC_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

	
//	int x = HAL_ADC_GetValue(&hadc1);
//	sprintf(temp,"%d",x);
//	
//	HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
		if(c){
			c = ClearScreen();
		}
		
		switch (mode){
			case 0:
				showTimeCalender(t,d,blinkCounter);
				break;
			case 1:
				showTemp();
				break;
//			case 2:
//				motionDtc();
//				break;
			case 3:
				showTimeCalender(t,d,blinkCounter);
				break;
			case 10:
			break;
			
		}

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
//		if (dd != 0x0D){
//			buffer[pos] = dd;
//			pos++;
//			buffer[pos] = '\0';
//		}
//		else{
//			pos = 0; 
//			if(buffer[0] == '*' && buffer[1] == '*' && buffer[2] == '*' && buffer[3] == '*' 
//				&& buffer[4] == '*' && buffer[5] == '\0'){
//					if(mode == 0){
//						mode = 1;
//						*(buffer + 1) = ' ';
//					}
//					else
//						mode = 0;
//					c = 1;
//			}
//		}

//		HAL_UART_Receive_IT(&huart2, &dd, sizeof(unsigned char));
  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles ADC3 global interrupt.
*/
void ADC3_IRQHandler(void)
{
  /* USER CODE BEGIN ADC3_IRQn 0 */

  /* USER CODE END ADC3_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC3_IRQn 1 */
	char a[100] = "Motion detected";
	HAL_UART_Transmit(&huart2,a,sizeof(char) * 100 , 1000);

	HAL_ADC_Start_IT(&hadc3);
  /* USER CODE END ADC3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
