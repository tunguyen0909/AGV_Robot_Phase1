/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#include <string.h>
#include <stdio.h>
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim9;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
int triTuyeDoi(int a)
{
	if(a < 0)
		a = -a;
	else
		a = a;
	return a;
}

int chuyen(char c)
{
		return (int)c-48;
}

void PID_control_DC1(double speed_des1_rps)
{
	Speed_des1_rps = triTuyeDoi((speed_des1_rps*330*0.025));
	rSpeed1 = pluse1 - pluse_pre1;
	pluse_pre1 = pluse1;
	Err1 = Speed_des1_rps - triTuyeDoi(rSpeed1);

	pPart1 = (float)(Kp1*(Err1));
	dPart1 = (float)(Kd1*(Err1 - pre_Err1)*inv_sampling_Time);
	iPart1 += (float)(Ki1*sampling_Time*Err1);
	Output1 += (int)(pPart1 + dPart1 + iPart1);

	if(Output1 > 4000)
		Output1 = 4000-1;
	if(Output1 <= 0)
		Output1 = 1;

	PWM1 = Output1;
	pre_Err1 = Err1;
	w1  = (rSpeed1)/(0.025*330);
}

void PID_control_DC2(double speed_des2_rps)
{
	Speed_des2_rps = triTuyeDoi((speed_des2_rps*330*0.025));
	rSpeed2 = pluse2 - pluse_pre2;
	pluse_pre2 = pluse2;
	Err2 = Speed_des2_rps - triTuyeDoi(rSpeed2);

	pPart2 = (float)(Kp2*(Err2));
	dPart2 = (float)(Kd2*(Err2 - pre_Err2)*inv_sampling_Time);
	iPart2 += (float)(Ki2*sampling_Time*Err2);
	Output2 += (int)(pPart2 + dPart2 + iPart2);

	if(Output2 > 4000)
		Output2 = 4000-1;
	if(Output2 <= 0)
		Output2 = 1;

	PWM2 = Output2;
	pre_Err2 = Err2;

	w2  = (rSpeed2)/(0.025*330);
}

void PID_control_DC3(double speed_des3_rps)
{
	Speed_des3_rps = triTuyeDoi((speed_des3_rps*330*0.025));
	rSpeed3 = pluse3 - pluse_pre3;
	pluse_pre3 = pluse3;
	Err3 = Speed_des3_rps - triTuyeDoi(rSpeed3);

	pPart3 = (float)(Kp3*(Err3));
	dPart3 = (float)(Kd3*(Err3 - pre_Err3)*inv_sampling_Time);
	iPart3 += (float)(Ki3*sampling_Time*Err3);
	Output3 += (int)(pPart3 + dPart3 + iPart3);

	if(Output3 > 4000)
		Output3 = 4000-1;
	if(Output3 <= 0)
		Output3 = 1;

	PWM3 = Output3;
	pre_Err3 = Err3;

	w3  = (rSpeed3)/(0.025*330);
}

void PID_control_DC4(double speed_des4_rps)
{
	Speed_des4_rps = triTuyeDoi((speed_des4_rps*330*0.025));
	rSpeed4 = pluse4 - pluse_pre4;
	pluse_pre4 = pluse4;
	Err4 = Speed_des4_rps - triTuyeDoi(rSpeed4);

	pPart4 = (float)(Kp4*(Err4));
	dPart4 = (float)(Kd4*(Err4 - pre_Err4)*inv_sampling_Time);
	iPart4 += (float)(Ki4*sampling_Time*Err4);
	Output4 += (int)(pPart4 + dPart4 + iPart4);

	if(Output4 > 4000)
		Output4 = 4000-1;
	if(Output4 <= 0)
		Output4 = 1;

	PWM4 = Output4;
	pre_Err4 = Err4;

	w4  = (rSpeed4)/(0.025*330);
}

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
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
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1) == 0)
	{
		pluse4--;
	}
	else if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1) == 1)
	{
		pluse4++;
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
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0) == 0)
	{
		pluse1++;
	}
	else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0) == 1)
	{
		pluse1--;
	}
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == 0)
	{
		pluse3++;
	}
	else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == 1)
	{
		pluse3--;
	}
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == 0)
  {
	  pluse2--;
  }
  else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == 1)
  {
	  pluse2++;
  }
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
	PID_control_DC1(W1);
	PID_control_DC2(W2);
	PID_control_DC3(W3);
	PID_control_DC4(W4);
	count++;
  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
