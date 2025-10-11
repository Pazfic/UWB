/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_RSTn_GPIO_Port, DW_RSTn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DW_WUP_Pin|LED_RUN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = DW_RSTn_Pin|DW_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DW_WUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DW_WUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DW_EXTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DW_EXTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DW_IRQN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DW_IRQN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_RUN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn led_off()
 *
 * @brief 该函数用于熄灭用户指定的LED灯。
 *
 * input parameters
 * @param led  - 选择LED灯。
 * @param enable- 0：LED驱动不使能，1：LED驱动使能。
 *
 * output parameters none
 *
 * no return value
 */
//void led_off (led_t led,uint16_t enable)
//{
//		if(enable == 1)
//		{
//			switch (led)
//			{

////			case LED_LANCONNECT:
////				LED_LANCONNECT_OFF();
////				break;
////			case LED_LANDATA:
////				LED_LANDATA_OFF();
////				break;
////			case LED_SYSRUN:
////				LED_SYSRUN_OFF();
////				break;
//			case LED_UWBDATA:
//				LED_UWBDATA_OFF();
//				break;
//			case LED_PC9:
//				LED_PC9_OFF();
//				break;
//			case LED_ALL:

//				LED_ALL_OFF();
//				break;
//			default:
//				// do nothing for undefined led number
//				break;
//			}
//		}
//}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn led_on()
 *
 * @brief 该函数用于点亮用户指定的LED灯。
 *
 * input parameters
 * @param led  - 选择LED灯。
 * @param enable- 0：LED驱动不使能，1：LED驱动使能。
 *
 * output parameters none
 *
 * no return value
 */
//void led_on (led_t led,uint16_t enable)
//{
//	if(enable == 1)
//	{
//		switch (led)
//		{
////			case LED_LANCONNECT:
////				LED_LANCONNECT_ON();
////				break;
////			case LED_LANDATA:
////				LED_LANDATA_ON();
////				break;
////			case LED_SYSRUN:
////				LED_SYSRUN_ON();
////				break;
//			case LED_UWBDATA:
//				LED_UWBDATA_ON();
//				break;
//			case LED_PC9:
//				LED_PC9_ON();
//				break;
//			case LED_ALL:

//				LED_ALL_ON();
//				break;
//			default:
//				// do nothing for undefined led number
//				break;
//		}
//	}
//	else if(enable == 0)
//		
//		LED_ALL_OFF();
//}
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
