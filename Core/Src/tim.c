/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "dwt_common.h"
#include "iwdg.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* TIM1 init function */
void MX_TIM1_Init(void) {
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 64 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 100 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {
    if (tim_baseHandle->Instance == TIM1) {
        /* USER CODE BEGIN TIM1_MspInit 0 */

        /* USER CODE END TIM1_MspInit 0 */
        /* TIM1 clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

        /* TIM1 interrupt Init */
        HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
        HAL_NVIC_SetPriority(TIM1_CC_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
        /* USER CODE BEGIN TIM1_MspInit 1 */

        /* USER CODE END TIM1_MspInit 1 */
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle) {
    if (tim_baseHandle->Instance == TIM1) {
        /* USER CODE BEGIN TIM1_MspDeInit 0 */

        /* USER CODE END TIM1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();

        /* TIM1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
        HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
        /* USER CODE BEGIN TIM1_MspDeInit 1 */

        /* USER CODE END TIM1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
int Sendcontrltime = 0;  // 控制发送时间
int Anchpollingtime = 0; // Anchor等待polling的时间
int Anchrxtime = 0;      // Anchor距离上一次接收到数据的时间
uint8_t Tag_RecNt = 0;   // Tag接收到数据的标志: 1-未接收到，0-接收到
uint8_t Anch_RecNt = 0;  // Anchor接收到数据的标志: 1-未接收到，0-接收到
int Tagrxtime = 0;       // Tag距离上一次接收到数据的时间
uint16_t waittime = 0;   // 休眠时间，在时钟外部中断中减少，减少至0时休眠结束
int ledrunnum = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == (&htim1)) {
        if (sys_para.role == 0x01) {
            if (Anchpollingtime >= T_Round * 10) {
                Anchpollingtime = 0;
            }
            if (Anch_RecNt == 1) {
                Anchrxtime++;
            } else if (Anch_RecNt == 0) {
                Anchrxtime = 0;
            }
            if (Anchrxtime > T_Round * 100) {
                ins.previousState = ins.testAppState;
                // 超时，重新初始化
                ins.nextState = TA_INIT;
                Anchrxtime = 0;
            }
            // 减少休眠时间
            if (waittime == 1) {
                if (ins.nextState == TA_SLEEP) {
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_SLEEP_DONE;
                }
            }
            if (waittime > 0) waittime--;

            Anchpollingtime++;
        } else if (sys_para.role == 0x02) {
            if (Tag_RecNt == 1) {
                Tagrxtime++;

            } else if (Tag_RecNt == 0) {
                Tagrxtime = 0;
            }
            if (Tagrxtime >= 50000) {
                ins.previousState = ins.testAppState;
                // 超时，重新初始化
                ins.nextState = TA_INIT;
                Tagrxtime = 0;
            }

            if (waittime == 1) {
                if (ins.nextState == TA_SLEEP) {
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_SLEEP_DONE;
                }
            }
            if (waittime > 0) {
                waittime--;
            }
        }
        ledrunnum++;
        if (sys_para.role == 0x02) {
            if (ledrunnum >= 900 && ledrunnum <= 1000) {
                HAL_GPIO_WritePin(GPIOB, LED_RUN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
            } else if (ledrunnum > 1000) {
                ledrunnum = 0;
                HAL_GPIO_WritePin(GPIOB, LED_RUN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(GPIOB, LED_RUN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_RESET);
            }
        } else {
            if (ledrunnum >= 900 && ledrunnum <= 1000) {
                HAL_GPIO_WritePin(GPIOB, LED_RUN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
            } else if (ledrunnum > 1000) {
                ledrunnum = 0;
                HAL_GPIO_WritePin(GPIOB, LED_RUN_Pin, GPIO_PIN_RESET);

            } else
                HAL_GPIO_WritePin(GPIOB, LED_RUN_Pin, GPIO_PIN_RESET);
        }
    }
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
