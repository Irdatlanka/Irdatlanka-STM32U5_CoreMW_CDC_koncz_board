/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_IOToggle/Src/stm32u5xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"



/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* External functions ------------------------------------------------------- */



/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{


  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 7, 0);


}



/**
  * @}
  */

/**
  * @}
  */

