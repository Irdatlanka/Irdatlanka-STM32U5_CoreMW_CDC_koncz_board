/**
  ******************************************************************************
  *
  *
  *
  *
  *
  ******************************************************************************
  *
  *
  *
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SEAHORSE_H
#define SEAHORSE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "seahorse_conf.h"
#include "seahorse_errno.h"


#if !defined (USE_SEAHORSE) //koncz_change
#define USE_SEAHORSE
#endif /* !defined (USE_SEAHORSE) */


// LEDs in order from right to left
typedef enum
{
  LED1 = 0,
  led_iolink1_g = LED1,
  LED2 = 1,
  led_iolink1_r = LED2,
  LED3 = 2,
  led_pwr_ok = LED3,
  LED4 = 3,
  led_pwr_fault = LED4,
  LED5 = 4,
  led_usb = LED5,
  LEDn//fontos, mert ez tarolja a LEDek szamat!
} Led_TypeDef;


#define BSP_BOARD_NAME "Seahorse_v1"


#define LED1_PIN                                GPIO_PIN_0
#define LED1_GPIO_PORT                          GPIOC
// #define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()
// #define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOC_CLK_DISABLE()

#define LED2_PIN                                GPIO_PIN_1
#define LED2_GPIO_PORT                          GPIOC
// #define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
// #define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED3_PIN                                GPIO_PIN_2
#define LED3_GPIO_PORT                          GPIOC
// #define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOG_CLK_ENABLE()
// #define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOG_CLK_DISABLE()

#define LED4_PIN                                GPIO_PIN_3
#define LED4_GPIO_PORT                          GPIOC

#define LED5_PIN                                GPIO_PIN_3
#define LED5_GPIO_PORT                          GPIOA

//ADC and measurement related stuff:
#define BSP_ADC1_RESOLUTION 4096 // 1 << 12
#define BSP_VREF 3300
#define BSP_VBUS_DIVIDER 0.3125f //10/(10+22)


//int32_t  BSP_GetVersion(void);
const uint8_t *BSP_GetBoardName(void);
//const uint8_t *BSP_GetBoardID(void);
// int32_t  BSP_LED_Init(Led_TypeDef Led); //koncz_change koncz_note
// int32_t  BSP_LED_DeInit(Led_TypeDef Led);
int32_t  BSP_LED_On(Led_TypeDef Led);
int32_t  BSP_LED_Off(Led_TypeDef Led);
int32_t  BSP_LED_Toggle(Led_TypeDef Led);
int32_t  BSP_LED_GetState(Led_TypeDef Led);
int32_t BSP_LedTest(void);
// int32_t  BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
// int32_t  BSP_PB_DeInit(Button_TypeDef Button);
// int32_t  BSP_PB_GetState(Button_TypeDef Button);
// void     BSP_PB_IRQHandler(Button_TypeDef Button);
// void     BSP_PB_Callback(Button_TypeDef Button);

// #if (USE_BSP_COM_FEATURE > 0)
// int32_t  BSP_COM_Init(COM_TypeDef COM, COM_InitTypeDef *COM_Init);
// int32_t  BSP_COM_DeInit(COM_TypeDef COM);
// #if (USE_COM_LOG > 0)
// int32_t  BSP_COM_SelectLogPort(COM_TypeDef COM);
// #endif /* USE_COM_LOG */

// #if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
// int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM);
// int32_t BSP_COM_RegisterMspCallbacks(COM_TypeDef COM, BSP_COM_Cb_t *Callback);
// #endif /* USE_HAL_UART_REGISTER_CALLBACKS */
// HAL_StatusTypeDef MX_USART1_Init(UART_HandleTypeDef *huart, MX_UART_InitTypeDef *COM_Init);
// #endif /* USE_BSP_COM_FEATURE */


int32_t BSP_Init_ADC1(void);
int32_t BSP_ADC1_Select_CH15(void);
int32_t BSP_ADC1_Select_CH16(void);
int32_t BSP_ADC1_Select_CH17(void);
int32_t BSP_MeasureVBUS(void);

#ifdef __cplusplus
}
#endif

#endif /* SEAHORSE_H */
