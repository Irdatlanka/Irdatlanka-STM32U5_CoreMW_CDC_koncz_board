/**
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
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "seahorse.h"
#include "main.h"


ADC_HandleTypeDef hadc1;


static GPIO_TypeDef *LED_PORT[LEDn] = {LED1_GPIO_PORT,
                                       LED2_GPIO_PORT,
                                       LED3_GPIO_PORT,
                                       LED4_GPIO_PORT,
                                       LED5_GPIO_PORT
                                      };

static const uint16_t LED_PIN[LEDn] = {LED1_PIN,
                                       LED2_PIN,
                                       LED3_PIN,
                                       LED4_PIN,
                                       LED5_PIN
                                      };




/**
  * @brief  This method returns the board name
  * @retval pointer to the board name string
  */
const uint8_t *BSP_GetBoardName(void)
{
  return (uint8_t *)BSP_BOARD_NAME;
}

/**
  * @brief  Turns selected LED On.
  * @param  Led Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  *     @arg  LED4
  *     @arg  LED5
  * @retval BSP status
  */
int32_t BSP_LED_On(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;

  if ((Led != LED1) && (Led != LED2) && (Led != LED3) && (Led != LED4) && (Led != LED5))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
  }

  return ret;
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  *     @arg  LED4
  *     @arg  LED5
  * @retval BSP status
  */
int32_t BSP_LED_Off(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;

  if ((Led != LED1) && (Led != LED2) && (Led != LED3) && (Led != LED4) && (Led != LED5))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  }

  return ret;
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  *     @arg  LED4
  *     @arg  LED5
  * @retval BSP status
  */
int32_t BSP_LED_Toggle(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;

  if ((Led != LED1) && (Led != LED2) && (Led != LED3) && (Led != LED4) && (Led != LED5))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
  }

  return ret;
}

/**
  * @brief  Get the state of the selected LED.
  * @param  Led LED to get its state
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  *     @arg  LED4
  *     @arg  LED5
  * @retval LED status
  */
int32_t BSP_LED_GetState(Led_TypeDef Led)
{
  int32_t ret;

  if ((Led != LED1) && (Led != LED2) && (Led != LED3) && (Led != LED4) && (Led != LED5))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    ret = (int32_t)HAL_GPIO_ReadPin(LED_PORT [Led], LED_PIN [Led]);
  }

  return ret;
}

/**
  * @brief  Blinking all LEDs for testing.
  * @retval BSP status
  */
int32_t BSP_LedTest(void) 
{
	int32_t ret = BSP_ERROR_NONE;
	
	ret = BSP_LED_On(LED1);
	if (ret != BSP_ERROR_NONE)
		return ret;
	HAL_Delay(500);
	ret = BSP_LED_On(LED2);
	if (ret != BSP_ERROR_NONE)
		return ret;
	HAL_Delay(500);
	ret = BSP_LED_On(LED3);
	if (ret != BSP_ERROR_NONE)
		return ret;
	HAL_Delay(500);
	ret = BSP_LED_On(LED4);
	if (ret != BSP_ERROR_NONE)
		return ret;
	HAL_Delay(500);
	ret = BSP_LED_On(LED5);
	if (ret != BSP_ERROR_NONE)
		return ret;
	
	HAL_Delay(500);
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);
	BSP_LED_Off(LED3);
	BSP_LED_Off(LED4);
	BSP_LED_Off(LED5);
	return ret;
}

/**
  * @brief  This method initializes ADC1 on Seahorse board.
  * @retval BSP status
  */
int32_t BSP_Init_ADC1(void)
{
	int32_t ret = BSP_ERROR_NONE;
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  return ret;
}

/**
  * @brief  This method selects CH15 on ADC1
  * @retval BSP status
  */
int32_t BSP_ADC1_Select_CH15(void)
{
		int32_t ret = BSP_ERROR_NONE;
	    ADC_ChannelConfTypeDef sConfig = {0};
	    /** Configure Regular Channel
	    */
	    sConfig.Channel = ADC_CHANNEL_15;
	    sConfig.Rank = ADC_REGULAR_RANK_1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_5CYCLE;
	    sConfig.SingleDiff = ADC_SINGLE_ENDED;
	    sConfig.OffsetNumber = ADC_OFFSET_NONE;
	    sConfig.Offset = 0;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	      Error_Handler();
	    }
		return ret;
}
/**
  * @brief  This method selects CH16 on ADC1
  * @retval BSP status
  */
int32_t BSP_ADC1_Select_CH16(void)
{
		int32_t ret = BSP_ERROR_NONE;
	    ADC_ChannelConfTypeDef sConfig = {0};
	    /** Configure Regular Channel
	    */
	    sConfig.Channel = ADC_CHANNEL_16;
	    sConfig.Rank = ADC_REGULAR_RANK_1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_5CYCLE;
	    sConfig.SingleDiff = ADC_SINGLE_ENDED;
	    sConfig.OffsetNumber = ADC_OFFSET_NONE;
	    sConfig.Offset = 0;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	      Error_Handler();
	    }
		return ret;
}
/**
  * @brief  This method selects CH17 on ADC1
  * @retval BSP status
  */
int32_t BSP_ADC1_Select_CH17(void)
{
		int32_t ret = BSP_ERROR_NONE;
	    ADC_ChannelConfTypeDef sConfig = {0};
	    /** Configure Regular Channel
	    */
	    sConfig.Channel = ADC_CHANNEL_17;
	    sConfig.Rank = ADC_REGULAR_RANK_1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_5CYCLE;
	    sConfig.SingleDiff = ADC_SINGLE_ENDED;
	    sConfig.OffsetNumber = ADC_OFFSET_NONE;
	    sConfig.Offset = 0;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	      Error_Handler();
	    }
		return ret;
}

/**
  * @brief  Measure VBUS voltage.
  * @retval VBUS voltage in mV
  */
int32_t BSP_MeasureVBUS(void)
{
  uint32_t adc_raw;
  BSP_ADC1_Select_CH15();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adc_raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  
  float adc_vlt = ((float)adc_raw / (float)BSP_ADC1_RESOLUTION) * (float)BSP_VREF;
  float vbus =  (adc_vlt / (float)BSP_VBUS_DIVIDER) ; 
  int32_t vbus_s = (int32_t) vbus;
  return vbus_s;
}
