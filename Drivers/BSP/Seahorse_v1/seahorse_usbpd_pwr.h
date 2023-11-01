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
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SEAHORSE_USBPD_PWR_H
#define SEAHORSE_USBPD_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "seahorse_conf.h"
#include "seahorse_errno.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32U5XX_NUCLEO
  * @{
  */

/** @addtogroup STM32U5XX_NUCLEO_USBPD_PWR
  * @{
  */

/** @defgroup STM32U5XX_NUCLEO_USBPD_PWR_Exported_Types Exported Types
  * @{
  */

/**
  * @brief  Power role
  */
typedef enum
{
  POWER_ROLE_SOURCE = 0,
  POWER_ROLE_SINK,
  POWER_ROLE_DUAL
}
USBPD_PWR_PowerRoleTypeDef;

/**
  * @brief  VBUS connection status
  */
typedef enum
{
  VBUS_CONNECTED = 0,
  VBUS_NOT_CONNECTED
} USBPD_PWR_VBUSConnectionStatusTypeDef;

/**
  * @brief VBUS Detection Callback
  */
typedef void USBPD_PWR_VBUSDetectCallbackFunc(uint32_t PortNum,
                                              USBPD_PWR_VBUSConnectionStatusTypeDef VBUSConnectionStatus);

/**
  * @}
  */

/** @defgroup STM32U5XX_NUCLEO_USBPD_PWR_Exported_Constants Exported Constants
  * @{
  */
/**
  * @brief  Number of TypeC ports
  */
#define USBPD_PWR_INSTANCES_NBR           (1U)

/**
  * @brief  Type-C port identifier
  */
#define USBPD_PWR_TYPE_C_PORT_1           (0U)

/**
  * @brief  CC pin identifier
  */
#define USBPD_PWR_TYPE_C_CC1              (1U)
#define USBPD_PWR_TYPE_C_CC2              (2U)

/**
  * @brief  VBUS disconnection threshold values (in mV)
  */
#define USBPD_PWR_HIGH_VBUS_THRESHOLD     (2800U)
#define USBPD_PWR_LOW_VBUS_THRESHOLD      (750U)
#define USBPD_PWR_VBUS_THRESHOLD_5V       (2150U)
#define USBPD_PWR_VBUS_THRESHOLD_9V       (7000U)
#define USBPD_PWR_VBUS_THRESHOLD_15V      (12500U)
#define USBPD_PWR_VBUS_THRESHOLD_20V      (17000U)
#define USBPD_PWR_VBUS_THRESHOLD_APDO     (2150U)

/**
  * @brief  VBUS discharge parameters
  */
#define USBPD_PWR_DISCHARGE_MARGIN        (500U)
#define USBPD_PWR_DISCHARGE_TIME          (6U)

/**
  * @brief  Standard VBUS voltage levels
  */
#define USBPD_PWR_VBUS_5V                 5000U
#define USBPD_PWR_VBUS_9V                 9000U
#define USBPD_PWR_VBUS_15V                15000U

/**
  * @brief  power timeout
  */
#define USBPD_PWR_TIMEOUT_PDO             250U         /* Timeout for PDO to PDO or PDO to APDO at 250ms */
#define USBPD_PWR_TIMEOUT_APDO            25U          /* Timeout for APDO to APDO at 25ms */

/**
  * @brief  Invalid value set during issue with voltage setting
  */
#define USBPD_PWR_INVALID_VALUE           0xFFFFFFFFU

/* On STM32U5xx-Nucleo Kit (MB1549), following setup has been used :
      VSENSE      => PC2 */ //koncz_change: ez PB0-on van nekünk
#define VSENSE_GPIO_PORT                  GPIOB
#define VSENSE_GPIO_PIN                   LL_GPIO_PIN_0
//#define VSENSE_GPIO_ENABLE_CLOCK()        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);


/* PC2 is used as ADC1_IN3 input for ADC measurement of VBUS voltage :
   ADC1 Channel 3.
*/        //koncz_change: ADC4_IN18, de ilyet nem játszunk
// #define VSENSE_ADC_INSTANCE               ADC4
// #define VSENSE_ADC_RANK                   ADC_REGULAR_RANK_1
// #define VSENSE_ADC_CHANNEL                ADC_CHANNEL_18

/* Enable ADC clock (core clock) */ //koncz_change: ADC4
//#define VSENSE_ADC_ENABLE_CLOCK()         LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC4);

/* Enable ADC clock (core clock) */  //koncz_change: ADC4
// #define VSENSE_ADC_DISABLE_CLOCK()        do {                                                        \
                                               // LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC4);     \
                                               // LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC4);   \
                                               // LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC4);   \
                                             // } while (0);

/* Clock enabling for TCPP01 DB signal : PB5 */ //koncz_change: PC5-ön van a Dead Battery
#define GPIO_TCPP01_DB_PORT               GPIOC
#define GPIO_TCPP01_DB_PIN                GPIO_PIN_5
//#define GPIO_TCPP01_DB_CLK_ENABLE()       LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

/**
  * @}
  */

/** @addtogroup STM32U5XX_NUCLEO_USBPD_PWR_Exported_Variables
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32U5XX_NUCLEO_USBPD_PWR_Exported_Functions STM32U5xx-Nucleo USBPD PWR Exported Functions
  * @{
  */
/* Common functions */
int32_t BSP_USBPD_PWR_Init(uint32_t PortNum); //<--------------------------

int32_t BSP_USBPD_PWR_Deinit(uint32_t PortNum);

int32_t BSP_USBPD_PWR_VBUSInit(uint32_t PortNum); //<----------------------3x

int32_t BSP_USBPD_PWR_VBUSDeInit(uint32_t PortNum); //<----------------------xx

int32_t BSP_USBPD_PWR_VBUSOn(uint32_t PortNum); //<----------------------

int32_t BSP_USBPD_PWR_VBUSOff(uint32_t PortNum); //<----------------------

int32_t BSP_USBPD_PWR_VBUSIsOn(uint32_t PortNum, uint8_t *pState); //<----------------------

int32_t BSP_USBPD_PWR_VBUSSetVoltage_Fixed(uint32_t PortNum,
                                           uint32_t VbusTargetInmv,
                                           uint32_t OperatingCurrent,
                                           uint32_t MaxOperatingCurrent);

int32_t BSP_USBPD_PWR_VBUSSetVoltage_Variable(uint32_t PortNum,
                                              uint32_t VbusTargetMaxInmv,
                                              uint32_t VbusTargetMinInmv,
                                              uint32_t OperatingCurrent,
                                              uint32_t MaxOperatingCurrent);

int32_t BSP_USBPD_PWR_VBUSSetVoltage_Battery(uint32_t PortNum,
                                             uint32_t VbusTargetMin,
                                             uint32_t VbusTargetMax,
                                             uint32_t OperatingPower,
                                             uint32_t MaxOperatingPower);

int32_t BSP_USBPD_PWR_VBUSSetVoltage_APDO(uint32_t PortNum,
                                          uint32_t VbusTargetInmv,
                                          uint32_t OperatingCurrent,
                                          int32_t Delta);

int32_t BSP_USBPD_PWR_SetVBUSDisconnectionThreshold(uint32_t PortNum,
                                                    uint32_t VoltageThreshold);

int32_t BSP_USBPD_PWR_RegisterVBUSDetectCallback(uint32_t PortNum,//<-------------------------------------------------------------------------
                                                 USBPD_PWR_VBUSDetectCallbackFunc *pfnVBUSDetectCallback);

int32_t BSP_USBPD_PWR_VBUSGetVoltage(uint32_t PortNum, uint32_t *pVoltage); //<-------------------------------------------------------------------------

int32_t BSP_USBPD_PWR_VBUSGetCurrent(uint32_t PortNum, int32_t *pCurrent);//<-------------------------------------------------------------------------

int32_t BSP_USBPD_PWR_VBUSDischargeOn(uint32_t PortNum);

int32_t BSP_USBPD_PWR_VBUSDischargeOff(uint32_t PortNum);

int32_t BSP_USBPD_PWR_VCONNInit(uint32_t PortNum,//<-------------------------------------------------------------------------
                                uint32_t CCPinId);
int32_t BSP_USBPD_PWR_VCONNDeInit(uint32_t PortNum,//<-------------------------------------------------------------------------3x
                                  uint32_t CCPinId);
int32_t BSP_USBPD_PWR_VCONNOn(uint32_t PortNum,//<-------------------------------------------------------------------------
                              uint32_t CCPinId);
int32_t BSP_USBPD_PWR_VCONNOff(uint32_t PortNum,//<-------------------------------------------------------------------------
                               uint32_t CCPinId);

int32_t BSP_USBPD_PWR_VCONNIsOn(uint32_t PortNum,
                                uint32_t CCPinId, uint8_t *pState);

int32_t BSP_USBPD_PWR_VCONNDischargeOn(uint32_t PortNum);

int32_t BSP_USBPD_PWR_VCONNDischargeOff(uint32_t PortNum);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32U5XX_NUCLEO_USBPD_PWR_H */
