/**
  ******************************************************************************
  * @file   bf0_hal_lcpu_config.h
  * @author Sifli software development team
  * @brief Header file of configure parameters to LCPU
  * @{
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef __BF0_HAL_LCPU_CONFIG_H
#define __BF0_HAL_LCPU_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */


/** @defgroup LPCONFIG LCPU configure
  * @brief LCOU configure to share parameter with HCPU
  * @{
  */


/**
  * @addtogroup  LPCONFIG_exported_constants
  * @{
*/

#define HAL_LCPU_CONFIG_SN_MAX_NUM 128

/**
  * @brief  ENUM definition of configration type.
*/
#ifndef SF32LB52X
typedef enum
{
    HAL_LCPU_CONFIG_ADC_CALIBRATION,      /*!< ADC calibration value. */
    HAL_LCPU_CONFIG_SDADC_CALIBRATION,    /*!< SDADC calibration value. */
    HAL_LCPU_CONFIG_SN,                   /*!< mcu serial number. */
    HAL_LCPU_CONFIG_CHIP_REV,             /*!< Chip revision. */
    HAL_LCPU_CONFIG_BATTERY_CALIBRATION,  /*!< Battery calibration value. */
    HAL_LCPU_CONFIG_MAX,
} HAL_LCPU_CONFIG_TYPE_T;
#else
typedef enum
{
    HAL_LCPU_CONFIG_XTAL_ENABLED,      /*!< ADC calibration value. */
    HAL_LCPU_CONFIG_LPCYCLE_CURR,             /*!< Chip revision. */
    HAL_LCPU_CONFIG_LPCYCLE_AVE,  /*!< Battery calibration value. */
    HAL_LCPU_CONFIG_WDT_TIME,          /*!< SDADC calibration value. */
    HAL_LCPU_CONFIG_WDT_STATUS,  /*!< Battery calibration value. */
    HAL_LCPU_CONFIG_WDT_CLK_FEQ,
    HAL_LCPU_CONFIG_BT_TX_PWR,  /*!< BT tx pwr config. */
    HAL_LCPU_CONFIG_MAX,
} HAL_LCPU_CONFIG_TYPE_T;
#endif // !

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_ADC_CALIBRATION.
*/
#ifndef SF32LB55X
typedef struct
{
    uint16_t vol10;     /*!< Reg value for low voltage. */
    uint16_t vol25;      /*!< Reg value for high voltage. */
    uint16_t low_mv;     /*!< voltage for low with mv . */
    uint16_t high_mv;    /*!< voltage for high with mv. */
} HAL_LCPU_CONFIG_ADC_T;

#else
typedef struct
{
    uint16_t vol10;
    uint16_t vol25;
} HAL_LCPU_CONFIG_ADC_T;
#endif
/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_SDADC_CALIBRATION.
*/
typedef struct
{
    uint32_t vol_mv;
    uint32_t value;
} HAL_LCPU_CONFIG_SDMADC_T;

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_SN.
*/
typedef struct
{
    uint8_t sn[HAL_LCPU_CONFIG_SN_MAX_NUM];
} HAL_LCPU_CONFIG_SN_T;


/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_BATTERY_CALIBRATION.  ax+b   a*10000 for integer
*/
typedef struct
{
    int magic;
    uint32_t a;
    int32_t b;
} HAL_LCPU_CONFIG_BATTERY_T;


/**
  * @} LPCONFIG_exported_constants
*/


/**
  * @defgroup LPCONFIG_exported_functions LCPU configure Exported functions
  * @ingroup LPCONFIG
  * @{
  *
 */



/**
* @brief  Set configure parameter, only support in HCPU
* @param type Parameter type
* @param value Parameter values
* @param length Parameter length
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_TYPE_T type, void *value, uint16_t length);

/**
* @brief  Get configure parameter
* @param type Parameter type
* @param value Parameter values
* @param length Parameter length
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_TYPE_T type, uint8_t *value, uint16_t *length);

/**
* @brief Force init context without check
* @return void
*/
void HAL_LCPU_CONFIG_InitContext(void);



/**
  *@} LPCONFIG_exported_functions
*/


/**
  *@} LPCONFIG
  */


/**
  *@} BF0_HAL_Driver
  */



#ifdef __cplusplus
}
#endif


#endif // __BF0_HAL_LCPU_CONFIG_H

/**
  *@}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

