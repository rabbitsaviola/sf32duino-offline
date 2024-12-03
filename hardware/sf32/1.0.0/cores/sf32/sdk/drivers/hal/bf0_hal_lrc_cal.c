/**
  ******************************************************************************
  * @file   bf0_hal_lrc_cal.c
  * @author Sifli software development team
  * @brief Lower power RC clock calibration.
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


#include "bf0_hal.h"
#include "string.h"
#include "bf0_hal_rtc.h"


static int ave_window = 100;

__HAL_ROM_USED uint32_t HAL_RC_CAL_get_reference_cycle_on_48M(void)
{
#if 1 //ndef SF32LB52X HAL_LCPU_CONFIG_LPCYCLE_CURR  rom patch used
    uint32_t cycle_cur = HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
#else
    uint32_t cycle_cur;
    uint16_t len = 4;
    HAL_StatusTypeDef ret = HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_LPCYCLE_CURR, (uint8_t *)&cycle_cur, &len);
    HAL_ASSERT(ret == HAL_OK);
#endif

    return cycle_cur;
}

__STATIC_INLINE uint32_t HAL_RC_CAL_DisableInterrupt(void)
{
    uint32_t mask;

    mask = __get_PRIMASK();
    __set_PRIMASK(1);
    return mask;
}

__STATIC_INLINE void HAL_RC_CAL_EnableInterrupt(uint32_t mask)
{
    __set_PRIMASK(mask);
}
uint8_t g_xt48_used = 0;

#ifdef SF32LB55X
//extern void rt_kprintf(const char *fmt, ...);
__HAL_ROM_USED int HAL_RC_CAL_update_reference_cycle_on_48M_ex(uint8_t lp_cycle, int clear_ave, int ave_window)
{
    static uint32_t count = 0;
    static uint64_t value = 0;
    uint32_t cur = 0;


    if ((hwp_pmuc->CR & PMUC_CR_SEL_LPCLK) != 0)
    {
        return -1;
    }

    {
        uint32_t rcc_reg, div, div1, div2, try_times, delta;
        uint32_t int_mask;

        int_mask = HAL_RC_CAL_DisableInterrupt();
        hwp_lpsys_aon->ACR |= LPSYS_AON_ACR_HXT48_REQ;
        g_xt48_used = 1;
        HAL_RC_CAL_EnableInterrupt(int_mask);

        if (!(hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
        {
            while (0 == (hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
            {
                /* wait until HXT48 ready */
            }
        }
        // switch system clock to HXT48
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, 1);

        rcc_reg = READ_REG(hwp_lpsys_rcc->CFGR);
        div = (rcc_reg & LPSYS_RCC_CFGR_HDIV1_Msk) >> LPSYS_RCC_CFGR_HDIV1_Pos;
        div1 = (rcc_reg & LPSYS_RCC_CFGR_PDIV1_Msk) >> LPSYS_RCC_CFGR_PDIV1_Pos;
        div2 = (rcc_reg & LPSYS_RCC_CFGR_PDIV2_Msk) >> LPSYS_RCC_CFGR_PDIV2_Pos;
        // Reduce LPAON frequency to avoid voltage unstable
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(-1, -1, 7);
        for (try_times = 0; try_times < 10; try_times++)
        {
            hwp_ble_mac->RCCAL_CTRL &= ~BLE_MAC_RCCAL_CTRL_RCCAL_AUTO;
            hwp_ble_mac->RCCAL_CTRL |= (0x0 << BLE_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
            hwp_ble_mac->RCCAL_CTRL &= ~BLE_MAC_RCCAL_CTRL_RCCAL_LENGTH;
            hwp_ble_mac->RCCAL_CTRL |= (lp_cycle << BLE_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
            hwp_ble_mac->RCCAL_CTRL |= (0x1 << BLE_MAC_RCCAL_CTRL_RCCAL_START_Pos);
            while (!(hwp_ble_mac->RCCAL_RESULT & BLE_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
            cur = (hwp_ble_mac->RCCAL_RESULT & BLE_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);
            if (cur > 1000000)
                delta = cur - 1000000;
            else
                delta = 1000000 - cur;
            if (delta < 300000)
                break;
            //rt_kprintf("LPCycles: failed cur=%d delta=%d\n", cur, delta);
        }

        HAL_Set_backup(RTC_BACKUP_LPCYCLE_CUR, cur);
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(div, div1, div2);
        g_xt48_used = 0;
    }

    if (clear_ave)
    {
        count = 0;
        value = 0;
    }
    if (count < ave_window)
        count++;
    if (count)
    {
        if (count < ave_window)
        {
            value += HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
        }
        else
        {
            value = (uint64_t)HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
            value = value * (ave_window - 1) + HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
        }
        HAL_Set_backup(RTC_BACKUP_LPCYCLE_AVE, (uint32_t)(value / count));

#if 0   // For Debug only.
        {
            extern void rt_kprintf(const char *fmt, ...);
            rt_kprintf("LPCycles: %d, ave=%d, count=%d\n", HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR), (uint32_t)(value / count), count);
        }
#endif
    }

    return 0;
}

#else

#define LRC_CAL_RETRY_OFFSET 5000
#define LRC_CAL_JUDGE_COUNT 1
#define LRC_CAL_COUNT_AVE_LIMIT 5
#define LRC_CAL_THRESHOLD_UPPER 600
#define LRC_CAL_THRESHOLD_RANGE 200
__HAL_ROM_USED int HAL_RC_CAL_update_reference_cycle_on_48M_ex(uint8_t lp_cycle, int clear_ave, int ave_window)
{
    // TODO: Implement this for PRO
    static uint32_t count = 0;
    static uint64_t value = 0;
    uint32_t cur = 0;
    uint8_t cal_ok = 1;
    uint32_t cal_ave = 0;
    // Whether need reduce interval, 0: no need; 1: Suggest; 2: Must
    int cal_lvl = 0;
    static uint8_t cal_err = 0;


    if (HAL_RTC_LXT_ENABLED())
    {
        return -1;
    }

#ifdef SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_Delay(5);
#endif
    {
        uint32_t rcc_reg, div, div1, div2, try_times, delta = 0;
        uint32_t int_mask;

        int_mask = HAL_RC_CAL_DisableInterrupt();
        hwp_lpsys_aon->ACR |= LPSYS_AON_ACR_HXT48_REQ;
        g_xt48_used = 1;
        HAL_RC_CAL_EnableInterrupt(int_mask);

        if (!(hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
        {
            while (0 == (hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
            {
                /* wait until HXT48 ready */
            }
        }
        HAL_Delay(10);
        // switch system clock to HXT48
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, 1);

        rcc_reg = READ_REG(hwp_lpsys_rcc->CFGR);
        div = (rcc_reg & LPSYS_RCC_CFGR_HDIV1_Msk) >> LPSYS_RCC_CFGR_HDIV1_Pos;
        div1 = (rcc_reg & LPSYS_RCC_CFGR_PDIV1_Msk) >> LPSYS_RCC_CFGR_PDIV1_Pos;
        div2 = (rcc_reg & LPSYS_RCC_CFGR_PDIV2_Msk) >> LPSYS_RCC_CFGR_PDIV2_Pos;
        // Reduce LPAON frequency to avoid voltage unstable
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(-1, -1, 7);
        for (try_times = 0; try_times < 10; try_times++)
        {

#ifdef SF32LB52X
            MODIFY_REG(hwp_bt_mac->RCCAL_CTRL, BT_MAC_RCCAL_CTRL_CON_NUM_Msk, 1 << BT_MAC_RCCAL_CTRL_CON_NUM_Pos);
            hwp_bt_mac->RCCAL_CTRL |= BT_MAC_RCCAL_CTRL_CON_MODE;
#endif

            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_AUTO;
            hwp_bt_mac->RCCAL_CTRL |= (0x0 << BT_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_LENGTH;
            hwp_bt_mac->RCCAL_CTRL |= (lp_cycle << BT_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
            hwp_bt_mac->RCCAL_CTRL |= (0x1 << BT_MAC_RCCAL_CTRL_RCCAL_START_Pos);
            HAL_Delay(1);
            while (!(hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
            cur = (hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);

            if (count >= LRC_CAL_JUDGE_COUNT)
            {
                cal_ave = HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
                delta = cur > cal_ave ? (cur - cal_ave) : (cal_ave - cur);

                if (delta < LRC_CAL_RETRY_OFFSET)
                    break;
            }
            else
            {
                break;
            }
        }
        HAL_DBG_printf("LPCycles: cur=%d delta=%d,try_times=%d\n", cur, delta, (try_times + 1));

        if (try_times == 10)
        {
            cal_ok = 0;
            cal_err++;
            if (cal_err == 4)
            {
                count = 0;
                value = 0;
            }
        }
        else
        {
            uint32_t delta_limit = LRC_CAL_THRESHOLD_UPPER - LRC_CAL_THRESHOLD_RANGE * count / ave_window;
            HAL_DBG_printf("Delta limit: %d\n", delta_limit);
            cal_err = 0;
            HAL_Set_backup(RTC_BACKUP_LPCYCLE_CUR, cur);
            if (delta > delta_limit)
                clear_ave = 1;
        }
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(div, div1, div2);
        g_xt48_used = 0;
    }

    if (0 == cal_ok)
    {
#ifdef SF32LB52X
        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
        return -2;
    }

    if (clear_ave)
    {
        count = 0;
        value = 0;
        cal_lvl = 2;
    }
    else if (count <= (LRC_CAL_COUNT_AVE_LIMIT - 1))
        cal_lvl = 1;

    if (count < ave_window)
        count++;
    if (count)
    {
        if (count < ave_window)
        {
            value += HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
        }
        else
        {
            value = (uint64_t)HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
            value = value * (ave_window - 1) + HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
        }
        HAL_Set_backup(RTC_BACKUP_LPCYCLE_AVE, (uint32_t)(value / count));
#ifdef SF32LB52X
        {
            uint32_t set_value = (uint32_t)(value / count);
            float lxtfreq;

            lxtfreq = (48000000UL / (float)set_value * lp_cycle);
            HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_LPCYCLE_AVE, &set_value, 4);
            HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_LPCYCLE_CURR, &lxtfreq, 4);
        }
#endif
        HAL_DBG_printf("LPCycles: %d, ave=%d, count=%d\n", HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR), (uint32_t)(value / count), count);
    }
#ifdef SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
    return cal_lvl;
}
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

