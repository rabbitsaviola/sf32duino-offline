/**
  ******************************************************************************
  * @file   bf0_hal_lcdc.c
  * @author Sifli software development team
  * @brief   lcdc HAL module driver.
  *          This file provides firmware functions to manage the following
  ******************************************************************************
*/
/**
 *
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include <string.h>
#include "bf0_hal.h"

//extern void rt_kprintf(const char *fmt, ...);

#define LCDC_LOG(...)   //do{rt_kprintf(__VA_ARGS__);rt_kprintf("\r\n");}while(0)

#define LCDC_LOG_D(...)  //LCDC_LOG(__VA_ARGS__)
#define LCDC_LOG_I(...)  //LCDC_LOG(__VA_ARGS__)
#define LCDC_LOG_E(...)  LCDC_LOG(__VA_ARGS__)
#define LCDC_PRINT_AREA(s,area) //LCDC_LOG("%s: x0y0=%d,%d  x1y1=%d,%d  \n",s,(area)->x0,(area)->y0,(area)->x1,(area)->y1)

#define LCDC_TIMEOUT_SECONDS  1

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup LCDC LCD Controller
  * @{
  */

#if defined(HAL_LCD_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)
typedef enum
{

    LCDC_SYNC_MODE  = 0,
    LCDC_ASYNC_MODE = 1

} LCDC_AsyncModeTypeDef;

typedef enum
{
    NEXT_FRAME_TE_OFF = 0,
    NEXT_FRAME_TE_ON  = 1,
    NEXT_FRAME_TE_DEFAULT = 0xFF, //Inherit LCDC init configuration
} Next_Frame_TE_TypeDef;

static void (*g_LCDC_CpltCallback)(LCDC_HandleTypeDef *lcdc);
static void LCDC_TransCpltCallback(LCDC_HandleTypeDef *lcdc);
static HAL_StatusTypeDef LayerUpdate(LCDC_HandleTypeDef *lcdc);
static HAL_StatusTypeDef _SendLayerData(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode);
static HAL_StatusTypeDef SendSingleCmd(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len);
static HAL_StatusTypeDef WaitBusy(LCDC_HandleTypeDef *lcdc);
static void HAL_LCDC_JDIParallelInit(LCDC_HandleTypeDef *lcdc);

#if defined(USE_FULL_ASSERT)||defined(_SIFLI_DOXYGEN_) || defined(USE_LOOP_ASSERT)
    #define HAL_LCDC_ASSERT  HAL_ASSERT
#else
    #define HAL_LCDC_ASSERT(expr) if ((expr)==0) while (1)
#endif /* USE_FULL_ASSERT */




#define __HAL_LCDC_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       (__HANDLE__)->ErrorCode |= HAL_LCDC_ERROR_HAL_LOCKED;\
                                       HAL_LCDC_ASSERT(0);                \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0)

#define __HAL_LCDC_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      HAL_LCDC_ASSERT(0 == ((__HANDLE__)->ErrorCode & HAL_LCDC_ERROR_HAL_LOCKED)); \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0)


#define SendLayerData_IT(lcdc)  _SendLayerData(lcdc, 1)

#define LCDC_IS_TLBR_OVERFLOW(layer, x0, y0, x1, y1) (0 != (((x0) & (~(layer##_TL_POS_X0_Msk >> layer##_TL_POS_X0_Pos))) \
                                                     |((y0) & (~(layer##_TL_POS_Y0_Msk >> layer##_TL_POS_Y0_Pos))) \
                                                     |((x1) & (~(layer##_BR_POS_X1_Msk >> layer##_BR_POS_X1_Pos))) \
                                                     |((y1) & (~(layer##_BR_POS_Y1_Msk >> layer##_BR_POS_Y1_Pos)))) \
                                                     || ((x0)>(x1) || ((y0)>(y1))))


#define HAL_LCDC_LOOKUP_TABLE_HEAD(pHwLCDC) ((uint32_t *)(((uint32_t)(pHwLCDC)) + 0x400))



#define WAIT_LCDC_SINGLE_BUSY(lcdc) while((lcdc)->Instance->LCD_SINGLE & LCD_IF_LCD_SINGLE_LCD_BUSY)
#define GET_LCDC_SYSID(lcdc) ((LCDC1 == ((lcdc)->Instance))?CORE_ID_HCPU:CORE_ID_LCPU)
#define LCDC_DELAY_NS(ns) HAL_Delay_us(((ns)/1000) + 1)

#ifdef HAL_DSI_MODULE_ENABLED
    /**
    * @brief  [Obsolete]   Send layer data to LCD by several memory write ops.
    * @param  lcdc LCD controller handle
    * @param  write_cmd - LCD memory write command
    * @param  rg_cb - Callback to set LCD current written colunms & rows, NULL - use continue mode
    * @retval HAL status
    */
    static HAL_StatusTypeDef _DSI_StartPartialWrite(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode, uint8_t write_cmd, SetRegion_cb_t rg_cb);
    static bool _PartialWriteContinue(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode);
    static HAL_StatusTypeDef DSI_SetOutFormat(LCDC_HandleTypeDef *lcdc, HAL_LCDC_PixelFormat color_mode);
    static HAL_StatusTypeDef DSI_SetFreq(LCDC_HandleTypeDef *lcdc, uint32_t freq);
    static void HAL_LCDC_DSI_Init(LCDC_HandleTypeDef *lcdc);
    #ifndef SF32LB55X
        #define DSI_DIRECT_SEND_MODE //DSI directly send interface
        static void DSI_Video_Init(LCDC_HandleTypeDef *lcdc);
        static bool _DSI_DirectWriteDone(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode);
        static HAL_StatusTypeDef _DSI_DirectWrite(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode, uint8_t write_cmd, SetRegion_cb_t rg_cb);
    #endif

    #ifndef DSI_DIRECT_SEND_MODE
        #define DSIMultiWriteStart(lcdc, write_cmd, rg_cb) _DSI_StartPartialWrite(lcdc,LCDC_SYNC_MODE,write_cmd,rg_cb)
        #define DSIMultiWriteStart_IT(lcdc, write_cmd, rg_cb) _DSI_StartPartialWrite(lcdc,LCDC_ASYNC_MODE,write_cmd,rg_cb)
        #define DSIMultiWriteContinue_IT(lcdc)                   _PartialWriteContinue(lcdc, LCDC_ASYNC_MODE)
    #else
        #define DSIMultiWriteStart(lcdc, write_cmd, rg_cb) _DSI_DirectWrite(lcdc,LCDC_SYNC_MODE,write_cmd,rg_cb)
        #define DSIMultiWriteStart_IT(lcdc, write_cmd, rg_cb) _DSI_DirectWrite(lcdc,LCDC_ASYNC_MODE,write_cmd,rg_cb)
        #define DSIMultiWriteContinue_IT(lcdc)                   _DSI_DirectWriteDone(lcdc, LCDC_ASYNC_MODE)
    #endif /* SF32LB55X */



#endif /* HAL_DSI_MODULE_ENABLED */
void HAL_LCDC_Enable_TE(LCDC_HandleTypeDef *lcdc, bool en);
void HAL_LCDC_Invert_TE_Pol(LCDC_HandleTypeDef *lcdc);
void HAL_LCDC_SPI_Sequence(LCDC_HandleTypeDef *lcdc, bool end);

#if defined(SF32LB55X)&&defined(LCDC_SUPPORT_LINE_DONE_IRQ)
static bool area_intersect(LCDC_AreaDef *res_p, const LCDC_AreaDef *a0_p, const LCDC_AreaDef *a1_p)
{
    /* Get the smaller area from 'a0_p' and 'a1_p' */
    res_p->x0 = HAL_MAX(a0_p->x0, a1_p->x0);
    res_p->y0 = HAL_MAX(a0_p->y0, a1_p->y0);
    res_p->x1 = HAL_MIN(a0_p->x1, a1_p->x1);
    res_p->y1 = HAL_MIN(a0_p->y1, a1_p->y1);

    /*If x0 or y0 greater then x1 or y1 then the areas union is empty*/
    bool union_ok = true;
    if ((res_p->x0 > res_p->x1) || (res_p->y0 > res_p->y1))
    {
        union_ok = false;
    }

    return union_ok;
}
#endif

__STATIC_INLINE void LCDC_SET_TE(LCDC_HandleTypeDef *lcdc, bool en)
{
    if (en)
        lcdc->Instance->TE_CONF |= LCD_IF_TE_CONF_ENABLE;
    else
        lcdc->Instance->TE_CONF &= ~LCD_IF_TE_CONF_ENABLE;
}

static uint8_t HAL_LCDC_GetPixelSize(HAL_LCDC_PixelFormat color_format)
{
    uint8_t bytes_per_pixel = 0;

    switch (color_format)
    {
    case LCDC_PIXEL_FORMAT_RGB332:
    case LCDC_PIXEL_FORMAT_A8:
    case LCDC_PIXEL_FORMAT_L8:
        bytes_per_pixel = 1;
        break;

    case LCDC_PIXEL_FORMAT_RGB565_SWAP:
    case LCDC_PIXEL_FORMAT_RGB565:
        bytes_per_pixel = 2;
        break;

    case LCDC_PIXEL_FORMAT_ARGB565:
    case LCDC_PIXEL_FORMAT_RGB888:
        bytes_per_pixel = 3;
        break;

    case LCDC_PIXEL_FORMAT_ARGB888:
        bytes_per_pixel = 4;
        break;

    default:
        //unknow color format
        HAL_ASSERT(0);
        break;
    }

    return bytes_per_pixel;
}

static HAL_StatusTypeDef SelectIntf(LCDC_HandleTypeDef *lcdc, HAL_LCDC_IF_TypeDef intf)
{
    LCDC_InitTypeDef *init;
    uint32_t reg_v;

    HAL_LCDC_ASSERT(lcdc);
    init = &lcdc->Init;
    init->lcd_itf = intf;

    lcdc->Instance->LCD_CONF &= ~(LCD_IF_LCD_CONF_LCD_INTF_SEL_Msk | LCD_IF_LCD_CONF_TARGET_LCD_Msk);

    if (HAL_LCDC_IS_DBI_IF(intf))
    {
        lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_DBI_LCD_SEL;

        reg_v = lcdc->Instance->LCD_IF_CONF;


        lcdc->Instance->LCD_IF_CONF  = reg_v | (1 << LCD_IF_LCD_IF_CONF_TAS_Pos) |
                                       (1 << LCD_IF_LCD_IF_CONF_TAH_Pos) |
                                       (init->cfg.dbi.CS_polarity << LCD_IF_LCD_IF_CONF_CS0_POL_Pos) |
                                       (init->cfg.dbi.RS_polarity << LCD_IF_LCD_IF_CONF_RS_POL_Pos) |
                                       (init->cfg.dbi.WR_polarity << LCD_IF_LCD_IF_CONF_WR_POL_Pos) |
                                       (init->cfg.dbi.RD_polarity << LCD_IF_LCD_IF_CONF_RD_POL_Pos);

    }
    else if (HAL_LCDC_IS_AHB_IF(intf))
    {
        lcdc->Instance->LCD_CONF |= (3 << LCD_IF_LCD_CONF_TARGET_LCD_Pos);  //set AHB RAM
        lcdc->Instance->LCD_MEM = init->cfg.ahb.lcd_mem;
        lcdc->Instance->LCD_O_WIDTH = init->cfg.ahb.lcd_o_width;
    }
    else if (HAL_LCDC_IS_SPI_IF(intf))
    {
        lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_SPI_LCD_SEL;

        switch (init->lcd_itf)
        {
        case LCDC_INTF_SPI_NODCX_1DATA:
            reg_v = LCD_IF_SPI_IF_CONF_3LINE;
            break;
        case LCDC_INTF_SPI_NODCX_2DATA:
            reg_v = LCD_IF_SPI_IF_CONF_3LINE_2_DATA_LINE;
            break;
        case LCDC_INTF_SPI_NODCX_4DATA:
            reg_v = LCD_IF_SPI_IF_CONF_3LINE_4_DATA_LINE;
            break;
        case LCDC_INTF_SPI_DCX_1DATA:
            reg_v = LCD_IF_SPI_IF_CONF_4LINE;
            break;
        case LCDC_INTF_SPI_DCX_2DATA:
            reg_v = LCD_IF_SPI_IF_CONF_4LINE_2_DATA_LINE;
            break;
        case LCDC_INTF_SPI_DCX_4DATA:
            reg_v = LCD_IF_SPI_IF_CONF_4LINE_4_DATA_LINE;
            break;
        case LCDC_INTF_SPI_DCX_DDR_4DATA:
#ifdef LCDC_SUPPORT_DDR_QSPI
#ifdef SF32LB52X
            HAL_LCDC_ASSERT(__HAL_SYSCFG_GET_REVID() >= 2);
#endif /* SF32LB52X */
#else
            HAL_LCDC_ASSERT(0);
#endif
            reg_v = LCD_IF_SPI_IF_CONF_4LINE_4_DATA_LINE;
            break;
        default:
            reg_v = LCD_IF_SPI_IF_CONF_4LINE;
            break;
        }

        reg_v |= MAKE_REG_VAL(init->cfg.spi.cs_polarity,  LCD_IF_SPI_IF_CONF_SPI_CLK_POL_Msk,  LCD_IF_SPI_IF_CONF_SPI_CLK_POL_Pos);
        reg_v |= MAKE_REG_VAL(!init->cfg.spi.clk_polarity, LCD_IF_SPI_IF_CONF_SPI_CLK_INIT_Msk, LCD_IF_SPI_IF_CONF_SPI_CLK_INIT_Pos);
        reg_v |= MAKE_REG_VAL(init->cfg.spi.clk_phase,    LCD_IF_SPI_IF_CONF_SPI_CLK_POL_Msk,  LCD_IF_SPI_IF_CONF_SPI_CLK_POL_Pos);
        reg_v |= MAKE_REG_VAL(init->cfg.spi.dummy_clock,  LCD_IF_SPI_IF_CONF_DUMMY_CYCLE_Msk,  LCD_IF_SPI_IF_CONF_DUMMY_CYCLE_Pos);
        reg_v |= LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS | LCD_IF_SPI_IF_CONF_SPI_CLK_AUTO_DIS | LCD_IF_SPI_IF_CONF_SPI_CS_NO_IDLE;
        lcdc->Instance->SPI_IF_CONF = reg_v;

#ifdef LCD_IF_LCD_CONF_SPI_RD_SEL
        HAL_LCDC_ASSERT(init->cfg.spi.readback_from_Dx <= (LCD_IF_LCD_CONF_SPI_RD_SEL_Msk >> LCD_IF_LCD_CONF_SPI_RD_SEL_Pos));
        lcdc->Instance->LCD_CONF &= ~LCD_IF_LCD_CONF_SPI_RD_SEL_Msk;
        lcdc->Instance->LCD_CONF |= init->cfg.spi.readback_from_Dx << LCD_IF_LCD_CONF_SPI_RD_SEL_Pos;
#endif /* LCD_IF_LCD_CONF_SPI_RD_SEL */
    }
#ifdef HAL_DSI_MODULE_ENABLED
    else if (HAL_LCDC_IS_DSI_CMD_IF(intf))
    {
        HAL_LCDC_DSI_Init(lcdc);
#ifndef DSI_DIRECT_SEND_MODE
        lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_DSI_LCD_SEL;  //set DSI
#else
        lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_DIRECT_INTF_EN  //set Direct INTF
                                    | (2 << LCD_IF_LCD_CONF_TARGET_LCD_Pos); //set AHB LCD
#endif /* SF32LB55X */

    }
#ifndef SF32LB55X
    else if (HAL_LCDC_IS_DSI_VID_IF(intf))
    {
        DSI_Video_Init(lcdc);
        lcdc->Instance->LCD_CONF |= 7 << LCD_IF_LCD_CONF_LCD_INTF_SEL_Pos; //DPI->DSI

        //Config DPI
        lcdc->Instance->DPI_IF_CONF1 = (init->cfg.dsi_vid.VidCfg.VS_width << LCD_IF_DPI_IF_CONF1_VSH_Pos) |
                                       (init->cfg.dsi_vid.VidCfg.HS_width << LCD_IF_DPI_IF_CONF1_HSW_Pos);

        lcdc->Instance->DPI_IF_CONF2 = (init->cfg.dsi_vid.VidCfg.bank_col_head << LCD_IF_DPI_IF_CONF2_VBP_Pos) |
                                       (init->cfg.dsi_vid.VidCfg.bank_row_head << LCD_IF_DPI_IF_CONF2_HBP_Pos);
        lcdc->Instance->DPI_IF_CONF3 = (init->cfg.dsi_vid.VidCfg.bank_col_tail << LCD_IF_DPI_IF_CONF3_VFP_Pos) |
                                       (init->cfg.dsi_vid.VidCfg.bank_row_tail << LCD_IF_DPI_IF_CONF3_HFP_Pos);
        lcdc->Instance->DPI_IF_CONF4 = (init->cfg.dsi_vid.VidCfg.valid_columns << LCD_IF_DPI_IF_CONF4_VAH_Pos) |
                                       (init->cfg.dsi_vid.VidCfg.valid_rows    << LCD_IF_DPI_IF_CONF4_HAW_Pos);

        lcdc->Instance->DPI_IF_CONF5 &=  ~(LCD_IF_DPI_IF_CONF5_INT_LINE_NUM_Msk |
                                           LCD_IF_DPI_IF_CONF5_PCLKPOL_Msk |
                                           LCD_IF_DPI_IF_CONF5_DEPOL_Msk |
                                           LCD_IF_DPI_IF_CONF5_VSPOL_Msk |
                                           LCD_IF_DPI_IF_CONF5_HSPOL_Msk);

        lcdc->Instance->DPI_IF_CONF5 |= (init->cfg.dsi_vid.VidCfg.interrupt_line_num << LCD_IF_DPI_IF_CONF5_INT_LINE_NUM_Pos) |
                                        (0 << LCD_IF_DPI_IF_CONF5_PCLKPOL_Pos) |
                                        (0 << LCD_IF_DPI_IF_CONF5_DEPOL_Pos) |
                                        (0 << LCD_IF_DPI_IF_CONF5_VSPOL_Pos) |
                                        (0 << LCD_IF_DPI_IF_CONF5_HSPOL_Pos);

    }
#endif /* SF32LB55X */
#endif /* HAL_DSI_MODULE_ENABLED */
    else if (HAL_LCDC_IS_JDI_PARALLEL_IF(intf))
    {
        lcdc->Instance->LCD_CONF |= 5 << LCD_IF_LCD_CONF_LCD_INTF_SEL_Pos;
    }
    else if (HAL_LCDC_IS_JDI_SERIAL_IF(lcdc->Init.lcd_itf))
    {
        lcdc->Instance->LCD_CONF |= 4 << LCD_IF_LCD_CONF_LCD_INTF_SEL_Pos;
    }
    else if (HAL_LCDC_IS_DPI_IF(intf))
    {
        lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_DPI_LCD_SEL;


        lcdc->Instance->DPI_IF_CONF1 = (init->cfg.dpi.VS_width << LCD_IF_DPI_IF_CONF1_VSH_Pos) |
                                       (init->cfg.dpi.HS_width << LCD_IF_DPI_IF_CONF1_HSW_Pos);

        lcdc->Instance->DPI_IF_CONF2 = (init->cfg.dpi.bank_col_head << LCD_IF_DPI_IF_CONF2_VBP_Pos) |
                                       (init->cfg.dpi.bank_row_head << LCD_IF_DPI_IF_CONF2_HBP_Pos);
        lcdc->Instance->DPI_IF_CONF3 = (init->cfg.dpi.bank_col_tail << LCD_IF_DPI_IF_CONF3_VFP_Pos) |
                                       (init->cfg.dpi.bank_row_tail << LCD_IF_DPI_IF_CONF3_HFP_Pos);
        lcdc->Instance->DPI_IF_CONF4 = (init->cfg.dpi.valid_columns << LCD_IF_DPI_IF_CONF4_VAH_Pos) |
                                       (init->cfg.dpi.valid_rows    << LCD_IF_DPI_IF_CONF4_HAW_Pos);

        lcdc->Instance->DPI_IF_CONF5 &=  ~(LCD_IF_DPI_IF_CONF5_INT_LINE_NUM_Msk |
                                           LCD_IF_DPI_IF_CONF5_PCLKPOL_Msk |
                                           LCD_IF_DPI_IF_CONF5_DEPOL_Msk |
                                           LCD_IF_DPI_IF_CONF5_VSPOL_Msk |
                                           LCD_IF_DPI_IF_CONF5_HSPOL_Msk);

        lcdc->Instance->DPI_IF_CONF5 |= (init->cfg.dpi.interrupt_line_num << LCD_IF_DPI_IF_CONF5_INT_LINE_NUM_Pos) |
                                        (init->cfg.dpi.PCLK_polarity         << LCD_IF_DPI_IF_CONF5_PCLKPOL_Pos) |
                                        (init->cfg.dpi.DE_polarity         << LCD_IF_DPI_IF_CONF5_DEPOL_Pos) |
                                        (init->cfg.dpi.VS_polarity         << LCD_IF_DPI_IF_CONF5_VSPOL_Pos) |
                                        (init->cfg.dpi.HS_polarity         << LCD_IF_DPI_IF_CONF5_HSPOL_Pos);
    }
    else
    {
        HAL_LCDC_ASSERT(0);//Not support now
        return HAL_ERROR;
    }

    return HAL_OK;
}


static HAL_StatusTypeDef SetFreq(LCDC_HandleTypeDef *lcdc, uint32_t freq)
{
    HAL_StatusTypeDef ret_v = HAL_OK;
    uint32_t reg_v, lcdc_clk;
    LCDC_InitTypeDef *init;

    HAL_LCDC_ASSERT(lcdc);
    HAL_LCDC_ASSERT(freq > 0);
    lcdc_clk = HAL_RCC_GetHCLKFreq(GET_LCDC_SYSID(lcdc));
    //uint32_t lcdc_pclk_Hz = HAL_RCC_GetPCLKFreq(GET_LCDC_SYSID(lcdc), 1);

    init = &lcdc->Init;
    init->freq = freq;

    if (HAL_LCDC_IS_SPI_IF(init->lcd_itf))
    {
        uint32_t clk_div;

        clk_div = (lcdc_clk + (freq - 1)) / freq;

        if (clk_div < 2) /*HW NOT support divider 1*/
        {
            clk_div = 2;
        }
        else if (clk_div > GET_REG_VAL(LCD_IF_SPI_IF_CONF_CLK_DIV_Msk, LCD_IF_SPI_IF_CONF_CLK_DIV_Msk, LCD_IF_SPI_IF_CONF_CLK_DIV_Pos))
        {
            clk_div = GET_REG_VAL(LCD_IF_SPI_IF_CONF_CLK_DIV_Msk, LCD_IF_SPI_IF_CONF_CLK_DIV_Msk, LCD_IF_SPI_IF_CONF_CLK_DIV_Pos);
        }
        else
        {
            ;//valid value, do nothing
        }

        reg_v = lcdc->Instance->SPI_IF_CONF;
        reg_v &= ~LCD_IF_SPI_IF_CONF_CLK_DIV_Msk;
        reg_v |= MAKE_REG_VAL(clk_div, LCD_IF_SPI_IF_CONF_CLK_DIV_Msk, LCD_IF_SPI_IF_CONF_CLK_DIV_Pos);
        lcdc->Instance->SPI_IF_CONF = reg_v;

    }
    else if (HAL_LCDC_IS_DBI_IF(init->lcd_itf))
    {
        uint32_t pw, pwl, pwh;


        pw = (lcdc_clk + (freq - 1)) / freq;
        pwl = pw / 2;
        pwh = pw - pwl;

        if (pwl < 1)
        {
            pwl = 1;
        }
        else if (pwl > GET_REG_VAL(LCD_IF_LCD_IF_CONF_PWL_Msk, LCD_IF_LCD_IF_CONF_PWL_Msk, LCD_IF_LCD_IF_CONF_PWL_Pos))
        {
            pwl = GET_REG_VAL(LCD_IF_LCD_IF_CONF_PWL_Msk, LCD_IF_LCD_IF_CONF_PWL_Msk, LCD_IF_LCD_IF_CONF_PWL_Pos);
        }

        if (pwh < 1)
        {
            pwh = 1;
        }
        else if (pwh > GET_REG_VAL(LCD_IF_LCD_IF_CONF_PWH_Msk, LCD_IF_LCD_IF_CONF_PWH_Msk, LCD_IF_LCD_IF_CONF_PWH_Pos))
        {
            pwh = GET_REG_VAL(LCD_IF_LCD_IF_CONF_PWH_Msk, LCD_IF_LCD_IF_CONF_PWH_Msk, LCD_IF_LCD_IF_CONF_PWH_Pos);
        }

        reg_v = lcdc->Instance->LCD_IF_CONF;
        reg_v &= ~(LCD_IF_LCD_IF_CONF_PWL_Msk | LCD_IF_LCD_IF_CONF_PWH_Msk);
        reg_v |= (pwl << LCD_IF_LCD_IF_CONF_PWL_Pos) | (pwh << LCD_IF_LCD_IF_CONF_PWH_Pos);
        lcdc->Instance->LCD_IF_CONF = reg_v;

    }
    else if (HAL_LCDC_IS_AHB_IF(lcdc->Init.lcd_itf))
    {
    }
#ifdef HAL_DSI_MODULE_ENABLED
    else if (HAL_LCDC_IS_DSI_CMD_IF(lcdc->Init.lcd_itf))
    {
        uint32_t pwl, pwh;


        if (DSI_LP_DLW_ENABLE == init->cfg.dsi.LPCmd.LPDcsLongWrite) //Low speed mode
        {
            //dsi_kbps = (init->freq / 1000) / 8 / init->cfg.dsi.Init.TXEscapeCkdiv;

            /*DBI send data at lowest speed at LP mode*/
            pwh = (LCD_IF_LCD_IF_CONF_PWH_Msk >> LCD_IF_LCD_IF_CONF_PWH_Pos);
            pwl = (LCD_IF_LCD_IF_CONF_PWL_Msk >> LCD_IF_LCD_IF_CONF_PWL_Pos);
            HAL_LCDC_ASSERT(lcdc_clk <= 48000000);//Make sure dsi fifo not over flow
        }
        else  //High speed mode
        {
            /*Match DBI output speed with DSI by change pwl&pwh.*/
            uint32_t pw, dsi_kbps, dbi_kbps, target_dbi_kbps;

            dbi_kbps = (lcdc_clk / 1000) * 16 /*16bit dbi bus*/;

            if (0) // (DSI_TWO_DATA_LANES == init->cfg.dsi.Init.NumberOfLanes) NOT twice of one lane
                dsi_kbps = (init->freq / 1000) * 2; //Two data lanes, double bps
            else
                dsi_kbps = (init->freq / 1000);


            /*Make sure target_dbi_kbps is lower than dsi_kbps so dsi fifo will not overflow */
            target_dbi_kbps = (dsi_kbps * 78) / 100;  //Factor 0.78

            pw = (dbi_kbps + (target_dbi_kbps - 1)/*Round up*/) / target_dbi_kbps;

            pwh = pw / 2;  //pwl > pwh is better
            pwl = pw - pwh;

        }

        //Slow down DBI speed if Ackknowledge is enable for testcase
        if (HAL_DSI_ISForceEnableAck())
        {
            pwl <<= 1;
            pwh <<= 1;
        }

        LCDC_LOG_I("Dsi pwl=%d, pwh=%d, lcdc_clk=%d", pwl, pwh, lcdc_clk);
        HAL_LCDC_ASSERT(pwh <= (LCD_IF_LCD_IF_CONF_PWH_Msk >> LCD_IF_LCD_IF_CONF_PWH_Pos));
        HAL_LCDC_ASSERT(pwl <= (LCD_IF_LCD_IF_CONF_PWL_Msk >> LCD_IF_LCD_IF_CONF_PWL_Pos));




        reg_v = lcdc->Instance->LCD_IF_CONF;

        if (DSI_LP_DLW_ENABLE == init->cfg.dsi.LPCmd.LPDcsLongWrite) //Low speed mode
        {
            reg_v |= (0 << LCD_IF_LCD_IF_CONF_WR_POL_Pos)
                     | (1 << LCD_IF_LCD_IF_CONF_TAS_Pos)
                     | (1 << LCD_IF_LCD_IF_CONF_TAH_Pos) ;
        }
        else
        {
            __HAL_DSI_IS_DBI_INF_DELAY(&lcdc->hdsi, 1);

            reg_v |= (1 << LCD_IF_LCD_IF_CONF_WR_POL_Pos)
                     | (6 << LCD_IF_LCD_IF_CONF_TAS_Pos)
                     | (6 << LCD_IF_LCD_IF_CONF_TAH_Pos) ;
#ifndef SF32LB55X
            reg_v |= LCD_IF_LCD_IF_CONF_CTRL_DLY_SET;
#endif /* !SF32LB55X */
        }



        lcdc->Instance->LCD_IF_CONF = reg_v |
                                      (pwl << LCD_IF_LCD_IF_CONF_PWL_Pos) |
                                      (pwh << LCD_IF_LCD_IF_CONF_PWH_Pos) |
                                      (0 << LCD_IF_LCD_IF_CONF_CS0_POL_Pos) |
                                      (0 << LCD_IF_LCD_IF_CONF_CS1_POL_Pos) |
                                      (0 << LCD_IF_LCD_IF_CONF_RS_POL_Pos) |
                                      (0 << LCD_IF_LCD_IF_CONF_RD_POL_Pos);


        ret_v = DSI_SetFreq(lcdc, freq);

    }
    else if (HAL_LCDC_IS_DSI_VID_IF(lcdc->Init.lcd_itf))
    {
        //Config DPI clk
        reg_v = (lcdc_clk + (freq - 1)) / freq;
        lcdc->Instance->DPI_IF_CONF5 &= ~LCD_IF_DPI_IF_CONF5_PCLK_DIV_Msk;
        lcdc->Instance->DPI_IF_CONF5 |= reg_v << LCD_IF_DPI_IF_CONF5_PCLK_DIV_Pos;

        ret_v = DSI_SetFreq(lcdc, freq);

    }
#endif /* HAL_DSI_MODULE_ENABLED */
    else if (HAL_LCDC_IS_JDI_PARALLEL_IF(lcdc->Init.lcd_itf))
    {
        HAL_LCDC_JDIParallelInit(lcdc);
    }
    else if (HAL_LCDC_IS_JDI_SERIAL_IF(lcdc->Init.lcd_itf))
    {
    }
    else if (HAL_LCDC_IS_DPI_IF(lcdc->Init.lcd_itf))
    {
        reg_v = (lcdc_clk + (freq - 1)) / freq;
        lcdc->Instance->DPI_IF_CONF5 &= ~LCD_IF_DPI_IF_CONF5_PCLK_DIV_Msk;
        lcdc->Instance->DPI_IF_CONF5 |= reg_v << LCD_IF_DPI_IF_CONF5_PCLK_DIV_Pos;
    }
    else
    {
        HAL_LCDC_ASSERT(0);//Not support now
        ret_v = HAL_ERROR;
    }

    return ret_v;
}

static HAL_StatusTypeDef SetOutFormat(LCDC_HandleTypeDef *lcdc, HAL_LCDC_PixelFormat color_mode)
{
    LCDC_InitTypeDef *init;
    uint32_t reg_v;

    HAL_LCDC_ASSERT(lcdc);


    init = &lcdc->Init;
    init->color_mode = color_mode;

    /*LCD output format*/
    reg_v = lcdc->Instance->LCD_CONF;

    reg_v &= ~(LCD_IF_LCD_CONF_LCD_FORMAT_Msk | LCD_IF_LCD_CONF_AHB_FORMAT_Msk | LCD_IF_LCD_CONF_SPI_LCD_FORMAT_Msk);
    reg_v &= ~(LCD_IF_LCD_CONF_DPI_LCD_FORMAT_Msk | LCD_IF_LCD_CONF_JDI_SER_FORMAT_Msk);

    switch (color_mode)
    {
    case LCDC_PIXEL_FORMAT_RGB565:
        reg_v |= (0 << LCD_IF_LCD_CONF_AHB_FORMAT_Pos)                // AHB LCD/RAM
                 | (2 << LCD_IF_LCD_CONF_DPI_LCD_FORMAT_Pos)           // DPI LCD
                 | (1 << LCD_IF_LCD_CONF_SPI_LCD_FORMAT_Pos);           // SPI LCD

        if (HAL_LCDC_IS_DBI_IF(lcdc->Init.lcd_itf))
        {
            reg_v |= LCD_IF_LCD_CONF_LCD_FORMAT_8BIT_RGB565;           // DBI RGB565 over 8-bit bus
        }
        else
        {
            reg_v |= LCD_IF_LCD_CONF_LCD_FORMAT_RGB565;               // DBI RGB565 over 16-bit bus
        }
        break;

#ifndef SF32LB55X
    case LCDC_PIXEL_FORMAT_RGB565_SWAP:
        reg_v |= (0 << LCD_IF_LCD_CONF_AHB_FORMAT_Pos)                // AHB LCD/RAM
                 | (2 << LCD_IF_LCD_CONF_DPI_LCD_FORMAT_Pos)           // DPI LCD
                 | (1 << LCD_IF_LCD_CONF_SPI_LCD_FORMAT_Pos)           // SPI LCD
                 | LCD_IF_LCD_CONF_ENDIAN;

        if (HAL_LCDC_IS_DBI_IF(lcdc->Init.lcd_itf))
        {
            reg_v |= LCD_IF_LCD_CONF_LCD_FORMAT_8BIT_RGB565;           // DBI RGB565 over 8-bit bus
        }
        else
        {
            reg_v |= LCD_IF_LCD_CONF_LCD_FORMAT_RGB565;               // DBI RGB565 over 16-bit bus
        }
        break;
#endif /* !SF32LB55X */

    case LCDC_PIXEL_FORMAT_RGB888:
        reg_v |= (1 << LCD_IF_LCD_CONF_AHB_FORMAT_Pos)               // AHB LCD/RAM
                 | (5 << LCD_IF_LCD_CONF_DPI_LCD_FORMAT_Pos)           // DPI LCD
                 | (2 << LCD_IF_LCD_CONF_SPI_LCD_FORMAT_Pos);           // SPI LCD

        if (HAL_LCDC_IS_DBI_IF(lcdc->Init.lcd_itf))
        {
            reg_v |= LCD_IF_LCD_CONF_LCD_FORMAT_RGB888_OVER8BUS;          // DBI
        }
#ifdef HAL_DSI_MODULE_ENABLED
        else if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
        {
            reg_v |= LCD_IF_LCD_CONF_LCD_FORMAT_RGB888_OVER16BUS;         // DBI
        }
#endif /* HAL_DSI_MODULE_ENABLED */
        else
        {
            reg_v |= LCD_IF_LCD_CONF_LCD_FORMAT_RGB888;               // DBI
        }
        break;

    case LCDC_PIXEL_FORMAT_RGB332:
        reg_v |= (3 << LCD_IF_LCD_CONF_AHB_FORMAT_Pos) |               // AHB LCD/RAM
                 (0 << LCD_IF_LCD_CONF_SPI_LCD_FORMAT_Pos);            // SPI LCD

        reg_v |= LCD_IF_LCD_CONF_LCD_FORMAT_RGB332;                   //DBI
        break;

    default:
        //unknow color format
        HAL_LCDC_ASSERT(0);
        break;
    }
    lcdc->Instance->LCD_CONF = reg_v;


#ifdef HAL_DSI_MODULE_ENABLED
    if (HAL_LCDC_IS_DSI_IF(init->lcd_itf))
    {
        DSI_SetOutFormat(lcdc, color_mode);
    }
#endif /* HAL_DSI_MODULE_ENABLED */

    return HAL_OK;
}


static HAL_StatusTypeDef SetupTE(LCDC_HandleTypeDef *lcdc)
{
    uint32_t vsyn_cycle, vsyn_us;
    LCDC_InitTypeDef *init;

    HAL_LCDC_ASSERT(lcdc);

    uint32_t lcdc_clk_MHz = HAL_RCC_GetHCLKFreq(GET_LCDC_SYSID(lcdc)) / 1000000;

    init = &lcdc->Init;


    lcdc->Instance->TE_CONF &= ~LCD_IF_TE_CONF_ENABLE;

    if (HAL_LCDC_IS_SPI_IF(lcdc->Init.lcd_itf))
    {

        vsyn_us = init->cfg.spi.vsyn_delay_us;

        switch (init->cfg.spi.syn_mode)
        {
        case HAL_LCDC_SYNC_VER:
        {
            lcdc->Instance->TE_CONF = LCD_IF_TE_CONF_ENABLE
                                      | MAKE_REG_VAL(0, LCD_IF_TE_CONF_MODE_Msk, LCD_IF_TE_CONF_MODE_Pos)
                                      | MAKE_REG_VAL(init->cfg.spi.vsyn_polarity, LCD_IF_TE_CONF_FMARK_POL_Msk, LCD_IF_TE_CONF_FMARK_POL_Pos)
                                      ;
        }
        break;

        case HAL_LCDC_SYNC_VERHOR:
        {
            lcdc->Instance->TE_CONF = LCD_IF_TE_CONF_ENABLE
                                      | MAKE_REG_VAL(init->cfg.spi.hsyn_num, LCD_IF_TE_CONF_VSYNC_DET_CNT_Msk, LCD_IF_TE_CONF_VSYNC_DET_CNT_Pos)
                                      | MAKE_REG_VAL(1, LCD_IF_TE_CONF_MODE_Msk, LCD_IF_TE_CONF_MODE_Pos)
                                      | MAKE_REG_VAL(init->cfg.spi.vsyn_polarity, LCD_IF_TE_CONF_FMARK_POL_Msk, LCD_IF_TE_CONF_FMARK_POL_Pos)
                                      ;
        }
        break;

        default:
            break;

        }
    }
#ifdef HAL_DSI_MODULE_ENABLED
    else if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {

#ifndef SF32LB55X
        if (DSI_TE_DSILINK == init->cfg.dsi.CmdCfg.TearingEffectSource)
            lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_DSI_LCD_SEL;  //DSI interface to receive TE form DSI bus
        else
            lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_DBI_LCD_SEL;  //DBI interface to recevie TE from gpio
#endif /* SF32LB55X */

        if (DSI_TE_ACKNOWLEDGE_ENABLE == init->cfg.dsi.CmdCfg.TEAcknowledgeRequest)
        {
            lcdc->Instance->TE_CONF = (1 << LCD_IF_TE_CONF_ENABLE_Pos) |
                                      (0 << LCD_IF_TE_CONF_FMARK_POL_Pos) |
                                      (0 << LCD_IF_TE_CONF_MODE_Pos) |
                                      (1 << LCD_IF_TE_CONF_FMARK_MODE_Pos) |
                                      (1 << LCD_IF_TE_CONF_FMARK_SOURCE_Pos) |
                                      (0 << LCD_IF_TE_CONF_VSYNC_DET_CNT_Pos);
        }
        else
        {
            ;
        }
        vsyn_us = init->cfg.dsi.vsyn_delay_us;


    }
#endif
    else if (HAL_LCDC_IS_DBI_IF(lcdc->Init.lcd_itf))
    {
        vsyn_us = init->cfg.dbi.vsyn_delay_us;

        switch (init->cfg.dbi.syn_mode)
        {
        case HAL_LCDC_SYNC_VER:
        {
            lcdc->Instance->TE_CONF = LCD_IF_TE_CONF_ENABLE
                                      | MAKE_REG_VAL(0, LCD_IF_TE_CONF_MODE_Msk, LCD_IF_TE_CONF_MODE_Pos)
                                      | MAKE_REG_VAL(init->cfg.dbi.vsyn_polarity, LCD_IF_TE_CONF_FMARK_POL_Msk, LCD_IF_TE_CONF_FMARK_POL_Pos)
                                      ;
        }
        break;

        case HAL_LCDC_SYNC_VERHOR:
        {
            lcdc->Instance->TE_CONF = LCD_IF_TE_CONF_ENABLE
                                      | MAKE_REG_VAL(init->cfg.dbi.hsyn_num, LCD_IF_TE_CONF_VSYNC_DET_CNT_Msk, LCD_IF_TE_CONF_VSYNC_DET_CNT_Pos)
                                      | MAKE_REG_VAL(1, LCD_IF_TE_CONF_MODE_Msk, LCD_IF_TE_CONF_MODE_Pos)
                                      | MAKE_REG_VAL(init->cfg.dbi.vsyn_polarity, LCD_IF_TE_CONF_FMARK_POL_Msk, LCD_IF_TE_CONF_FMARK_POL_Pos)
                                      ;
        }
        break;

        default:
            break;

        }

    }
    else
    {
        vsyn_us = 0;
    }

    vsyn_cycle = lcdc_clk_MHz * vsyn_us;
    lcdc->Instance->TE_CONF2 = MAKE_REG_VAL(vsyn_cycle, LCD_IF_TE_CONF2_DLY_CNT_Msk, LCD_IF_TE_CONF2_DLY_CNT_Pos);

#if defined(SF32LB58X) && defined(SOC_BF0_HCPU)
    if (lcdc->use_lcdc2_te)
    {
        //Use LCDC2's TE
        hwp_hpsys_cfg->SYSCR |= HPSYS_CFG_SYSCR_TE_SEL;
    }
#endif

    return HAL_OK;
}

static HAL_StatusTypeDef EnableInf(LCDC_HandleTypeDef *lcdc)
{
    HAL_StatusTypeDef ret = HAL_OK;
#ifdef HAL_DSI_MODULE_ENABLED
    if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
        ret = HAL_DSI_Start(&lcdc->hdsi);
    }
#endif /* HAL_DSI_MODULE_ENABLED */

    return ret;
}

#ifdef LCDC_SUPPORT_LINE_DONE_IRQ
#ifdef SF32LB55X
static void PTC_CH_SETUP(uint8_t ch, uint8_t trigger, uint32_t addr, uint8_t op, uint32_t data)
{
    PTC_HandleTypeDef    PtcHandle;

    PtcHandle.Instance = hwp_ptc1 ;
    PtcHandle.Init.Channel = ch - 1;
    PtcHandle.Init.Address = addr;
    PtcHandle.Init.data = data;
    PtcHandle.Init.Operation = op;
    PtcHandle.Init.Sel = trigger;
    if (HAL_PTC_Init(&PtcHandle) != HAL_OK)                     // Initialize PTC
    {
        /* Initialization Error */
        HAL_ASSERT(0);
    }
    HAL_PTC_Enable(&PtcHandle, 1);                              // Enable PTC
}

static void ClearLineIrq(LCDC_HandleTypeDef *lcdc)
{
    //stop PTC, clear busmon1/ptc1
    HAL_RCC_ResetModule(RCC_MOD_PTC1);
    hwp_busmon1->CCR |= BUSMON_CCR_CLR4;
    NVIC_DisableIRQ(PTC1_IRQn);
}

static void SetupLineIrq(LCDC_HandleTypeDef *lcdc)
{
    HAL_RCC_EnableModule(RCC_MOD_PTC1);
    HAL_RCC_EnableModule(RCC_MOD_BUSMON1);

    ClearLineIrq(lcdc);

    if ((lcdc->irq_lines > 0) && ((lcdc->roi.y0 + lcdc->irq_lines) < lcdc->roi.y1))
    {
        LCDC_AreaDef inter_area;

        LCDC_LayerCfgTypeDef *cfg = &(lcdc->Layer[HAL_LCDC_LAYER_DEFAULT]);

        if (area_intersect(&inter_area, &(lcdc->roi), &(cfg->data_area)))
        {
            uint32_t first_half_line_addr;//The address of first half line in intersect area
            uint32_t bytes_per_pixel, layer_1line_total_bytes;
            uint32_t irq_address_offset;



            //area base on default layer's TL
            LCDC_AreaDef clip_area =
            {
                inter_area.x0 - cfg->data_area.x0,
                inter_area.y0 - cfg->data_area.y0,
                inter_area.x1 - cfg->data_area.x0,
                inter_area.y1 - cfg->data_area.y0
            };
            LCDC_PRINT_AREA("\r\n>>>ROI:", &lcdc->roi);
            LCDC_PRINT_AREA("Layer", &cfg->data_area);
            LCDC_PRINT_AREA("inter_area", &inter_area);
            LCDC_PRINT_AREA("clip_area", &clip_area);

            bytes_per_pixel = HAL_LCDC_GetPixelSize(cfg->data_format);
            HAL_ASSERT(0 != bytes_per_pixel);

            if (INVALID_TOTAL_WIDTH == cfg->total_width)
                layer_1line_total_bytes = (cfg->data_area.x1 - cfg->data_area.x0 + 1) * bytes_per_pixel;
            else
                layer_1line_total_bytes = cfg->total_width * bytes_per_pixel;




            first_half_line_addr = ((uint32_t)cfg->data)
                                   + (layer_1line_total_bytes * clip_area.y0)
                                   + (((clip_area.x1 - clip_area.x0 + 1) / 2) * bytes_per_pixel);

            irq_address_offset = layer_1line_total_bytes * lcdc->irq_lines;

            LCDC_LOG_D("first_half_line_addr=%x, irq_address_offset=%x", first_half_line_addr, irq_address_offset);



            {

                hwp_busmon1->CR4 = (1 << BUSMON_CR4_CMP_Pos) | BUSMON_CR4_CMPEN | BUSMON_CR4_AUTOCLR | (0x0 << BUSMON_CR4_OP_Pos) | 3; //select LCDC1 read
                hwp_busmon1->MIN4 = first_half_line_addr + irq_address_offset;
                hwp_busmon1->MAX4 = first_half_line_addr + irq_address_offset + 4;
                hwp_busmon1->CCR |= BUSMON_CCR_CLR4;
                hwp_busmon1->CER |= BUSMON_CER_EN4; //enable channel 4

#define PTC_BUSMON1_OF4 43

                hwp_ptc1->MEM4 = 0;
                PTC_CH_SETUP(6, PTC_BUSMON1_OF4, (uint32_t) & (hwp_ptc1->MEM4),      PTC_OP_ADD,  lcdc->irq_lines);
                PTC_CH_SETUP(7, PTC_BUSMON1_OF4, (uint32_t) & (hwp_busmon1->MIN4),   PTC_OP_ADD,  irq_address_offset);
                PTC_CH_SETUP(8, PTC_BUSMON1_OF4, (uint32_t) & (hwp_busmon1->MAX4),   PTC_OP_ADD,  irq_address_offset);

                NVIC_EnableIRQ(PTC1_IRQn);
            }
        }
    }


}


void HAL_LCD_LineIRQHandler(LCDC_HandleTypeDef *lcdc)
{
    uint32_t v = hwp_ptc1->MEM4;
    hwp_ptc1->ICR = 0xFF;

    LCDC_LOG_D("hwp_ptc1->MEM4 = %d", v);

    if (lcdc->XferLineCallback)
    {
        lcdc->XferLineCallback(lcdc, v);
    }

    HAL_LCDC_SendLineCpltCbk(lcdc, v);
}

#else /* !SF32LB55X*/

static void SetupLineIrq(LCDC_HandleTypeDef *lcdc)
{
    __IO uint32_t setting_reg_v = lcdc->Instance->SETTING;
    setting_reg_v &= ~(LCD_IF_SETTING_LINE_DONE_NUM_Msk | LCD_IF_SETTING_LINE_DONE_MASK); //Clear irq defaultly.

    if (lcdc->irq_lines > 0)
    {
        uint32_t roi_top = GET_REG_VAL(lcdc->Instance->CANVAS_TL_POS,  LCD_IF_CANVAS_TL_POS_Y0_Msk, LCD_IF_CANVAS_TL_POS_Y0_Pos);
        uint32_t roi_bottom = GET_REG_VAL(lcdc->Instance->CANVAS_BR_POS,  LCD_IF_CANVAS_BR_POS_Y1_Msk, LCD_IF_CANVAS_BR_POS_Y1_Pos);

        uint32_t v = roi_top + lcdc->irq_lines;

        if (v <= roi_bottom)
        {
            setting_reg_v |= LCD_IF_SETTING_LINE_DONE_MASK | MAKE_REG_VAL(v, LCD_IF_SETTING_LINE_DONE_NUM_Msk, LCD_IF_SETTING_LINE_DONE_NUM_Pos);
        }
    }

    lcdc->Instance->SETTING = setting_reg_v;
    lcdc->Instance->IRQ = LCD_IF_IRQ_LINE_DONE_STAT_Msk | LCD_IF_IRQ_LINE_DONE_RAW_STAT_Msk;

}

static HAL_StatusTypeDef SetupNextLineIrq(LCDC_HandleTypeDef *lcdc)
{
    if (lcdc->irq_lines > 0)
    {
        uint32_t roi_bottom = GET_REG_VAL(lcdc->Instance->CANVAS_BR_POS,  LCD_IF_CANVAS_BR_POS_Y1_Msk, LCD_IF_CANVAS_BR_POS_Y1_Pos);
        uint32_t roi_cur;

        __IO uint32_t setting_reg_v = lcdc->Instance->SETTING;
        uint32_t v = GET_REG_VAL(setting_reg_v, LCD_IF_SETTING_LINE_DONE_NUM_Msk, LCD_IF_SETTING_LINE_DONE_NUM_Pos);

        do
        {
            v += lcdc->irq_lines;

            if (v > roi_bottom)
            {
                return HAL_ERROR;
            }

            setting_reg_v &= ~LCD_IF_SETTING_LINE_DONE_NUM_Msk;
            setting_reg_v |= LCD_IF_SETTING_LINE_DONE_MASK | MAKE_REG_VAL(v, LCD_IF_SETTING_LINE_DONE_NUM_Msk, LCD_IF_SETTING_LINE_DONE_NUM_Pos);
            lcdc->Instance->SETTING = setting_reg_v;

            roi_cur = GET_REG_VAL(lcdc->Instance->CANVAS_STAT0, LCD_IF_CANVAS_STAT0_Y_COR_Msk, LCD_IF_CANVAS_STAT0_Y_COR_Pos);
            if (v > roi_cur)
            {
                break;
            }
            else
            {
                v = roi_cur;
                //Clear irq if line num already reached.
                lcdc->Instance->IRQ = LCD_IF_IRQ_LINE_DONE_STAT_Msk | LCD_IF_IRQ_LINE_DONE_RAW_STAT_Msk;
            }
        }
        while (1);

        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

#endif /* SF32LB55X */
#endif /* LCDC_SUPPORT_LINE_DONE_IRQ */

static HAL_StatusTypeDef SendSingleCmd(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len)
{
    if (0 == addr_len) return HAL_OK;

    HAL_LCDC_ASSERT((addr_len > 0) && (addr_len <= 4));

    if (HAL_LCDC_IS_SPI_IF(lcdc->Init.lcd_itf))
    {
        uint32_t config;
        uint32_t byte_gap = lcdc->Init.cfg.spi.bytes_gap_us;

        WaitBusy(lcdc);
        config = lcdc->Instance->SPI_IF_CONF;
        config &= ~(LCD_IF_SPI_IF_CONF_RD_LEN_Msk | LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk | LCD_IF_SPI_IF_CONF_WR_LEN_Msk);

        if (byte_gap > 0)
        {
            config |= MAKE_REG_VAL(0, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Pos)
                      | MAKE_REG_VAL(0, LCD_IF_SPI_IF_CONF_WR_LEN_Msk, LCD_IF_SPI_IF_CONF_WR_LEN_Pos);

            while (addr_len > 0)
            {
                WAIT_LCDC_SINGLE_BUSY(lcdc);
                lcdc->Instance->LCD_WR = (addr >> ((addr_len - 1) << 3)) & 0xFF;
                lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG;

                if (byte_gap > 1) HAL_Delay_us(byte_gap);

                addr_len--;
            }
        }
        else
        {
            config |= MAKE_REG_VAL(0, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Pos)
                      | MAKE_REG_VAL((addr_len - 1), LCD_IF_SPI_IF_CONF_WR_LEN_Msk, LCD_IF_SPI_IF_CONF_WR_LEN_Pos);


            lcdc->Instance->SPI_IF_CONF = config;
            lcdc->Instance->LCD_WR = (addr);
            lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG;
        }
    }
    else if (HAL_LCDC_IS_DBI_IF(lcdc->Init.lcd_itf) || HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
        do
        {
            WaitBusy(lcdc);
            lcdc->Instance->LCD_WR = (addr >> ((addr_len - 1) << 3)) & 0xFF;
            lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG;

            addr_len--;
        }
        while (addr_len > 0);
    }


    return HAL_OK;

}


static HAL_StatusTypeDef WaitBusy(LCDC_HandleTypeDef *lcdc)
{
    const uint32_t timeout_ms = LCDC_TIMEOUT_SECONDS * 1000;

    uint32_t start_tick = HAL_GetTick();

    if (HAL_LCDC_IS_DPI_IF(lcdc->Init.lcd_itf))   return HAL_OK;

    do
    {
        if ((lcdc->Instance->STATUS & LCD_IF_STATUS_LCD_BUSY) || (lcdc->Instance->LCD_SINGLE & LCD_IF_LCD_SINGLE_LCD_BUSY)) continue;

#ifdef HAL_DSI_MODULE_ENABLED
        if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
        {
#ifndef HAL_USING_HTOL
            if (HAL_DSI_IsBusy(&lcdc->hdsi)) continue;
#endif /* HAL_USING_HTOL */
        }
#endif /* HAL_DSI_MODULE_ENABLED */

        return HAL_OK;
    }
    while (HAL_GetTick() - start_tick < timeout_ms);

    lcdc->ErrorCode |= HAL_LCDC_ERROR_TIMEOUT;
    LCDC_LOG_E("WaitBusy Timeout");
    //HAL_LCDC_ASSERT(0);

    return HAL_TIMEOUT;
}




static HAL_StatusTypeDef LayerUpdate(LCDC_HandleTypeDef *lcdc)
{
    LCDC_LayerCfgTypeDef *cfg;
    uint32_t bytes_per_pixel, data_w, data_h, roi_w, roi_h;
    uint32_t reg, layer_1line_total_bytes;

    if (NULL == lcdc)
    {
        return HAL_ERROR;
    }

    WaitBusy(lcdc);


    /*** 1. setup canvas info ***/
    roi_w   = lcdc->roi.x1  - lcdc->roi.x0  + 1;
    roi_h   = lcdc->roi.y1  - lcdc->roi.y0  + 1;
    lcdc->Instance->CANVAS_BG = (lcdc->bg.r << LCD_IF_CANVAS_BG_RED_Pos) | (lcdc->bg.g << LCD_IF_CANVAS_BG_GREEN_Pos) | (lcdc->bg.b << LCD_IF_CANVAS_BG_BLUE_Pos);

    if (LCDC_IS_TLBR_OVERFLOW(LCD_IF_CANVAS, lcdc->roi.x0, lcdc->roi.y0, lcdc->roi.x1, lcdc->roi.y1))
    {
        HAL_LCDC_ASSERT(0);
        return HAL_ERROR;
    }

    if (HAL_LCDC_IS_JDI_PARALLEL_IF(lcdc->Init.lcd_itf))
    {
        // canvas area
        lcdc->Instance->CANVAS_TL_POS = (lcdc->roi.x0 << LCD_IF_CANVAS_TL_POS_X0_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_TL_POS_Y0_Pos);
        lcdc->Instance->CANVAS_BR_POS = (lcdc->roi.x1 << LCD_IF_CANVAS_BR_POS_X1_Pos)
                                        | ((lcdc->roi.y0 + (roi_h * 2) - 1) << LCD_IF_CANVAS_BR_POS_Y1_Pos);
    }
    else
    {
        // canvas area
        lcdc->Instance->CANVAS_TL_POS = (lcdc->roi.x0 << LCD_IF_CANVAS_TL_POS_X0_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_TL_POS_Y0_Pos);
        lcdc->Instance->CANVAS_BR_POS = (lcdc->roi.x1 << LCD_IF_CANVAS_BR_POS_X1_Pos) | (lcdc->roi.y1 << LCD_IF_CANVAS_BR_POS_Y1_Pos);
    }



    /*** 2. setup layer ***/
    for (HAL_LCDC_LayerDef layeridx = HAL_LCDC_LAYER_0; layeridx < HAL_LCDC_LAYER_MAX; layeridx++)
    {
#ifndef SF32LB55X
        uint32_t swap_endian = 0;
#endif /* !SF32LB55X */

        LCDC_LayerxTypeDef *pHwLayerx = (LCDC_LayerxTypeDef *)((layeridx == HAL_LCDC_LAYER_0) ? (&(lcdc->Instance->LAYER0_CONFIG)) : (&(lcdc->Instance->LAYER1_CONFIG)));
        cfg = &(lcdc->Layer[layeridx]);

        if (cfg->disable)
        {
            pHwLayerx->CONFIG &= ~LCD_IF_LAYER0_CONFIG_ACTIVE;
            continue;
        }
#ifdef SF32LB52X
        HAL_ASSERT(HAL_LCDC_LAYER_0 == layeridx);//Only layer 0 is present on 54x
#endif /* SF32LB52X */
        data_w = cfg->data_area.x1 - cfg->data_area.x0 + 1;
        data_h = cfg->data_area.y1 - cfg->data_area.y0 + 1;
        bytes_per_pixel = HAL_LCDC_GetPixelSize(cfg->data_format);
        HAL_ASSERT(0 != bytes_per_pixel);

        //a.1 Layer format
        reg = 0;
        switch (cfg->data_format)
        {
        case LCDC_PIXEL_FORMAT_RGB565:
            reg |= LCD_IF_LAYER0_CONFIG_FORMAT_RGB565;
            break;

        case LCDC_PIXEL_FORMAT_RGB888:
            reg |= LCD_IF_LAYER0_CONFIG_FORMAT_RGB888;
            break;
        case LCDC_PIXEL_FORMAT_ARGB565:
            reg |= LCD_IF_LAYER0_CONFIG_FORMAT_ARGB8565;
            break;
        case LCDC_PIXEL_FORMAT_ARGB888:
            reg |= LCD_IF_LAYER0_CONFIG_FORMAT_ARGB8888;
            break;

#ifndef SF32LB55X
        case LCDC_PIXEL_FORMAT_A8:
            reg |= LCD_IF_LAYER0_CONFIG_FORMAT_A8;
            break;

#ifndef SF32LB52X //Not support L8 on 54x
        case LCDC_PIXEL_FORMAT_L8:
            reg |= LCD_IF_LAYER0_CONFIG_FORMAT_L8;
            break;
#endif /* SF32LB52X */

        case LCDC_PIXEL_FORMAT_RGB565_SWAP:
            reg |= LCD_IF_LAYER0_CONFIG_FORMAT_RGB565;
            /*Set endian below*/
            swap_endian = LCD_IF_LAYER0_FILL_ENDIAN;
            break;
#endif /* !SF32LB55X */

        default:
            HAL_ASSERT(0);
            break;
        }


        //a.2 Layer alpha,chroma,
        reg |= (cfg->layer_alpha_en   << LCD_IF_LAYER0_CONFIG_ALPHA_SEL_Pos) |     // use layer alpha
               (cfg->alpha            << LCD_IF_LAYER0_CONFIG_ALPHA_Pos) |         // layer alpha value is 255
               (cfg->chroma_key_en    << LCD_IF_LAYER0_CONFIG_FILTER_EN_Pos) |     // disable filter
               (1                     << LCD_IF_LAYER0_CONFIG_ACTIVE_Pos) |        // Enable layer
               (1                     << LCD_IF_LAYER0_CONFIG_PREFETCH_EN_Pos);    // prefetch enable


        //a.3 line fetch mode & h,v mirror
        if (HAL_LCDC_IS_JDI_PARALLEL_IF(lcdc->Init.lcd_itf))
            reg |= LCD_IF_LAYER0_CONFIG_LINE_FETCH_MODE;
        else
            reg &= ~LCD_IF_LAYER0_CONFIG_LINE_FETCH_MODE;

#ifndef SF32LB55X
        if (cfg->v_mirror) reg |= LCD_IF_LAYER0_CONFIG_V_MIRROR;

        if ((cfg->h_mirror) && (HAL_LCDC_LAYER_0 == layeridx))
        {
#ifdef SF32LB58X
            reg |= LCD_IF_LAYER0_CONFIG_H_MIRROR;
#elif defined(SF32LB56X)
            lcdc->Instance->CANVAS_BG |= LCD_IF_CANVAS_BG_H_MIRROR;
#else
            //54x NOT support H mirror
#endif /* SF32LB58X */
        }
#endif /* SF32LB55X */


        //a.4 Layer width
        reg &= ~LCD_IF_LAYER0_CONFIG_WIDTH_Msk;

        if (INVALID_TOTAL_WIDTH == cfg->total_width)
            layer_1line_total_bytes = data_w * bytes_per_pixel;
        else
            layer_1line_total_bytes = cfg->total_width * bytes_per_pixel;


        // a.5 setup compressed buffer info
        if (HAL_LCDC_LAYER_0 == layeridx)
        {
#ifdef SF32LB58X
            lcdc->Instance->LAYER0_SIZE = (data_h << LCD_IF_LAYER0_SIZE_MAX_LINE_Pos) | (data_w << LCD_IF_LAYER0_SIZE_MAX_COL_Pos);
#endif /* SF32LB58X */

#ifdef LCDC_SUPPORTED_COMPRESSED_LAYER
            if (cfg->cmpr_en)
            {
                uint32_t cfg0;
                uint32_t cfg1;
                uint32_t compressed_size;
                uint32_t target_size;

                HAL_EXT_DMA_CalcCompressedSize(data_w * 1 * bytes_per_pixel / 4, cfg->cmpr_rate,
                                               1, bytes_per_pixel, &compressed_size, &target_size);
                lcdc->Instance->LAYER0_DECOMP = LCD_IF_LAYER0_DECOMP_ENABLE |
                                                MAKE_REG_VAL(target_size, LCD_IF_LAYER0_DECOMP_TARGET_WORDS_Msk, LCD_IF_LAYER0_DECOMP_TARGET_WORDS_Pos) |
                                                MAKE_REG_VAL(data_w, LCD_IF_LAYER0_DECOMP_COL_SIZE_Msk, LCD_IF_LAYER0_DECOMP_COL_SIZE_Pos);


                layer_1line_total_bytes = compressed_size * 4 / 1;

                // set decompression
                HAL_EXT_DMA_GetConfig(target_size, &cfg0, &cfg1);
                lcdc->Instance->LAYER0_DECOMP_CFG1 = cfg0;
                lcdc->Instance->LAYER0_DECOMP_CFG0 = cfg1;

            }
            else
#endif /* LCDC_SUPPORTED_COMPRESSED_LAYER */
            {
                lcdc->Instance->LAYER0_DECOMP = 0;
            }
        }


        reg |= (layer_1line_total_bytes << LCD_IF_LAYER0_CONFIG_WIDTH_Pos); // layer line width
        pHwLayerx->CONFIG = reg;


        // b. Layer area
        if (LCDC_IS_TLBR_OVERFLOW(LCD_IF_LAYER0, cfg->data_area.x0, cfg->data_area.y0, cfg->data_area.x1, cfg->data_area.y1))
        {
            HAL_ASSERT(0);
            return HAL_ERROR;
        }
        if (HAL_LCDC_IS_JDI_PARALLEL_IF(lcdc->Init.lcd_itf))
        {
            pHwLayerx->TL_POS = (cfg->data_area.x0 << LCD_IF_LAYER0_TL_POS_X0_Pos) | (cfg->data_area.y0 << LCD_IF_LAYER0_TL_POS_Y0_Pos);
            pHwLayerx->BR_POS = (cfg->data_area.x1 << LCD_IF_LAYER0_BR_POS_X1_Pos)
                                | ((cfg->data_area.y0 + (data_h * 2) - 1) << LCD_IF_LAYER0_BR_POS_Y1_Pos);

        }
        else
        {
            pHwLayerx->TL_POS = (cfg->data_area.x0 << LCD_IF_LAYER0_TL_POS_X0_Pos) | (cfg->data_area.y0 << LCD_IF_LAYER0_TL_POS_Y0_Pos);
            pHwLayerx->BR_POS = (cfg->data_area.x1 << LCD_IF_LAYER0_BR_POS_X1_Pos) | (cfg->data_area.y1 << LCD_IF_LAYER0_BR_POS_Y1_Pos);
        }


        // c. Layer src
#ifndef SF32LB55X
        if (cfg->v_mirror)
        {
#if 1
            //Set src to the head of last line
            reg = ((uint32_t)cfg->data) + (layer_1line_total_bytes * (cfg->data_area.y1 - cfg->data_area.y0));// + (off_x*bytes_per_pixel);
#else
            LCDC_AreaDef inter_area;

            if (1)
            {
            }
            else if (0)//area_intersect(&inter_area, &cfg->data_area, &lcdc->roi))
            {
                uint32_t off_x, off_y;

                LCDC_LOG_E("\r\n>>>ROI:%d,%d,%d,%d ", lcdc->roi.x0, lcdc->roi.x1, lcdc->roi.y0, lcdc->roi.y1);
                LCDC_LOG_E("Layer:%d,%d,%d,%d ", cfg->data_area.x0, cfg->data_area.x1, cfg->data_area.y0, cfg->data_area.y1);
                LCDC_LOG_E("inter_area:%d,%d,%d,%d ", inter_area.x0, inter_area.x1, inter_area.y0, inter_area.y1);


                off_x = inter_area.x0 - cfg->data_area.x0;
                off_y = (inter_area.y1 - cfg->data_area.y0) + (inter_area.y0 - cfg->data_area.y0);

                //Set src to the head of last line of intersect area.
                reg = ((uint32_t)cfg->data) + (layer_1line_total_bytes * off_y) + (off_x * bytes_per_pixel);

                LCDC_LOG_E("1line_bytes:%d,offset:%d", layer_1line_total_bytes, reg - ((uint32_t)cfg->data));
            }
            else
            {
                reg = (uint32_t)cfg->data;
            }
#endif
        }
        else
#endif /* SF32LB55X */
        {
            reg = (uint32_t)cfg->data;
        }

        mpu_dcache_clean((uint32_t *)cfg->data, layer_1line_total_bytes * (cfg->data_area.y1 - cfg->data_area.y0 + 1));

#ifndef SF32LB55X
        pHwLayerx->SRC = (HCPU_MPI_SBUS_ADDR(reg)) << LCD_IF_LAYER0_SRC_ADDR_Pos;
#else
        pHwLayerx->SRC = (reg) << LCD_IF_LAYER0_SRC_ADDR_Pos;
#endif /* SF32LB55X */



        //d. Layer chroma key
        pHwLayerx->FILTER = (cfg->chroma_key.r << LCD_IF_LAYER0_FILTER_FILTER_R_Pos)
                            | (cfg->chroma_key.g << LCD_IF_LAYER0_FILTER_FILTER_G_Pos)
                            | (cfg->chroma_key.b << LCD_IF_LAYER0_FILTER_FILTER_B_Pos);

#ifndef SF32LB55X
        pHwLayerx->FILL = (cfg->bg_color.r << LCD_IF_LAYER0_FILL_BG_R_Pos)
                          | (cfg->bg_color.g << LCD_IF_LAYER0_FILL_BG_G_Pos)
                          | (cfg->bg_color.b << LCD_IF_LAYER0_FILL_BG_B_Pos)
                          | swap_endian;
        //if ((HAL_LCDC_IS_JDI_PARALLEL_IF(lcdc->Init.lcd_itf))&&(HAL_LCDC_LAYER_0 == layeridx))
        //pHwLayerx->FILL |= LCD_IF_LAYER0_FILL_BG_MODE; //Skip line buffer
#endif /* SF32LB55X */
    }

    return HAL_OK;
}

#ifdef LCDC_SUPPORT_DDR_QSPI
static HAL_StatusTypeDef EnableSPIDDRSent(LCDC_HandleTypeDef *lcdc)
{
    uint32_t reg_v, clk_div;

    lcdc->Instance->LAYER0_FILL |= LCD_IF_LAYER0_FILL_BG_MODE;//Enable DDR

    reg_v = lcdc->Instance->SPI_IF_CONF;
    clk_div = GET_REG_VAL(reg_v, LCD_IF_SPI_IF_CONF_CLK_DIV_Msk, LCD_IF_SPI_IF_CONF_CLK_DIV_Pos);
    clk_div = HAL_MAX(clk_div >> 1, 2);
    reg_v &= ~LCD_IF_SPI_IF_CONF_CLK_DIV_Msk;
    reg_v |= MAKE_REG_VAL(clk_div, LCD_IF_SPI_IF_CONF_CLK_DIV_Msk, LCD_IF_SPI_IF_CONF_CLK_DIV_Pos);
    lcdc->Instance->SPI_IF_CONF = reg_v;


    return HAL_OK;
}

static HAL_StatusTypeDef DisableSPIDDRSent(LCDC_HandleTypeDef *lcdc)
{
    lcdc->Instance->LAYER0_FILL &= ~LCD_IF_LAYER0_FILL_BG_MODE; //Disable DDR

    if (SPI_LCD_FLAG_DDR_DUMMY_CLOCK & lcdc->Init.cfg.spi.flags)
    {
        //Dummy clock
        lcdc->Instance->SPI_IF_CONF |= LCD_IF_SPI_IF_CONF_SPI_CLK_POL;
        LCDC_DELAY_NS(20);
        lcdc->Instance->SPI_IF_CONF &= ~LCD_IF_SPI_IF_CONF_SPI_CLK_POL;
        LCDC_DELAY_NS(20);

        //Reset LCDC to restore CS
        lcdc->Instance->SPI_IF_CONF |= LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS;
        lcdc->Instance->COMMAND |= LCD_IF_COMMAND_RESET;
        lcdc->Instance->COMMAND &= ~LCD_IF_COMMAND_RESET;
    }

    return SetFreq(lcdc, lcdc->Init.freq); //Restore freq
}
#endif /* LCDC_SUPPORT_DDR_QSPI */




// async_mode = 0, synchronized mode;    async_mode=1, asynchronized.
static HAL_StatusTypeDef _SendLayerData(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef async_mode)
{
    HAL_StatusTypeDef err;

    if (HAL_TIMEOUT == WaitBusy(lcdc))
    {
        return HAL_TIMEOUT;
    }


    //1. Update layer
    err = LayerUpdate(lcdc);

    if (HAL_OK != err)
    {
        return err;
    }



#if 0 //For debug, send random background color
    {
        extern int rand(void);

        lcdc->Instance->LAYER0_CONFIG &= ~LCD_IF_LAYER0_CONFIG_ACTIVE_Msk;
        lcdc->Instance->LAYER1_CONFIG &= ~LCD_IF_LAYER1_CONFIG_ACTIVE_Msk;
        lcdc->Instance->CANVAS_BG = (uint32_t)rand();
    }
#endif

#if 0//!(defined(SF32LB55X) || defined(SF32LB58X))
    if ((LCDC_PIXEL_FORMAT_RGB888 == lcdc->Layer[0].data_format) && (LCDC_PIXEL_FORMAT_RGB565 == lcdc->Init.color_mode))
    {
        lcdc->Instance->DITHER_CONF = (4 << LCD_IF_DITHER_CONF_W_R_Pos)
                                      | (3 << LCD_IF_DITHER_CONF_W_G_Pos)
                                      | (4 << LCD_IF_DITHER_CONF_W_B_Pos)
                                      | LCD_IF_DITHER_CONF_EN;

    }
#endif /* ! (SF32LB55X|| SF32LB58X) */

    g_LCDC_CpltCallback = LCDC_TransCpltCallback;


    //2. Send layer data
    if (0)
    {
    }
    else if (HAL_LCDC_IS_JDI_PARALLEL_IF(lcdc->Init.lcd_itf))
    {

        uint32_t max_col, max_line, start_line, end_line, start_col, end_col;


        max_col = (lcdc->Init.cfg.jdi.bank_col_head + lcdc->Init.cfg.jdi.valid_columns + lcdc->Init.cfg.jdi.bank_col_tail) / 2;
        max_line = (lcdc->Init.cfg.jdi.bank_row_head + lcdc->Init.cfg.jdi.valid_rows + lcdc->Init.cfg.jdi.bank_row_tail) * 2;


        start_line = (lcdc->Init.cfg.jdi.bank_row_head + lcdc->roi.y0) * 2 + 1;
        end_line   = (lcdc->Init.cfg.jdi.bank_row_head + lcdc->roi.y1 + 1) * 2;

        start_col = (lcdc->Init.cfg.jdi.bank_col_head + lcdc->roi.x0) / 2;
        end_col   = (lcdc->Init.cfg.jdi.bank_col_head + lcdc->roi.x1 + 1) / 2  - 1;


        lcdc->Instance->JDI_PAR_CONF2 = (start_line << LCD_IF_JDI_PAR_CONF2_ST_LINE_Pos) | (end_line << LCD_IF_JDI_PAR_CONF2_END_LINE_Pos);
        lcdc->Instance->JDI_PAR_CONF9 = ((start_line + 1) << LCD_IF_JDI_PAR_CONF9_ENB_ST_LINE_Pos) | ((end_line + 1) << LCD_IF_JDI_PAR_CONF9_ENB_END_LINE_Pos);
        lcdc->Instance->JDI_PAR_CONF3 = (start_col << LCD_IF_JDI_PAR_CONF3_ST_COL_Pos)   | (end_col << LCD_IF_JDI_PAR_CONF3_END_COL_Pos);
#ifndef SF32LB55X
        lcdc->Instance->JDI_PAR_CONF10 = ((start_line) << LCD_IF_JDI_PAR_CONF10_HC_ST_LINE_Pos) | ((end_line + 2) << LCD_IF_JDI_PAR_CONF10_HC_END_LINE_Pos);
#endif /* SF32LB55X */

        /* Interrupt after send half of 'max_line' data */
        lcdc->Instance->JDI_PAR_CTRL &= ~LCD_IF_JDI_PAR_CTRL_INT_LINE_NUM_Msk;
        lcdc->Instance->JDI_PAR_CTRL |= ((max_line / 2) - 1) << LCD_IF_JDI_PAR_CTRL_INT_LINE_NUM_Pos;


        if (lcdc->Instance->JDI_PAR_EX_CTRL & LCD_IF_JDI_PAR_EX_CTRL_CNT_EN)
        {
            //wait fall edge
            while (0 == (lcdc->Instance->JDI_PAR_EX_CTRL & LCD_IF_JDI_PAR_EX_CTRL_XFRP));
            while (0 != (lcdc->Instance->JDI_PAR_EX_CTRL & LCD_IF_JDI_PAR_EX_CTRL_XFRP));
        }

        lcdc->debug_cnt1++;

        lcdc->Instance->SETTING |= LCD_IF_SETTING_JDI_PARL_INTR_MASK | LCD_IF_SETTING_EOF_MASK;
        //Pull up rst
        lcdc->Instance->JDI_PAR_CTRL |= LCD_IF_JDI_PAR_CTRL_XRST;
        HAL_Delay_us(35);
        lcdc->Instance->JDI_PAR_CTRL |=  LCD_IF_JDI_PAR_CTRL_ENABLE; //send data
        HAL_Delay_us(1); //Wait digital start send data



    }
    else if (HAL_LCDC_IS_DPI_IF(lcdc->Init.lcd_itf) || HAL_LCDC_IS_DSI_VID_IF(lcdc->Init.lcd_itf))
    {
        if (LCDC_ASYNC_MODE == async_mode)
        {
            lcdc->debug_cnt1++;
            /* unmask */
            lcdc->Instance->SETTING |= LCD_IF_SETTING_DPIL_INTR_MASK | LCD_IF_SETTING_DPI_UDR_MASK | LCD_IF_SETTING_EOF_MASK;
            /* trigger lcdc */
            lcdc->Instance->DPI_CTRL = LCD_IF_DPI_CTRL_DPI_EN;
        }
        else
        {
            lcdc->Instance->DPI_CTRL = LCD_IF_DPI_CTRL_DPI_EN;
        }
    }
    else
    {

#ifdef LCDC_SUPPORT_DDR_QSPI
        if (LCDC_INTF_SPI_DCX_DDR_4DATA == lcdc->Init.lcd_itf)
        {
            EnableSPIDDRSent(lcdc);
        }
#endif /* LCDC_SUPPORT_DDR_QSPI */

        if (LCDC_ASYNC_MODE == async_mode)
        {
            lcdc->debug_cnt1++;
            /* unmask */
            lcdc->Instance->SETTING |= LCD_IF_SETTING_EOF_MASK;
#ifdef LCDC_SUPPORT_LINE_DONE_IRQ
            SetupLineIrq(lcdc);
#endif /* LCDC_SUPPORT_LINE_DONE_IRQ */
            /* trigger lcdc */
            lcdc->Instance->COMMAND = LCD_IF_COMMAND_START;
        }
        else
        {
            lcdc->Instance->COMMAND = LCD_IF_COMMAND_START;
        }
    }



    return HAL_OK;
}


static HAL_StatusTypeDef _WaitSendLayerDone(LCDC_HandleTypeDef *lcdc)
{
    if (0)
    {
    }
#ifndef SOC_BF_Z0
    else if (HAL_LCDC_IS_DPI_IF(lcdc->Init.lcd_itf) || HAL_LCDC_IS_DSI_VID_IF(lcdc->Init.lcd_itf))
    {
        while (0 == (lcdc->Instance->IRQ & LCD_IF_IRQ_DPIL_INTR_RAW_STAT_Msk));

        //Clear EOF IRQ.
        lcdc->Instance->IRQ = LCD_IF_IRQ_DPIL_INTR_RAW_STAT;
    }
#endif
    else
    {
        while (0 == (lcdc->Instance->IRQ & LCD_IF_IRQ_EOF_RAW_STAT));

        //Clear EOF IRQ.
        lcdc->Instance->IRQ = LCD_IF_IRQ_EOF_RAW_STAT;
    }

#ifdef LCDC_SUPPORT_DDR_QSPI
    if (LCDC_INTF_SPI_DCX_DDR_4DATA == lcdc->Init.lcd_itf)
    {
        DisableSPIDDRSent(lcdc);
    }
#endif /* LCDC_SUPPORT_DDR_QSPI */


    return HAL_OK;
}

static HAL_StatusTypeDef SendLayerData(LCDC_HandleTypeDef *lcdc)
{
    HAL_StatusTypeDef err;

    err = _SendLayerData(lcdc, LCDC_SYNC_MODE);

    if (HAL_OK == err) err = _WaitSendLayerDone(lcdc);

    return err;
}

#if 0//def SOC_BF0_HCPU

uint8_t ram_lcd_buf[454 * 454 * 3];
volatile uint32_t halt_lcdc = 0;
static void LCDC_OutputToRAM(LCDC_HandleTypeDef *lcdc)
{
    LCDC_InitTypeDef dsi_lcdc_int_cfg;
    LCDC_InitTypeDef ram_lcdc_int_cfg;

    memcpy(&dsi_lcdc_int_cfg, &lcdc->Init, sizeof(LCDC_InitTypeDef));


    ram_lcdc_int_cfg.lcd_itf = LCDC_INTF_AHB;
    ram_lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
    ram_lcdc_int_cfg.cfg.ahb.lcd_mem = (uint32_t) &ram_lcd_buf[(lcdc->roi.x0 + lcdc->roi.y0 * 454) * 3];
    ram_lcdc_int_cfg.cfg.ahb.lcd_o_width = (454 - (lcdc->roi.x1 - lcdc->roi.x0 + 1)) * 3;

    memcpy(&lcdc->Init, &ram_lcdc_int_cfg, sizeof(LCDC_InitTypeDef));

    //SelectIntf(lcdc,LCDC_INTF_AHB);
    lcdc->Instance->LCD_CONF &= ~(LCD_IF_LCD_CONF_LCD_INTF_SEL_Msk | LCD_IF_LCD_CONF_TARGET_LCD_Msk);
    lcdc->Instance->LCD_CONF |= (3 << LCD_IF_LCD_CONF_TARGET_LCD_Pos);  //set AHB RAM
    lcdc->Instance->LCD_MEM = lcdc->Init.cfg.ahb.lcd_mem;
    lcdc->Instance->LCD_O_WIDTH = lcdc->Init.cfg.ahb.lcd_o_width;
    SetOutFormat(lcdc, lcdc->Init.color_mode);

    SendLayerData(lcdc);
    while (halt_lcdc) {;};


    memcpy(&lcdc->Init, &dsi_lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    //SelectIntf(lcdc,LCDC_INTF_DSI);

    lcdc->Instance->LCD_CONF &= ~(LCD_IF_LCD_CONF_LCD_INTF_SEL_Msk | LCD_IF_LCD_CONF_TARGET_LCD_Msk);
#ifndef DSI_DIRECT_SEND_MODE
    lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_DSI_LCD_SEL;  //set DSI
#else
    lcdc->Instance->LCD_CONF |= LCD_IF_LCD_CONF_DIRECT_INTF_EN  //set Direct INTF
                                | (2 << LCD_IF_LCD_CONF_TARGET_LCD_Pos); //set AHB LCD
#endif /* SF32LB55X */
    SetOutFormat(lcdc, lcdc->Init.color_mode);


    while (halt_lcdc) {;};
}
#endif /* SOC_BF0_HCPU */
static void LCDC_TransCpltCallback(LCDC_HandleTypeDef *lcdc)
{

#ifdef HAL_DSI_MODULE_ENABLED
    if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
        if (DSIMultiWriteContinue_IT(lcdc))  return; //DSI not finished
    }
#endif /* HAL_DSI_MODULE_ENABLED */


#if 0//def SOC_BF0_HCPU
    if (halt_lcdc)
        LCDC_OutputToRAM(lcdc);
#endif /* SOC_BF0_HCPU */


    lcdc->debug_cnt2++;

    lcdc->State = HAL_LCDC_STATE_READY;

    //MODIFY_REG(lcdc->Instance->GD_LCD_CTRL, GFX_GD_LCD_CTRL_HIGH_BYTE_Msk, 0);
    if (lcdc->XferCpltCallback)
    {
        lcdc->XferCpltCallback(lcdc);
    }

    HAL_LCDC_SendLayerDataCpltCbk(lcdc);
}

static void LCDC_TransErrCallback(LCDC_HandleTypeDef *lcdc, HAL_StatusTypeDef err)
{
    UNUSED(err);
    if (lcdc->XferErrorCallback)
    {
        lcdc->XferErrorCallback(lcdc);
    }
}

static void HAL_LCDC_JDIParallelInit(LCDC_HandleTypeDef *lcdc)
{
    uint32_t lcdc_clk_MHz = HAL_RCC_GetHCLKFreq(GET_LCDC_SYSID(lcdc)) / 1000000;
    uint32_t lcdc_pclk_Hz = HAL_RCC_GetPCLKFreq(GET_LCDC_SYSID(lcdc), 1);


    uint32_t max_col, max_line;

    max_col = (lcdc->Init.cfg.jdi.bank_col_head + lcdc->Init.cfg.jdi.valid_columns + lcdc->Init.cfg.jdi.bank_col_tail) / 2;
    max_line = (lcdc->Init.cfg.jdi.bank_row_head + lcdc->Init.cfg.jdi.valid_rows + lcdc->Init.cfg.jdi.bank_row_tail) * 2;

    const uint32_t hck_ns = 660;  //hck pulse width, half period time


    uint32_t hck_tk     = ((hck_ns  * lcdc_clk_MHz) + 500) / 1000;
    uint32_t hst_tk     = hck_tk;
    uint32_t hst_dly_tk = hst_tk;
    uint32_t hck_dly_tk = hck_tk / 2;

    uint32_t vck_tk     = hck_tk * max_col;
    uint32_t vst_tk     = vck_tk;
    uint32_t vck_dly_tk = vck_tk / 2;


    HAL_LCDC_ASSERT(hst_tk <= GET_REG_VAL(LCD_IF_JDI_PAR_CONF4_HST_WIDTH_Msk, LCD_IF_JDI_PAR_CONF4_HST_WIDTH_Msk, LCD_IF_JDI_PAR_CONF4_HST_WIDTH_Pos));
    HAL_LCDC_ASSERT(hck_tk <= GET_REG_VAL(LCD_IF_JDI_PAR_CONF4_HCK_WIDTH_Msk, LCD_IF_JDI_PAR_CONF4_HCK_WIDTH_Msk, LCD_IF_JDI_PAR_CONF4_HCK_WIDTH_Pos));
    HAL_LCDC_ASSERT(hst_dly_tk <= GET_REG_VAL(LCD_IF_JDI_PAR_CONF6_HST_DLY_Msk, LCD_IF_JDI_PAR_CONF6_HST_DLY_Msk, LCD_IF_JDI_PAR_CONF6_HST_DLY_Pos));
    HAL_LCDC_ASSERT(hck_dly_tk <= GET_REG_VAL(LCD_IF_JDI_PAR_CONF7_HCK_DLY_Msk, LCD_IF_JDI_PAR_CONF7_HCK_DLY_Msk, LCD_IF_JDI_PAR_CONF7_HCK_DLY_Pos));
    HAL_LCDC_ASSERT(vst_tk <= GET_REG_VAL(LCD_IF_JDI_PAR_CONF5_VST_WIDTH_Msk, LCD_IF_JDI_PAR_CONF5_VST_WIDTH_Msk, LCD_IF_JDI_PAR_CONF5_VST_WIDTH_Pos));
    HAL_LCDC_ASSERT(vck_tk <= GET_REG_VAL(LCD_IF_JDI_PAR_CONF5_VCK_WIDTH_Msk, LCD_IF_JDI_PAR_CONF5_VCK_WIDTH_Msk, LCD_IF_JDI_PAR_CONF5_VCK_WIDTH_Pos));
    HAL_LCDC_ASSERT(vck_dly_tk <= GET_REG_VAL(LCD_IF_JDI_PAR_CONF6_VCK_DLY_Msk, LCD_IF_JDI_PAR_CONF6_VCK_DLY_Msk, LCD_IF_JDI_PAR_CONF6_VCK_DLY_Pos));

    lcdc->Instance->JDI_PAR_CONF1 = ((max_col - 1) << LCD_IF_JDI_PAR_CONF1_MAX_COL_Pos) | ((max_line - 1) << LCD_IF_JDI_PAR_CONF1_MAX_LINE_Pos);

    lcdc->Instance->JDI_PAR_CONF4 = (hst_tk << LCD_IF_JDI_PAR_CONF4_HST_WIDTH_Pos)
                                    | (hck_tk << LCD_IF_JDI_PAR_CONF4_HCK_WIDTH_Pos);

    lcdc->Instance->JDI_PAR_CONF5 = (vst_tk << LCD_IF_JDI_PAR_CONF5_VST_WIDTH_Pos)
                                    | (vck_tk << LCD_IF_JDI_PAR_CONF5_VCK_WIDTH_Pos);

    lcdc->Instance->JDI_PAR_CONF6 = (hst_dly_tk << LCD_IF_JDI_PAR_CONF6_HST_DLY_Pos)
                                    | (vck_dly_tk << LCD_IF_JDI_PAR_CONF6_VCK_DLY_Pos);

    lcdc->Instance->JDI_PAR_CONF7 = (hck_dly_tk << LCD_IF_JDI_PAR_CONF7_HCK_DLY_Pos)
                                    | LCD_IF_JDI_PAR_CONF7_DP_MODE;


    lcdc->Instance->JDI_PAR_CONF8 = ((32) << LCD_IF_JDI_PAR_CONF8_ENB_ST_COL_Pos) | ((95) << LCD_IF_JDI_PAR_CONF8_ENB_END_COL_Pos);
    lcdc->Instance->JDI_PAR_CTRL = (0 << LCD_IF_JDI_PAR_CTRL_ENBPOL_Pos)
                                   | (0 << LCD_IF_JDI_PAR_CTRL_HCKPOL_Pos)
                                   | (0 << LCD_IF_JDI_PAR_CTRL_HSTPOL_Pos)
                                   | (0 << LCD_IF_JDI_PAR_CTRL_VCKPOL_Pos)
                                   | (0 << LCD_IF_JDI_PAR_CTRL_VSTPOL_Pos)
                                   | (0 << LCD_IF_JDI_PAR_CTRL_INT_LINE_NUM_Pos);



    lcdc->Instance->JDI_PAR_CTRL &= ~LCD_IF_JDI_PAR_CTRL_XRST;


    lcdc->Instance->JDI_PAR_EX_CTRL &= ~(LCD_IF_JDI_PAR_EX_CTRL_MAX_CNT_Msk | LCD_IF_JDI_PAR_EX_CTRL_CNT_EN);
    lcdc->Instance->JDI_PAR_EX_CTRL |= (lcdc_pclk_Hz / 60) << LCD_IF_JDI_PAR_EX_CTRL_MAX_CNT_Pos;





}



#ifdef HAL_DSI_MODULE_ENABLED


static void HAL_LCDC_DSI_Init(LCDC_HandleTypeDef *lcdc)
{
    DSI_HandleTypeDef *hdsi = &lcdc->hdsi;
    LCDC_InitTypeDef *int_cfg = &lcdc->Init;

    /* Base address of DSI Host/Wrapper registers to be set before calling De-Init */
    hdsi->Instance = hwp_dsi_host;
    hdsi->PHY_Instance = hwp_dsi_phy;

    HAL_DSI_DeInit(hdsi);

    HAL_Delay(10);

    memcpy(&hdsi->Init, &int_cfg->cfg.dsi.Init, sizeof(DSI_InitTypeDef));


    HAL_DSI_Init(hdsi);

    //HAL_DSI_ConfigPhyFreq(hdsi, int_cfg->freq);

    /* Configure the DSI for Command mode */
    //HAL_DSI_ConfigAdaptedCommandMode(hdsi, &int_cfg->cfg.dsi.CmdCfg);

    HAL_DSI_ConfigCommand(hdsi, &int_cfg->cfg.dsi.LPCmd);

    HAL_DSI_ConfigHostTimeouts(hdsi, &int_cfg->cfg.dsi.HostTimeouts);

    HAL_DSI_ConfigFlowControl(hdsi, DSI_FLOW_CONTROL_BTA);
    /* Configure DSI PHY HS2LP and LP2HS timings */
    HAL_DSI_ConfigPhyTimer(hdsi, &int_cfg->cfg.dsi.PhyTimings);

    HAL_DSI_SetGenericVCID(hdsi, int_cfg->cfg.dsi.CmdCfg.VirtualChannelID);
    HAL_DSI_ConfigErrorMonitor(hdsi, 0);



    HAL_DSI_ConfigErrorMonitor(hdsi, 0xFFF);
}


#ifndef SF32LB55X
static void DSI_Video_Init(LCDC_HandleTypeDef *lcdc)
{
    DSI_HandleTypeDef *hdsi = &lcdc->hdsi;
    LCDC_InitTypeDef *int_cfg = &lcdc->Init;

    /* Base address of DSI Host/Wrapper registers to be set before calling De-Init */
    hdsi->Instance = hwp_dsi_host;
    hdsi->PHY_Instance = hwp_dsi_phy;

    HAL_DSI_DeInit(hdsi);

    HAL_Delay(10);

    memcpy(&hdsi->Init, &int_cfg->cfg.dsi_vid.Init, sizeof(DSI_InitTypeDef));


    HAL_DSI_Init(hdsi);


    HAL_DSI_ConfigCommand(hdsi, &int_cfg->cfg.dsi_vid.LPCmd);

    HAL_DSI_ConfigHostTimeouts(hdsi, &int_cfg->cfg.dsi_vid.HostTimeouts);

    HAL_DSI_ConfigFlowControl(hdsi, DSI_FLOW_CONTROL_BTA);
    /* Configure DSI PHY HS2LP and LP2HS timings */
    HAL_DSI_ConfigPhyTimer(hdsi, &int_cfg->cfg.dsi_vid.PhyTimings);

    HAL_DSI_SetGenericVCID(hdsi, int_cfg->cfg.dsi_vid.VidCfg.VirtualChannelID);
    HAL_DSI_ConfigErrorMonitor(hdsi, 0);
    HAL_DSI_ConfigErrorMonitor(hdsi, 0xFFF);
}

#endif /* SF32LB58X */
static HAL_StatusTypeDef _DSI_StartPartialWrite(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode, uint8_t write_cmd, SetRegion_cb_t rg_cb)
{
    HAL_StatusTypeDef ret = HAL_OK;

    lcdc->lcd_rg_cb = rg_cb;
    lcdc->dsi_lcd_w_cmd = write_cmd;

    if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
        uint32_t write_size;
        if (((lcdc->roi.y1 - lcdc->roi.y0 + 1) * (lcdc->roi.x1 - lcdc->roi.x0 + 1) * HAL_LCDC_GetPixelSize(lcdc->Init.color_mode))  > (lcdc->Init.cfg.dsi.CmdCfg.CommandSize - 1))
        {
            memcpy(&lcdc->part_write_roi_bak, &lcdc->roi, sizeof(LCDC_AreaDef));
            //memcpy(&lcdc->part_write_data_area_bak, &lcdc->Layer.data_area,  sizeof(LCDC_AreaDef));
            //lcdc->Layer.part_write_data_bak = lcdc->Layer.data;

            lcdc->part_write_lines = lcdc->Init.cfg.dsi.CmdCfg.CommandSize / HAL_LCDC_GetPixelSize(lcdc->Init.color_mode) / (lcdc->roi.x1 - lcdc->roi.x0 + 1);
            lcdc->part_write_lines &= ~1; //Adapt to some LCD: roi height must can be divisible by 2
            HAL_LCDC_ASSERT(lcdc->part_write_lines > 0);
            lcdc->part_write_y = lcdc->roi.y0;

            HAL_LCDC_ASSERT(lcdc->part_write_y <= lcdc->part_write_roi_bak.y1);

            //Change ROI area
            lcdc->roi.y0 = lcdc->part_write_y;
            lcdc->roi.y1 = lcdc->part_write_y + lcdc->part_write_lines - 1;


            lcdc->part_write_y += lcdc->part_write_lines;
        }
        else
        {
            lcdc->part_write_y = 0xDEAD;
        }

        if (lcdc->lcd_rg_cb)
        {
            lcdc->lcd_rg_cb(lcdc,
                            lcdc->roi.x0,
                            lcdc->roi.y0,
                            lcdc->roi.x1,
                            lcdc->roi.y1
                           );
        }

        write_size = (lcdc->roi.y1 - lcdc->roi.y0 + 1) * (lcdc->roi.x1 - lcdc->roi.x0 + 1) * HAL_LCDC_GetPixelSize(lcdc->Init.color_mode) + 1 /*0X2C cmd*/;

        LCDC_LOG_D("LCD_IO_PartialWriteSetup[%d,%d,%d,%d] write_size=%d\n", lcdc->roi.x0,
                   lcdc->roi.y0,
                   lcdc->roi.x1,
                   lcdc->roi.y1,
                   write_size);

        HAL_DSI_SetPaticalSend(&lcdc->hdsi, write_size);
        if (DSI_LP_DLW_DISABLE == lcdc->Init.cfg.dsi.LPCmd.LPDcsLongWrite)
        {
            HAL_DSI_EnterCKLaneHSM(&lcdc->hdsi);//Disbale auto control
        }
        SendSingleCmd(lcdc, write_cmd, 1);


#ifndef HAL_USING_HTOL
        if ((DSI_TE_ACKNOWLEDGE_ENABLE == lcdc->Init.cfg.dsi.CmdCfg.TEAcknowledgeRequest) && (NEXT_FRAME_TE_OFF != lcdc->Next_Frame_TE))
        {
            LCDC_SET_TE(lcdc, true);
            ret = _SendLayerData(lcdc, sync_mode);
            if (DSI_TE_DSILINK == lcdc->Init.cfg.dsi.CmdCfg.TearingEffectSource)
            {
                HAL_DSI_ShortWrite(&lcdc->hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x00); //TE on
            }
        }
        else
#endif /* HAL_USING_HTOL */
        {
            ret = _SendLayerData(lcdc, sync_mode);
        }

        if ((HAL_OK == ret) && (LCDC_SYNC_MODE == sync_mode)) ret = _WaitSendLayerDone(lcdc);
    }


    if ((HAL_OK == ret) && (LCDC_SYNC_MODE == sync_mode))
    {
        while (true == _PartialWriteContinue(lcdc, sync_mode));
    }

    return ret;
}


static bool _PartialWriteContinue(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode)
{
    if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
        HAL_StatusTypeDef ret;
        uint32_t write_size;

        if ((lcdc->part_write_y <= lcdc->part_write_roi_bak.y1) && (lcdc->part_write_y < 0xDEAD))
        {
            lcdc->roi.y0 = lcdc->part_write_y;
            lcdc->roi.y1 = lcdc->part_write_y + lcdc->part_write_lines - 1;

            if (lcdc->roi.y1 > lcdc->part_write_roi_bak.y1)
            {
                lcdc->roi.y1 = lcdc->part_write_roi_bak.y1;
            }

            if (lcdc->lcd_rg_cb)
            {
                lcdc->lcd_rg_cb(lcdc,
                                lcdc->roi.x0,
                                lcdc->roi.y0,
                                lcdc->roi.x1,
                                lcdc->roi.y1
                               );
            }
            write_size = (lcdc->roi.y1 - lcdc->roi.y0 + 1) * (lcdc->roi.x1 - lcdc->roi.x0 + 1) * HAL_LCDC_GetPixelSize(lcdc->Init.color_mode) + 1 /*0X3C cmd*/;
            LCDC_LOG_D("PartialWriteContinue_IT[%d,%d,%d,%d] write_size=%d\n", lcdc->roi.x0,
                       lcdc->roi.y0,
                       lcdc->roi.x1,
                       lcdc->roi.y1,
                       write_size);


            HAL_DSI_SetPaticalSend(&lcdc->hdsi, write_size);

            if (lcdc->lcd_rg_cb) //Use partial write mode
                SendSingleCmd(lcdc, lcdc->dsi_lcd_w_cmd, 1);
            else //Use continue mode if lcd_rg_cb is NULL
                SendSingleCmd(lcdc, 0x3C, 1);

            if (DSI_TE_ACKNOWLEDGE_ENABLE == lcdc->Init.cfg.dsi.CmdCfg.TEAcknowledgeRequest)
            {
                LCDC_SET_TE(lcdc, false);
            }
            ret = _SendLayerData(lcdc, sync_mode);
            HAL_LCDC_ASSERT(HAL_OK == ret);
            if (LCDC_SYNC_MODE == sync_mode) ret = _WaitSendLayerDone(lcdc);

            lcdc->part_write_y += lcdc->part_write_lines;
            return true;
        }
        else if (lcdc->part_write_y != 0xDEAD)
        {
            /*restore LCD region after partial write finished*/
            if (lcdc->lcd_rg_cb)
            {
                lcdc->lcd_rg_cb(lcdc,
                                lcdc->part_write_roi_bak.x0,
                                lcdc->part_write_roi_bak.y0,
                                lcdc->part_write_roi_bak.x1,
                                lcdc->part_write_roi_bak.y1
                               );
            }

            /*Restore roi region*/
            memcpy(&lcdc->roi, &lcdc->part_write_roi_bak, sizeof(LCDC_AreaDef));

            LCDC_LOG_D("PartialWrite Done.[debug_cnt1:%x]\n", lcdc->debug_cnt0);
        }

        if (DSI_LP_DLW_DISABLE == lcdc->Init.cfg.dsi.LPCmd.LPDcsLongWrite)
        {
            HAL_DSI_ExitCKLaneHSM(&lcdc->hdsi); //Resume auto control
        }
    }

    return false; //finish

}

#ifdef DSI_DIRECT_SEND_MODE
static HAL_StatusTypeDef _DSI_DirectWrite(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode, uint8_t write_cmd, SetRegion_cb_t rg_cb)
{
    HAL_StatusTypeDef ret = HAL_OK;

    lcdc->lcd_rg_cb = rg_cb;
    lcdc->dsi_lcd_w_cmd = write_cmd;

    if (HAL_LCDC_IS_DSI_CMD_IF(lcdc->Init.lcd_itf))
    {
        uint32_t write_size;

        write_size = (lcdc->roi.y1 - lcdc->roi.y0 + 1) * (lcdc->roi.x1 - lcdc->roi.x0 + 1) * HAL_LCDC_GetPixelSize(lcdc->Init.color_mode);

        LCDC_LOG_D("_DSI_DirectWrite[%d,%d,%d,%d] write_size=%d\n", lcdc->roi.x0,
                   lcdc->roi.y0,
                   lcdc->roi.x1,
                   lcdc->roi.y1,
                   write_size);

        HAL_DSI_DirectSend(&lcdc->hdsi, write_size);
        if (DSI_LP_DLW_DISABLE == lcdc->Init.cfg.dsi.LPCmd.LPDcsLongWrite)
        {
            HAL_DSI_EnterCKLaneHSM(&lcdc->hdsi);//Disbale auto control
        }

#ifndef HAL_USING_HTOL
        if ((DSI_TE_ACKNOWLEDGE_ENABLE == lcdc->Init.cfg.dsi.CmdCfg.TEAcknowledgeRequest) && (NEXT_FRAME_TE_OFF != lcdc->Next_Frame_TE))
        {
            LCDC_SET_TE(lcdc, true);
            ret = _SendLayerData(lcdc, sync_mode);
            if (DSI_TE_DSILINK == lcdc->Init.cfg.dsi.CmdCfg.TearingEffectSource)
            {
                HAL_DSI_ShortWrite(&lcdc->hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x00); //TE on
            }
        }
        else
#endif /* HAL_USING_HTOL */
        {
            ret = _SendLayerData(lcdc, sync_mode);
        }

        if ((HAL_OK == ret) && (LCDC_SYNC_MODE == sync_mode)) ret = _WaitSendLayerDone(lcdc);
    }
    else if (HAL_LCDC_IS_DSI_VID_IF(lcdc->Init.lcd_itf))
    {
        HAL_DSI_EnterCKLaneHSM(&lcdc->hdsi);//Disbale auto control
        ret = _SendLayerData(lcdc, sync_mode);

        if ((HAL_OK == ret) && (LCDC_SYNC_MODE == sync_mode)) ret = _WaitSendLayerDone(lcdc);
    }


    if ((HAL_OK == ret) && (LCDC_SYNC_MODE == sync_mode))
    {
        while (true == _DSI_DirectWriteDone(lcdc, sync_mode));
    }

    return ret;
}


static bool _DSI_DirectWriteDone(LCDC_HandleTypeDef *lcdc, LCDC_AsyncModeTypeDef sync_mode)
{
    if (HAL_LCDC_IS_DSI_CMD_IF(lcdc->Init.lcd_itf))
    {
        if (DSI_LP_DLW_DISABLE == lcdc->Init.cfg.dsi.LPCmd.LPDcsLongWrite)
        {
            HAL_DSI_ExitCKLaneHSM(&lcdc->hdsi); //Resume auto control
        }
    }

    return false; //finish

}

#endif /* SF32LB55X */


static HAL_StatusTypeDef DSI_SetOutFormat(LCDC_HandleTypeDef *lcdc, HAL_LCDC_PixelFormat color_mode)
{
    DSI_HandleTypeDef *hdsi = &lcdc->hdsi;
    uint32_t dsi_cf;

    switch (color_mode)
    {
    case LCDC_PIXEL_FORMAT_RGB888:
        dsi_cf = DSI_RGB888;
        break;
    case LCDC_PIXEL_FORMAT_RGB565:
    case LCDC_PIXEL_FORMAT_RGB565_SWAP: //Use RGB565 directly
        dsi_cf = DSI_RGB565;
        break;
    case LCDC_PIXEL_FORMAT_RGB666:
        dsi_cf = DSI_RGB666;
        break;
    default:
        HAL_LCDC_ASSERT(0);/* INVALID PARAM */           break;
    }

    if (HAL_LCDC_IS_DSI_CMD_IF(lcdc->Init.lcd_itf))
    {
        DSI_CmdCfgTypeDef *pCmdCfg = &lcdc->Init.cfg.dsi.CmdCfg;

        pCmdCfg->ColorCoding = dsi_cf;
        /* Configure the DSI for Command mode */
        return HAL_DSI_ConfigAdaptedCommandMode(&(lcdc->hdsi), pCmdCfg);
    }
    else
    {
#ifdef SF32LB55X
        HAL_LCDC_ASSERT(0);//Not support
        return HAL_ERROR;
#else
        DSI_VidCfgTypeDef VidCfg;


        return HAL_DSI_ConfigVideoMode(hdsi, &VidCfg);
#endif /* SF32LB55X */
    }
}

static HAL_StatusTypeDef DSI_SetFreq(LCDC_HandleTypeDef *lcdc, uint32_t freq)
{
    DSI_HandleTypeDef *hdsi = &lcdc->hdsi;
    LCDC_InitTypeDef *int_cfg = &lcdc->Init;

    switch (freq)
    {
    case DSI_FREQ_240Mbps:
    case DSI_FREQ_288Mbps:
    case DSI_FREQ_336Mbps:
    case DSI_FREQ_384Mbps:
    case DSI_FREQ_480Mbps:
        break;

    case DSI_FREQ_96Mbps:
    case DSI_FREQ_144Mbps:
    case DSI_FREQ_168Mbps:
    case DSI_FREQ_192Mbps:
    default:
        HAL_LCDC_ASSERT(0);// INVALID PARAM
        break;
    }

    HAL_LCDC_ASSERT((freq / 16 / int_cfg->cfg.dsi.Init.TXEscapeCkdiv)  < 20000000); //LP bps must < 20Mbps

    return HAL_DSI_ConfigPhyFreq(hdsi, freq);
}

#endif /* HAL_DSI_MODULE_ENABLED */


static void SPI_Sequence(LCDC_HandleTypeDef *lcdc, bool end)
{
    if (end)
        lcdc->Instance->SPI_IF_CONF |= LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS;
    else
        lcdc->Instance->SPI_IF_CONF &= ~LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS;
}

static HAL_StatusTypeDef LCDC_HW_Init(LCDC_HandleTypeDef *lcdc)
{
    LCDC_InitTypeDef *init;

    HAL_LCDC_ASSERT(lcdc);
    HAL_RCC_EnableModule((lcdc->Instance == hwp_lcdc1) ? RCC_MOD_LCDC1 : RCC_MOD_LCDC2);
    HAL_RCC_ResetModule((lcdc->Instance == hwp_lcdc1) ? RCC_MOD_LCDC1 : RCC_MOD_LCDC2);

    init = &lcdc->Init;

    lcdc->Instance->SETTING |= LCD_IF_SETTING_AUTO_GATE_EN;


    /*Select LCD interface*/
    SelectIntf(lcdc, lcdc->Init.lcd_itf);

    /*LCDC output frequecy*/
    SetFreq(lcdc, init->freq);

    /*LCDC output format*/
    SetOutFormat(lcdc, init->color_mode);

    /* Setup TE  */
    SetupTE(lcdc);

    /* Enable LCD interface output*/
    EnableInf(lcdc);

    lcdc->Instance->LCD_IF_CONF |= LCD_IF_LCD_IF_CONF_LCD_RSTB; //Release reset signal

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_Init(LCDC_HandleTypeDef *lcdc)
{
    LCDC_HW_Init(lcdc);

    /*Enable layer0 only default*/
    lcdc->Layer[HAL_LCDC_LAYER_0].disable = 0;
    lcdc->Layer[HAL_LCDC_LAYER_1].disable = 1;

    lcdc->ErrorCode = HAL_LCDC_ERROR_NONE;
    lcdc->State = HAL_LCDC_STATE_READY;
    //LCDC_LOG_D("HAL_LCDC_Init cnf=%d,cnf2=%d,status=%d\n",lcdc->Instance->TE_CONF,
    lcdc->Next_Frame_TE = NEXT_FRAME_TE_DEFAULT;

    lcdc->debug_cnt0 = 0;
    lcdc->debug_cnt1 = 0;
    lcdc->debug_cnt2 = 0;

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_DeInit(LCDC_HandleTypeDef *lcdc)
{
#ifdef HAL_DSI_MODULE_ENABLED
    if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
        HAL_DSI_DeInit(&lcdc->hdsi);
    }
#endif /* HAL_DSI_MODULE_ENABLED */

    if (HAL_LCDC_IS_DPI_IF(lcdc->Init.lcd_itf))
    {
        /* Disable DPI */
        lcdc->Instance->DPI_CTRL &= ~LCD_IF_DPI_CTRL_DPI_EN;
    }

    return HAL_OK;
}

//Obsolete
__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_ResetLCD(LCDC_HandleTypeDef *lcdc, uint8_t pulse_type, uint32_t pulse_us)
{
    uint32_t bak;

    /*Select DSI will not send trig LCD_RESB pin */
    bak = lcdc->Instance->LCD_CONF & LCD_IF_LCD_CONF_LCD_INTF_SEL_Msk;
    lcdc->Instance->LCD_CONF &=  ~LCD_IF_LCD_CONF_LCD_INTF_SEL_Msk;

    /*No matter what the previous level was, just keep LOW/HIGH level for pulse_us*/
    if (LCDC_RESX_NEG_PULSE == pulse_type)
    {
        lcdc->Instance->LCD_IF_CONF &= ~LCD_IF_LCD_IF_CONF_LCD_RSTB;
        HAL_Delay_us(pulse_us);
        lcdc->Instance->LCD_IF_CONF |= LCD_IF_LCD_IF_CONF_LCD_RSTB;
    }
    else
    {
        lcdc->Instance->LCD_IF_CONF |= LCD_IF_LCD_IF_CONF_LCD_RSTB;
        HAL_Delay_us(pulse_us);
        lcdc->Instance->LCD_IF_CONF &= ~LCD_IF_LCD_IF_CONF_LCD_RSTB;
    }

    lcdc->Instance->LCD_CONF |= bak;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_Reset(LCDC_HandleTypeDef *lcdc)
{
    LCDC_HW_Init(lcdc);

    lcdc->ErrorCode = HAL_LCDC_ERROR_NONE;
    lcdc->State = HAL_LCDC_STATE_READY;

    return HAL_OK;
}


__HAL_ROM_USED HAL_LCDC_IF_TypeDef HAL_LCDC_SetSPICfg(LCDC_HandleTypeDef *lcdc, HAL_LCDC_IF_TypeDef config)
{
    uint32_t config_raw = lcdc->Instance->SPI_IF_CONF;
    HAL_LCDC_IF_TypeDef r = LCDC_INTF_SPI_START;

    HAL_LCDC_ASSERT(config >= LCDC_INTF_SPI_START && config <= LCDC_INTF_SPI_END);

    r = ((config_raw & LCD_IF_SPI_IF_CONF_LINE_Msk) >> LCD_IF_SPI_IF_CONF_LINE_Pos) + LCDC_INTF_SPI_START;
    config_raw &= ~LCD_IF_SPI_IF_CONF_LINE_Msk;
    config_raw |= MAKE_REG_VAL(config - LCDC_INTF_SPI_START, LCD_IF_SPI_IF_CONF_LINE_Msk, LCD_IF_SPI_IF_CONF_LINE_Pos);
    lcdc->Instance->SPI_IF_CONF = config_raw;
    return r;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_SetOutFormat(LCDC_HandleTypeDef *lcdc, HAL_LCDC_PixelFormat format)
{
    WaitBusy(lcdc);

    return SetOutFormat(lcdc, format);
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_SetFreq(LCDC_HandleTypeDef *lcdc, uint32_t freq)
{
    WaitBusy(lcdc);

    return SetFreq(lcdc, freq);
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_ReadDatas(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len, uint8_t *p_data, uint32_t data_len)
{
    uint32_t i;
    uint32_t data;
    HAL_StatusTypeDef err = HAL_OK;

    //HAL_LCDC_ASSERT((lcdc->Instance->STATUS&LCD_IF_STATUS_LCD_BUSY) == 0x0);
    HAL_LCDC_ASSERT((NULL != lcdc) && (NULL != p_data));
    HAL_LCDC_ASSERT((addr_len > 0) && (addr_len <= 4));

    if (0 == data_len) return HAL_OK;

    __HAL_LCDC_LOCK(lcdc);
    if (HAL_LCDC_STATE_READY != lcdc->State)
    {
        __HAL_LCDC_UNLOCK(lcdc);
        return HAL_BUSY;
    }

    if (HAL_LCDC_IS_SPI_IF(lcdc->Init.lcd_itf))
    {
        uint32_t config;
        HAL_LCDC_ASSERT((data_len > 0) && (data_len <= 4));

        HAL_LCDC_SPI_Sequence(lcdc, 1);

        if (HAL_TIMEOUT == WaitBusy(lcdc)) goto READ_TIMEOUT;

#ifndef LCD_IF_LCD_CONF_SPI_RD_SEL
        if (lcdc->Init.cfg.spi.readback_from_Dx != 0)
        {
            data = HAL_LCDC_SoftSPI_Read(lcdc, addr, addr_len, data_len);
        }
        else
#endif /*LCD_IF_LCD_CONF_SPI_RD_SEL*/
        {
            config = lcdc->Instance->SPI_IF_CONF;
            config &= ~(LCD_IF_SPI_IF_CONF_RD_LEN_Msk | LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk | LCD_IF_SPI_IF_CONF_WR_LEN_Msk);

            config |= MAKE_REG_VAL((data_len - 1), LCD_IF_SPI_IF_CONF_RD_LEN_Msk, LCD_IF_SPI_IF_CONF_RD_LEN_Pos)
                      | MAKE_REG_VAL(0, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Pos)
                      | MAKE_REG_VAL((addr_len - 1), LCD_IF_SPI_IF_CONF_WR_LEN_Msk, LCD_IF_SPI_IF_CONF_WR_LEN_Pos);


            lcdc->Instance->SPI_IF_CONF = config;
            lcdc->Instance->LCD_WR = (addr);
            lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_RD_TRIG;

            if (HAL_TIMEOUT == WaitBusy(lcdc)) goto READ_TIMEOUT;


            data = lcdc->Instance->LCD_RD;
        }

        for (i = 0; i < data_len; i++)
        {
            *(p_data + i) = (uint8_t)((data >> (i * 8)) & 0xff);
        }


    }
    else if (HAL_LCDC_IS_DBI_IF(lcdc->Init.lcd_itf))
    {
        SendSingleCmd(lcdc, addr, addr_len);
        if (HAL_TIMEOUT == WaitBusy(lcdc)) goto READ_TIMEOUT;

        for (i = 0; i < data_len; i++)
        {
            lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_RD_TRIG | LCD_IF_LCD_SINGLE_TYPE;
            if (HAL_TIMEOUT == WaitBusy(lcdc)) goto READ_TIMEOUT;
            *(p_data + i) = (uint8_t)(lcdc->Instance->LCD_RD & 0xff);
        }
    }
#ifdef HAL_DSI_MODULE_ENABLED
    else if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
#ifndef HAL_USING_HTOL
        HAL_LCDC_ASSERT(1 == addr_len);
        if (HAL_TIMEOUT == WaitBusy(lcdc)) goto READ_TIMEOUT;

        err = HAL_DSI_Read(&lcdc->hdsi, lcdc->Init.cfg.dsi.CmdCfg.VirtualChannelID, p_data, data_len, DSI_GEN_SHORT_PKT_READ_P1, 0, (uint8_t *)&addr);
#endif /* HAL_USING_HTOL */
    }
#endif /* HAL_DSI_MODULE_ENABLED */
    else
    {
        HAL_LCDC_ASSERT(0); //Not support!
        err = HAL_ERROR;
    }


    __HAL_LCDC_UNLOCK(lcdc);
    return err;

READ_TIMEOUT:
    __HAL_LCDC_UNLOCK(lcdc);
    return HAL_TIMEOUT;

}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_WriteDatas(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len, uint8_t *p_data, uint32_t data_len)
{
    HAL_StatusTypeDef err = HAL_OK;

    HAL_LCDC_ASSERT(NULL != lcdc);
    HAL_LCDC_ASSERT((addr_len > 0) && (addr_len <= 4));
    if (data_len > 0)
    {
        HAL_LCDC_ASSERT(NULL != p_data);
    }

    if (HAL_TIMEOUT == WaitBusy(lcdc)) return HAL_TIMEOUT;


    __HAL_LCDC_LOCK(lcdc);
    if (HAL_LCDC_STATE_READY != lcdc->State)
    {
        __HAL_LCDC_UNLOCK(lcdc);
        return HAL_BUSY;
    }


    if (0)
    {
    }
#ifdef HAL_DSI_MODULE_ENABLED
    else if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
        HAL_LCDC_ASSERT(1 == addr_len);

#ifndef HAL_USING_HTOL
        if (0 == data_len)
        {
            err = HAL_DSI_ShortWrite(&lcdc->hdsi, lcdc->Init.cfg.dsi.CmdCfg.VirtualChannelID, DSI_DCS_SHORT_PKT_WRITE_P0, (uint8_t)addr, 0);
        }
        else if (1 == data_len)
        {
            err = HAL_DSI_ShortWrite(&lcdc->hdsi, lcdc->Init.cfg.dsi.CmdCfg.VirtualChannelID, DSI_DCS_SHORT_PKT_WRITE_P1, (uint8_t)addr, p_data[0]);
        }
        else
        {
            err = HAL_DSI_LongWrite(&lcdc->hdsi,  lcdc->Init.cfg.dsi.CmdCfg.VirtualChannelID, DSI_DCS_LONG_PKT_WRITE, data_len, (uint8_t)addr, p_data);
        }
#endif /* HAL_USING_HTOL */

        if (err != HAL_OK)
        {
            LCDC_LOG_E("DSI WRITE ERROR %d", err);
        }
    }
#endif /* HAL_DSI_MODULE_ENABLED */
    else if (HAL_LCDC_IS_SPI_IF(lcdc->Init.lcd_itf))
    {
        uint32_t total_len = addr_len + data_len;
        uint32_t config;
        uint32_t byte_gap = lcdc->Init.cfg.spi.bytes_gap_us;

        if (byte_gap > 0)
        {
            /****************Send command  ****************/
            config = lcdc->Instance->SPI_IF_CONF;
            config &= ~(LCD_IF_SPI_IF_CONF_RD_LEN_Msk | LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk | LCD_IF_SPI_IF_CONF_WR_LEN_Msk);
            config |= MAKE_REG_VAL(0, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Pos)
                      | MAKE_REG_VAL(0, LCD_IF_SPI_IF_CONF_WR_LEN_Msk, LCD_IF_SPI_IF_CONF_WR_LEN_Pos);
            lcdc->Instance->SPI_IF_CONF = config;


            while (addr_len > 0)
            {
                HAL_LCDC_SPI_Sequence(lcdc, (1 == total_len) ? 1 : 0);
                total_len--;

                WAIT_LCDC_SINGLE_BUSY(lcdc);
                lcdc->Instance->LCD_WR = (addr >> ((addr_len - 1) << 3)) & 0xFF;
                lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG;

                if (byte_gap > 1) HAL_Delay_us(byte_gap);

                addr_len--;
            }


            /****************Send data   ****************/
            if (data_len > 0)
            {

                HAL_LCDC_ASSERT(NULL != p_data);

                while (data_len > 0)
                {
                    HAL_LCDC_SPI_Sequence(lcdc, (1 == total_len) ? 1 : 0);
                    total_len--;

                    WAIT_LCDC_SINGLE_BUSY(lcdc);
                    lcdc->Instance->LCD_WR = *p_data;
                    lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG | LCD_IF_LCD_SINGLE_TYPE;

                    if (byte_gap > 1) HAL_Delay_us(byte_gap);

                    data_len--;
                    p_data++;
                }
            }
        }
        else   //Send address & datas  maximum 4bytes each time.
        {
            /****************Send command  ****************/
            total_len -= addr_len;
            SPI_Sequence(lcdc, (0 == total_len)); //Auto reset CS if no data to send

            config = lcdc->Instance->SPI_IF_CONF;
            config &= ~(LCD_IF_SPI_IF_CONF_RD_LEN_Msk | LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk | LCD_IF_SPI_IF_CONF_WR_LEN_Msk);
            config |= MAKE_REG_VAL(0, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk, LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Pos)
                      | MAKE_REG_VAL(addr_len - 1, LCD_IF_SPI_IF_CONF_WR_LEN_Msk, LCD_IF_SPI_IF_CONF_WR_LEN_Pos);
            lcdc->Instance->SPI_IF_CONF = config;


            lcdc->Instance->LCD_WR = addr;
            lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG;



            /****************Send data   ****************/
            if (data_len > 0)
            {
                do
                {
                    uint32_t v, l;

                    //Convert 0xAA,0xBB,0xCC ->  0x00AABBCC
                    for (v = 0, l = 0; (l < 4) && (data_len > 0); l++)
                    {
                        v = (v << 8) | (*p_data);
                        data_len--;
                        p_data++;
                    }

                    WAIT_LCDC_SINGLE_BUSY(lcdc);

                    total_len -= l;
                    SPI_Sequence(lcdc, (0 == total_len)); //Auto reset CS if no data to send

                    MODIFY_REG(lcdc->Instance->SPI_IF_CONF, LCD_IF_SPI_IF_CONF_WR_LEN_Msk, (l - 1) << LCD_IF_SPI_IF_CONF_WR_LEN_Pos);

                    lcdc->Instance->LCD_WR = v;
                    lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG | LCD_IF_LCD_SINGLE_TYPE;
                }
                while (data_len > 0);
            }
        }
        HAL_LCDC_ASSERT(0 == total_len); //Make sure CS pin is reset
    }
    else
    {
        /****************Send cmd   ****************/
        do
        {
            WaitBusy(lcdc);
            lcdc->Instance->LCD_WR = (addr >> ((addr_len - 1) << 3)) & 0xFF;
            lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG;

            addr_len--;
        }
        while (addr_len > 0);

        /****************Send data   ****************/
        if (data_len > 0)
        {
            do
            {
                WaitBusy(lcdc);
                lcdc->Instance->LCD_WR = *p_data;
                lcdc->Instance->LCD_SINGLE = LCD_IF_LCD_SINGLE_WR_TRIG | LCD_IF_LCD_SINGLE_TYPE;

                data_len--;
                p_data++;
            }
            while (data_len > 0);
        }
    }

    if (HAL_TIMEOUT == WaitBusy(lcdc)) //Wait the last data be sent
    {
        err = HAL_TIMEOUT;
    }

    __HAL_LCDC_UNLOCK(lcdc);
    return err;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_SetROI(LCDC_HandleTypeDef *lcdc, LCDC_AreaDef *area)
{
    if ((NULL == area) || (NULL == lcdc))
    {
        return HAL_ERROR;
    }

    if ((area->x0 > area->x1) || (area->y0 > area->y1))
    {
        return HAL_ERROR;
    }

    lcdc->roi.x0 = area->x0;
    lcdc->roi.x1 = area->x1;
    lcdc->roi.y0 = area->y0;
    lcdc->roi.y1 = area->y1;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_SetROIArea(LCDC_HandleTypeDef *lcdc, uint16_t x_0, uint16_t y_0, uint16_t x_1, uint16_t y_1)
{
    LCDC_AreaDef a;
    a.x0 = (x_0);
    a.y0 = (y_0);
    a.x1 = (x_1);
    a.y1 = (y_1);

    return HAL_LCDC_SetROI(lcdc, &a);
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerReset(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx)
{
    __HAL_LCDC_LOCK(lcdc);

    memset(&lcdc->Layer[layeridx], 0, sizeof(LCDC_LayerCfgTypeDef));
    lcdc->Layer[layeridx].alpha = 255;
    lcdc->Layer[layeridx].total_width = INVALID_TOTAL_WIDTH;

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerSetCmpr(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx, uint8_t cmpr)
{
    __HAL_LCDC_LOCK(lcdc);

    if (HAL_LCDC_LAYER_0 == layeridx)
    {
        lcdc->Layer[layeridx].cmpr_rate = cmpr;
        lcdc->Layer[layeridx].cmpr_en = (0 == cmpr) ? 0 : 1;
    }
    else
    {
        lcdc->Layer[layeridx].cmpr_rate = 0;
        lcdc->Layer[layeridx].cmpr_en = 0;
    }

#ifndef LCDC_SUPPORTED_COMPRESSED_LAYER
    HAL_LCDC_ASSERT((0 == cmpr));
#endif /* LCDC_SUPPORTED_COMPRESSED_LAYER */

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;

}

__HAL_ROM_USED uint8_t HAL_LCDC_LayerGetCmpr(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx)
{
    if (lcdc->Layer[layeridx].cmpr_en)
    {
        return lcdc->Layer[layeridx].cmpr_rate;
    }
    else
    {
        return 0;
    }

}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerSetFormat(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx, HAL_LCDC_PixelFormat format)
{
    lcdc->Layer[layeridx].data_format = format;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerSetData(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t *pData,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1)
{
    if ((x0 > x1) || (y0 > y1))
    {
        return HAL_ERROR;
    }

    lcdc->Layer[layeridx].data = pData;

    lcdc->Layer[layeridx].data_area.x0 = x0;
    lcdc->Layer[layeridx].data_area.y0 = y0;
    lcdc->Layer[layeridx].data_area.x1 = x1;
    lcdc->Layer[layeridx].data_area.y1 = y1;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerSetDataExt(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t *pData,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1, uint16_t total_width)
{
    if ((x0 > x1) || (y0 > y1))
    {
        return HAL_ERROR;
    }

    lcdc->Layer[layeridx].data = pData;

    lcdc->Layer[layeridx].data_area.x0 = x0;
    lcdc->Layer[layeridx].data_area.y0 = y0;
    lcdc->Layer[layeridx].data_area.x1 = x1;
    lcdc->Layer[layeridx].data_area.y1 = y1;

    lcdc->Layer[layeridx].total_width = total_width;

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerSetBgColor(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t r, uint8_t g, uint8_t b)
{
    __HAL_LCDC_LOCK(lcdc);

    lcdc->Layer[layeridx].bg_color.r = r;
    lcdc->Layer[layeridx].bg_color.g = g;
    lcdc->Layer[layeridx].bg_color.b = b;

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerSetLTab(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx, const LCDC_AColorDef *pLTab)
{
    __HAL_LCDC_LOCK(lcdc);

    if (HAL_TIMEOUT == WaitBusy(lcdc))
    {
        __HAL_LCDC_UNLOCK(lcdc);
        return HAL_TIMEOUT;
    }

    memcpy(HAL_LCDC_LOOKUP_TABLE_HEAD(lcdc->Instance), pLTab, HAL_LCDC_LOOKUP_TABLE_SIZE);
    lcdc->Layer[layeridx].lookup_table = pLTab;

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerEnableChromaKey(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t r, uint8_t g, uint8_t b)
{
    __HAL_LCDC_LOCK(lcdc);

    lcdc->Layer[layeridx].chroma_key.r = r;
    lcdc->Layer[layeridx].chroma_key.g = g;
    lcdc->Layer[layeridx].chroma_key.b = b;
    lcdc->Layer[layeridx].chroma_key_en = 1;

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerDisableChromaKey(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx)
{
    __HAL_LCDC_LOCK(lcdc);

    lcdc->Layer[layeridx].chroma_key_en = 0;

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerEnableAlpha(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx, uint8_t alpha)
{
    __HAL_LCDC_LOCK(lcdc);

    lcdc->Layer[layeridx].alpha = alpha;
    lcdc->Layer[layeridx].layer_alpha_en = 1;

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerDisableAlpha(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx)
{
    __HAL_LCDC_LOCK(lcdc);

    lcdc->Layer[layeridx].layer_alpha_en = 0;

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerRotate(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx, HAL_LCDC_RotateDef angle)
{
    __HAL_LCDC_LOCK(lcdc);

    if (HAL_LCDC_ROTATE_0 == angle)
    {
        lcdc->Layer[layeridx].v_mirror = 0;
        lcdc->Layer[layeridx].h_mirror = 0;
    }
    else if (HAL_LCDC_ROTATE_180 == angle)
    {
        lcdc->Layer[layeridx].v_mirror = 1;
        lcdc->Layer[layeridx].h_mirror = 1;
    }
    else
    {
        __HAL_LCDC_UNLOCK(lcdc);
        return HAL_ERROR;
    }

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerVMirror(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx, bool en)
{
    __HAL_LCDC_LOCK(lcdc);

    lcdc->Layer[layeridx].v_mirror = en ? 1 : 0;

    __HAL_LCDC_UNLOCK(lcdc);

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerEnable(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx)
{
    lcdc->Layer[layeridx].disable = 0;
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_LayerDisable(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx)
{
    lcdc->Layer[layeridx].disable = 1;
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_SetBgColor(LCDC_HandleTypeDef *lcdc, uint8_t r, uint8_t g, uint8_t b)
{
    lcdc->bg.r = r;
    lcdc->bg.g = g;
    lcdc->bg.b = b;

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_SendLayerData2Reg_IT(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len)
{
    HAL_StatusTypeDef err;

    __HAL_LCDC_LOCK(lcdc);

    if (HAL_LCDC_STATE_READY != lcdc->State)
    {
        err = HAL_BUSY;
    }
    else if (HAL_TIMEOUT == WaitBusy(lcdc))
    {
        err = HAL_TIMEOUT;
    }
    else if (0 == addr_len) //Send data directly if address free
    {
        HAL_LCDC_ASSERT(HAL_LCDC_IS_DPI_IF(lcdc->Init.lcd_itf) || HAL_LCDC_IS_JDI_IF(lcdc->Init.lcd_itf) || HAL_LCDC_IS_AHB_IF(lcdc->Init.lcd_itf));
        lcdc->State = HAL_LCDC_STATE_BUSY;
        err =  SendLayerData_IT(lcdc);
        if (HAL_OK != err)
        {
            lcdc->State = HAL_LCDC_STATE_READY;
        }
    }
    else
    {
        lcdc->State = HAL_LCDC_STATE_BUSY;

        if (NEXT_FRAME_TE_OFF == lcdc->Next_Frame_TE)
        {
            LCDC_SET_TE(lcdc, false);
        }
        else
        {
            SetupTE(lcdc); //Follow default TE configuration
        }


#ifdef HAL_DSI_MODULE_ENABLED
        if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
        {
            err = DSIMultiWriteStart_IT(lcdc, (uint8_t)(addr & 0xFF), NULL);
        }
        else
#endif /* HAL_DSI_MODULE_ENABLED */
        {

            HAL_LCDC_SPI_Sequence(lcdc, 0);
            SendSingleCmd(lcdc, addr, addr_len);

#ifdef LCDC_SUPPORT_DDR_QSPI
            if ((LCDC_INTF_SPI_DCX_DDR_4DATA == lcdc->Init.lcd_itf)
                    && (SPI_LCD_FLAG_DDR_DUMMY_CLOCK & lcdc->Init.cfg.spi.flags))
            {
                HAL_LCDC_SPI_Sequence(lcdc, 0);
            }
            else
#endif /* LCDC_SUPPORT_DDR_QSPI */
            {
                HAL_LCDC_SPI_Sequence(lcdc, 1);
            }
            err = SendLayerData_IT(lcdc);
        }

        //Reset to default after, as DSI need it to send 0x35 cmd.
        lcdc->Next_Frame_TE = NEXT_FRAME_TE_DEFAULT;
        if (HAL_OK != err)
        {
            lcdc->State = HAL_LCDC_STATE_READY;
        }
    }

    if (HAL_OK != err)
    {
        LCDC_TransErrCallback(lcdc, err);
    }
    __HAL_LCDC_UNLOCK(lcdc);
    return err;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_SendLayerData2Reg(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len)
{
    HAL_StatusTypeDef err;

    __HAL_LCDC_LOCK(lcdc);

    if (HAL_LCDC_STATE_READY != lcdc->State)
    {
        err = HAL_BUSY;
    }
    else if (HAL_TIMEOUT == WaitBusy(lcdc))
    {
        err = HAL_TIMEOUT;
    }
    else if (0 == addr_len) //Send data directly if address free
    {
        HAL_LCDC_ASSERT(HAL_LCDC_IS_DPI_IF(lcdc->Init.lcd_itf) || HAL_LCDC_IS_JDI_IF(lcdc->Init.lcd_itf) || HAL_LCDC_IS_AHB_IF(lcdc->Init.lcd_itf));
        lcdc->State = HAL_LCDC_STATE_BUSY;
        err = SendLayerData(lcdc);
        lcdc->State = HAL_LCDC_STATE_READY;
    }
    else
    {
        lcdc->State = HAL_LCDC_STATE_BUSY;

        if (NEXT_FRAME_TE_OFF == lcdc->Next_Frame_TE)
        {
            LCDC_SET_TE(lcdc, false);
        }
        else
        {
            SetupTE(lcdc); //Follow default TE configuration
        }


#ifdef HAL_DSI_MODULE_ENABLED
        if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
        {
            err = DSIMultiWriteStart(lcdc, (uint8_t)(addr & 0xFF), NULL);
        }
        else
#endif /* HAL_DSI_MODULE_ENABLED */
        {

            HAL_LCDC_SPI_Sequence(lcdc, 0);
            SendSingleCmd(lcdc, addr, addr_len);


#ifdef LCDC_SUPPORT_DDR_QSPI
            if ((LCDC_INTF_SPI_DCX_DDR_4DATA == lcdc->Init.lcd_itf)
                    && (SPI_LCD_FLAG_DDR_DUMMY_CLOCK & lcdc->Init.cfg.spi.flags))
            {
                HAL_LCDC_SPI_Sequence(lcdc, 0);
            }
            else
#endif /* LCDC_SUPPORT_DDR_QSPI */
            {
                HAL_LCDC_SPI_Sequence(lcdc, 1);
            }
            err = SendLayerData(lcdc);

        }

        lcdc->Next_Frame_TE = NEXT_FRAME_TE_DEFAULT;
        lcdc->State = HAL_LCDC_STATE_READY;
    }


    __HAL_LCDC_UNLOCK(lcdc);
    return err;

}


#ifdef HAL_RAMLESS_LCD_ENABLED

#ifndef SF32LB55X
#define  p_ptc     hwp_ptc1
#define  p_lcd     hwp_lcdc1
#define  PTC_IRQ_NUM     PTC1_IRQn

#define PTC_BTIM_UPDATE      PTC_HCPU_BTIM2_UPDATE
#define PTC_btim   hwp_btim2
#define BTIM_RCC_MOD  RCC_MOD_BTIM2
#ifdef SF32LB58X
    #define  p_DMACH0  DMA1_Channel7
    #define  p_DMACH1  DMA1_Channel5
    #define PTC_DMACH0_TC PTC_HCPU_DMAC1_DONE7
    #define PTC_DMACH1_TC PTC_HCPU_DMAC1_DONE5
#else
    #define  p_DMACH0  DMA1_Channel8
    #define  p_DMACH1  DMA1_Channel6
    #define PTC_DMACH0_TC PTC_HCPU_DMAC1_DONE8
    #define PTC_DMACH1_TC PTC_HCPU_DMAC1_DONE6

#endif /* SF32LB58X */


#define PTC_TABLE_WORD  (8*4)
#define PTC_TABLE_BYTE  PTC_TABLE_WORD*4
#define PTC_CODE_PHASE_MAX 16
#define RAMLESS_STOP_MAGIC_FLAG 0xaabbccdd
#if (RAMLESS_AUTO_REFR_CODE_SIZE_IN_WORD != (PTC_CODE_PHASE_MAX*PTC_TABLE_WORD))
    #error "Export PTC_CODE_SIZE is incorrect!"
#endif

#define PTC_PHASE_INIT()  uint8_t ptc_phase =0, ptc_pahse_i=0;
#define PTC_PHASE_ADDR(phase) (lcdc->ptc_code + (PTC_TABLE_WORD * (phase)))
#define PTC_PHASE_START(phase) do{ptc_phase = (phase); ptc_pahse_i = 0;}while(0)
#define PTC_CODE_EXT(trigger, delay, repeat, addr, op, data) do{\
        uint32_t _ptc_ch = ptc_pahse_i/4;                         \
        uint32_t _ptc_reg = (((op) << PTC_TCR1_OP_Pos) | (trigger));\
        _ptc_reg |= ((repeat) > 0) ? (PTC_TCR1_REPEN|PTC_TCR1_REPTRIG) : 0; \
        if((delay)>0) HAL_LCDC_ASSERT(_ptc_ch < 4);                \
        PTC_PHASE_ADDR(ptc_phase)[ptc_pahse_i++] = (uint32_t) _ptc_reg; \
        PTC_PHASE_ADDR(ptc_phase)[ptc_pahse_i++] = (uint32_t) (addr);            \
        PTC_PHASE_ADDR(ptc_phase)[ptc_pahse_i++] = (uint32_t) (data);            \
        PTC_PHASE_ADDR(ptc_phase)[ptc_pahse_i++] = (uint32_t) (((delay) << PTC_RCR1_DLY_Pos) | ((repeat) << PTC_RCR1_REP_Pos));  \
        }while(0)


#define PTC_CODE(trigger, addr, op, data)  PTC_CODE_EXT(trigger, 0, 0, addr, op, data)


#define PTC_CODE_SET_PHASE(trigger,phase)   PTC_CODE_EXT((trigger), 0, 0, &(p_DMACH0->CM0AR), PTC_OP_WRITE, PTC_PHASE_ADDR(phase))
#define PTC_CODE_SET_NEXT_PHASE(trigger)    PTC_CODE_EXT((trigger), 0, 0, &(p_DMACH0->CM0AR), PTC_OP_ADD, PTC_TABLE_BYTE)
#define PTC_CODE_JUMP_PHASE(trigger)        PTC_CODE_EXT((trigger), 0, 0, &(p_DMACH0->CNDTR), PTC_OP_WRITE, PTC_TABLE_WORD)
#define PTC_CODE_DELAY_TO_JUMP_PHASE(delay, trigger)        PTC_CODE_EXT((trigger), (delay), 0, &(p_DMACH0->CNDTR), PTC_OP_WRITE, PTC_TABLE_WORD)
#define PTC_CODE_JUMP_SUB_PHASE(trigger)        PTC_CODE_EXT((trigger), 0, 0, &(p_DMACH0->CNDTR), PTC_OP_WRITE, PTC_TABLE_WORD - (4 * 5))
#define PTC_CODE_DUMMY()        PTC_CODE_EXT(0, 0, 0, 0, 0, 0)
#define PTC_CODE_LOOP_RESERVED()        PTC_CODE_EXT(0, 0, 0, 0, 0, 0)


#define PTC_PHASE_ENDS(phase) HAL_LCDC_ASSERT(PTC_TABLE_WORD == ptc_pahse_i)
static void RAMLESS_RST_HW_FSM(void)
{
    HAL_RCC_ResetModule(RCC_MOD_PTC1);
    //stop PTC, clear btim2/dmac1/ptc1
    PTC_btim->CR1 &= ~BTIM_CR1_CEN;
    p_DMACH1->CCR &= ~DMAC_CCR2_EN;
    p_DMACH0->CCR &= ~DMAC_CCR1_EN;
}

static void RAMLESS_HW_FSM_START(LCDC_HandleTypeDef *lcdc)
{
    uint32_t prescaler_value_1us;
    uint32_t ptc_delay_1us = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU) / 1000000;


    uint32_t single_cmd_cost_us = (32 * 2) / (lcdc->Init.freq / 1000000) ;   /*Three 32bit single cmd cost time*/


    uint16_t line_num = lcdc->roi.y1 - lcdc->roi.y0 + 1; //10;
    uint16_t front_porch_num = lcdc->ramless.front_porch; //5-1;
    uint16_t back_porch_num = lcdc->ramless.back_porch; //3-1;

    uint16_t porch_interval = lcdc->ramless.porch_line_gap - 1; //40-1;
    uint16_t line_interval = lcdc->ramless.data_line_gap - 1; //5-1;
    uint16_t frame_interval = lcdc->ramless.frame_gap - 1; //300-1;
    uint16_t cmd_duration = single_cmd_cost_us - 1; //2-1;


    uint16_t porch_interval_tick = lcdc->ramless.porch_line_gap * ptc_delay_1us;
    uint16_t line_interval_tick = lcdc->ramless.data_line_gap * ptc_delay_1us;
    uint16_t cmd_duration_tick = single_cmd_cost_us * ptc_delay_1us;



    uint32_t frame_cmd = lcdc->ramless.frame_cmd; //0xDE006100
    uint32_t line_cmd  = lcdc->ramless.line_cmd; //0xDE006000;
    uint32_t porch_line_cmd = lcdc->ramless.porch_cmd;
    uint32_t lcdc_single_wr = (0 << LCD_IF_LCD_SINGLE_TYPE_Pos) |
                              (1 << LCD_IF_LCD_SINGLE_WR_TRIG_Pos) |
                              (0 << LCD_IF_LCD_SINGLE_RD_TRIG_Pos);


    uint16_t half_screen_lines = line_num / 2;

    if (PTC_btim == hwp_btim1)
        prescaler_value_1us = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1) / 1000000 - 1;
    else
        prescaler_value_1us = 24 - 1; //Fixed 24MHz

    HAL_RCC_EnableModule(BTIM_RCC_MOD);
    HAL_RCC_EnableModule(RCC_MOD_PTC1);

    RAMLESS_RST_HW_FSM();

    //Set canvas area as 1-line at roi.y0
    lcdc->Instance->CANVAS_TL_POS = (lcdc->roi.x0 << LCD_IF_CANVAS_TL_POS_X0_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_TL_POS_Y0_Pos);
    lcdc->Instance->CANVAS_BR_POS = (lcdc->roi.x1 << LCD_IF_CANVAS_BR_POS_X1_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_BR_POS_Y1_Pos);

///////////////////////////////////
// init btim/dmac/busmon         //
///////////////////////////////////
    PTC_btim->CR1 |= BTIM_CR1_OPM | BTIM_CR1_URS;
    PTC_btim->PSC = prescaler_value_1us; //set time tick to 1us
    PTC_btim->ARR = porch_interval;
    PTC_btim->EGR |= BTIM_EGR_UG;

    p_DMACH1->CPAR = (uint32_t) & (hwp_ptc1->TCR1);
    p_DMACH1->CM0AR = (uint32_t) PTC_PHASE_ADDR(0);
    p_DMACH1->CNDTR = 0;
    p_DMACH1->CCR = DMAC_CCR1_MEM2MEM | DMAC_CCR1_DIR;
    p_DMACH1->CCR |= DMAC_CCR1_TCIE | (3 << DMAC_CCR1_PL_Pos);
    p_DMACH1->CCR |= DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos);
    p_DMACH1->CCR |= DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos);
    p_DMACH1->CCR |= DMAC_CCR1_EN;



    p_DMACH0->CPAR = (uint32_t) & (hwp_ptc1->TCR1);
    p_DMACH0->CM0AR = (uint32_t) PTC_PHASE_ADDR(0);
    p_DMACH0->CNDTR = 0;
    p_DMACH0->CCR = DMAC_CCR1_MEM2MEM | DMAC_CCR1_DIR;
    p_DMACH0->CCR |= DMAC_CCR1_TCIE | (3 << DMAC_CCR1_PL_Pos);
    p_DMACH0->CCR |= DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos);
    p_DMACH0->CCR |= DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos);
    p_DMACH0->CCR |= DMAC_CCR1_EN;

    //hwp_ptc1->MEM4 = 0;
    hwp_ptc1->IER = 0;



    ///////////////////////////////////
    // prepare PTC code //
    ///////////////////////////////////
    PTC_PHASE_INIT();
    ///////////////////////////////////
    // FRAME START   //
    ///////////////////////////////////
    PTC_PHASE_START(0);
#if 0
    PTC_CODE_EXT(0, 0,  1, &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE_EXT(PTC_HCPU_PTC1_DONE1,        cmd_duration_tick + porch_interval_tick,  1, &(hwp_ptc1->TCR1),             PTC_OP_OR,   PTC_TCR1_SWTRIG);
    PTC_CODE_SET_NEXT_PHASE(PTC_HCPU_PTC1_DONE2);
    PTC_CODE_DELAY_TO_JUMP_PHASE(cmd_duration_tick, PTC_HCPU_PTC1_DONE2);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, frame_cmd);    //set command,
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_ptc1->TCR1),             PTC_OP_OR,   PTC_TCR1_SWTRIG);    //start loop,
#endif
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, frame_cmd);    //set command,
    PTC_CODE(PTC_HCPU_PTC1_DONE1,        &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE_EXT(PTC_HCPU_PTC1_DONE2, cmd_duration_tick + porch_interval_tick, 1, 0,      0, 0);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_SET_NEXT_PHASE(PTC_HCPU_PTC1_DONE3);
    PTC_CODE_JUMP_PHASE(PTC_HCPU_PTC1_DONE3);

    PTC_PHASE_ENDS(0);

    ///////////////////////////////////
    // FRONT PORCH   //
    ///////////////////////////////////
    PTC_PHASE_START(1);
#if 1
    PTC_CODE_EXT(0, 0,  0, &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE_EXT(PTC_HCPU_PTC1_DONE1,        cmd_duration_tick + porch_interval_tick,  front_porch_num - 1, &(hwp_ptc1->TCR1),             PTC_OP_OR,   PTC_TCR1_SWTRIG);
    PTC_CODE_SET_NEXT_PHASE(PTC_HCPU_PTC1_DONE2);
    PTC_CODE_DELAY_TO_JUMP_PHASE(cmd_duration_tick + porch_interval_tick, PTC_HCPU_PTC1_DONE2);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, porch_line_cmd);    //set command,
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_ptc1->TCR1),             PTC_OP_OR,   PTC_TCR1_SWTRIG);    //start loop,

#else
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, porch_line_cmd);    //set command,
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_ptc1->TCR4),             PTC_OP_OR,   PTC_TCR1_SWTRIG);    //start loop,
    PTC_CODE_EXT(PTC_HCPU_PTC1_DONE4, cmd_duration_tick + porch_interval_tick, front_porch_num, &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);  //send command,
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_SET_NEXT_PHASE(PTC_HCPU_PTC1_DONE3);
    PTC_CODE_JUMP_PHASE(PTC_HCPU_PTC1_DONE3);
#endif
    PTC_PHASE_ENDS(1);


    ///////////////////////////////////
    // Top half screen   //
    ///////////////////////////////////
    //phase 2 for line cmd, loop start
#if 0
    {
        PTC_PHASE_START(2);
        PTC_CODE(PTC_DMACH0_TC,      &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, line_cmd);    //set command,
        PTC_CODE(PTC_DMACH0_TC,        &(hwp_ptc1->TCR4),             PTC_OP_OR,   PTC_TCR1_SWTRIG);    //Loop variable + 1
        PTC_CODE_DUMMY(); //Dummy code

        PTC_CODE_EXT(0, 0, half_screen_lines, &(p_DMACH0->CM0AR), PTC_OP_WRITE, PTC_PHASE_ADDR(3));
        PTC_CODE_EXT(PTC_HCPU_PTC1_DONE4, 0, half_screen_lines, &(p_DMACH0->CNDTR), PTC_OP_WRITE, PTC_TABLE_WORD - 20);
        PTC_CODE_EXT(PTC_HCPU_PTC1_DONE5, 0, half_screen_lines + 1, 0, 0, 0); //Loop variable
        PTC_CODE(PTC_HCPU_PTC1_DONE6, &(p_DMACH0->CM0AR), PTC_OP_WRITE, PTC_PHASE_ADDR(11)); //Loop variable
        PTC_CODE_JUMP_PHASE(PTC_HCPU_PTC1_DONE7);//Loop variable
        PTC_PHASE_ENDS(2);

#if 1
        PTC_PHASE_START(3);
        PTC_CODE_EXT(PTC_DMACH0_TC,   line_interval_tick, 0,   &(hwp_lcdc1->COMMAND),         PTC_OP_WRITE, 0x1);    //Send lcdc line data
        PTC_CODE_EXT(PTC_DMACH0_TC,   line_interval_tick, 0,    &(hwp_ptc1->TCR4),             PTC_OP_OR,   PTC_TCR1_SWTRIG);    //Loop variable + 1
        PTC_CODE_DUMMY(); //Dummy code

        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_PHASE_ENDS(3);

#else
        //phase 3 for line data
        PTC_PHASE_START(3);
        PTC_CODE(PTC_DMACH0_TC,      &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_AND, ~LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //hold spi_cs,
        PTC_CODE(PTC_DMACH0_TC,      &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
        PTC_CODE_EXT(PTC_DMACH0_TC,  cmd_duration_tick,  0,       &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_OR, LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //release spi_cs,
        PTC_CODE_EXT(PTC_DMACH0_TC,  cmd_duration_tick,  0,        &(hwp_lcdc1->COMMAND),         PTC_OP_WRITE, 0x1);    //Send lcdc line data

        PTC_CODE_SET_NEXT_PHASE(PTC_HCPU_LCDC1_DONE);
        PTC_CODE_JUMP_SUB_PHASE(PTC_HCPU_PTC1_DONE5);
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_PHASE_ENDS(3);
#endif

        //phase 4 for line cfg
        PTC_PHASE_START(4);
        PTC_CODE(PTC_DMACH0_TC,     &(hwp_lcdc1->CANVAS_TL_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_TL_POS_Y0_Pos);    //set data address for next line,
        PTC_CODE(PTC_DMACH0_TC,     &(hwp_lcdc1->CANVAS_BR_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_BR_POS_Y1_Pos);    //set data address for next line,
        PTC_CODE_EXT(PTC_DMACH0_TC,        line_interval_tick,   0, 0, 0, 0);//Line interval
        PTC_CODE_DUMMY(); //Dummy code

        PTC_CODE_SET_NEXT_PHASE(PTC_HCPU_PTC1_DONE3);
        PTC_CODE_JUMP_SUB_PHASE(PTC_HCPU_PTC1_DONE3);
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_PHASE_ENDS(4);


        //phase 5 for line done
        PTC_PHASE_START(5);
        PTC_CODE(PTC_DMACH0_TC,        &(hwp_ptc1->TCR7),             PTC_OP_OR,   PTC_TCR1_SWTRIG);    //Loop variable + 1
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE_DUMMY(); //Dummy code

        PTC_CODE_SET_PHASE(PTC_DMACH0_TC, 2);
        PTC_CODE_JUMP_SUB_PHASE(PTC_DMACH0_TC);
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_CODE_LOOP_RESERVED(); //Reserved for loop
        PTC_PHASE_ENDS(5);
    }
#else
    {
        PTC_PHASE_START(2);
        PTC_CODE(PTC_DMACH0_TC,    &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, line_cmd);    //set command,
        PTC_CODE(PTC_DMACH0_TC,    &(p_DMACH1->CM0AR), PTC_OP_WRITE, PTC_PHASE_ADDR(4));
        PTC_CODE(PTC_DMACH0_TC,    &(PTC_btim->ARR),             PTC_OP_WRITE, line_interval);
        PTC_CODE(PTC_DMACH0_TC,    &(PTC_btim->CR1),             PTC_OP_OR, BTIM_CR1_CEN);
        PTC_CODE_DUMMY();
        PTC_CODE_DUMMY();
        PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
        PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
        PTC_PHASE_ENDS(2);


        //phase 3 for line data
        PTC_PHASE_START(3);
        PTC_CODE(PTC_BTIM_UPDATE,      &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_AND, ~LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //hold spi_cs,
        PTC_CODE(PTC_BTIM_UPDATE,      &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
        PTC_CODE_EXT(PTC_BTIM_UPDATE,  cmd_duration_tick,  0,       &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_OR, LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //release spi_cs,
        PTC_CODE_EXT(PTC_BTIM_UPDATE,  cmd_duration_tick,  0,        &(hwp_lcdc1->COMMAND),         PTC_OP_WRITE, 0x1);    //Send lcdc line data
        PTC_CODE(PTC_HCPU_LCDC1_DONE,     &(hwp_lcdc1->CANVAS_TL_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_TL_POS_Y0_Pos);    //set data address for next line,
        PTC_CODE(PTC_HCPU_LCDC1_DONE,     &(hwp_lcdc1->CANVAS_BR_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_BR_POS_Y1_Pos);    //set data address for next line,
        PTC_CODE_EXT(PTC_HCPU_LCDC1_DONE,    0, half_screen_lines - 2,   &(PTC_btim->CR1),             PTC_OP_OR, BTIM_CR1_CEN);
        PTC_CODE_EXT(PTC_HCPU_PTC1_DONE7, 0, 0, &(p_DMACH1->CNDTR), PTC_OP_WRITE, PTC_TABLE_WORD);
        PTC_PHASE_ENDS(3);


        PTC_PHASE_START(4);
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE(PTC_DMACH1_TC,  &(hwp_ptc1->MEM4),              PTC_OP_WRITE, half_screen_lines);
        PTC_CODE_DUMMY();
        PTC_CODE_DUMMY();
        PTC_CODE_SET_PHASE(PTC_DMACH1_TC, 5);
        PTC_CODE_JUMP_PHASE(PTC_DMACH1_TC);
        PTC_PHASE_ENDS(4);

        PTC_PHASE_START(5);
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE_DUMMY(); //Dummy code
        PTC_CODE_DUMMY();
        PTC_CODE_DUMMY();
        PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
        PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
        PTC_PHASE_ENDS(5);
    }
#endif
    ///////////////////////////////////
    // Bottom half screen   //
    ///////////////////////////////////
#if 1
    PTC_PHASE_START(6);
    PTC_CODE(PTC_DMACH0_TC,    &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, line_cmd);    //set command,
    PTC_CODE(PTC_DMACH0_TC,    &(p_DMACH1->CM0AR), PTC_OP_WRITE, PTC_PHASE_ADDR(8));
    PTC_CODE(PTC_DMACH0_TC,    &(PTC_btim->ARR),             PTC_OP_WRITE, line_interval);
    PTC_CODE(PTC_DMACH0_TC,    &(PTC_btim->CR1),             PTC_OP_OR, BTIM_CR1_CEN);
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(6);


    //phase 3 for line data
    PTC_PHASE_START(7);
    PTC_CODE(PTC_BTIM_UPDATE,      &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_AND, ~LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //hold spi_cs,
    PTC_CODE(PTC_BTIM_UPDATE,      &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE_EXT(PTC_BTIM_UPDATE,  cmd_duration_tick,  0,       &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_OR, LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //release spi_cs,
    PTC_CODE_EXT(PTC_BTIM_UPDATE,  cmd_duration_tick,  0,        &(hwp_lcdc1->COMMAND),         PTC_OP_WRITE, 0x1);    //Send lcdc line data
    PTC_CODE(PTC_HCPU_LCDC1_DONE,     &(hwp_lcdc1->CANVAS_TL_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_TL_POS_Y0_Pos);    //set data address for next line,
    PTC_CODE(PTC_HCPU_LCDC1_DONE,     &(hwp_lcdc1->CANVAS_BR_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_BR_POS_Y1_Pos);    //set data address for next line,
    PTC_CODE_EXT(PTC_HCPU_LCDC1_DONE,    0, half_screen_lines - 2,   &(PTC_btim->CR1),             PTC_OP_OR, BTIM_CR1_CEN);
    PTC_CODE_EXT(PTC_HCPU_PTC1_DONE7, 0, 0, &(p_DMACH1->CNDTR), PTC_OP_WRITE, PTC_TABLE_WORD);
    PTC_PHASE_ENDS(7);



    //phase 8 for line data
    PTC_PHASE_START(8);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE(PTC_DMACH1_TC,  &(hwp_ptc1->MEM4),              PTC_OP_WRITE, 0);    //clear counter,
    PTC_CODE_DUMMY();
    PTC_CODE_SET_PHASE(PTC_DMACH1_TC, 9);
    PTC_CODE_JUMP_PHASE(PTC_DMACH1_TC);
    PTC_PHASE_ENDS(8);


    //phase 9 for line cfg
    PTC_PHASE_START(9);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(9);


    //phase 10 for line done
    PTC_PHASE_START(10);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(10);

#else
    //phase 6 for half frame done
    PTC_PHASE_START(6);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE(PTC_DMACH0_TC,      &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, line_cmd);    //set command,
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_DMACH0_TC,      &(PTC_btim->CR1),              PTC_OP_OR, BTIM_CR1_CEN);    //start line interval,
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(6);


    //phase 7 for line cmd
    PTC_PHASE_START(7);
    PTC_CODE_EXT(PTC_BTIM_UPDATE,  cmd_duration_tick,  0,       &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_OR, LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //release spi_cs,
    PTC_CODE_EXT(PTC_BTIM_UPDATE,  cmd_duration_tick,  0,        &(hwp_lcdc1->COMMAND),         PTC_OP_WRITE, 0x1);    //Send lcdc line data
    PTC_CODE(PTC_BTIM_UPDATE,      &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_AND, ~LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //hold spi_cs,
    PTC_CODE(PTC_BTIM_UPDATE,      &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,

    PTC_CODE(PTC_HCPU_LCDC1_DONE,     &(hwp_lcdc1->CANVAS_TL_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_TL_POS_Y0_Pos);    //set data address for next line,
    PTC_CODE(PTC_HCPU_LCDC1_DONE,     &(hwp_lcdc1->CANVAS_BR_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_BR_POS_Y1_Pos);    //set data address for next line,
    PTC_CODE_EXT(PTC_HCPU_LCDC1_DONE,     0, (line_num - half_screen_lines - 1),    &(PTC_btim->CR1),              PTC_OP_OR, BTIM_CR1_CEN);     //start line interval,
    PTC_CODE(PTC_HCPU_PTC1_DONE6,              &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(7);


    //phase 8 for line data
    PTC_PHASE_START(8);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(8);


    //phase 9 for line cfg
    PTC_PHASE_START(9);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(9);


    //phase 10 for line done
    PTC_PHASE_START(10);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(10);

#endif


    ///////////////////////////////////
    // BACK_PORCH   //
    ///////////////////////////////////
    //phase 11 for the first command of back porch
    PTC_PHASE_START(11);
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, porch_line_cmd);    //set command,
    PTC_CODE(PTC_DMACH0_TC,        &(hwp_ptc1->TCR3),             PTC_OP_OR,   PTC_TCR1_SWTRIG);    //start loop,
    PTC_CODE_EXT(PTC_HCPU_PTC1_DONE3,    cmd_duration_tick + porch_interval_tick,   0, 0, 0, 0);  //start loop,
    PTC_CODE_EXT(PTC_HCPU_PTC1_DONE3,        0,  back_porch_num, &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_SET_NEXT_PHASE(PTC_HCPU_PTC1_DONE4);
    PTC_CODE_JUMP_PHASE(PTC_HCPU_PTC1_DONE4);
    PTC_PHASE_ENDS(11);


    //phase 12 for subsequent command of back porch
    PTC_PHASE_START(12);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(12);


    ///////////////////////////////////
    // RESTORE CANVAS   //
    ///////////////////////////////////
    //phase 13 for frame done

    PTC_PHASE_START(13);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->MEM4),              PTC_OP_WRITE, 0);    //clear counter,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->ICR),               PTC_OP_WRITE, PTC_ICR_CTCIF4);    //clear interrupt,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->IER),               PTC_OP_OR, PTC_IER_TCIE4);    //enable interrupt,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->IER),               PTC_OP_AND, ~PTC_IER_TCIE4);         //disable interrupt,
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_SET_NEXT_PHASE(PTC_DMACH0_TC);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(13);


    //phase 14 for next frame
    PTC_PHASE_START(14);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->CANVAS_TL_POS),   PTC_OP_WRITE, (lcdc->roi.x0 << LCD_IF_CANVAS_TL_POS_X0_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_TL_POS_Y0_Pos));    //init address for next frame,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->CANVAS_BR_POS),   PTC_OP_WRITE, (lcdc->roi.x1 << LCD_IF_CANVAS_BR_POS_X1_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_BR_POS_Y1_Pos));    //init address for next frame,
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_DMACH0_TC,  &(PTC_btim->ARR),             PTC_OP_WRITE, frame_interval);    //set frame interval,
    PTC_CODE(PTC_DMACH0_TC,  &(PTC_btim->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start frame interval,
    PTC_CODE_SET_NEXT_PHASE(PTC_BTIM_UPDATE);
    PTC_CODE_JUMP_PHASE(PTC_BTIM_UPDATE);
    PTC_PHASE_ENDS(14);

    //phase 15 reserved for Stop FSM overwrite
    PTC_PHASE_START(15);
    PTC_CODE_DUMMY(); //Dummy code
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_SET_PHASE(PTC_DMACH0_TC, 0);
    PTC_CODE_JUMP_PHASE(PTC_DMACH0_TC);
    PTC_PHASE_ENDS(15);



    //Start!!!
    memcpy((void *) & (hwp_ptc1->TCR1), (void *)PTC_PHASE_ADDR(0), PTC_TABLE_BYTE);
    hwp_ptc1->TCR1 |= PTC_TCR1_SWTRIG;



}

static void RAMLESS_HW_FSM_STOP(LCDC_HandleTypeDef *lcdc)
{
    //Overwrite phase 15 to Stop FSM
    hwp_ptc1->MEM3 = RAMLESS_STOP_MAGIC_FLAG;
    PTC_PHASE_INIT();

    //Stop it at end of one frame
    PTC_PHASE_START(PTC_CODE_PHASE_MAX - 1);
    PTC_CODE(PTC_DMACH0_TC,  &(PTC_btim->CR1),    PTC_OP_AND,   ~BTIM_CR1_CEN);    //Stop BTIM2
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->MEM3),    PTC_OP_WRITE, 0); //Set MEM3 0
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CCR),   PTC_OP_WRITE, ~DMAC_CCR8_EN);    //Stop dma
    PTC_PHASE_ENDS(PTC_CODE_PHASE_MAX - 1);

    for (uint32_t start_tick = HAL_GetTick(); hwp_ptc1->MEM3;)
    {
        if (HAL_GetTick() - start_tick > LCDC_TIMEOUT_SECONDS * 1000)
        {
            break;
        }
    }

    HAL_LCDC_ASSERT(RAMLESS_STOP_MAGIC_FLAG != hwp_ptc1->MEM3);

}




#else /*SF32LB55X*/

#define  p_ptc     hwp_ptc1
#define  p_lcd     hwp_lcdc1
#define  PTC_IRQ_NUM     PTC1_IRQn

#define PTC_BTIM1       PTC_HCPU_BTIM1_UPDATE
#define PTC_BTIM2       PTC_HCPU_BTIM2_UPDATE

#define PTC_DMACH0_IFCR     DMAC_IFCR_CTCIF8
#define PTC_DMACH0_TC   PTC_HCPU_DMAC1_DONE8
#define  p_DMACH0  DMA1_Channel8

#ifdef hwp_busmon1
    #define PTC_BUSMON1_OF7 PTC_HCPU_BUSMON1_OF7
#endif /* hwp_busmon1 */
#define PTC_DONE6       PTC_HCPU_PTC1_DONE6
#define PTC_DONE7       PTC_HCPU_PTC1_DONE7



#define PTC_TABLE_WORD  24
#define PTC_TABLE_BYTE  PTC_TABLE_WORD*4
#define PTC_CODE_PHASE_MAX 16
#define RAMLESS_STOP_MAGIC_FLAG 0xaabbccdd
#if (RAMLESS_AUTO_REFR_CODE_SIZE_IN_WORD != (PTC_CODE_PHASE_MAX*PTC_TABLE_WORD))
    #error "Export PTC_CODE_SIZE is incorrect!"
#endif

#define PTC_PHASE_INIT()  uint8_t ptc_phase =0, ptc_pahse_i=0;
#define PTC_PHASE_ADDR(phase) (lcdc->ptc_code + (PTC_TABLE_WORD * (phase)))
#define PTC_PHASE_START(phase) do{ptc_phase = (phase); ptc_pahse_i = 0;}while(0)
#define PTC_CODE(trigger, addr, op, data) do{\
        PTC_PHASE_ADDR(ptc_phase)[ptc_pahse_i++] = (uint32_t) (((op) << PTC_TCR1_OP_Pos) | (trigger)); \
        PTC_PHASE_ADDR(ptc_phase)[ptc_pahse_i++] = (uint32_t) (addr);            \
        PTC_PHASE_ADDR(ptc_phase)[ptc_pahse_i++] = (uint32_t) (data);            \
        }while(0)
#define PTC_PHASE_ENDS(phase) HAL_LCDC_ASSERT(PTC_TABLE_WORD == ptc_pahse_i)

#define PTC_LINK_CLEAR   PTC_CODE(PTC_DMACH0_TC, &(hwp_dmac1->IFCR), PTC_OP_OR, PTC_DMACH0_IFCR);
#define PTC_LINK_TO_PHASE(phase, trigger) \
            PTC_CODE((trigger), &(p_DMACH0->CM0AR), PTC_OP_WRITE, PTC_PHASE_ADDR(phase)); \
            PTC_CODE((trigger), &(p_DMACH0->CNDTR), PTC_OP_WRITE, PTC_TABLE_WORD);
#define PTC_CODE_DUMMY()        PTC_CODE(0, 0, 0, 0)

static void RAMLESS_RST_HW_FSM(void)
{
    HAL_RCC_ResetModule(RCC_MOD_PTC1);
    //stop PTC, clear btim1/btim2/dmac1/busmon1/ptc1
    hwp_btim1->CR1 &= ~BTIM_CR1_CEN;
    hwp_btim2->CR1 &= ~BTIM_CR1_CEN;
#ifdef hwp_busmon1
    hwp_busmon1->CCR |= BUSMON_CCR_CLR7;
#endif
    p_DMACH0->CCR &= ~DMAC_CCR8_EN;
}

static void RAMLESS_HW_FSM_START(LCDC_HandleTypeDef *lcdc)
{
    uint32_t prescaler_value_1us = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1) / 1000000 - 1;
    uint32_t single_cmd_cost_us = (32 * 3) / (lcdc->Init.freq / 1000000) ;   /*Three 32bit single cmd cost time*/

#ifdef hwp_busmon1
    uint16_t line_num = lcdc->roi.y1 - lcdc->roi.y0 + 1; //10;
    uint16_t front_porch_num = lcdc->ramless.front_porch - 1; //5-1;
    uint16_t back_porch_num = lcdc->ramless.back_porch - 1; //3-1;
#endif /* hwp_busmon1 */
    uint16_t porch_interval = lcdc->ramless.porch_line_gap - 1; //40-1;
    uint16_t line_interval = lcdc->ramless.data_line_gap - 1; //5-1;
    uint16_t frame_interval = lcdc->ramless.frame_gap - 1; //300-1;
    uint16_t cmd_duration = single_cmd_cost_us - 1; //2-1;

    uint32_t frame_cmd = lcdc->ramless.frame_cmd; //0xDE006100
    uint32_t line_cmd  = lcdc->ramless.line_cmd; //0xDE006000;
    uint32_t porch_line_cmd = lcdc->ramless.porch_cmd;
    uint32_t lcdc_single_wr = (0 << LCD_IF_LCD_SINGLE_TYPE_Pos) |
                              (1 << LCD_IF_LCD_SINGLE_WR_TRIG_Pos) |
                              (0 << LCD_IF_LCD_SINGLE_RD_TRIG_Pos);


#ifdef hwp_busmon1
    uint16_t half_screen_lines = line_num / 2;
#endif /* hwp_busmon1 */

    HAL_RCC_EnableModule(RCC_MOD_BTIM1);
    HAL_RCC_EnableModule(RCC_MOD_BTIM2);
    HAL_RCC_EnableModule(RCC_MOD_PTC1);
    HAL_RCC_EnableModule(RCC_MOD_BUSMON1);

    RAMLESS_RST_HW_FSM();

    //Set canvas area as 1-line at roi.y0
    lcdc->Instance->CANVAS_TL_POS = (lcdc->roi.x0 << LCD_IF_CANVAS_TL_POS_X0_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_TL_POS_Y0_Pos);
    lcdc->Instance->CANVAS_BR_POS = (lcdc->roi.x1 << LCD_IF_CANVAS_BR_POS_X1_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_BR_POS_Y1_Pos);

///////////////////////////////////
// init btim/dmac/busmon         //
///////////////////////////////////
    hwp_btim1->CR1 |= BTIM_CR1_OPM | BTIM_CR1_URS;
    hwp_btim1->PSC = prescaler_value_1us; //set time tick to 1us
    hwp_btim1->ARR = cmd_duration;
    hwp_btim1->EGR |= BTIM_EGR_UG;

    hwp_btim2->CR1 |= BTIM_CR1_OPM | BTIM_CR1_URS;
    hwp_btim2->PSC = prescaler_value_1us; //set time tick to 1us
    hwp_btim2->ARR = porch_interval;
    hwp_btim2->EGR |= BTIM_EGR_UG;

    p_DMACH0->CPAR = (uint32_t) & (hwp_ptc1->TCR1);
    p_DMACH0->CM0AR = (uint32_t) PTC_PHASE_ADDR(0);
    p_DMACH0->CNDTR = 0;
    p_DMACH0->CCR = DMAC_CCR8_MEM2MEM | DMAC_CCR8_DIR;
    p_DMACH0->CCR |= DMAC_CCR8_TCIE | (3 << DMAC_CCR8_PL_Pos);
    p_DMACH0->CCR |= DMAC_CCR8_MINC | (0x2 << DMAC_CCR8_MSIZE_Pos);
    p_DMACH0->CCR |= DMAC_CCR8_PINC | (0x2 << DMAC_CCR8_PSIZE_Pos);
    p_DMACH0->CCR |= DMAC_CCR8_EN;

    //hwp_ptc1->MEM4 = 0;
    hwp_ptc1->IER = 0;

#ifdef hwp_busmon1
    hwp_busmon1->CR7 = (front_porch_num << BUSMON_CR7_CMP_Pos) | BUSMON_CR7_CMPEN | BUSMON_CR7_AUTOCLR | (0x1 << BUSMON_CR7_OP_Pos) | 6; //select apb write
    hwp_busmon1->MIN7 = (uint32_t) & (hwp_ptc1->MEM4);
    hwp_busmon1->MAX7 = (uint32_t) & (hwp_ptc1->MEM4) + 4;
    hwp_busmon1->CCR |= BUSMON_CCR_CLR7;
    hwp_busmon1->CER |= BUSMON_CER_EN7; //enable channel 7
#endif


    PTC_PHASE_INIT();
    ///////////////////////////////////
    // prepare PTC code, FRONT PORCH              //
    ///////////////////////////////////
    //phase 0 for the first command of front porch
    PTC_PHASE_START(0);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),            PTC_OP_OR,     PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_BTIM2,      &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE,  frame_cmd);    //set command,
    PTC_CODE(PTC_BTIM2,      &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE,  lcdc_single_wr);    //send command,
    PTC_CODE(PTC_BTIM2,      &(hwp_btim1->CR1),             PTC_OP_OR,     BTIM_CR1_CEN);    //start command duration,
    PTC_CODE(PTC_BTIM1,      &(hwp_btim2->ARR),             PTC_OP_WRITE,   porch_interval);    //set porch interval,
    PTC_CODE(PTC_BTIM1,      &(hwp_btim2->CR1),             PTC_OP_OR,      BTIM_CR1_CEN);    //start porch interval,
    PTC_CODE(PTC_BTIM1,      &(p_DMACH0->CM0AR),          PTC_OP_ADD,     PTC_TABLE_BYTE);    //link to next phase,
    PTC_CODE(PTC_BTIM1,      &(p_DMACH0->CNDTR),          PTC_OP_WRITE,   PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(0);

    //phase 1 for subsequent command of front porch
    PTC_PHASE_START(1);
    PTC_CODE(PTC_DMACH0_TC,    &(hwp_dmac1->IFCR),            PTC_OP_OR,    PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_BTIM2,        &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, porch_line_cmd);    //set command,
    PTC_CODE(PTC_BTIM2,        &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE(PTC_BTIM2,        &(hwp_btim1->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start command duration,
    PTC_CODE(PTC_BTIM1,        &(hwp_btim2->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start porch interval,
    PTC_CODE(PTC_BTIM1,        &(hwp_ptc1->MEM4),             PTC_OP_ADD, 1);    //add counter,
    PTC_CODE(PTC_BUSMON1_OF7,  &(p_DMACH0->CM0AR),          PTC_OP_ADD, PTC_TABLE_BYTE);    //link to next phase,
    PTC_CODE(PTC_BUSMON1_OF7,  &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(1);

    ///////////////////////////////////
    // VALID DATA LINES   //
    ///////////////////////////////////
    //phase 2 for line cmd
    PTC_PHASE_START(2);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),            PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_BTIM2,      &(hwp_lcdc1->LCD_WR),          PTC_OP_WRITE, line_cmd);    //set command,
    PTC_CODE(PTC_BTIM2,      &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_AND, ~LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //hold spi_cs,
    PTC_CODE(PTC_BTIM2,      &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE(PTC_BTIM2,      &(hwp_btim1->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start command duration,
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_BTIM1,      &(p_DMACH0->CM0AR),          PTC_OP_ADD, PTC_TABLE_BYTE);    //link to next phase,
    PTC_CODE(PTC_BTIM1,      &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(2);


    //phase 3 for line data
    PTC_PHASE_START(3);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),            PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->SPI_IF_CONF),     PTC_OP_OR, LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //release spi_cs,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->COMMAND),         PTC_OP_WRITE, 0x1);    //trigger lcdc line,
#ifdef hwp_busmon1
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_busmon1->CR7),           PTC_OP_WRITE, (half_screen_lines << BUSMON_CR7_CMP_Pos) | BUSMON_CR7_CMPEN | BUSMON_CR7_AUTOCLR | (0x1 << BUSMON_CR7_OP_Pos) | 6);    //set line counter limit,
#endif
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_HCPU_LCDC1_DONE,  &(p_DMACH0->CM0AR),          PTC_OP_ADD, PTC_TABLE_BYTE);    //link to next phase,
    PTC_CODE(PTC_HCPU_LCDC1_DONE,  &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(3);


    //phase 4 for line cfg
    PTC_PHASE_START(4);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),            PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->CANVAS_TL_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_TL_POS_Y0_Pos);    //set data address for next line,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->CANVAS_BR_POS),   PTC_OP_ADD, 1 << LCD_IF_CANVAS_BR_POS_Y1_Pos);    //set data address for next line,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_btim2->ARR),             PTC_OP_WRITE, line_interval);    //set line interval,
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CM0AR),          PTC_OP_ADD, PTC_TABLE_BYTE);    //link to next phase,
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(4);


    //phase 5 for line done
    PTC_PHASE_START(5);
    PTC_CODE(PTC_BUSMON1_OF7,  &(hwp_ptc1->TCR7),             PTC_OP_WRITE, 0);    //stop jumping to phase 2 if line counter limit reached,
    PTC_CODE(PTC_BUSMON1_OF7,  &(p_DMACH0->CM0AR),          PTC_OP_ADD, PTC_TABLE_BYTE);    //link to next phase,
    PTC_CODE(PTC_BUSMON1_OF7,  &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_CODE(PTC_DMACH0_TC,    &(hwp_dmac1->IFCR),            PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,    &(hwp_ptc1->MEM4),             PTC_OP_ADD, 1);    //add line counter,
    PTC_CODE(PTC_DMACH0_TC,    &(hwp_btim2->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start line interval,
    PTC_CODE(PTC_DONE6,        &(p_DMACH0->CM0AR),          PTC_OP_WRITE,  PTC_PHASE_ADDR(2));    //jump to phase 2 to start a new line,
    PTC_CODE(PTC_DONE7,        &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //jump to phase 2,
    PTC_PHASE_ENDS(5);


    ///////////////////////////////////
    // Bottom half screen   //
    ///////////////////////////////////

    //phase 6 for half frame done
    PTC_PHASE_START(6);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),             PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->ICR),               PTC_OP_WRITE, PTC_ICR_CTCIF4);    //clear interrupt,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->IER),               PTC_OP_OR, PTC_IER_TCIE4);    //enable interrupt,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->IER),               PTC_OP_AND, ~PTC_IER_TCIE4);         //disable interrupt,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_btim2->CR1),              PTC_OP_OR, BTIM_CR1_CEN);    //start line interval,
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CM0AR),           PTC_OP_ADD, PTC_TABLE_BYTE);         //link to next phase,
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CNDTR),           PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(6);


    //phase 7 for line cmd
    PTC_PHASE_START(7);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),              PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_BTIM2,     &(hwp_lcdc1->LCD_WR),           PTC_OP_WRITE, line_cmd);          //set command,
    PTC_CODE(PTC_BTIM2,     &(hwp_lcdc1->SPI_IF_CONF),      PTC_OP_AND, ~LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);         //hold spi_cs,
    PTC_CODE(PTC_BTIM2,     &(hwp_lcdc1->LCD_SINGLE),       PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE(PTC_BTIM2,     &(hwp_btim1->CR1),              PTC_OP_OR, BTIM_CR1_CEN);    //start command duration,
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_BTIM1,     &(p_DMACH0->CM0AR),           PTC_OP_ADD, PTC_TABLE_BYTE);         //link to next phase,
    PTC_CODE(PTC_BTIM1,     &(p_DMACH0->CNDTR),           PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(7);


    //phase 8 for line data
    PTC_PHASE_START(8);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),             PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->SPI_IF_CONF),      PTC_OP_OR, LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS);    //release spi_cs,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->COMMAND),          PTC_OP_WRITE, 0x1);    //trigger lcdc line,
#ifdef hwp_busmon1
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_busmon1->CR7),            PTC_OP_WRITE, ((line_num - half_screen_lines) << BUSMON_CR7_CMP_Pos) | BUSMON_CR7_CMPEN | BUSMON_CR7_AUTOCLR | (0x1 << BUSMON_CR7_OP_Pos) | 6);    //set line counter limit,
#endif
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_HCPU_LCDC1_DONE,  &(p_DMACH0->CM0AR),           PTC_OP_ADD, PTC_TABLE_BYTE);         //link to next phase,
    PTC_CODE(PTC_HCPU_LCDC1_DONE,  &(p_DMACH0->CNDTR),           PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(8);


    //phase 9 for line cfg
    PTC_PHASE_START(9);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),             PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->CANVAS_TL_POS),    PTC_OP_ADD, 1 << LCD_IF_CANVAS_TL_POS_Y0_Pos);         //set data address for next line,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->CANVAS_BR_POS),    PTC_OP_ADD, 1 << LCD_IF_CANVAS_BR_POS_Y1_Pos);         //set data address for next line,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_btim2->ARR),              PTC_OP_WRITE, line_interval);    //set line interval,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->LCD_WR),           PTC_OP_WRITE, porch_line_cmd);    //set command,
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CM0AR),           PTC_OP_ADD, PTC_TABLE_BYTE);         //link to next phase,
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CNDTR),           PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(9);


    //phase 10 for line done
    PTC_PHASE_START(10);
    PTC_CODE(PTC_BUSMON1_OF7,  &(hwp_ptc1->TCR7),              PTC_OP_WRITE, 0);    //stop jumping to phase 7 if line counter limit reached,
    PTC_CODE(PTC_BUSMON1_OF7,  &(p_DMACH0->CM0AR),           PTC_OP_ADD, PTC_TABLE_BYTE);         //link to next phase,
    PTC_CODE(PTC_BUSMON1_OF7,  &(p_DMACH0->CNDTR),           PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_CODE(PTC_DMACH0_TC,    &(hwp_dmac1->IFCR),             PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,    &(hwp_ptc1->MEM4),              PTC_OP_ADD, 1);         //add line counter,
    PTC_CODE(PTC_DMACH0_TC,    &(hwp_btim2->CR1),              PTC_OP_OR, BTIM_CR1_CEN);    //start line interval,
    PTC_CODE(PTC_DONE6,        &(p_DMACH0->CM0AR),           PTC_OP_WRITE,  PTC_PHASE_ADDR(7));    //jump to phase 7 to start a new line,
    PTC_CODE(PTC_DONE7,        &(p_DMACH0->CNDTR),           PTC_OP_WRITE, PTC_TABLE_WORD);    //jump to phase 7,
    PTC_PHASE_ENDS(10);
    ///////////////////////////////////
    // BACK_PORCH   //
    ///////////////////////////////////
    //phase 11 for the first command of back porch
    PTC_PHASE_START(11);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),            PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
#ifdef hwp_busmon1
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_busmon1->CR7),           PTC_OP_WRITE, (back_porch_num << BUSMON_CR7_CMP_Pos) | BUSMON_CR7_CMPEN | BUSMON_CR7_AUTOCLR | (0x1 << BUSMON_CR7_OP_Pos) | 6);    //set porch counter limit,
#endif
    PTC_CODE(PTC_BTIM2,  &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE(PTC_BTIM2,  &(hwp_btim1->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start command duration,
    PTC_CODE(PTC_BTIM1,  &(hwp_btim2->ARR),             PTC_OP_WRITE, porch_interval);    //set porch interval,
    PTC_CODE(PTC_BTIM1,  &(hwp_btim2->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start porch interval,
    PTC_CODE(PTC_BTIM1,  &(p_DMACH0->CM0AR),          PTC_OP_ADD, PTC_TABLE_BYTE);    //link to next phase,
    PTC_CODE(PTC_BTIM1,  &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(11);


    //phase 12 for subsequent command of back porch
    PTC_PHASE_START(12);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),            PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_BTIM2,  &(hwp_lcdc1->LCD_SINGLE),      PTC_OP_WRITE, lcdc_single_wr);    //send command,
    PTC_CODE(PTC_BTIM2,  &(hwp_btim1->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start command duration,
    PTC_CODE(PTC_BTIM1,  &(hwp_btim2->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start porch interval,
    PTC_CODE(PTC_BTIM1,  &(hwp_ptc1->MEM4),             PTC_OP_ADD, 1);    //add counter,
    PTC_CODE(PTC_BUSMON1_OF7,  &(hwp_btim2->CR1),             PTC_OP_AND, ~BTIM_CR1_CEN);    //stop porch interval if porch done,
    PTC_CODE(PTC_BUSMON1_OF7,  &(p_DMACH0->CM0AR),          PTC_OP_ADD, PTC_TABLE_BYTE);    //link to next phase,
    PTC_CODE(PTC_BUSMON1_OF7,  &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(12);


    ///////////////////////////////////
    // send frame repeatly example   //
    ///////////////////////////////////
    //phase 13 for frame done

    PTC_PHASE_START(13);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),             PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->MEM4),              PTC_OP_WRITE, 0);    //clear counter,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->ICR),               PTC_OP_WRITE, PTC_ICR_CTCIF4);    //clear interrupt,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->IER),               PTC_OP_OR, PTC_IER_TCIE4);    //enable interrupt,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->IER),               PTC_OP_AND, ~PTC_IER_TCIE4);         //disable interrupt,
#ifdef hwp_busmon1
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_busmon1->CCR),            PTC_OP_OR, BUSMON_CCR_CLR7);    //clear busmon,
#endif
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CM0AR),           PTC_OP_ADD, PTC_TABLE_BYTE);         //link to next phase,
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CNDTR),           PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(13);


    //phase 14 for next frame
    PTC_PHASE_START(14);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),            PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->CANVAS_TL_POS),   PTC_OP_WRITE, (lcdc->roi.x0 << LCD_IF_CANVAS_TL_POS_X0_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_TL_POS_Y0_Pos));    //init address for next frame,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_lcdc1->CANVAS_BR_POS),   PTC_OP_WRITE, (lcdc->roi.x1 << LCD_IF_CANVAS_BR_POS_X1_Pos) | (lcdc->roi.y0 << LCD_IF_CANVAS_BR_POS_Y1_Pos));    //init address for next frame,
#ifdef hwp_busmon1
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_busmon1->CR7),           PTC_OP_WRITE, (front_porch_num << BUSMON_CR7_CMP_Pos) | BUSMON_CR7_CMPEN | BUSMON_CR7_AUTOCLR | (0x1 << BUSMON_CR7_OP_Pos) | 6);    //set porch counter limit,
#endif
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_btim2->ARR),             PTC_OP_WRITE, frame_interval);    //set frame interval,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_btim2->CR1),             PTC_OP_OR, BTIM_CR1_CEN);    //start frame interval,
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CM0AR),           PTC_OP_ADD, PTC_TABLE_BYTE);         //link to next phase,
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CNDTR),           PTC_OP_WRITE, PTC_TABLE_WORD);    //link to next phase,
    PTC_PHASE_ENDS(14);

    //phase 15 for Stop FSM
    PTC_PHASE_START(15);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),            PTC_OP_OR, PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CM0AR),          PTC_OP_WRITE,  PTC_PHASE_ADDR(0));    //jump to phase 0,
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CNDTR),          PTC_OP_WRITE, PTC_TABLE_WORD);    //jump to phase 0,
    PTC_PHASE_ENDS(15);


    memcpy((void *) & (hwp_ptc1->TCR1), (void *)PTC_PHASE_ADDR(0), PTC_TABLE_BYTE);
    hwp_btim2->ARR = porch_interval;
    hwp_btim2->CR1 |= BTIM_CR1_CEN;



}

static void RAMLESS_HW_FSM_STOP(LCDC_HandleTypeDef *lcdc)
{
    //Overwrite phase 15 to Stop FSM
    hwp_ptc1->MEM3 = RAMLESS_STOP_MAGIC_FLAG;
    PTC_PHASE_INIT();

    //Stop it at end of one frame
    PTC_PHASE_START(PTC_CODE_PHASE_MAX - 1);
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_dmac1->IFCR),   PTC_OP_OR,    PTC_DMACH0_IFCR);    //clear dmac trig,
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_btim1->CR1),    PTC_OP_AND,   ~BTIM_CR1_CEN);    //Stop BTIM1
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_btim2->CR1),    PTC_OP_AND,   ~BTIM_CR1_CEN);    //Stop BTIM2
#ifdef hwp_busmon1
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_busmon1->CR7),  PTC_OP_OR,    BUSMON_CCR_CLR7);    //Stop bus monitor ch7
#endif
    PTC_CODE(PTC_DMACH0_TC,  &(hwp_ptc1->MEM3),    PTC_OP_WRITE, 0); //Set MEM3 0
    PTC_CODE_DUMMY();
    PTC_CODE_DUMMY();
    PTC_CODE(PTC_DMACH0_TC,  &(p_DMACH0->CCR),   PTC_OP_WRITE, ~DMAC_CCR8_EN);    //Stop dma
    PTC_PHASE_ENDS(PTC_CODE_PHASE_MAX - 1);

    for (uint32_t start_tick = HAL_GetTick(); hwp_ptc1->MEM3;)
    {
        if (HAL_GetTick() - start_tick > LCDC_TIMEOUT_SECONDS * 1000)
        {
            break;
        }
    }

    HAL_LCDC_ASSERT(RAMLESS_STOP_MAGIC_FLAG != hwp_ptc1->MEM3);

}

#endif /*!SF32LB55X*/
__weak void HAL_RAMLESS_LCD_HalfScreenCbk(void)
{
}

__weak void HAL_RAMLESS_LCD_FullScreenCbk(void)
{
}

void HAL_RAMLESS_LCD_IRQHandler(LCDC_HandleTypeDef *lcdc)
{
    uint32_t cur_line, half_valid_line_num, isr, ier;
    hwp_ptc1->ICR = 0xFF;
    isr = hwp_ptc1->ISR;
    ier = hwp_ptc1->IER;

#ifndef SF32LB52X  //Fix me!!
    if (isr & 0x00ff0000)
    {
        NVIC_DisableIRQ(PTC_IRQ_NUM);
        LCDC_LOG_E("BUS ERROR detected!  %x\n", isr);
        return;
    }
#endif /* SF32LB52X */

    cur_line = hwp_ptc1->MEM4; //Include back_porch+valid_lines+front_porch, except frame start cmd
    half_valid_line_num = (lcdc->roi.y1 - lcdc->roi.y0 + 1) / 2;


    if (cur_line >= lcdc->ramless.front_porch + half_valid_line_num - 1)   //half frame interrupt
    {
        LCDC_LOG_D("half frame! %d\n", cur_line);
        HAL_RAMLESS_LCD_HalfScreenCbk();
    }
    else
    {
        //frame done interrupt
        LCDC_LOG_D("frame done! %d\n", cur_line);
        HAL_RAMLESS_LCD_FullScreenCbk();

        if (lcdc->need_cplt_ckb > 0)
        {
            lcdc->need_cplt_ckb--;
            //Refresh frame data
            HAL_LCDC_ASSERT(LCDC_INTF_SPI_DCX_4DATA == lcdc->Init.lcd_itf);
            lcdc->Instance->LAYER0_SRC = (__IO uint32_t)lcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data;

            LCDC_LayerCfgTypeDef *cfg = &(lcdc->Layer[HAL_LCDC_LAYER_DEFAULT]);
            uint32_t reg, line_width = HAL_LCDC_GetPixelSize(cfg->data_format) * (cfg->data_area.x1 - cfg->data_area.x0 + 1);

            reg = lcdc->Instance->LAYER0_CONFIG;
            reg &= ~LCD_IF_LAYER0_CONFIG_WIDTH_Msk;
            reg |= (line_width << LCD_IF_LAYER0_CONFIG_WIDTH_Pos); // layer line width
            lcdc->Instance->LAYER0_CONFIG = reg;

            lcdc->Instance->LAYER0_TL_POS = (cfg->data_area.x0 << LCD_IF_LAYER0_TL_POS_X0_Pos) | (cfg->data_area.y0 << LCD_IF_LAYER0_TL_POS_Y0_Pos);
            lcdc->Instance->LAYER0_BR_POS = (cfg->data_area.x1 << LCD_IF_LAYER0_BR_POS_X1_Pos) | (cfg->data_area.y1 << LCD_IF_LAYER0_BR_POS_Y1_Pos);


            LCDC_TransCpltCallback(lcdc);
        }

    }

}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_RAMLESS_Init(LCDC_HandleTypeDef *lcdc, uint32_t *buf)
{
    lcdc->ptc_code = buf;
    lcdc->running = 0;
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_RAMLESS_Start(LCDC_HandleTypeDef *lcdc, RAMLESS_CFG_T *p_config)
{
    HAL_StatusTypeDef err;

    LCDC_LOG_I("HAL_LCDC_RAMLESS_Start\n");

    if (LCDC_INTF_SPI_DCX_4DATA != lcdc->Init.lcd_itf)
    {
        /*ONLY SUPPORT QADSPI NOW*/
        return HAL_ERROR;
    }
    else if (0 == lcdc->running)
    {
        memcpy(&lcdc->ramless, p_config, sizeof(RAMLESS_CFG_T));

        err = LayerUpdate(lcdc);
        if (HAL_OK != err)
        {
            LCDC_TransErrCallback(lcdc, err);
            return err;
        }

        lcdc->running = 1;
        RAMLESS_HW_FSM_START(lcdc);
        NVIC_EnableIRQ(PTC_IRQ_NUM);
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_RAMLESS_UpdateData(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t *pData,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1)
{
    lcdc->Layer[layeridx].data = pData;

    lcdc->Layer[layeridx].data_area.x0 = x0;
    lcdc->Layer[layeridx].data_area.y0 = y0;
    lcdc->Layer[layeridx].data_area.x1 = x1;
    lcdc->Layer[layeridx].data_area.y1 = y1;

    lcdc->debug_cnt1++;
    lcdc->need_cplt_ckb++;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_RAMLESS_Stop(LCDC_HandleTypeDef *lcdc)
{
    LCDC_LOG_I("HAL_LCDC_RAMLESS_Stop\n");

    if (LCDC_INTF_SPI_DCX_4DATA != lcdc->Init.lcd_itf)
    {
        /*ONLY SUPPORT QADSPI NOW*/
        return HAL_ERROR;
    }
    else
    {
        if (lcdc->running)
        {
            RAMLESS_HW_FSM_STOP(lcdc);
        }

        RAMLESS_RST_HW_FSM();

        if (lcdc->need_cplt_ckb > 0)
        {
            lcdc->need_cplt_ckb--;
            LCDC_TransCpltCallback(lcdc);
        }

        HAL_LCDC_ASSERT(0 == lcdc->need_cplt_ckb);
        lcdc->running = 0;

        return HAL_OK;
    }
}
#endif /* HAL_DSI_MODULE_ENABLED */

__HAL_ROM_USED void HAL_LCDC_IRQHandler(LCDC_HandleTypeDef *lcdc)
{
    __IO uint32_t irq_value;
    void (*cb)(LCDC_HandleTypeDef * lcdc);

    irq_value = lcdc->Instance->IRQ;
    lcdc->Instance->IRQ = irq_value; //clear irq

    if (irq_value & LCD_IF_IRQ_ICB_OF_RAW_STAT)
    {
        lcdc->ErrorCode |= HAL_LCDC_ERROR_OVERFLOW;
    }

#if (!defined(SF32LB55X))&&defined(LCDC_SUPPORT_LINE_DONE_IRQ)
    if (irq_value & LCD_IF_IRQ_LINE_DONE_STAT)
    {
        lcdc->Instance->SETTING &= ~LCD_IF_SETTING_LINE_DONE_MASK_Msk;

        __IO uint32_t v = (lcdc->Instance->SETTING & LCD_IF_SETTING_LINE_DONE_NUM_Msk) >> LCD_IF_SETTING_LINE_DONE_NUM_Pos;
        __IO uint32_t roi_top = (lcdc->Instance->CANVAS_TL_POS &  LCD_IF_CANVAS_TL_POS_Y0_Msk) >> LCD_IF_CANVAS_TL_POS_Y0_Pos;
        SetupNextLineIrq(lcdc);

        if (lcdc->XferLineCallback)
        {

            lcdc->XferLineCallback(lcdc, v - roi_top);
        }

        HAL_LCDC_SendLineCpltCbk(lcdc, v - roi_top);
    }
#endif /* !SF32LB55X && LCDC_SUPPORT_LINE_DONE_IRQ*/

    if (HAL_LCDC_IS_DPI_IF(lcdc->Init.lcd_itf))
    {
        lcdc->Instance->SETTING &= ~LCD_IF_SETTING_DPIL_INTR_MASK;

        if (irq_value & LCD_IF_IRQ_DPI_UDR_RAW_STAT)
        {
            lcdc->ErrorCode |= HAL_LCDC_ERROR_UNDERRUN;
            HAL_LCDC_ASSERT(0); //Check DPI under run error.
        }
    }

    if (HAL_LCDC_IS_JDI_PARALLEL_IF(lcdc->Init.lcd_itf))
    {
        if (irq_value & LCD_IF_IRQ_JDI_PAR_UDR_RAW_STAT)
        {
            lcdc->ErrorCode |= HAL_LCDC_ERROR_UNDERRUN;
            HAL_LCDC_ASSERT(0); //Check JDI under run error.
        }

        if (irq_value & LCD_IF_IRQ_JDI_PARL_INTR_RAW_STAT)
        {
            uint32_t int_line_num = GET_REG_VAL(lcdc->Instance->JDI_PAR_CTRL, LCD_IF_JDI_PAR_CTRL_INT_LINE_NUM_Msk, LCD_IF_JDI_PAR_CTRL_INT_LINE_NUM_Pos);
            uint32_t max_line     = GET_REG_VAL(lcdc->Instance->JDI_PAR_CONF1, LCD_IF_JDI_PAR_CONF1_MAX_LINE_Msk, LCD_IF_JDI_PAR_CONF1_MAX_LINE_Pos);

            if (int_line_num != max_line)
            {
                /* Interrupt after send 'max_line' data */
                lcdc->Instance->JDI_PAR_CTRL &= ~LCD_IF_JDI_PAR_CTRL_INT_LINE_NUM_Msk;
                lcdc->Instance->JDI_PAR_CTRL |= (max_line / 1) << LCD_IF_JDI_PAR_CTRL_INT_LINE_NUM_Pos;
                lcdc->Instance->JDI_PAR_CTRL &= ~LCD_IF_JDI_PAR_CTRL_ENABLE; //Send frambuffer once

                return;
            }
            else
            {
                lcdc->Instance->SETTING &= ~LCD_IF_SETTING_JDI_PARL_INTR_MASK;
                lcdc->Instance->JDI_PAR_CTRL &= ~LCD_IF_JDI_PAR_CTRL_XRST;   //send data finished
                lcdc->Instance->JDI_PAR_EX_CTRL |= LCD_IF_JDI_PAR_EX_CTRL_CNT_EN;
            }
        }
    }

    if (irq_value & LCD_IF_IRQ_EOF_STAT)
    {
        lcdc->Instance->SETTING &= ~LCD_IF_SETTING_EOF_MASK;
#if defined(SF32LB55X)&&defined(LCDC_SUPPORT_LINE_DONE_IRQ)
        ClearLineIrq(lcdc);
#endif /* SF32LB55X && LCDC_SUPPORT_LINE_DONE_IRQ*/
#ifdef LCDC_SUPPORT_DDR_QSPI
        if (LCDC_INTF_SPI_DCX_DDR_4DATA == lcdc->Init.lcd_itf)
        {
            DisableSPIDDRSent(lcdc);
        }
#endif /* LCDC_SUPPORT_DDR_QSPI */

        cb = g_LCDC_CpltCallback;
        if (cb)
        {
            g_LCDC_CpltCallback = NULL;
            cb(lcdc);
        }
    }
}

__HAL_ROM_USED void HAL_LCDC_TE_IRQHandler(LCDC_HandleTypeDef *lcdc)
{
    HAL_LCDC_Invert_TE_Pol(lcdc);
    HAL_LCDC_Invert_TE_Pol(lcdc);
}




__HAL_ROM_USED void HAL_LCDC_Next_Frame_TE(LCDC_HandleTypeDef *lcdc, bool en)
{
    lcdc->Next_Frame_TE = en ? NEXT_FRAME_TE_ON : NEXT_FRAME_TE_OFF;

    return;
}


__HAL_ROM_USED void HAL_LCDC_Invert_TE_Pol(LCDC_HandleTypeDef *lcdc)
{
    if (lcdc->Instance->TE_CONF & LCD_IF_TE_CONF_FMARK_POL)
        lcdc->Instance->TE_CONF &= ~LCD_IF_TE_CONF_FMARK_POL;
    else
        lcdc->Instance->TE_CONF |= LCD_IF_TE_CONF_FMARK_POL;


    return;
}


__HAL_ROM_USED void HAL_LCDC_Enable_TE(LCDC_HandleTypeDef *lcdc, bool en)
{
    if (HAL_LCDC_IS_SPI_IF(lcdc->Init.lcd_itf))
    {
        lcdc->Init.cfg.spi.syn_mode = (0 == en) ? HAL_LCDC_SYNC_DISABLE : HAL_LCDC_SYNC_VER;
    }
#ifdef HAL_DSI_MODULE_ENABLED
    else if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
    {
        lcdc->Init.cfg.dsi.CmdCfg.TEAcknowledgeRequest  = (0 == en) ? DSI_TE_ACKNOWLEDGE_DISABLE : DSI_TE_ACKNOWLEDGE_ENABLE;
    }
#endif /* HAL_DSI_MODULE_ENABLED */
    else
    {
        HAL_LCDC_ASSERT(0);
    }

    return;
}

#if defined(SF32LB58X) && defined(SOC_BF0_HCPU)
__HAL_ROM_USED void HAL_LCDC_UseLCDC2TE(LCDC_HandleTypeDef *lcdc)
{
    lcdc->use_lcdc2_te = 1;
    hwp_hpsys_cfg->SYSCR |= HPSYS_CFG_SYSCR_TE_SEL;
}
#endif

__HAL_ROM_USED void HAL_LCDC_SPI_Sequence(LCDC_HandleTypeDef *lcdc, bool end)
{
    WaitBusy(lcdc);

    if (end)
        lcdc->Instance->SPI_IF_CONF |= LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS;
    else
        lcdc->Instance->SPI_IF_CONF &= ~LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS;
}


__weak void HAL_LCDC_SendLayerDataCpltCbk(LCDC_HandleTypeDef *lcdc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(lcdc);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LCDC_SendLayerDataCpltCbk should be implemented in the user file
     */
}

__weak void HAL_LCDC_SendLineCpltCbk(LCDC_HandleTypeDef *lcdc, uint32_t line)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(lcdc);
    UNUSED(line);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LCDC_SendLineCpltCbk should be implemented in the user file
     */
}

#ifndef LCD_IF_LCD_CONF_SPI_RD_SEL
__weak void HAL_LCDC_SoftSpiInit(SOFT_SPI_PIN_Def pin, SOFT_SPI_IO_Def inout, uint32_t high1low0)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(pin);
    UNUSED(inout);
    UNUSED(high1low0);

    /* NOTE : Implement this function by user */
}

__weak void HAL_LCDC_SoftSpiDeinit(SOFT_SPI_PIN_Def pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(pin);

    /* NOTE : Implement this function by user */
}

__weak uint32_t HAL_LCDC_SoftSpiGetPin(SOFT_SPI_PIN_Def pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(pin);

    /* NOTE : Implement this function by user */
    return 0;
}

__weak void HAL_LCDC_SoftSpiSetPin(SOFT_SPI_PIN_Def pin, uint32_t high1low0)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(pin);
    UNUSED(high1low0);

    /* NOTE : Implement this function by user */
}

/**
 * @brief CPOL=0,CPHA=0,MSB
 * @param lcdc - lcdc handler
 * @param addr - address value
 * @param addr_len - address length in byte
 * @param data_len - Read data length in byte
 * @return
 */
__weak uint32_t HAL_LCDC_SoftSPI_Read(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len, uint32_t data_len)
{
    uint32_t spi_clk_MHz = lcdc->Init.freq / 1000000;
    uint32_t half_cycle_delay_ns = (1000 / spi_clk_MHz) / 2;
    uint8_t  with_dcx;
    uint32_t  ret_v = 0;
    SOFT_SPI_PIN_Def readpin = SW_SPI_D0 + lcdc->Init.cfg.spi.readback_from_Dx;

#define SPI_DELAY_NS(ns) HAL_Delay_us(((ns)/1000) + 1)
#define SPI_DELAY()  SPI_DELAY_NS(half_cycle_delay_ns<<1)
#define SPI_H_DELAY() SPI_DELAY_NS(half_cycle_delay_ns)

    if ((LCDC_INTF_SPI_DCX_1DATA == lcdc->Init.lcd_itf)
            || (LCDC_INTF_SPI_DCX_2DATA == lcdc->Init.lcd_itf)
            || (LCDC_INTF_SPI_DCX_4DATA == lcdc->Init.lcd_itf))
    {
        with_dcx = 1;
    }
    else
    {
        with_dcx = 0;
    }

    //1.Set GPIO output value and switch to GPIO pinmux
    HAL_LCDC_SoftSpiInit(SW_SPI_CS, SW_SPI_OUTPUT, 1);
    HAL_LCDC_SoftSpiInit(SW_SPI_CLK, SW_SPI_OUTPUT, 0);
    HAL_LCDC_SoftSpiInit(SW_SPI_D0, SW_SPI_OUTPUT, 0);
    if (with_dcx) HAL_LCDC_SoftSpiInit(SW_SPI_DCX, SW_SPI_OUTPUT, 1);


    //2.Select chip
    HAL_LCDC_SoftSpiSetPin(SW_SPI_CS, 0);
    if (with_dcx)
    {
        HAL_LCDC_SoftSpiSetPin(SW_SPI_DCX, 0);
    }
    SPI_DELAY();



    //3.Write address
    for (int32_t i = (addr_len * 8) - 1; i >= 0 ; i--)
    {
        HAL_LCDC_SoftSpiSetPin(SW_SPI_D0, (addr >> i) & 1);

        SPI_H_DELAY();
        HAL_LCDC_SoftSpiSetPin(SW_SPI_CLK, 1);

        SPI_H_DELAY();
        HAL_LCDC_SoftSpiSetPin(SW_SPI_CLK, 0);
    }


    //4.Read datas
    if (with_dcx)
    {
        HAL_LCDC_SoftSpiSetPin(SW_SPI_DCX, 1);
    }
    HAL_LCDC_SoftSpiInit(readpin, SW_SPI_INPUT, 0);
    for (int32_t i = ((data_len * 8) + lcdc->Init.cfg.spi.dummy_clock) - 1; i >= 0; i--)
    {
        SPI_H_DELAY();
        HAL_LCDC_SoftSpiSetPin(SW_SPI_CLK, 1);

        ret_v <<= 1;
        ret_v |= HAL_LCDC_SoftSpiGetPin(readpin);

        SPI_H_DELAY();
        HAL_LCDC_SoftSpiSetPin(SW_SPI_CLK, 0);
    }
    HAL_LCDC_SoftSpiInit(readpin, SW_SPI_OUTPUT, 0);


    //5.UnSelect chip
    SPI_DELAY();
    if (with_dcx)
    {
        HAL_LCDC_SoftSpiSetPin(SW_SPI_DCX, 1);
    }
    HAL_LCDC_SoftSpiSetPin(SW_SPI_CS, 1);




    //6.Restore pinmux at last
    HAL_LCDC_SoftSpiDeinit(SW_SPI_CS);
    HAL_LCDC_SoftSpiDeinit(SW_SPI_CLK);
    HAL_LCDC_SoftSpiDeinit(SW_SPI_D0);
    HAL_LCDC_SoftSpiDeinit(readpin);
    if (with_dcx) HAL_LCDC_SoftSpiDeinit(SW_SPI_DCX);

    return ret_v;
}
#endif /* LCD_IF_LCD_CONF_SPI_RD_SEL */


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_Enter_LP(LCDC_HandleTypeDef *lcdc)
{
    HAL_StatusTypeDef err = HAL_OK;
    HAL_LCDC_ASSERT(lcdc);

    __HAL_LCDC_LOCK(lcdc);


    LCDC_LOG_D("Enter_LowPower\n");

    if (HAL_LCDC_STATE_LOWPOWER == lcdc->State)
    {
        err = HAL_OK;
    }
    else if (HAL_LCDC_STATE_READY != lcdc->State)
    {
        err = HAL_BUSY;
    }
    else
    {
        if (HAL_TIMEOUT == WaitBusy(lcdc))
        {
            err = HAL_TIMEOUT;
        }
        else
        {
            lcdc->State = HAL_LCDC_STATE_LOWPOWER;

#ifdef HAL_DSI_MODULE_ENABLED
            if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
            {
                err = HAL_DSI_EnterULPM(&lcdc->hdsi);
            }
#endif /* HAL_DSI_MODULE_ENABLED */
        }
    }

    __HAL_LCDC_UNLOCK(lcdc);

    return err;
}



__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_Exit_LP(LCDC_HandleTypeDef *lcdc)
{
    HAL_StatusTypeDef err = HAL_OK;
    HAL_LCDC_ASSERT(lcdc);

    __HAL_LCDC_LOCK(lcdc);



    LCDC_LOG_D("Exit_LowPower\n");

    if (HAL_LCDC_STATE_LOWPOWER == lcdc->State)
    {
#ifdef HAL_DSI_MODULE_ENABLED
        if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
        {
            DSI_HandleTypeDef *hdsi = &lcdc->hdsi;
            err = HAL_DSI_ExitULPM(hdsi);
        }
#endif /* HAL_DSI_MODULE_ENABLED */

        lcdc->State = HAL_LCDC_STATE_READY;
    }

    __HAL_LCDC_UNLOCK(lcdc);

    return err;
}




__HAL_ROM_USED HAL_StatusTypeDef HAL_LCDC_Resume(LCDC_HandleTypeDef *lcdc)
{
    HAL_StatusTypeDef err = HAL_OK;
    HAL_LCDC_ASSERT(lcdc);

    __HAL_LCDC_LOCK(lcdc);

    {
        //Re-initialize LCDC regs(DSI regs too)
        LCDC_InitTypeDef *init;

        HAL_RCC_ResetModule((lcdc->Instance == hwp_lcdc1) ? RCC_MOD_LCDC1 : RCC_MOD_LCDC2);

        init = &lcdc->Init;

#ifndef SOC_BF_Z0  /*Z0 version auto gate will block LCDC IRQ*/
        lcdc->Instance->SETTING |= LCD_IF_SETTING_AUTO_GATE_EN;
#endif


        /*Select LCD interface*/
        SelectIntf(lcdc, lcdc->Init.lcd_itf);

        /*LCDC output frequecy*/
        SetFreq(lcdc, init->freq);

        /*LCDC output format*/
        SetOutFormat(lcdc, init->color_mode);

        /* Setup TE  */
        SetupTE(lcdc);

        /* Enable LCD interface output*/
        //EnableInf(lcdc);

        lcdc->Instance->LCD_IF_CONF |= LCD_IF_LCD_IF_CONF_LCD_RSTB; //Release reset signal
    }

    if (HAL_LCDC_STATE_LOWPOWER == lcdc->State)
    {
#ifdef HAL_DSI_MODULE_ENABLED
        if (HAL_LCDC_IS_DSI_IF(lcdc->Init.lcd_itf))
        {
            DSI_HandleTypeDef *hdsi = &lcdc->hdsi;
            HAL_DSI_bypass_output(hdsi, true);
            HAL_DSI_Start(hdsi);
            err = HAL_DSI_EnterULPM(hdsi);
            HAL_DSI_bypass_output(hdsi, false);
        }
#endif /* HAL_DSI_MODULE_ENABLED */

    }

    __HAL_LCDC_UNLOCK(lcdc);

    return err;
}

#endif /*defined(HAL_LCD_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)*/


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
