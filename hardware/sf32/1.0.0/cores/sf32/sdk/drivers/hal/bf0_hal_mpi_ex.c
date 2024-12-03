/**
  ******************************************************************************
  * @file   bf0_hal_mpi_ex.c
  * @author Sifli software development team
  * @brief   QSPI extension HAL module driver.
  *
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

#include "bf0_hal.h"
#include <string.h>
#include <stdlib.h>

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup FLASH FLASH
  * @brief FLASH HAL module driver
  * @{
  */

#define DEBUG_JLINK         0
#if DEBUG_JLINK
    extern void debug_print(char *str);
    extern uint8_t *htoa(uint8_t *p, uint32_t d);
#endif
#if defined(HAL_MPI_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#include "flash_table.h"

int nand_read_id(FLASH_HandleTypeDef *handle, uint8_t dummy);
uint32_t HAL_QSPI_GET_SRC_CLK(FLASH_HandleTypeDef *fhandle);
#define FLASH_CLK_INVERT_THD            (60000000)

#define QSPI_USE_CMD2

// for some HYF nand chip, need more oip for read, and set protect register 2 before clear all protect
#define HYF_SPECIAL_SUPPORT

__weak void HAL_RAM_FLASH_INIT(void)
{
}

__weak uint32_t HAL_GET_FLASH_MID(MPI_TypeDef *fhandle)
{
    return 0;
}

static inline int HAL_IS_ID_VALID(uint32_t mid)
{
    if (mid == 0 || mid == 0xffffff)
        return 0;

    return 1;
}

__HAL_ROM_USED int HAL_FLASH_DETECT_DUAL(QSPI_FLASH_CTX_T *ctx, qspi_configure_t *cfg)
{
    FLASH_HandleTypeDef *hflash = NULL;
    int res = 0;

    if (ctx == NULL || cfg == NULL)
        return 0;

    hflash = &(ctx->handle);
    HAL_QSPI_Init(hflash, cfg);
    //hflash->Mode = 0;

    // set dual mode , it should not set by user
    HAL_FLASH_SET_DUAL_MODE(hflash, 1);

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);

    // get device id, then get table,
    res = HAL_FLASH_ID_DUAL_ID(hflash);

    // disable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 0);
    // recover to single mode
    HAL_FLASH_SET_DUAL_MODE(hflash, 0);

    return res;
}

__HAL_ROM_USED int HAL_FLASH_DETECT_SINGLE(QSPI_FLASH_CTX_T *ctx, qspi_configure_t *cfg)
{
    FLASH_HandleTypeDef *hflash = NULL;
    int res = 0;

    if (ctx == NULL || cfg == NULL)
        return 0;

    hflash = &(ctx->handle);
    HAL_QSPI_Init(hflash, cfg);
    //hflash->Mode = 0;

    // set single mode , it should not set by user
    HAL_FLASH_SET_DUAL_MODE(hflash, 0);
    hflash->dualFlash = 0;

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);

    // get device id, then get table,
    res = HAL_FLASH_GET_NOR_ID(hflash);

    // disable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 0);

    if (res == 0 || res == 0xffffff)
        return 0;

    return 1;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_BOOT(QSPI_FLASH_CTX_T *ctx, qspi_configure_t *cfg,
        DMA_HandleTypeDef *dma, struct dma_config *dma_cfg, uint16_t clk_div)
{
    FLASH_HandleTypeDef *hflash = NULL;
    uint8_t fid, did, mtype;

    if (ctx == NULL || cfg == NULL)
        return HAL_ERROR;

    hflash = &(ctx->handle);
    HAL_QSPI_Init(hflash, cfg);

    // init context
    //ctx->dev_id = FLASH_UNKNOW_ID;
    ctx->flash_mode = cfg->SpiMode;
    ctx->base_addr = cfg->base;
    ctx->total_size = cfg->msize * 0x100000;
    ctx->cache_flag = 2;

    // for bootloader with reset, forec single line and nor flash mode to avoid set qspi mode
    hflash->isNand = 0;
    hflash->Mode = 0;
    hflash->dma = dma;
    if (hflash->dma != NULL && dma_cfg != NULL)
    {
        hflash->dma->Instance                 = dma_cfg->Instance;
        hflash->dma->Init.Request             = dma_cfg->request;
        hflash->dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hflash->dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        hflash->dma->Init.MemInc              = DMA_MINC_ENABLE;
        //hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        //hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hflash->dma->Init.Mode                = DMA_NORMAL;
        hflash->dma->Init.Priority            = DMA_PRIORITY_MEDIUM;
        hflash->dma->Init.BurstSize           = 8;
    }

    HAL_FLASH_SET_CLK_rom(hflash, clk_div);
    // set dual mode , it should not set by user
    if (ctx->dual_mode != 0)
    {
        if (ctx->dual_mode == 1)
            HAL_FLASH_SET_DUAL_MODE(hflash, 0);
        else if (ctx->dual_mode == 2)
            HAL_FLASH_SET_DUAL_MODE(hflash, 1);
    }
    // get dual mode for user image
    hflash->dualFlash = HAL_FLASH_GET_DUAL_MODE(hflash);

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);

    // get device id, then get table,
    ctx->dev_id = HAL_QSPI_READ_ID(hflash);
    fid = (uint8_t)ctx->dev_id & 0xff;
    mtype = (uint8_t)((ctx->dev_id >> 8) & 0xff);
    did = (uint8_t)((ctx->dev_id >> 16) & 0xff);
    if (hflash->isNand)
        hflash->ctable = spi_nand_get_cmd_by_id(fid, did, mtype);
    else
        hflash->ctable = spi_flash_get_cmd_by_id(fid, did, mtype);
    if (hflash->ctable  == NULL)
        return HAL_ERROR;

    // add software reset to make chip reset to default status (single line ?)
    // set local controller to single line, it only work for bootloader?
    HAL_QSPIEX_FLASH_RESET(hflash);
    HAL_Delay_us(30);

    return HAL_OK;
}

int nand_clear_status(FLASH_HandleTypeDef *handle);

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_Init(QSPI_FLASH_CTX_T *ctx, qspi_configure_t *cfg,
        DMA_HandleTypeDef *dma, struct dma_config *dma_cfg, uint16_t clk_div)
{
    FLASH_HandleTypeDef *hflash = NULL;
    uint8_t fid, did, mtype;
    int size = 0;
    //uint32_t freq;

    if (ctx == NULL || cfg == NULL)
        return HAL_ERROR;

    hflash = &(ctx->handle);

    HAL_QSPI_Init(hflash, cfg);
    uint32_t mid = HAL_GET_FLASH_MID(hflash->Instance);

    // init context
    //ctx->dev_id = FLASH_UNKNOW_ID;
    ctx->flash_mode = cfg->SpiMode;
    ctx->base_addr = cfg->base;
    ctx->total_size = cfg->msize * 0x100000;
    ctx->cache_flag = 2;

    hflash->isNand = cfg->SpiMode == 1 ? 1 : 0;
    hflash->dma = dma;
    if (hflash->dma != NULL && dma_cfg != NULL)
    {
        hflash->dma->Instance                 = dma_cfg->Instance;
        hflash->dma->Init.Request             = dma_cfg->request;
        hflash->dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hflash->dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        hflash->dma->Init.MemInc              = DMA_MINC_ENABLE;
        //hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        //hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hflash->dma->Init.Mode                = DMA_NORMAL;
        hflash->dma->Init.Priority            = DMA_PRIORITY_MEDIUM;
        hflash->dma->Init.BurstSize           = 1;
        if (HAL_IS_ID_VALID(mid) == 0)
            HAL_FLASH_SET_TXSLOT(hflash, hflash->dma->Init.BurstSize);
    }

    if (HAL_IS_ID_VALID(mid) == 0)
    {
        //freq = HAL_QSPI_GET_SRC_CLK(hflash);
        if (hflash->freq > FLASH_CLK_INVERT_THD)
            HAL_QSPI_SET_CLK_INV(hflash, 1, 0);
        else
            HAL_QSPI_SET_CLK_INV(hflash, 0, 0);

        HAL_FLASH_SET_CLK_rom(hflash, clk_div);
        // set dual mode , it should not set by user
        if (ctx->dual_mode != 0)
        {
            //#ifndef  CFG_BOOTLOADER
            //        ASSERT(0);  // only  bootloader can use this branch !
            //#endif
            if (ctx->dual_mode == 1)
                HAL_FLASH_SET_DUAL_MODE(hflash, 0);
            else if (ctx->dual_mode == 2)
            {
                HAL_FLASH_SET_DUAL_MODE(hflash, 1);
            }
        }

        // enable QSPI
        HAL_FLASH_ENABLE_QSPI(hflash, 1);
    }
    // get dual mode for user image
    hflash->dualFlash = HAL_FLASH_GET_DUAL_MODE(hflash);

#if 0
    HAL_FLASH_DEEP_PWRDOWN(hflash);
    HAL_Delay_us(0);
    HAL_Delay_us(3);
#endif

    if (hflash->isNand == 0)
    {
        if (HAL_IS_ID_VALID(mid) == 0)
        {
            // force release from Deep Power-Down mode by default
            HAL_FLASH_RELEASE_DPD(hflash);
            HAL_Delay_us(0);
            HAL_Delay_us(50);   // change to 50us to meet boya request, others with 8,20, 30 us can be cover
        }
    }
    else
    {
        MODIFY_REG(hflash->Instance->DCR, MPI_DCR_CSHMIN, 0xF << MPI_DCR_CSHMIN_Pos);

        // add chip reset for nand
        HAL_FLASH_MANUAL_CMD(hflash, 0, 0, 0, 0, 0, 0, 0, 1);
        HAL_FLASH_SET_CMD(hflash, 0xff, 0);
        HAL_Delay_us(0);
        HAL_Delay_us(200);
    }

    //extern void rt_kprintf(const char *fmt, ...);
    // get device id, then get table,
    //rt_kprintf("Before get Id\n");
    //if ((ctx->dev_id == 0) || (ctx->dev_id == 0xff))
    if (HAL_IS_ID_VALID(mid) == 0)
        ctx->dev_id = HAL_QSPI_READ_ID(hflash);
    else
        ctx->dev_id = mid;

    fid = (uint8_t)ctx->dev_id & 0xff;
    mtype = (uint8_t)((ctx->dev_id >> 8) & 0xff);
    did = (uint8_t)((ctx->dev_id >> 16) & 0xff);
    //rt_kprintf("Fid 0x%x\n",ctx->dev_id);
#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("fid:");
    debug_print((char *)htoa(hex, fid));
    debug_print(" mtype:");
    debug_print((char *)htoa(hex, mtype));
    debug_print(" did:");
    debug_print((char *)htoa(hex, did));
    debug_print("\r\n");
#endif
    if (hflash->isNand)
        hflash->ctable = spi_nand_get_cmd_by_id(fid, did, mtype);
    else
        hflash->ctable = spi_flash_get_cmd_by_id(fid, did, mtype);
    if (hflash->ctable  == NULL)
    {
        if (hflash->isNand) // for nand, try another timing to read id
        {
            ctx->dev_id = nand_read_id(hflash, 8);
            fid = (uint8_t)ctx->dev_id & 0xff;
            mtype = (uint8_t)((ctx->dev_id >> 8) & 0xff);
            did = (uint8_t)((ctx->dev_id >> 16) & 0xff);
            //rt_kprintf("Fid2 0x%x\n",ctx->dev_id);
            hflash->ctable = spi_nand_get_cmd_by_id(fid, did, mtype);
            if (hflash->ctable == NULL)   // try to output fix level or addr for dummy bits
            {
                ctx->dev_id = nand_read_id(hflash, 0xf);
                fid = (uint8_t)ctx->dev_id & 0xff;
                mtype = (uint8_t)((ctx->dev_id >> 8) & 0xff);
                did = (uint8_t)((ctx->dev_id >> 16) & 0xff);
                //rt_kprintf("Fid3 0x%x\n",ctx->dev_id);
                hflash->ctable = spi_nand_get_cmd_by_id(fid, did, mtype);
                if (hflash->ctable == NULL)   // try to get default table if support
                {
                    hflash->ctable = spi_nand_get_default_ctable();
                }
            }
        }
        if (hflash->ctable  == NULL)
        {
            // disable QSPI
            HAL_FLASH_ENABLE_QSPI(hflash, 0);
            ctx->base_addr = 0;
            ctx->total_size = 0;
            return HAL_ERROR;
        }
    }

    if (hflash->isNand)
        size = spi_nand_get_size_by_id(fid, did, mtype);
    else
        size = spi_flash_get_size_by_id(fid, did, mtype);
    if (size != 0)  // use size from table to replace configure size
    {
        ctx->total_size = size << hflash->dualFlash;    // dual flash size double
        hflash->size = size << hflash->dualFlash; // ?? 2 size ?
    }

    // add software reset to make chip reset to default status (single line ?)
    // set local controller to single line, it only work for bootloader?
    //HAL_QSPIEX_FLASH_RESET(hflash);
    //HAL_Delay_us(300);

    // only nor need set QE mode at initial
    //rt_kprintf("DTR %d, SPI %d, cmd 0x%x\n", hflash->buf_mode,hflash->Mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].cmd);
    if (hflash->isNand == SPI_MODE_NOR)
    {
        if (HAL_IS_ID_VALID(mid) == 0)
        {
            HAL_FLASH_CLR_PROTECT(hflash);
            //HAL_Delay_us(30);
            if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
                    || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
            {
                //hflash->buf_mode = 0; // not support dtr for large size flash
                // enter 4 byte address mode
                HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_EN4BM, 0);

                if (hflash->Mode == HAL_FLASH_NOR_MODE)
                {
                    hflash->buf_mode = 0;   // only support 4 lines dtr
                    HAL_FLASH_FADDR_SET_QSPI(hflash, false);
                }
                else
                {
                    HAL_FLASH_FADDR_SET_QSPI(hflash, true);
                    if (hflash->buf_mode == 1)   // dtr enabled, check if support by flash command
                    {
                        // cover pre read command configure
                        if (spi_flash_is_support_dtr(fid, did, mtype) != 0)
                        {
#if defined(SF32LB56X) || defined(SF32LB52X)
                            HAL_NOR_DTR_CAL(hflash);
#endif
                            HAL_NOR_CFG_DTR(hflash, 1);
                        }
                        else // not support dtr
                        {
                            HAL_MPI_CFG_DTR(hflash, false, 0);
                            hflash->buf_mode = 0;
                        }
                    }
                }

                // set 1KB boundary to avoid some large NOR wrap around at 16MB position
                // some 32MB NOR look like 2 16MB connect, it not support continue read.
                HAL_FLASH_SET_ROW_BOUNDARY(hflash, 7);
            }
            else
            {
                if (hflash->Mode == HAL_FLASH_NOR_MODE)
                {
                    hflash->buf_mode = 0;   // only support 4 lines dtr
                    HAL_FLASH_SET_QUAL_SPI(hflash, false);
                }
                else
                {
                    HAL_FLASH_SET_QUAL_SPI(hflash, true);
                    if (hflash->buf_mode == 1)   // dtr enabled, check if support by flash command
                    {
                        // cover pre read command configure
                        if (spi_flash_is_support_dtr(fid, did, mtype) != 0)
                        {
#if 0
                            HAL_FLASH_CFG_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].data_mode,
                                                   hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].dummy_cycle, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].ab_size,
                                                   hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].ab_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].addr_size,
                                                   hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].addr_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].ins_mode);
                            HAL_FLASH_SET_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].cmd);

                            HAL_MPI_CFG_DTR(hflash, true, hflash->ecc_en);
#endif
#if defined(SF32LB56X) || defined(SF32LB52X)
                            HAL_NOR_DTR_CAL(hflash);
#endif
                            HAL_NOR_CFG_DTR(hflash, 1);
                        }
                        else // not support dtr
                        {
                            HAL_MPI_CFG_DTR(hflash, false, 0);
                            hflash->buf_mode = 0;
                        }
                    }
                    else
                        HAL_MPI_CFG_DTR(hflash, false, 0);

                }
            }
        }
        else // external FLASH do not use dtr
        {
            hflash->buf_mode = 0;
        }
    }
    else
    {
        uint32_t sta;
        do
        {
            HAL_FLASH_WRITE_DLEN(hflash, 1);
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, hflash->ctable->status_reg);
            //HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 0, 1, 1);
            //HAL_FLASH_SET_CMD(hflash, 0x0f, 0xc0);
            sta = HAL_FLASH_READ32(hflash);
            HAL_Delay_us(10);
        }
        while (sta & 0x1);    // busy/iop

        nand_clear_status(hflash);
        if (hflash->Mode == HAL_FLASH_QMODE)    // ONLY qspi need switch QE
        {
            HAL_NAND_EN_QUAL(hflash, 1);
        }
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_DeInit(FLASH_HandleTypeDef *hflash)
{
    HAL_FLASH_ENABLE_QSPI(hflash, 0);
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_DMA_START(FLASH_HandleTypeDef *hflash, char *buf, int write, uint32_t len)
{
    uint32_t src, dst, value, size;
    HAL_StatusTypeDef res = HAL_OK;
    if ((hflash == NULL) || (hflash->dma == NULL) || (len == 0))
        return HAL_ERROR;

    value = hflash->Instance->CR;

    if (write)
    {
        hflash->dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hflash->dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        hflash->dma->Init.MemInc              = DMA_MINC_ENABLE;
        src = (uint32_t) buf;
        dst = (uint32_t)(&hflash->Instance->DR);
        size = len; //(len + 3) / 4; // must up aligned
    }
    else // read, fifo word mode, length should change to word lenght(/4)
    {
        hflash->dma->Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; //DMA_PDATAALIGN_BYTE; //DMA_PDATAALIGN_WORD;
        hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;  //DMA_MDATAALIGN_BYTE; //DMA_MDATAALIGN_WORD;
        hflash->dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        hflash->dma->Init.MemInc              = DMA_MINC_ENABLE;
        dst = (uint32_t) buf;
        src = (uint32_t)(&hflash->Instance->DR);
        size = (len + 3) / 4; // must up aligned
    }

    res = HAL_DMA_DeInit(hflash->dma);
    if (res != HAL_OK)
        return res;
    res = HAL_DMA_Init(hflash->dma);
    if (res != HAL_OK)
        return res;

    //value |= FLASHC_CR_DMAE;
    hflash->Instance->CR = value | MPI_CR_DMAE;
    //hflash->Instance->DLR = (len - 1) & FLASHC_DLR_DLEN_Msk;
    hflash->Instance->DLR1 = (len - 1);

    res = HAL_DMA_Start(hflash->dma, src, dst, size);
    //res = HAL_DMA_PollForTransfer(hflash->dma, HAL_DMA_FULL_TRANSFER, 1000);

    // recover dmae bit
    //hflash->Instance->CR = value;

    return res;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_DMA_WAIT_DONE(FLASH_HandleTypeDef *hflash, uint32_t timeout)
{
    HAL_StatusTypeDef res = HAL_OK;
    uint32_t value;
    if ((hflash == NULL) || (hflash->dma == NULL))
        return HAL_ERROR;

    res = HAL_DMA_PollForTransfer(hflash->dma, HAL_DMA_FULL_TRANSFER, timeout);

    // clear dmae bit
    value = hflash->Instance->CR;
    hflash->Instance->CR = value & (~MPI_CR_DMAE);

    return res;
}

__HAL_ROM_USED void HAL_FLASH_ALIAS_CFG(FLASH_HandleTypeDef *fhandle, uint32_t start, uint32_t len, uint32_t offset)
{
    if (fhandle == NULL)
        return;
    // for pro, start should set to relative address
    if (start >= fhandle->base)
        start -= fhandle->base;
    HAL_FLASH_SET_ALIAS_RANGE(fhandle, start, len);

    HAL_FLASH_SET_ALIAS_OFFSET(fhandle, offset);
}


__HAL_ROM_USED void HAL_FLASH_NONCE_CFG(FLASH_HandleTypeDef *fhandle, uint32_t start, uint32_t end, uint8_t *nonce)
{
    if (fhandle == NULL || nonce == NULL)
        return;
    HAL_FLASH_SET_NONCE(fhandle, nonce);

    // for pro, start/end should set to relative address
    if (start >= fhandle->base)
        start -= fhandle->base;
    if (end >= fhandle->base)
        end -= fhandle->base;
    HAL_FLASH_SET_CTR(fhandle, start, end);
}


__HAL_ROM_USED void HAL_FLASH_AES_CFG(FLASH_HandleTypeDef *fhandle, uint8_t aes256)
{
    if (fhandle == NULL)
        return;

    if (aes256 != 0)
        HAL_FLASH_SET_AES(fhandle, HAL_FLASH_AES256);
    else
        HAL_FLASH_SET_AES(fhandle, HAL_FLASH_AES128);

    HAL_FLASH_ENABLE_AES(fhandle, 1);
}


static int flash_handle_valid(FLASH_HandleTypeDef *handle)
{
    if ((handle == NULL) || (handle->ctable == NULL))
        return 0;

    return 1;
}

static int flash_cmd_valid(FLASH_HandleTypeDef *handle, SPI_FLASH_CMD_E cmd)
{
    //HAL_StatusTypeDef res;
    if ((handle->ctable == NULL) || (cmd >= SPI_FLASH_CMD_COUNT)
            || (handle->ctable->cmd_cfg[cmd].cmd == 0))
    {
        return 0;
    }

    return 1;
}

/***************** Interface for nand flash ********************************/

// all nand read id should be same, or cannot recognize them
__HAL_ROM_USED int nand_read_id(FLASH_HandleTypeDef *handle, uint8_t dummy)
{
    //uint8_t mid, did1, did2;
    uint32_t rdata;

    if (handle == NULL)
        return FLASH_UNKNOW_ID;

    //HAL_FLASH_CLEAR_FIFO(handle, HAL_FLASH_CLR_RX_FIFO);

    // configure CCR

    if (dummy <= 8) // use dummy bit
        HAL_FLASH_MANUAL_CMD(handle, 0, 1, dummy, 0, 0, 0, 0, 1);
    else // force output data but not random data for dummy bits
        HAL_FLASH_MANUAL_CMD(handle, 0, 1, 0, 0, 0, 0, 1, 1);

    // configure data length
    HAL_FLASH_WRITE_DLEN(handle, 3);

    // send command, read id command as 0x9f, all device with same read ID?
    HAL_FLASH_SET_CMD(handle, 0x9f, 0); // fix output low for dummy if CCR addr mode != 0

    rdata = HAL_FLASH_READ32(handle);
    //mid  = rdata & 0xff;
    //did1 = (rdata >> 8) & 0xff;
    //did2 = (rdata >> 16) & 0xff;
    //LOG_D("Nand ID: 0x%x, 0x%x, 0x%x\n", mid, did1, did2);

    return rdata & 0xffffff;
}

//extern void rt_kprintf(const char *fmt, ...);
__HAL_ROM_USED int HAL_NAND_CONF_ECC(FLASH_HandleTypeDef *handle, uint8_t en)
{
    uint32_t sta;
    if (handle == NULL || handle->ctable == NULL)
        return -1;

    if (handle->ctable->mode_reg == 0 || handle->ctable->ecc_en_mask == 0)
        return -2;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);
    //rt_kprintf("Sta = 0x%x\n",sta);

    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WVSR, 0);
    //sta |= 8; // set to buf mode
    if (en)
        sta |= handle->ctable->ecc_en_mask; // SET ECC-E
    else
        sta &= ~handle->ctable->ecc_en_mask; // clear ecc bit
    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    handle->ecc_en = en;

    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);
    //rt_kprintf("Sta2 = 0x%x\n",sta);

    return 0;
}

__HAL_ROM_USED int HAL_NAND_GET_ECC_STATUS(FLASH_HandleTypeDef *handle)
{
    uint32_t sta;

    if (handle == NULL || handle->ctable == NULL)
        return 0;

    if (handle->ctable->status_reg == 0 || handle->ctable->ecc_sta_mask == 0)
        return 0;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
    sta = HAL_FLASH_READ32(handle);

    return (sta & handle->ctable->ecc_sta_mask);
}

__HAL_ROM_USED int HAL_NAND_GET_ECC_RESULT(FLASH_HandleTypeDef *handle)
{
    int sta;
    int bitc, bits, i;

    if (handle->ecc_en == 0)
        return 0;

    sta = HAL_NAND_GET_ECC_STATUS(handle);
    if (sta == 0)
        return 0;

    //if(sta != 0)
    //    return sta;

    handle->ErrorCode |= 0X8000;
    bits = 0;   // ecc status start bit
    //while((handle->ctable->ecc_sta_mask & (1<<bits)) == 0)
    //    bits++;
    for (i = 0; i < 8; i++)
        if ((handle->ctable->ecc_sta_mask & (1 << i)) != 0)
        {
            bits = i;
            break;
        }

    bitc = 0; // ecc status bit count
    for (i = bits; i < 8; i++)
        if ((handle->ctable->ecc_sta_mask & (1 << i)) != 0)
        {
            bitc++;
        }

    // for 2 bit mode, only 0 and 1 correct, for 3 bit mode, only 7 can not correct
    if (bitc == 2)
        if ((sta >> bits) < 2)
            return 0;
        else
            return sta >> bits ;
    else if (bitc == 3)
        if ((sta >> bits) < 7)
            return 0;
        else
            return sta >> bits ;
    else // not support yet
        return 0xfc;

    //return (sta & handle->ctable->ecc_sta_mask)>>bits;
    return 0xfd;
}

// NAND operations ---------------------------------
__HAL_ROM_USED int HAL_NAND_READ_PAGE(FLASH_HandleTypeDef *handle, uint32_t addr, uint8_t *buff, uint32_t len)
{
    return HAL_NAND_READ_WITHOOB(handle, addr, buff, len, NULL, 0);
}

#define NAND_CPY_EDMA_THD                     (256)  /*!< NAND cache copy with ext-dma threshold     */

__HAL_ROM_USED int HAL_NAND_READ_WITHOOB(FLASH_HandleTypeDef *handle, uint32_t addr,
        uint8_t *dbuff, uint32_t dlen, uint8_t *oob_buf, uint32_t olen)
{
    int busy, oip_cnt;
    uint32_t offset = addr & (SPI_NAND_PAGE_SIZE - 1);
    //HAL_StatusTypeDef ret;

    if (handle == NULL || handle->ctable == NULL || handle->data_buf == NULL
            || (dlen + offset) > SPI_NAND_PAGE_SIZE || olen > SPI_NAND_MAXOOB_SIZE)
    {
        //LOG_E(" error param\n");
        handle->ErrorCode = 1;
        return 0;
    }

    handle->ErrorCode = 0;
    // switch addr to local addr
    if (addr >= handle->base)
        addr -= handle->base;

    // load page to cache, page read with block+page address, support 3 bytes address for larger than 1Gb chips
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PREAD, (addr / SPI_NAND_PAGE_SIZE) & 0xffffff);
    HAL_Delay_us_(20);

    // check busy
    HAL_FLASH_WRITE_DLEN(handle, 1);
    // set cs low manual, use gpio output 0 or just pin setting low?
    //HAL_PIN_Set(PAD_PA06, GPIO_A6,  PIN_PULLDOWN, 1);
    //HAL_GPIO_WritePin(hwp_gpio1, 6, 0);
    oip_cnt = 0;
    do
    {
        HAL_Delay_us_(5);
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
        busy = HAL_FLASH_READ32(handle) & 0x1;
        if (busy == 0)
            oip_cnt++;
#ifdef HYF_SPECIAL_SUPPORT
        if (oip_cnt >= 2)   // only HYF request oip twice
            break;
#else
        if (oip_cnt >= 1)   // only HYF request oip twice
            break;
#endif
    }
    while (1);
    //while (busy);
    // recover CS pin setting
    //HAL_GPIO_WritePin(hwp_gpio1, 6, 1);
    //HAL_PIN_Set(PAD_PA06, MPI3_CS,  PIN_NOPULL, 1);

    int res = HAL_NAND_GET_ECC_RESULT(handle);
    if (res != 0)
    {
        handle->ErrorCode = res | 0x8000;
        return 0;
    }

    // use AHB read
    {
        // set ahb read command
        if (handle->Mode == HAL_FLASH_NOR_MODE)
        {
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].cmd);
            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].addr_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ins_mode);
        }
        else
        {
#define NAND_READ_CMD   SPI_FLASH_CMD_4READ
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[NAND_READ_CMD].cmd);
            HAL_FLASH_CFG_AHB_RCMD(handle,  handle->ctable->cmd_cfg[NAND_READ_CMD].data_mode,
                                   handle->ctable->cmd_cfg[NAND_READ_CMD].dummy_cycle, handle->ctable->cmd_cfg[NAND_READ_CMD].ab_size,
                                   handle->ctable->cmd_cfg[NAND_READ_CMD].ab_mode, handle->ctable->cmd_cfg[NAND_READ_CMD].addr_size,
                                   handle->ctable->cmd_cfg[NAND_READ_CMD].addr_mode, handle->ctable->cmd_cfg[NAND_READ_CMD].ins_mode);
        }
        {
            if ((((uint32_t)dbuff & 3) != 0) || ((offset & 3) != 0)
                    || ((dlen & 3) != 0) || ((dlen > 0) && (dlen < NAND_CPY_EDMA_THD))) // buffer/addr is not word aligned for too small, use memcpy
            {
                memcpy(dbuff, (const void *)(handle->base + offset), dlen);
            }
            else if (dbuff != NULL)
            {
#if (NAND_BUF_CPY_MODE == 0)    // memcpy
                memcpy(dbuff, (const void *)(handle->base + offset), dlen);

#elif (NAND_BUF_CPY_MODE == 1)  // ext-dma
                {
                    //EXT_DMA_Config(1, 1);
                    //res = EXT_DMA_TRANS_SYNC(handle->base, (uint32_t)dbuff, dlen/4, 8000);
                    EXT_DMA_HandleTypeDef DMA_Handle;
                    /*Data copy config    */
                    DMA_Handle.Init.SrcInc = HAL_EXT_DMA_SRC_INC | HAL_EXT_DMA_SRC_BURST16; //Source address auto-increment and burst 16
                    DMA_Handle.Init.DstInc = HAL_EXT_DMA_DST_INC | HAL_EXT_DMA_DST_BURST16; //Dest address auto-increment and burst 16
                    DMA_Handle.Init.cmpr_en = false;

                    res = HAL_EXT_DMA_Init(&DMA_Handle);
                    if (res != HAL_OK)
                    {
                        handle->ErrorCode = res | 0x80000000 | (DMA_Handle.ErrorCode << 16) | (DMA_Handle.State << 8);
                        return 0;
                    }
                    res = HAL_EXT_DMA_Start(&DMA_Handle, handle->base + offset, (uint32_t)dbuff, dlen / 4);
                    if (HAL_OK == res)
                    {
                        res = HAL_EXT_DMA_PollForTransfer(&DMA_Handle, HAL_EXT_DMA_FULL_TRANSFER, 1000);
                    }
                    else
                    {
                        handle->ErrorCode = res | 0xc0000000 | (DMA_Handle.ErrorCode << 16) | (DMA_Handle.State << 8);
                        return 0;
                    }
                    if (res != HAL_OK)
                    {
                        handle->ErrorCode = res | 0xe0000000 | (DMA_Handle.ErrorCode << 16) | (DMA_Handle.State << 8);
                        return 0;
                    }
                }
#elif (NAND_BUF_CPY_MODE==2)    // common dma
                handle->dma->Init.Direction           = DMA_MEMORY_TO_MEMORY;
                handle->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; //DMA_PDATAALIGN_BYTE; //DMA_PDATAALIGN_WORD;
                handle->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;  //DMA_MDATAALIGN_BYTE; //DMA_MDATAALIGN_WORD;
                handle->dma->Init.PeriphInc           = DMA_PINC_ENABLE;
                handle->dma->Init.MemInc              = DMA_MINC_ENABLE;

                res = HAL_DMA_DeInit(handle->dma);
                if (res != HAL_OK)
                {
                    handle->ErrorCode = res | 0x4100;
                    return 0;
                }
                res = HAL_DMA_Init(handle->dma);
                if (res != HAL_OK)
                {
                    handle->ErrorCode = res | 0x4200;
                    return 0;
                }

                res = HAL_DMA_Start(handle->dma, handle->base + offset, (uint32_t)dbuff, dlen / 4);
                if (res != HAL_OK)
                {
                    handle->ErrorCode = res | 0x4300;
                    return 0;
                }

                res = HAL_DMA_PollForTransfer(handle->dma, HAL_DMA_FULL_TRANSFER, 1000);
                if (res != HAL_OK)
                {
                    handle->ErrorCode = res | 0x4400;
                    return 0;
                }

                //HAL_Delay_us_(5);
#endif
            }
            if (oob_buf != NULL)
                memcpy(oob_buf, (const void *)(handle->base + SPI_NAND_PAGE_SIZE), olen);
        }
    }

    return dlen + olen;
}


#define QSPI_FIFO_SIZE      (64)
__HAL_ROM_USED int HAL_NAND_WRITE_PAGE(FLASH_HandleTypeDef *handle, uint32_t addr, const uint8_t *buff, uint32_t len)
{
    return HAL_NAND_WRITE_WITHOOB(handle, addr, buff, len, NULL, 0);
}

__HAL_ROM_USED int HAL_NAND_WRITE_WITHOOB(FLASH_HandleTypeDef *handle, uint32_t addr,
        const uint8_t *buff, uint32_t len, const uint8_t *oob_buf, uint32_t olen)
{
    int busy;
    uint32_t *tbuf = NULL; // = (uint32_t *)buff;
    int res;
    HAL_StatusTypeDef ret;
    int row_addr = 0;

    if (handle == NULL || handle->ctable == NULL || handle->data_buf == NULL)
    {
        handle->ErrorCode = 1;
        return 0;
    }
    if ((buff == NULL) && (oob_buf == NULL))
    {
        handle->ErrorCode = 2;
        return 0;
    }
    if ((len > SPI_NAND_PAGE_SIZE) || (olen > SPI_NAND_MAXOOB_SIZE))
    {
        handle->ErrorCode = 3;
        return 0;
    }

    handle->ErrorCode = 0;

    // if (handle->ecc_en && (olen > 0)) // when ecc en, user can not write oob , it should fill by flash chip
    //     return 0;

    if (addr >= handle->base)
    {
        addr -= handle->base;
    }
    if ((oob_buf == NULL || olen == 0) && (((uint32_t)buff & 3) == 0))
    {
        tbuf = (uint32_t *)buff;
    }
    else
    {
        // copy buffer data and oob data to internal buffer
        if (oob_buf != NULL)
            memcpy(handle->data_buf + SPI_NAND_PAGE_SIZE, oob_buf, olen);
        if (buff != NULL)
        {
            memcpy(handle->data_buf, buff, len);
            tbuf = (uint32_t *)handle->data_buf;
            len = SPI_NAND_PAGE_SIZE + olen;
        }
        else
        {
            row_addr = SPI_NAND_PAGE_SIZE;
            tbuf = (uint32_t *)(handle->data_buf + SPI_NAND_PAGE_SIZE);
            len = olen;
        }
        // if olen > 0, data buffer should fill full page
    }

    // address include flash start address, there only use offset
    //addr &= (FLASH_SUPPORT_MAX_SIZE - 1);

    //if (handle->dma != NULL) // dma enable
#ifndef PSRAM_CACHE_WB
    if (!IS_DMA_ACCROSS_1M_BOUNDARY((uint32_t)tbuf, len)) // buffer not across MB range for DMA issue
    {
        ret = HAL_FLASH_DMA_START(handle, (char *)tbuf, 1, len);
        if (ret != HAL_OK)
        {
            handle->ErrorCode = 4;
            return 0;
        }
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
        if (handle->Mode == HAL_FLASH_NOR_MODE)
            HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PLD, row_addr);
        else
            HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_QPLD, row_addr);

        ret = HAL_FLASH_DMA_WAIT_DONE(handle, 1000);
        if (ret != HAL_OK)
        {
            handle->ErrorCode = 5;
            return 0;
        }
    }
    else
#endif
    {
        //return 0;
        int i, cnt;
        int remain = len;
        int fill = remain > QSPI_FIFO_SIZE ? QSPI_FIFO_SIZE : remain;
        cnt = 0;

        // write enable
        res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
        if (res != 0)
            return 0;
        // controller fifo 64 bytes, for page write, need 32 times to cache
        // first 64 bytes
        // write data to fifo
        for (i = 0; i < fill / 4; i++)
        {
            HAL_FLASH_WRITE_WORD(handle, *tbuf++);
        }

        // for first write in one page, use load program data
        HAL_FLASH_WRITE_DLEN(handle, fill);
        if (handle->Mode == HAL_FLASH_NOR_MODE)
            res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PLD, row_addr);
        else
            res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_QPLD, row_addr);
        if (res != 0)
            return 0;

        remain -= fill;
        cnt += fill;

        while (remain > 0)
        {
            fill = remain > QSPI_FIFO_SIZE ? QSPI_FIFO_SIZE : remain;

            // write enable
            res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
            // write data to fifo
            for (i = 0; i < fill / 4; i++)
            {
                HAL_FLASH_WRITE_WORD(handle, *tbuf++);
            }

            // use random load program data
            HAL_FLASH_WRITE_DLEN(handle, fill);
            if (handle->Mode == HAL_FLASH_NOR_MODE)
                res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PLDR, cnt);
            else
                res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_QPLDR, cnt);

            remain -= fill;
            cnt += fill;
        }
    }
    // write cache data to physical memory
    // write enable
    res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);

    // program execute, use page address, max support 3 bytes address for larger than 1Gb chips
    res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PEXE, (addr / SPI_NAND_PAGE_SIZE) & 0xffffff);

    // check busy, wait until write done
    do
    {
        HAL_FLASH_WRITE_DLEN(handle, 1);
        res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
        busy = HAL_FLASH_READ32(handle);
    }
    while (busy & 0x1);

    if (busy & NAND_P_FAIL_BIT)
    {
        //handle->ErrorCode = NAND_P_FAIL_BIT;
        handle->ErrorCode = busy;
        return -1;
    }
#if 0
    if (busy & handle->ctable->ecc_sta_mask)
    {
        int i, bits, bitc, mask;
        bits = 0;   // ecc status start bit
        for (i = 0; i < 8; i++)
            if ((handle->ctable->ecc_sta_mask & (1 << i)) != 0)
            {
                bits = i;
                break;
            }

        bitc = 0; // ecc status bit count
        for (i = bits; i < 8; i++)
            if ((handle->ctable->ecc_sta_mask & (1 << i)) != 0)
            {
                bitc++;
            }

        mask = busy & handle->ctable->ecc_sta_mask;
        // for 2 bit mode, only 0 and 1 correct, for 3 bit mode, only 7 can not correct
        if (((bitc == 2) && ((mask >> bits)  >= 2))
                || ((bitc == 3) && ((mask >> bits) >= 7)))
        {
            handle->ErrorCode = 0x8000 | (mask >> bits) ;
            return -2;
        }
    }
#endif
    return len;
}

__HAL_ROM_USED int HAL_NAND_ERASE_BLK(FLASH_HandleTypeDef *handle, uint32_t addr)
{
    int busy;
    //int8_t id = Addr2Id(addr);
    if (handle == NULL || handle->ctable == NULL)
    {
        //LOG_E("HAL_NAND_ERASE_BLK error param\n");
        handle->ErrorCode = 1;
        return -1;
    }
    handle->ErrorCode = 0;
    // address include flash start address, there only use offset
    //addr &= (FLASH_SUPPORT_MAX_SIZE - 1);
    if (addr >= handle->base)
        addr -= handle->base;

    // write enable
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);

    // block erase, address should be RA: row address(like page based), 3 bytes address but include some dummy at high bits
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_BE, (addr >> 11) & 0xffffff);

    // check busy
    do
    {
        HAL_FLASH_WRITE_DLEN(handle, 1);
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
        busy = HAL_FLASH_READ32(handle);
    }
    while (busy & 0x1);

    if (busy & NAND_E_FAIL_BIT)
    {
        //handle->ErrorCode = NAND_E_FAIL_BIT;
        handle->ErrorCode = busy;
        return -2;
    }

    return 0;
}

__HAL_ROM_USED int HAL_NAND_CONF_BUF(FLASH_HandleTypeDef *handle, uint8_t en)
{
    uint32_t sta;
    if (handle == NULL || handle->ctable == NULL)
        return -1;

    if (handle->ctable->mode_reg == 0 || handle->ctable->buf_mod_mask == 0)
        return -2;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);
    //rt_kprintf("Sta = 0x%x\n",sta);

    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WVSR, 0);

    if (en)
        sta |= handle->ctable->buf_mod_mask; // set to buf mode
    else
        sta &= ~handle->ctable->buf_mod_mask; // clear buf mode

    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    handle->buf_mode = en;

    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    //sta = HAL_FLASH_READ32(handle);
    //rt_kprintf("Sta2 = 0x%x\n",sta);

    return 0;
}

__HAL_ROM_USED int HAL_NAND_SET_CONTINUE(FLASH_HandleTypeDef *handle, uint32_t addr, uint8_t en, uint32_t data_len)
{
    if ((handle == NULL) || (handle->ctable == NULL) || (handle->ctable->buf_mod_mask == 0))    // NOT SUPPORT MODE SWITCH
        return HAL_ERROR;

    // switch addr to local addr
    if (addr >= handle->base)
        addr -= handle->base;

    if (en) // enable continue mode, set buf = 0, load page as addr, set ahb as continue timing
    {
        // 1. load page to cache, page read with block+page address
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PREAD, (addr / SPI_NAND_PAGE_SIZE) & 0xffff);
        // check busy
        uint32_t busy = 1;
        do
        {
            HAL_FLASH_WRITE_DLEN(handle, 1);
            HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
            busy = HAL_FLASH_READ32(handle) & 0x1;
        }
        while (busy);

        // 2. set buf bit, continue = !buf
        HAL_NAND_CONF_BUF(handle, 0);

        // 3. enable Hardware Interface
        __HAL_QSPI_EN_HWI(handle);

        HAL_FLASH_WRITE_DLEN(handle, data_len);

        // 4. configure read command
        if (handle->Mode != 0)
        {
            if (handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd == 0)
                return HAL_ERROR;
#if 1
            HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].func_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].data_mode,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_size,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_size,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ins_mode);
            __HAL_QSPI_SET_CMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd);
#else

            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_mode, 1);
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd);
#endif
        }
        else
        {
            if (handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd == 0)
                return HAL_ERROR;
#if 1
            HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].func_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].data_mode,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_size,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_size,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ins_mode);
            __HAL_QSPI_SET_CMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd);
#else

            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_mode, 1);
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd);
#endif
        }

    }
    else // set buf mode, set AHB read
    {
        __HAL_QSPI_DIS_HWI(handle);
        // clear TCF before next mpi command
        HAL_FLASH_CLR_CMD_DONE(handle);
        //HAL_FLASH_CONFIG_AHB_READ(handle, handle->Mode);
        HAL_NAND_CONF_BUF(handle, 1);
    }

    return HAL_OK;
}

__HAL_ROM_USED int HAL_NAND_AHB_CONTINUE(FLASH_HandleTypeDef *handle, uint32_t addr, uint8_t en)
{
    if ((handle == NULL) || (handle->ctable == NULL) || (handle->ctable->buf_mod_mask == 0))    // NOT SUPPORT MODE SWITCH
        return HAL_ERROR;

    // switch addr to local addr
    if (addr >= handle->base)
        addr -= handle->base;

    if (en) // enable continue mode, set buf = 0, load page as addr, set ahb as continue timing
    {
        // 1. load page to cache, page read with block+page address
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PREAD, (addr / SPI_NAND_PAGE_SIZE) & 0xffff);
        // check busy
        uint32_t busy = 1;
        do
        {
            HAL_FLASH_WRITE_DLEN(handle, 1);
            HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
            busy = HAL_FLASH_READ32(handle) & 0x1;
        }
        while (busy);

        // 2. set buf bit, continue = !buf
        HAL_NAND_CONF_BUF(handle, 0);

        // 3. configure ahb read command
        if (handle->Mode != 0)
        {
            if (handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd == 0)
                return HAL_ERROR;

            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_mode, 1);
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd);
        }
        else
        {
            if (handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd == 0)
                return HAL_ERROR;

            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_mode, 1);
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd);
        }

    }
    else // set buf mode, set AHB read
    {
        //__HAL_QSPI_DIS_HWI(handle);
        //HAL_FLASH_CONFIG_AHB_READ(handle, handle->Mode);
        HAL_NAND_CONF_BUF(handle, 1);
    }

    return HAL_OK;
}


__HAL_ROM_USED int HAL_NAND_MARK_BADBLK(FLASH_HandleTypeDef *handle, uint32_t blk, uint8_t bad)
{
    uint32_t addr;
    int res;
    uint8_t tbuf[4];

    if ((handle == NULL) || (handle->data_buf == NULL))
        return HAL_ERROR;

    memset(tbuf, 0xff, 4);
    // block to addr, check first page
    addr = blk << 17;
    //addr += handle->base;

    if (bad)
    {
        tbuf[0] = 0;
        tbuf[1] = 0xbd;
    }
    else    // it can not be recover except erase, bits only 1 -> 0
    {
        tbuf[0] = 0xff;
        tbuf[1] = 0xff;
        return HAL_ERROR;
    }
    res = HAL_NAND_WRITE_WITHOOB(handle, addr, NULL, 0, tbuf, 4);
    if (res <= 0)
        return HAL_ERROR;

    return HAL_OK;
}

__HAL_ROM_USED int HAL_NAND_GET_BADBLK(FLASH_HandleTypeDef *handle, uint32_t blk)
{
    uint32_t addr;
    uint32_t value;
    uint8_t *tbuf = (uint8_t *)&value;

    if ((handle == NULL) || (handle->data_buf == NULL))
        return 0;

    // block to addr, check first page
    addr = blk << 17;
    //addr += handle->base;

    int res = HAL_NAND_READ_WITHOOB(handle, addr, NULL, 0, tbuf, 4);
    if (res == 0)
        return 0;

    if (tbuf[0] == 0xff)
        return 0;

    if (value == 0)
        return 1;

    return (int)value;
}

__HAL_ROM_USED int HAL_NAND_EN_QUAL(FLASH_HandleTypeDef *handle, uint8_t en)
{
    uint32_t sta;
    if (handle == NULL || handle->ctable == NULL)
        return -1;

    if (handle->ctable->mode_reg == 0 || handle->ctable->qe_mod_mask == 0)
        return 0;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);

    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WVSR, 0);
    if (en)
        sta |= handle->ctable->qe_mod_mask; // SET QE bit
    else
        sta &= ~handle->ctable->qe_mod_mask; // clear QE bit
    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    return 0;
}


__HAL_ROM_USED int nand_clear_status(FLASH_HandleTypeDef *handle)
{
    uint32_t status;

    status = 0;
    HAL_FLASH_WRITE_DLEN(handle, 1);
#ifdef HYF_SPECIAL_SUPPORT
    // for some HYF chips, need set 2 before clear protect, others no this request
    HAL_FLASH_WRITE_WORD(handle, 2);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->protect_reg);
#endif
    HAL_FLASH_WRITE_WORD(handle, 0);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->protect_reg);
    //HAL_FLASH_WRITE_WORD(handle, 0);
    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);
    //HAL_FLASH_WRITE_WORD(handle, 0);
    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->status_reg);

    return status;
}

__HAL_ROM_USED int nand_get_status(FLASH_HandleTypeDef *handle)
{
    uint32_t status, sta;

    status = 0;
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->protect_reg);
    sta = HAL_FLASH_READ32(handle);
    status = (sta & 0xff);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);
    status = (status << 8) | (sta & 0xff);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
    sta = HAL_FLASH_READ32(handle);
    status = (status << 8) | (sta & 0xff);
    return status;
}

/************************************************
* Check some chip, all otp bit in B0 register is same, so defined to fix bit
* Include: winbond, giga, ziguang, dosilicon, hyf
* If they changed in some new chip, need add a new member for it in SPI_FLASH_FACT_CFG_T
* Note: OTP can only write once even not lock, bits can noly 1 to 0, erase not support !!!
**************************************************/
#define NAND_OTP_EN_BIT     (1<<6)
#define NAND_OTP_LOCK_BIT   (1<<7)
#define NAND_ECC_EN_BIT     (1<<4)  /* TODO: if ECC_E not this bit, need modify it ! */

__HAL_ROM_USED int HAL_NAND_SWITCH_OTP(FLASH_HandleTypeDef *handle, uint8_t otp)
{
    uint32_t sta;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);

    if (otp)
    {
        if ((sta & NAND_OTP_EN_BIT) != 0) // otp enable has been set
        {
            return 0;
        }
        else
        {
            sta &= 0xff;
            sta |= NAND_OTP_EN_BIT;
            sta &= ~(NAND_ECC_EN_BIT); // disable ECC to make sure OTP can program more times
        }
    }
    else
    {
        if ((sta & NAND_OTP_EN_BIT) == 0) // otp enable has been disable
        {
            return 0;
        }
        else
        {
            sta &= 0xff;
            sta &= ~NAND_OTP_EN_BIT;
            if (handle->ecc_en)
                sta |= NAND_ECC_EN_BIT;
        }
    }
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    return 0;
}

__HAL_ROM_USED int HAL_NAND_LOCK_OTP(FLASH_HandleTypeDef *handle)
{
    uint32_t busy, sta;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);

    if ((sta & NAND_OTP_LOCK_BIT) != 0) // otp enable has been locked
    {
        return 0;
    }
    else
    {
        sta &= 0xff;
        sta |= (NAND_OTP_EN_BIT | NAND_OTP_LOCK_BIT);
    }

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    // begin lock process
    int res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
    if (res != 0)
        return 1;

    // program execute, use page address
    res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PEXE, 0);
    if (res != 0)
        return 1;

    // check busy, wait until program done
    HAL_FLASH_WRITE_DLEN(handle, 1);
    do
    {
        res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
        busy = HAL_FLASH_READ32(handle);
    }
    while (busy & 0x1);

    return 0;
}

/***************** Interface for nor flash ********************************/


__HAL_ROM_USED int HAL_QSPIEX_WRITE_PAGE(FLASH_HandleTypeDef *hflash, uint32_t addr, const uint8_t *buf, uint32_t size)
{
    HAL_StatusTypeDef ret;
    int i, aligned_size;
    SPI_FLASH_CMD_E cid;
    uint16_t dlen;
    uint32_t param;
    //rt_base_t level;

    if (!flash_handle_valid(hflash))
        return 0;
    if (size == 0)
        return 0;

    if (hflash->buf_mode == 1)  // for nor, it means open dtr, set normal timing for write
    {
        //HAL_NOR_CFG_DTR(hflash, 0);
    }
    aligned_size = QSPI_NOR_PAGE_SIZE << hflash->dualFlash;
    if (size > aligned_size)
        size = aligned_size;
    //level = rt_hw_interrupt_disable();
    if (hflash->dma != NULL)
    {
        //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_FIFO);
        //LOG_D("HAL_QSPIEX_WRITE_PAGE dma: 0x%x\n", addr);
        if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
                || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        {
            if (hflash->Mode == HAL_FLASH_QMODE)
                cid = SPI_FLASH_CMD_QPP4BA;
            else
                cid = SPI_FLASH_CMD_PP4BA;
        }
        else
        {
            if (hflash->Mode == HAL_FLASH_QMODE)
                cid = SPI_FLASH_CMD_QPP;
            else
                cid = SPI_FLASH_CMD_PP;
        }
        // add pre command process to make FLASH as write mode , to avoid prev read error.
        HAL_FLASH_PRE_CMD(hflash, cid);
        ret = HAL_FLASH_DMA_START(hflash, (char *)buf, 1, size);
        if (ret != HAL_OK)
        {
            //rt_hw_interrupt_enable(level);
            //LOG_E("nor dma start fail\n");
            size = 0;
            goto exit;
            //return 0;
        }

        //HAL_FLASH_SET_CMD(hflash, FCMD_WREN, addr);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);

#ifdef QSPI_USE_CMD2
        dlen = 1;
        param = 0x01;
        dlen = dlen << hflash->dualFlash;
        if (hflash->dualFlash)
            param = 0x0101;
        HAL_FLASH_WRITE_DLEN2(hflash, dlen);
        i = HAL_FLASH_ISSUE_CMD_SEQ(hflash, cid, addr, SPI_FLASH_CMD_RDSR, param);
        if (i != 0)
        {
            size = 0;
            goto exit;
            //return 0;
        }
#else

        HAL_FLASH_ISSUE_CMD(hflash, cid, addr);

        ret = HAL_FLASH_DMA_WAIT_DONE(hflash, 1000);
        if (ret != HAL_OK)
        {
            //rt_hw_interrupt_enable(level);
            //LOG_E("nor wait dma done fail\n");
            size = 0;
            goto exit;
            //return 0;
        }

        // make sure program done
        HAL_FLASH_WRITE_DLEN(hflash, 2);    // cover data len
        bool res;
        do
        {
            //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR, addr);
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
        //rt_hw_interrupt_enable(level);
#endif
        // clear DMAE
        hflash->Instance->CR &= ~MPI_CR_DMAE;
    }
    else
    {
        //LOG_E("FLASH must enable DMA !");
        //ASSERT(0);
        size = 0;
        goto exit;
        //return 0;
    }
exit:
    if (hflash->buf_mode == 1)  // for nor, it means open dtr, recover read config
    {
        //HAL_MPI_CFG_DTR(hflash, true, hflash->ecc_en);
        //HAL_NOR_CFG_DTR(hflash, 1);
    }
    return size;
}

// write length alway be 1
// if address not 2 aligned, prev = 1 to fill prev flash byte to buffer
// if address 2 aligned, prev = 0 to fill next byte of flash to buffer
__HAL_ROM_USED int HAL_QSPIEX_FILL_EVEN(FLASH_HandleTypeDef *hflash, uint32_t addr, uint8_t *buf, uint8_t prev)
{
    uint16_t temp;
    uint8_t *dst_buf;
    uint8_t *align_buf = (uint8_t *)&temp;
    dst_buf = (uint8_t *)(addr | hflash->base);
    if (prev) // get prev byte from flash
    {
        dst_buf--;
        align_buf[0] = *dst_buf;    // read a byte from flash with prev address
        align_buf[1] = *buf;        // fill the second byte by buf 1st byte
        // fill these 2 bytes to flash
        HAL_QSPIEX_WRITE_PAGE(hflash, addr - 1, align_buf, 2);
    }
    else    // get next byte from flash
    {
        align_buf[0] = *buf;    // fill this byte to buffer
        dst_buf++;
        align_buf[1] = *dst_buf;        // fill the second byte by flash next byte
        // fill these 2 bytes to flash
        HAL_QSPIEX_WRITE_PAGE(hflash, addr, align_buf, 2);
    }

    return 1;
}

__HAL_ROM_USED int nor_write_rom(FLASH_HandleTypeDef *hflash, uint32_t addr, const uint8_t *buf, uint32_t size)
{
    int i, cnt, taddr, tsize, aligned_size, start;
    uint8_t *tbuf;

    if (hflash == NULL || addr < hflash->base || size == 0)
        return 0;

    cnt = 0;
    tsize = size;
    tbuf = (uint8_t *)buf;
    taddr = addr - hflash->base;
#if 1
    if (hflash->dualFlash) // need lenght and address 2 aligned
    {
        if (taddr & 1) // dst odd, make 2 bytes write
        {
            HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 1);
            // update buffer and address
            taddr++;
            tbuf++;
            tsize--;
            cnt++;
        }
    }
#endif
    if (tsize <= 0)
        return cnt;

    // check address page align
    aligned_size = QSPI_NOR_PAGE_SIZE << hflash->dualFlash;
    //cnt = taddr - (taddr & (~(aligned_size - 1)));
    start = taddr & (aligned_size - 1);
    if (start > 0)    // start address not page aligned
    {
        start = aligned_size - start;
        if (start > tsize)    // not over one page
        {
            start = tsize;
        }
#if 1
        if (hflash->dualFlash && (tsize & 1))   // for this case, it should be the laster write
        {
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize & (~1));

            taddr += i;
            tbuf += i;
            //tsize -= i;
            HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 0);
            cnt += tsize;

            return cnt;
        }
        else
#endif
        {
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, start);
            if (i != start)
            {
                return 0;
            }
        }
        taddr += start;
        tbuf += start;
        tsize -= start;
        cnt += start;
        //rt_hw_interrupt_enable(level);
    }
    // process page aligned data
    while (tsize >= aligned_size)
    {
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, aligned_size);
        cnt += aligned_size;
        taddr += aligned_size;
        tbuf += aligned_size;
        tsize -= aligned_size;
        //LOG_D("write:  %d\n", cnt);
    }

    //level = rt_hw_interrupt_disable();
    // remain size
    if (tsize > 0)
    {
#if 1
        if (hflash->dualFlash && (tsize & 1))
        {
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize & (~1));

            if (tsize & 1)  // remain 1 byte
            {
                //cnt += i;
                taddr += i;
                tbuf += i;
                //tsize -= i;
                HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 0);
                //taddr++;
                //tbuf++;
                //tsize--;
                //cnt++;
            }
            cnt += tsize;
        }
        else
#endif
        {
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize);
            if (i != tsize)
            {
                return 0;
            }
            cnt += tsize;
        }
    }

    return cnt;
}

__HAL_ROM_USED int nor_page_erase(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint16_t dlen;
    uint32_t param;
    //rt_base_t level;
    //int8_t id = Addr2Id(addr);
    //if (id < 0)
    //    return RT_ERROR;
    if (!flash_handle_valid(hflash))
        return -1;
    if (flash_cmd_valid(hflash, SPI_FLASH_CMD_PE) == 0)
        return -1;
    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        return -1;

    //level = rt_hw_interrupt_disable();

    //HAL_FLASH_SET_CMD(hflash, FCMD_WREN, addr);
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;
    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_PE, addr, SPI_FLASH_CMD_RDSR, param) != 0)
        return -1;
#else

    //HAL_FLASH_SET_CMD(hflash, FCMD_PE, addr);
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_PE, addr);
    bool res = true;
    do
    {
        //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR, addr);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif
    //rt_hw_interrupt_enable(level);
    return 0;
}
__HAL_ROM_USED int HAL_QSPIEX_SECT_ERASE(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint16_t dlen;
    uint32_t param;
    SPI_FLASH_CMD_E ecmd;
    int ret = 0;
    //rt_base_t level;
    //int8_t id = Addr2Id(addr);
    //if (id < 0)
    //    return RT_ERROR;

    if (!flash_handle_valid(hflash))
        return -1;

    if (hflash->buf_mode == 1)  // for nor, it means open dtr, set normal timing for erase
    {
        //HAL_MPI_CFG_DTR(hflash, false, 0);
        //HAL_NOR_CFG_DTR(hflash, 0);
    }
    //level = rt_hw_interrupt_disable();
    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        ecmd = SPI_FLASH_CMD_SE4BA;
    else
        ecmd = SPI_FLASH_CMD_SE;

    //HAL_FLASH_SET_CMD(hflash, FCMD_WREN, addr);
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;

    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, ecmd, addr, SPI_FLASH_CMD_RDSR, param) != 0)
    {
        ret = -1;
        goto exit;
        //return -1;
    }
#else
    bool res = true;

    // HAL_FLASH_SET_CMD(hflash, FCMD_SE, addr);
    HAL_FLASH_ISSUE_CMD(hflash, ecmd, addr);
    HAL_FLASH_WRITE_DLEN(hflash, 1);
    do
    {
        //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR, addr);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif
    //rt_hw_interrupt_enable(level);
exit:
    if (hflash->buf_mode == 1)  // for nor, it means open dtr, recover read config
    {
        //HAL_MPI_CFG_DTR(hflash, true, hflash->ecc_en);
        //HAL_NOR_CFG_DTR(hflash, 1);
    }

    return ret;
}

__HAL_ROM_USED int HAL_QSPIEX_BLK32_ERASE(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint16_t dlen;
    uint32_t param;
    int ret = 0;
    //rt_base_t level;
    //int8_t id = Addr2Id(addr);
    //if (id < 0)
    //    return RT_ERROR;
    if (!flash_handle_valid(hflash))
        return -1;

    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        return -1;
    //level = rt_hw_interrupt_disable();
    if (hflash->buf_mode == 1)  // for nor, it means open dtr, set normal timing for erase
    {
        //HAL_MPI_CFG_DTR(hflash, false, 0);
        //HAL_NOR_CFG_DTR(hflash, 0);
    }

    //HAL_FLASH_SET_CMD(hflash, FCMD_WREN, addr);
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;

    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_BE32, addr, SPI_FLASH_CMD_RDSR, param) != 0)
    {
        ret = -1;
        goto exit;
        //return -1;
    }
#else
    bool res = true;

    //HAL_FLASH_SET_CMD(hflash, FCMD_BE32, addr);
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_BE32, addr);
    HAL_FLASH_WRITE_DLEN(hflash, 1);
    do
    {
        //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR, addr);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
    //rt_hw_interrupt_enable(level);
#endif

exit:
    if (hflash->buf_mode == 1)  // for nor, it means open dtr, recover read config
    {
        //HAL_MPI_CFG_DTR(hflash, true, hflash->ecc_en);
        //HAL_NOR_CFG_DTR(hflash, 1);
    }

    return ret;
}

__HAL_ROM_USED int HAL_QSPIEX_BLK64_ERASE(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint16_t dlen;
    uint32_t param;
    SPI_FLASH_CMD_E ecmd;
    int ret = 0;
    //rt_base_t level;
    //int8_t id = Addr2Id(addr);
    //if (id < 0)
    //    return RT_ERROR;
    if (!flash_handle_valid(hflash))
        return -1;

    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        ecmd = SPI_FLASH_CMD_BE4BA;
    else
        ecmd = SPI_FLASH_CMD_BE64;

    //level = rt_hw_interrupt_disable();
    if (hflash->buf_mode == 1)  // for nor, it means open dtr, set normal timing for erase
    {
        //HAL_MPI_CFG_DTR(hflash, false, 0);
        //HAL_NOR_CFG_DTR(hflash, 0);
    }

    //HAL_FLASH_SET_CMD(hflash, FCMD_WREN, addr);
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;

    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, ecmd, addr, SPI_FLASH_CMD_RDSR, param) != 0)
    {
        ret = -1;
        goto exit;
        //return -1;
    }
#else
    bool res = true;

    //HAL_FLASH_SET_CMD(hflash, FCMD_BE64, addr);
    HAL_FLASH_ISSUE_CMD(hflash, ecmd, addr);
    HAL_FLASH_WRITE_DLEN(hflash, 1);
    do
    {
        //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR, addr);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif
    //rt_hw_interrupt_enable(level);
exit:
    if (hflash->buf_mode == 1)  // for nor, it means open dtr, recover read config
    {
        //HAL_MPI_CFG_DTR(hflash, true, hflash->ecc_en);
        //HAL_NOR_CFG_DTR(hflash, 1);
    }

    return ret;
}

__HAL_ROM_USED int HAL_QSPIEX_CHIP_ERASE(FLASH_HandleTypeDef *hflash)
{
    uint16_t dlen;
    uint32_t param;
    int ret = 0;

    if (!flash_handle_valid(hflash))
        return -1;

    //if (hflash->Mode == HAL_FLASH_QMODE)
    //    HAL_FLASH_SET_QUAL_SPI(hflash, false);

    //level = rt_hw_interrupt_disable();
    if (hflash->buf_mode == 1)  // for nor, it means open dtr, set normal timing for erase
    {
        //HAL_MPI_CFG_DTR(hflash, false, 0);
        //HAL_NOR_CFG_DTR(hflash, 0);
    }

    //HAL_FLASH_SET_CMD(hflash, FCMD_WREN, 0);
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;

    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_CE, 0, SPI_FLASH_CMD_RDSR, param) != 0)
    {
        ret = -1;
        goto exit;
        //return -1;
    }
#else
    //HAL_FLASH_SET_CMD(hflash, FCMD_CE, 0);
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_CE, 0);
    HAL_FLASH_WRITE_DLEN(hflash, 1);
    bool res = true;
    do
    {
        //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR, 0);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif

exit:

    if (hflash->Mode == HAL_FLASH_QMODE)
    {
        if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
                || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
            HAL_FLASH_FADDR_SET_QSPI(hflash, true);
        else
        {
            HAL_FLASH_SET_QUAL_SPI(hflash, true);
            if (hflash->buf_mode == 1)  // for nor, it means open dtr, recover read config
            {
                //HAL_MPI_CFG_DTR(hflash, true, hflash->ecc_en);
                //HAL_NOR_CFG_DTR(hflash, 1);
            }
        }
    }

    return ret;
}

__HAL_ROM_USED int nor_erase_rom(FLASH_HandleTypeDef *hflash, uint32_t addr, uint32_t size)
{
    uint32_t al_size;
    uint32_t al_addr;
    //rt_base_t level;
    int ret = 0;

    if (size == 0)
        return 0;

    if (size >= hflash->size)
        return HAL_QSPIEX_CHIP_ERASE(hflash);
    //level = rt_hw_interrupt_disable();

    if (!IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), addr))
    {
        HAL_ASSERT(0);
        ret = -1;
        goto _exit;
    }
    if (!IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), size))
    {
        HAL_ASSERT(0);
        ret = -1;
        goto _exit;
    }
    // set to single before erase, recover later
    //if (hflash->Mode == HAL_FLASH_QMODE)
    //    HAL_FLASH_SET_QUAL_SPI(hflash, false);

    //addr &= (NOR_SUPPORT_MAX_SIZE - 1);

    // address alinged down to page, size aligned up to page size
    //al_addr = GetPage(addr);
    //al_size = GetPSize((addr-al_addr)+size);
    // page erase not support, start addr should be aligned.
    al_addr = GET_ALIGNED_DOWN((QSPI_NOR_SECT_SIZE << hflash->dualFlash), addr);
    al_size = GET_ALIGNED_UP((QSPI_NOR_SECT_SIZE << hflash->dualFlash), size);

    //HAL_FLASH_CLR_SR(hflash);
    //LOG_D("flash erase from 0x%x + %d to 0x%x + %d\n", addr, size, al_addr, al_size);
    // 1 block 64k aligned, for start addr not aligned do not process, need support later
    if (IS_ALIGNED((QSPI_NOR_BLK64_SIZE << hflash->dualFlash), al_addr) && (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash))) // block erease first
    {
        while (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash))
        {
            HAL_QSPIEX_BLK64_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_BLK64_SIZE << hflash->dualFlash;
            al_addr += QSPI_NOR_BLK64_SIZE << hflash->dualFlash;
        }
        //LOG_D("Block64 erase to 0x%x\n", al_addr);
    }
#if 0   // for some chip like 32MB winbond, it not support 4 byte blcok32 erase
    // 2 block 32 aligned.
    if ((al_size >= (QSPI_NOR_BLK32_SIZE << hflash->dualFlash)) && IS_ALIGNED((QSPI_NOR_BLK32_SIZE << hflash->dualFlash), al_addr))
    {
        while (al_size >= (QSPI_NOR_BLK32_SIZE << hflash->dualFlash))
        {
            HAL_QSPIEX_BLK32_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_BLK32_SIZE << hflash->dualFlash;
            al_addr += QSPI_NOR_BLK32_SIZE << hflash->dualFlash;
        }
        //LOG_D("Block32 erase to 0x%x\n", al_addr);
    }
#endif
    // sector aligned
    if ((al_size >= (QSPI_NOR_SECT_SIZE << hflash->dualFlash)) && IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), al_addr))
    {
        while (al_size >= (QSPI_NOR_SECT_SIZE << hflash->dualFlash))
        {
            HAL_QSPIEX_SECT_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_SECT_SIZE << hflash->dualFlash;
            al_addr += QSPI_NOR_SECT_SIZE << hflash->dualFlash;
        }
        //LOG_D("sector erase to 0x%x\n", al_addr);
    }

#if 0
    // page aligned
    if ((al_size >= FLASH_PAGE_SIZE) && ADDR_PAGE_ALIGN(al_addr))
    {
        cnt = 0;
        while (cnt < al_size)
        {
            rt_flash_page_erase(hflash, al_addr);
            cnt += FLASH_PAGE_SIZE;
            al_addr += FLASH_PAGE_SIZE;
        }
        al_size -= cnt;
        //LOG_D("page erase to 0x%x\n", al_addr);
    }
#endif

    if (al_size > 0)    // something wrong
    {
        ret = -1;
        goto _exit;
    }
    //todo , size lager than block but start addr not aligned, only page erase
    //
    if (hflash->Mode == HAL_FLASH_QMODE)
    {
        HAL_FLASH_SET_QUAL_SPI(hflash, true);
    }


_exit:

    //rt_hw_interrupt_enable(level);

    return ret;

}

__HAL_ROM_USED uint32_t HAL_QSPI_GET_SRC_CLK(FLASH_HandleTypeDef *fhandle)
{
    int src;
    uint32_t freq;
    int clk_module;

    if (NULL == fhandle)
        return 0;

    if (FLASH1 == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH1;
    }
    else if (FLASH2 == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH2;
    }
#ifdef FLASH3
    else if (FLASH3 == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH3;
    }
#endif
#ifdef SF32LB58X
    else if (FLASH4 == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH4;
    }
    else if (FLASH3 + HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH3;
    }
    else if (FLASH4 + HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH4;
    }
#endif
#ifdef FLASH5
    else if (FLASH5 == fhandle->Instance)
    {
        freq = HAL_RCC_GetHCLKFreq(CORE_ID_LCPU);
        return freq;
    }
#endif
    else
    {
        return 0;
    }

    src = HAL_RCC_HCPU_GetClockSrc(clk_module);
    if (RCC_CLK_FLASH_DLL2 == src)
    {
        freq = HAL_RCC_HCPU_GetDLL2Freq();
    }
    else if (RCC_CLK_FLASH_DLL3 == src)
    {
        freq = HAL_RCC_HCPU_GetDLL3Freq();
    }
    else
    {
        freq = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
    }

    return freq;
}

__HAL_ROM_USED uint32_t HAL_QSPI_GET_CLK(FLASH_HandleTypeDef *fhandle)
{
    uint8_t div;
    uint32_t freq;

    if (NULL == fhandle)
        return 0;

    div = HAL_FLASH_GET_DIV(fhandle);
    if (div <= 0)
        return 0;

    freq = HAL_QSPI_GET_SRC_CLK(fhandle);
    freq /= div;

    return freq;
}

__HAL_ROM_USED int HAL_QSPI_READ_ID(FLASH_HandleTypeDef *fhandle)
{
    if (fhandle == NULL)
        return FLASH_UNKNOW_ID;

    if (fhandle->isNand)
        return nand_read_id(fhandle, 0);
    else
        return HAL_FLASH_GET_NOR_ID(fhandle);

    return FLASH_UNKNOW_ID;
}

__HAL_ROM_USED int HAL_QSPI_GET_MEMSIZE(FLASH_HandleTypeDef *fhandle)
{
    if (fhandle == NULL)
        return 0;
    return fhandle->size;
}


// it only called in bootloader to detect dual flash, can not called in xip
__HAL_ROM_USED int qspi_flash_get_id2(FLASH_HandleTypeDef *hflash)
{
    uint32_t id, temp, id2;;
    if (hflash->dualFlash == 0)
        return 0;

    //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_FIFO);

    // configure CCR
    //HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 2, 1, 1);
    HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 0, 0, 1);

    // configure data length, 2 byte, manufacturer id and device id, 0X90
    // configure data length, 3 byte, manufacturer id , memory type id and capacity id, 0X9F
    HAL_FLASH_WRITE_DLEN(hflash, 6);

    // send command, read id command
    //HAL_FLASH_SET_CMD(hflash, 0x90, 0);
    HAL_FLASH_SET_CMD(hflash, 0x9f, 0);

    //while (HAL_FLASH_IS_RX_EMPTY(hflash));

    id = HAL_FLASH_READ32(hflash);
    id2 = HAL_FLASH_READ32(hflash);
    // big/little endian?
    // ------- memory type --------| --- manufacturer id
    temp = ((id & 0xff00) >> 8) | ((id & 0xff000000) >> 16);
    // --- mem desity-----
    temp |= (id2 & 0xff00) << 8;
    id = temp;

    return (int)(id & 0xffffff);
}

__HAL_ROM_USED int HAL_QSPIEX_FLASH_WRITE(FLASH_HandleTypeDef *hflash, uint32_t addr, const uint8_t *buf, uint32_t size)
{
    int cnt = 0;

    if (hflash == NULL || addr < hflash->base || buf == NULL)
        return 0;

    if (hflash->isNand == 0)
        cnt = nor_write_rom(hflash, addr, buf, size);
    else
        cnt = HAL_NAND_WRITE_PAGE(hflash, addr, buf, size);

    return cnt;
}

__HAL_ROM_USED int HAL_QSPIEX_FLASH_ERASE(FLASH_HandleTypeDef *hflash, uint32_t addr, uint32_t size)
{
    int res = 0;
    int i;

    if (hflash == NULL || addr < hflash->base)
        return 0;

    if (hflash->isNand == 0)
        res = nor_erase_rom(hflash, addr, size);
    else
        for (i = 0; i < size / SPI_NAND_BLK_SIZE; i++)
        {
            res = HAL_NAND_GET_BADBLK(hflash, (addr >> 17) + i);
            if (res) // block is bad, do not erase, or bad mark will be cover
                return -2;
            res = HAL_NAND_ERASE_BLK(hflash, addr + i * SPI_NAND_BLK_SIZE);
            if (res) // erase fail
                return -1;
        }

    return res;

}

__HAL_ROM_USED void HAL_QSPIEX_FLASH_RESET(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return ;

    HAL_FLASH_MANUAL_CMD(hflash, 0, 0, 0, 0, 0, 0, 0, 1);
    HAL_FLASH_SET_CMD(hflash, 0x66, 0);

    HAL_Delay_us(300);
    // add a delay?
    HAL_FLASH_MANUAL_CMD(hflash, 0, 0, 0, 0, 0, 0, 0, 1);
    HAL_FLASH_SET_CMD(hflash, 0x99, 0);

    return ;
}

__HAL_ROM_USED void HAL_QSPIEX_FLASH_RESET2(MPI_TypeDef *hmpi)
{
    uint32_t value, en;
    if (hmpi == NULL)
        return ;

    en = hmpi->CR & MPI_CR_EN_Msk;
    hmpi->CR |= MPI_CR_EN;

    value = ((0 << MPI_CCR1_FMODE_Pos) | (0 << MPI_CCR1_DMODE_Pos)
             | (0 << MPI_CCR1_DCYC_Pos) | (0 << MPI_CCR1_ADSIZE_Pos)
             | (0 << MPI_CCR1_ABMODE_Pos) | (0 << MPI_CCR1_ABSIZE_Pos)
             | (0 << MPI_CCR1_ADMODE_Pos) | (1 << MPI_CCR1_IMODE_Pos));
    hmpi->CCR1 = value;

    hmpi->AR1 = 0;
    hmpi->CMDR1 = 0x66;

    while (!(hmpi->SR & MPI_SR_TCF));
    hmpi->SCR |= MPI_SCR_TCFC;


    HAL_Delay_us(30);   // need delay 30 us
    // add a delay?
    value = ((0 << MPI_CCR1_FMODE_Pos) | (0 << MPI_CCR1_DMODE_Pos)
             | (0 << MPI_CCR1_DCYC_Pos) | (0 << MPI_CCR1_ADSIZE_Pos)
             | (0 << MPI_CCR1_ABMODE_Pos) | (0 << MPI_CCR1_ABSIZE_Pos)
             | (0 << MPI_CCR1_ADMODE_Pos) | (1 << MPI_CCR1_IMODE_Pos));
    hmpi->CCR1 = value;

    hmpi->AR1 = 0;
    hmpi->CMDR1 = 0x99;

    while (!(hmpi->SR & MPI_SR_TCF));
    hmpi->SCR |= MPI_SCR_TCFC;

    HAL_Delay_us(12000);    // delay 12ms

    if (en == 0)
    {
        hmpi->CR &= ~MPI_CR_EN;
    }
    return ;
}


__HAL_ROM_USED int HAL_QSPI_SET_DMABURST(FLASH_HandleTypeDef *hflash, uint8_t blength)
{
    if (hflash == NULL || hflash->dma == NULL)
        return -1;

    if (blength >= 32)
        return -2;

    hflash->dma->Init.BurstSize  = blength;

    HAL_FLASH_SET_TXSLOT(hflash, hflash->dma->Init.BurstSize);

    return 0;
}

__HAL_ROM_USED int HAL_QSPI_ENABLE_WDT()
{
    FLASH_HandleTypeDef hflash;
    uint16_t value = 0x3ff;

    hflash.Instance = FLASH1;
    HAL_FLASH_SET_WDT(&hflash, value);
    hflash.Instance = FLASH2;
    HAL_FLASH_SET_WDT(&hflash, value);
#ifdef FLASH3
    hflash.Instance = FLASH3;
    HAL_FLASH_SET_WDT(&hflash, value);
#endif
    return 0;
}

__HAL_ROM_USED uint32_t HAL_QSPI_GET_SR(FLASH_HandleTypeDef *hflash)
{
    uint32_t srl, srh, sr;

    if (hflash == NULL)
        return 0;

    srl = srh = sr = 0;
    if (hflash->isNand == 0)
    {
        //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_TX_FIFO);

        HAL_FLASH_WRITE_DLEN(hflash, 1 << hflash->dualFlash);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        srl = HAL_FLASH_READ32(hflash) ;

        int res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
        if (res == 0)
            srh = HAL_FLASH_READ32(hflash);

        //rt_kprintf("Get flash SRL 0x%x, SRH 0x%x\n", srl, srh);
        sr = ((srh) << 16) | (srl);
    }
    else
    {
        sr = nand_get_status(hflash);
        //rt_kprintf("Get flash SR 0x%08x\n", srl);
    }

    return sr;
}

__HAL_ROM_USED int HAL_QSPI_ERASE_OTP(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint32_t srh;
    uint16_t dlen;
    int res, opbit;
    uint32_t param;

    if (hflash == NULL || hflash->ctable == NULL)
        return -1;
    if (addr < SPI_FLASH_OTP_BASE || addr > SPI_FLASH_OTP_BASE + (hflash->ctable->mode_reg << 12))
        return -1;

    srh = HAL_QSPI_GET_OTP_LB(hflash, addr);
    //rt_kprintf("srh = %d\n", srh);
    opbit = addr >> 12;
    if (opbit < 1 || opbit > hflash->ctable->mode_reg)
        return -1;
    opbit = 1 << (opbit - 1);
    if (opbit & srh) // this security register has been locked, can not erase any more
        return -2;
    //rt_kprintf("opbit = %d\n", opbit);

    addr = addr << hflash->dualFlash;
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = param | (param << 8);
    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    res = HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_ERSCUR, addr, SPI_FLASH_CMD_RDSR, param);
    if (res != 0)
        return -3;
#else
    res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_ERSCUR, addr);

    HAL_FLASH_WRITE_DLEN(hflash, 1 << hflash->dualFlash);
    if (res == 0)
    {
        do
        {
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
    }
#endif
    return 0;
}

__HAL_ROM_USED int HAL_QSPI_WRITE_OTP(FLASH_HandleTypeDef *hflash, uint32_t addr, uint8_t *buf, uint32_t size)
{
    uint32_t srh;
    uint16_t dlen;
    int res, opbit;
    HAL_StatusTypeDef ret;
    uint32_t param;

    if (hflash == NULL || hflash->ctable == NULL)
        return 0;
    if (addr < SPI_FLASH_OTP_BASE || addr > SPI_FLASH_OTP_BASE + (hflash->ctable->mode_reg << 12))
        return 0;

    if ((addr & 0x3ff) + size   > hflash->ctable->oob_size * 256)
        return 0;

    srh = HAL_QSPI_GET_OTP_LB(hflash, addr);
    opbit = addr >> 12;
    if (opbit < 1 || opbit > hflash->ctable->mode_reg)
        return 0;
    opbit = 1 << (opbit - 1);
    if (opbit & srh) // this security register has been locked, can not write any more
        return 0;

    //HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
    //res = HAL_FLASH_ISSUE_CMD(hflash,SPI_FLASH_CMD_PRSCUR, addr);
    addr = addr << hflash->dualFlash;
    if (hflash->dma != NULL)
    {
        // add pre command process to make FLASH as write mode , to avoid prev read error.
        HAL_FLASH_PRE_CMD(hflash, SPI_FLASH_CMD_PRSCUR);
        ret = HAL_FLASH_DMA_START(hflash, (char *)buf, 1, size);
        if (ret != HAL_OK)
        {
            return 0;
        }

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);

#ifdef QSPI_USE_CMD2
        dlen = 1;
        param = 0x01;
        dlen = dlen << hflash->dualFlash;
        if (hflash->dualFlash)
            param = param | (param << 8);
        HAL_FLASH_WRITE_DLEN2(hflash, dlen);
        res = HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_PRSCUR, addr, SPI_FLASH_CMD_RDSR, param);
        if (res != 0)
            return 0;
#else
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_PRSCUR, addr);

        ret = HAL_FLASH_DMA_WAIT_DONE(hflash, 1000);
        if (ret != HAL_OK)
        {
            return 0;
        }

        // make sure program done
        HAL_FLASH_WRITE_DLEN(hflash, 2);    // cover data len
        do
        {
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
        //rt_hw_interrupt_enable(level);
#endif
        // clear DMAE
        hflash->Instance->CR &= ~MPI_CR_DMAE;
    }

    return size;
}

__HAL_ROM_USED int HAL_FLASH_READ_SFDP(FLASH_HandleTypeDef *hflash, uint32_t *buf, uint32_t start, uint32_t len)
{
    int res, i;
    uint32_t *ptr = (uint32_t *)buf;

    if (hflash == NULL || len % 4 != 0)
        return 0;

    HAL_FLASH_WRITE_DLEN(hflash, len);
    res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSFDP, start);
    if (res != 0)
        return 0;

    for (i = 0; i < len / 4; i++)
    {
        *ptr = HAL_FLASH_READ32(hflash);
        ptr++;
    }

    return len;
}

extern HAL_StatusTypeDef HAL_FLASH_CONFIG_FULL_AHB_READ(FLASH_HandleTypeDef *hflash, bool qmode);
__HAL_ROM_USED HAL_StatusTypeDef HAL_NOR_CFG_DTR(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (en)
    {
        if (hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].cmd == 0)  // not support
            return HAL_ERROR;
        if (hflash->Mode == 0) // only 4 line need set dtr
            return HAL_ERROR;

        // only 4 line need set dtr
        HAL_FLASH_CFG_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].data_mode,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].dummy_cycle, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].ab_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].ab_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].addr_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].addr_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].ins_mode);
        HAL_FLASH_SET_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_DTR4R].cmd);

        HAL_MPI_CFG_DTR(hflash, true, hflash->ecc_en);
    }
    else
    {
        HAL_MPI_CFG_DTR(hflash, false, 0);
        if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
                || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        {
            if (hflash->Mode == 0)  // single line
                HAL_FLASH_CONFIG_FULL_AHB_READ(hflash, 0);
            else // four line
                HAL_FLASH_CONFIG_FULL_AHB_READ(hflash, 1);
        }
        else
        {
            if (hflash->Mode == 0)  // single line
                HAL_FLASH_CONFIG_AHB_READ(hflash, 0);
            else // four line
                HAL_FLASH_CONFIG_AHB_READ(hflash, 1);
        }
    }

    return HAL_OK;
}

__HAL_ROM_USED int HAL_FLASH_NOP_CMD(FLASH_HandleTypeDef *handle)
{
    if (handle == NULL)
        return 1;

    HAL_FLASH_MANUAL_CMD(handle, 0, 0, 0, 0, 0, 0, 0, 1);

    HAL_FLASH_SET_CMD(handle, 0, 0);

    return 0;
}


#if defined(SF32LB56X) || defined(SF32LB52X)

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_FORCE_CONTINUE(FLASH_HandleTypeDef *handle, uint32_t *table, uint32_t length)
{
    if ((handle == NULL) || (table == NULL) || (length <= 0))
        return HAL_ERROR;

    if (handle->ctable == NULL)
        return HAL_ERROR;

    // get first member
    uint32_t addr = *table;

    // enalbe hardware interface
    __HAL_QSPI_EN_HWI(handle);

    // configure CMD1 and CMD2 sequence and loop counter if needed
    if (handle->isNand) // nand command
    {
        SPI_FLASH_CMD_E cmd1 = SPI_FLASH_CMD_PREAD;
        HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[cmd1].func_mode, handle->ctable->cmd_cfg[cmd1].data_mode,
                             handle->ctable->cmd_cfg[cmd1].dummy_cycle, handle->ctable->cmd_cfg[cmd1].ab_size,
                             handle->ctable->cmd_cfg[cmd1].ab_mode, handle->ctable->cmd_cfg[cmd1].addr_size,
                             handle->ctable->cmd_cfg[cmd1].addr_mode, handle->ctable->cmd_cfg[cmd1].ins_mode);
        HAL_FLASH_CFG_CMD(handle, handle->ctable->cmd_cfg[cmd1].cmd, addr, 0);

        HAL_FLASH_ENABLE_CMD2(handle, 1);
        HAL_FLASH_SET_LOOP(handle, length - 1);
        HAL_FLASH_SET_INTERVAL(handle, 0x2800, 0);  // 0x2800 about 100us for 100MHz, change it if needed

        SPI_FLASH_CMD_E cmd2 = SPI_FLASH_CMD_4READ; // default use 4 line, changed it if 4 line now work
        HAL_FLASH_WRITE_DLEN2(handle, 2048);
        HAL_FLASH_MANUAL_CMD2(handle, handle->ctable->cmd_cfg[cmd2].func_mode, handle->ctable->cmd_cfg[cmd2].data_mode,
                              handle->ctable->cmd_cfg[cmd2].dummy_cycle, handle->ctable->cmd_cfg[cmd2].ab_size,
                              handle->ctable->cmd_cfg[cmd2].ab_mode, handle->ctable->cmd_cfg[cmd2].addr_size,
                              handle->ctable->cmd_cfg[cmd2].addr_mode, handle->ctable->cmd_cfg[cmd2].ins_mode);
        HAL_FLASH_CFG_CMD(handle, handle->ctable->cmd_cfg[cmd2].cmd, 0, 1);
    }
    else // nor command
    {
        SPI_FLASH_CMD_E cmd1 = SPI_FLASH_CMD_4READ; // default use 4 line, changed it if 4 line now work
        if (handle->size > NOR_FLASH_MAX_3B_SIZE)
            cmd1 = SPI_FLASH_CMD_4RD4BA;
        HAL_FLASH_WRITE_DLEN(handle, 2048);
        HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[cmd1].func_mode, handle->ctable->cmd_cfg[cmd1].data_mode,
                             handle->ctable->cmd_cfg[cmd1].dummy_cycle, handle->ctable->cmd_cfg[cmd1].ab_size,
                             handle->ctable->cmd_cfg[cmd1].ab_mode, handle->ctable->cmd_cfg[cmd1].addr_size,
                             handle->ctable->cmd_cfg[cmd1].addr_mode, handle->ctable->cmd_cfg[cmd1].ins_mode);
        HAL_FLASH_CFG_CMD(handle, handle->ctable->cmd_cfg[cmd1].cmd, addr, 0);

        HAL_FLASH_ENABLE_CMD2(handle, 1);
        HAL_FLASH_SET_LOOP(handle, length - 1);
        HAL_FLASH_SET_INTERVAL(handle, 100, 0);  // 100 about 1us for 100MHz, change it if needed

        // set command as empty command
        HAL_FLASH_MANUAL_CMD2(handle, 1, 0, 0, 0, 0, 0, 0, 1);
        HAL_FLASH_CFG_CMD(handle, 0, 0, 1);
    }
    if (length <= 1) // set once, do not need dma any more
        return HAL_OK;

    // set dma
    if (handle->dma)
    {
        handle->dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        handle->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; //DMA_PDATAALIGN_BYTE; //DMA_PDATAALIGN_WORD;
        handle->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;  //DMA_MDATAALIGN_BYTE; //DMA_MDATAALIGN_WORD;
        handle->dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        handle->dma->Init.MemInc              = DMA_MINC_ENABLE;
        uint32_t dst = (uint32_t)(&handle->Instance->AR1);
        uint32_t src = (uint32_t)(table);
        src += 4; // first member has set to CMD1 AR

        int res = HAL_DMA_DeInit(handle->dma);
        if (res != HAL_OK)
            goto err;

        res = HAL_DMA_Init(handle->dma);
        if (res != HAL_OK)
            goto err;

        res = HAL_DMA_Start(handle->dma, src, dst, length - 1);
        if (res != HAL_OK)
            goto err;

        HAL_FLASH_DMA_EN(handle, 1);
    }

    return HAL_OK;
err:
    // enalbe hardware interface
    __HAL_QSPI_DIS_HWI(handle);

    return HAL_ERROR;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_QUIT_CONTINUE(FLASH_HandleTypeDef *handle)
{
    if (handle == NULL)
        return HAL_ERROR;

    __HAL_QSPI_DIS_HWI(handle);

    HAL_FLASH_ENABLE_CMD2(handle, 0);

    // clear TCF before next mpi command
    HAL_FLASH_CLR_CMD_DONE(handle);

    return HAL_OK;
}

__HAL_ROM_USED uint8_t HAL_NOR_DTR_CAL(FLASH_HandleTypeDef *hflash)
{
    uint8_t dtr_dly;
    if (hflash == NULL)
        return 0;

    hflash->Instance->CALCR |= MPI_CALCR_EN;
    HAL_Delay_us(20);
    while ((hflash->Instance->CALCR & MPI_CALCR_DONE_Msk) == 0)
    {
        // add delay?
    }
    dtr_dly = (uint8_t)((hflash->Instance->CALCR & MPI_CALCR_DELAY_Msk) >> MPI_CALCR_DELAY_Pos);
    hflash->Instance->CALCR &= ~MPI_CALCR_EN;

    hflash->ecc_en &= (~0X7F) ; //MPI_CALCR_DELAY_Msk   // only 7 bits work
    hflash->ecc_en |= dtr_dly;

    return dtr_dly;
}

#endif

#endif  // HAL_QSPI_MODULE_ENABLED

/// @} FLASH

/// @} BF0_HAL_Driver


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
