/**
  ******************************************************************************
  * @file   sdio_config.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
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

#ifndef __SDIO_CONFIG_H__
#define __SDIO_CONFIG_H__

#include <rtconfig.h>
#include "bf0_hal.h"
#include "dma_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// FIX ME: With real HW setting.
#ifdef BSP_USING_SD_LINE
#ifdef SDMMC1_DMA_INSTANCE
#define SDIO_BUS_CONFIG                                  \
    {                                                    \
        .Instance = SDIO1,                                \
        .dma_rx.dma_rcc = SDMMC1_DMA_RCC,            \
        .dma_tx.dma_rcc = SDMMC1_DMA_RCC,            \
        .dma_rx.Instance = SDMMC1_DMA_INSTANCE,                 \
        .dma_rx.request = SDMMC1_DMA_REQUEST,                 \
        .dma_rx.dma_irq = SDMMC1_DMA_IRQ,             \
        .dma_tx.Instance = SDMMC1_DMA_INSTANCE,                 \
        .dma_tx.request = SDMMC1_DMA_REQUEST,                 \
        .dma_tx.dma_irq = SDMMC1_DMA_IRQ,             \
    }

#else
#define SDIO_BUS_CONFIG                                  \
    {                                                    \
        .Instance = SDIO1,                                \
        .dma_rx.dma_rcc = 0,            \
        .dma_tx.dma_rcc = 0,            \
        .dma_rx.Instance = 0,                 \
        .dma_rx.request = 0,                 \
        .dma_rx.dma_irq = 0,             \
        .dma_tx.Instance = 0,                 \
        .dma_tx.request = 0,                 \
        .dma_tx.dma_irq = 0,             \
    }

#endif  //SDMMC2_DMA_INSTANCE
#endif //BSP_USING_SD_LINE

#ifdef __cplusplus
}
#endif

#endif /*__SDIO_CONFIG_H__ */



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
