/**
  ******************************************************************************
  * @file   uart_config.h
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

#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__


#ifdef __cplusplus
extern "C" {
#endif

/* NOTE: old version g++ doesn't support out-of-order initialization, all members should be in the same order as declaration
 *       https://stackoverflow.com/questions/31215971/non-trivial-designated-initializers-not-supported/46067271#46067271
 *       https://en.cppreference.com/w/cpp/language/aggregate_initialization
 */

/**********************UART1***************************/
#define UART1_CONFIG                                                \
    {                                                               \
        .Instance = USART1,                                         \
        .irq_type = USART1_IRQn,                                    \
        .dma_rx = UART1_RX_DMA_CONFIG,                              \
    }

#define UART1_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART1_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART1_RX_DMA_RCC,                               \
        .dma_irq  = UART1_RX_DMA_IRQ,                               \
        .request  = UART1_RX_DMA_REQUEST,                           \
    }

#define UART1_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART1_TX_DMA_INSTANCE,                          \
        .dma_rcc  = UART1_TX_DMA_RCC,                               \
        .dma_irq  = UART1_TX_DMA_IRQ,                               \
        .request  = UART1_TX_DMA_REQUEST,                           \
    }

/**********************UART2***************************/
#define UART2_CONFIG                                                \
    {                                                               \
        .Instance = USART2,                                         \
        .irq_type = USART2_IRQn,                                    \
        .dma_rx = UART2_RX_DMA_CONFIG,                              \
    }

#define UART2_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART2_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART2_RX_DMA_RCC,                               \
        .dma_irq  = UART2_RX_DMA_IRQ,                               \
        .request  = UART2_RX_DMA_REQUEST,                           \
    }

#define UART2_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART2_TX_DMA_INSTANCE,                          \
        .dma_rcc  = UART2_TX_DMA_RCC,                               \
        .dma_irq  = UART2_TX_DMA_IRQ,                               \
        .request  = UART2_TX_DMA_REQUEST,                           \
    }


/**********************UART3***************************/
#define UART3_CONFIG                                                \
    {                                                               \
        .Instance = USART3,                                         \
        .irq_type = USART3_IRQn,                                    \
        .dma_rx = UART3_RX_DMA_CONFIG,                              \
    }

#define UART3_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART3_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART3_RX_DMA_RCC,                               \
        .dma_irq  = UART3_RX_DMA_IRQ,                               \
        .request  = UART3_RX_DMA_REQUEST,                           \
    }

#define UART3_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART3_TX_DMA_INSTANCE,                          \
        .dma_rcc  = UART3_TX_DMA_RCC,                               \
        .dma_irq  = UART3_TX_DMA_IRQ,                               \
        .request  = UART3_TX_DMA_REQUEST,                           \
    }

/**********************UART4***************************/
#define UART4_CONFIG                                                \
    {                                                               \
        .Instance = USART4,                                         \
        .irq_type = USART4_IRQn,                                    \
        .dma_rx = UART4_RX_DMA_CONFIG,                              \
    }

#define UART4_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART4_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART4_RX_DMA_RCC,                               \
        .dma_irq  = UART4_RX_DMA_IRQ,                               \
        .request  = UART4_RX_DMA_REQUEST,                           \
    }

#define UART4_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART4_TX_DMA_INSTANCE,                          \
        .dma_rcc  = UART4_TX_DMA_RCC,                               \
        .dma_irq  = UART4_TX_DMA_IRQ,                               \
        .request  = UART4_TX_DMA_REQUEST,                           \
    }

/**********************UART5***************************/
#define UART5_CONFIG                                                \
    {                                                               \
        .Instance = USART5,                                         \
        .irq_type = USART5_IRQn,                                    \
        .dma_rx = UART5_RX_DMA_CONFIG,                              \
    }

#define UART5_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART5_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART5_RX_DMA_RCC,                               \
        .dma_irq  = UART5_RX_DMA_IRQ,                               \
        .request  = UART5_RX_DMA_REQUEST,                           \
    }

#define UART5_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART5_TX_DMA_INSTANCE,                          \
        .dma_rcc  = UART5_TX_DMA_RCC,                               \
        .dma_irq  = UART5_TX_DMA_IRQ,                               \
        .request  = UART5_TX_DMA_REQUEST,                           \
    }

#ifdef __cplusplus
}
#endif

#endif /* __UART_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
