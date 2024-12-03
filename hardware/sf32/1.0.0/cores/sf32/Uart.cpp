/*
  Copyright (c) 2016 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// #include "register.h"
#include "Uart.h"
#include "dma_config.h"
#include "uart_config.h"

#define UART_INSTANCE_CLEAR_FUNCTION    __HAL_UART_CLEAR_FLAG

#define UART_EVENT_RX_IND          0x01    /* Rx indication */
#define UART_EVENT_TX_DONE         0x02    /* Tx complete   */
#define UART_EVENT_RX_DMADONE      0x03    /* Rx DMA transfer done */
#define UART_EVENT_TX_DMADONE      0x04    /* Tx DMA transfer done */
#define UART_EVENT_RX_TIMEOUT      0x05    /* Rx timeout    */


// //tx and rx buffer for EasyDMA
// static uint8_t tx_buffer[1];
// static uint8_t rx_buffer[1];

enum
{
#ifdef USART1_BASE
    UART1_INDEX,
#endif /* USART1_BASE */
#ifdef USART2_BASE
    UART2_INDEX,
#endif /* USART2_BASE */	
#ifdef USART3_BASE	
    UART3_INDEX,
#endif /* USART3_BASE */
#ifdef USART4_BASE
    UART4_INDEX,
#endif /* USART4_BASE */
#ifdef USART5_BASE
    UART5_INDEX,
#endif /* USART5_BASE */
#ifdef USART6_BASE
    UART6_INDEX,
#endif /* USART6_BASE */
};


#ifdef __cplusplus
extern "C" {
#endif
static const uart_config_t _uart_config[] =
{
#ifdef USART1_BASE
    UART1_CONFIG,
#endif
#ifdef USART2_BASE
    UART2_CONFIG,
#endif
#ifdef USART3_BASE
    UART3_CONFIG,
#endif
#ifdef USART4_BASE
    UART4_CONFIG,
#endif
#ifdef USART5_BASE
    UART5_CONFIG,
#endif
#ifdef USART6_BASE
    UART6_CONFIG,
#endif
};

#ifdef __cplusplus
}
#endif

static Uart *serial[sizeof(_uart_config) / sizeof(_uart_config[0])];

Uart::Uart(uint8_t _inst, int16_t _pinRX, int16_t _pinTX)
{
	pinRX = _pinRX;
	pinTX = _pinTX;
	instance = _inst;
    uart_obj.uart_config = &_uart_config[_inst];
    uart_obj.serial = this;
    //TODO: 
    serial[_inst] = this;
}

void Uart::begin(unsigned long baudrate)
{
	begin(baudrate, SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint16_t config)
{
    IRQn_Type irq;

    _setConfig(baudrate, config);

    if (HAL_UART_Init(&uart_obj.handle) != HAL_OK)
    {
        return;
    }

    

    __HAL_LINKDMA(&uart_obj.handle, hdmarx, uart_obj.rx_dma_handle);
    uart_obj.rx_dma_handle.Instance = uart_obj.uart_config->dma_rx.Instance;
    uart_obj.rx_dma_handle.Init.Request = uart_obj.uart_config->dma_rx.request;
    irq = uart_obj.uart_config->dma_rx.dma_irq;

    HAL_UART_DmaTransmit(&uart_obj.handle, rxBuffer._aucBuffer, 512, DMA_PERIPH_TO_MEMORY);

    HAL_NVIC_SetPriority(irq, 0, 0);
    HAL_NVIC_EnableIRQ(irq);

    __HAL_UART_ENABLE_IT(&uart_obj.handle, UART_IT_IDLE);
    HAL_NVIC_SetPriority(uart_obj.uart_config->irq_type, 1, 0);
    HAL_NVIC_EnableIRQ(uart_obj.uart_config->irq_type);

}

void Uart::end()
{
	rxBuffer.clear();
}

void Uart::flush()
{
	rxBuffer.clear();
}

void Uart::onReceive(UartReceiveCallback cb)
{
    rx_cb = cb;
}

void Uart::handleEvent(int event)
{
    switch (event & 0xff)
    {
#if 0
    case UART_EVENT_RX_IND:
    {
        int ch = -1;
        uint32_t level;
        struct rt_serial_rx_fifo *rx_fifo;

        /* interrupt mode receive */
        rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
        RT_ASSERT(rx_fifo != RT_NULL);

        while (1)
        {
            ch = serial->ops->getc(serial);
            if (ch == -1) break;


            /* disable interrupt */
            level = HAL_DisableInterrupt();

            rxBuffer.store_char()

            rx_fifo->buffer[rx_fifo->put_index] = ch;
            rx_fifo->put_index += 1;
            if (rx_fifo->put_index >= serial->config.bufsz) rx_fifo->put_index = 0;

            /* if the next position is read index, discard this 'read char' */
            if (rx_fifo->put_index == rx_fifo->get_index)
            {
                rx_fifo->get_index += 1;
                rx_fifo->is_full = RT_TRUE;
                if (rx_fifo->get_index >= serial->config.bufsz) rx_fifo->get_index = 0;
            }

            /* enable interrupt */
            HAL_EnableInterrupt(level);
        }

        /* invoke callback */
        if (serial->parent.rx_indicate != RT_NULL)
        {
            rt_size_t rx_length;

            /* get rx length */
            level = HAL_DisableInterrupt();
            rx_length = (rx_fifo->put_index >= rx_fifo->get_index) ? (rx_fifo->put_index - rx_fifo->get_index) :
                        (serial->config.bufsz - (rx_fifo->get_index - rx_fifo->put_index));
            HAL_EnableInterrupt(level);

            if (rx_length)
            {
                serial->parent.rx_indicate(&serial->parent, rx_length);
            }
        }
        break;
    }
    
    case UART_EVENT_TX_DONE:
    {
        struct rt_serial_tx_fifo *tx_fifo;

        tx_fifo = (struct rt_serial_tx_fifo *)serial->serial_tx;
        rt_completion_done(&(tx_fifo->completion));
        break;
    }
#endif
#if 0
    case UART_EVENT_TX_DMADONE:
    {
        const void *data_ptr;
        rt_size_t data_size;
        const void *last_data_ptr;
        struct rt_serial_tx_dma *tx_dma;

        tx_dma = (struct rt_serial_tx_dma *) serial->serial_tx;

        rt_data_queue_pop(&(tx_dma->data_queue), &last_data_ptr, &data_size, 0);
        if (rt_data_queue_peak(&(tx_dma->data_queue), &data_ptr, &data_size) == RT_EOK)
        {
            /* transmit next data node */
            tx_dma->activated = RT_TRUE;
            serial->ops->dma_transmit(serial, (rt_uint8_t *)data_ptr, data_size, UART_DMA_TX);
        }
        else
        {
            tx_dma->activated = RT_FALSE;
        }

        /* invoke callback */
        if (serial->parent.tx_complete != RT_NULL)
        {
            serial->parent.tx_complete(&serial->parent, (void *)last_data_ptr);
        }
        break;
    }
#endif    
    case UART_EVENT_RX_DMADONE:
    {
        int length;
        uint32_t level;

        /* get DMA rx length */
        length = (event & (~0xff)) >> 8;

        /* disable interrupt */
        level = HAL_DisableInterrupt();
        /* update fifo put index */
        if ((rxBuffer._numElems + length) > UART_RX_BUF_SIZE)
        {
            length = UART_RX_BUF_SIZE - rxBuffer._numElems;
        }
        rxBuffer._numElems += length;
        rxBuffer._iTail += length;
        if (rxBuffer._iTail >= UART_RX_BUF_SIZE)
        {
            rxBuffer._iTail -= UART_RX_BUF_SIZE;
        }
        /* enable interrupt */
        HAL_EnableInterrupt(level);
        /* invoke callback */
        if (rx_cb)
        {
            rx_cb(this);
        }
        break;
    }
    }
}

void Uart::irqHandler()
{
    size_t recv_total_index, recv_len;
    uint32_t level;

    /* UART in mode Receiver -------------------------------------------------*/
#if 0 //always use dma for rx   
    if ((__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart_obj.andle, UART_IT_RXNE) != RESET))
    {
        handleEvent(UART_EVENT_RX_IND);
    }
    else 
#endif    
    if ((__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_IDLE) != RESET) &&
             (__HAL_UART_GET_IT_SOURCE(&uart_obj.handle, UART_IT_IDLE) != RESET))
    {
        level = HAL_DisableInterrupt();
        recv_total_index = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&uart_obj.rx_dma_handle);
        recv_len = recv_total_index - uart_obj.last_rx_pos;
        uart_obj.last_rx_pos = recv_total_index;
        HAL_EnableInterrupt(level);

        if (recv_len)
        {
            handleEvent(UART_EVENT_RX_DMADONE | (recv_len << 8));
        }
        __HAL_UART_CLEAR_IDLEFLAG(&uart_obj.handle);
    }
#if 0  //not support
    else if ((uart->uart_tx_dma_flag)
             && (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_TC) != RESET)
             && (__HAL_UART_GET_IT_SOURCE(&uart_obj.handle, UART_IT_TC) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&uart->handle, UART_CLEAR_TCF);
        uart->handle.gState = HAL_UART_STATE_READY;
        handleEvent(UART_EVENT_TX_DMADONE);
    }
#endif    
    else
    {
        if (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(&uart_obj.handle);
        }
        if (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_NE) != RESET)
        {
            __HAL_UART_CLEAR_NEFLAG(&uart_obj.handle);
        }
        if (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_FE) != RESET)
        {
            __HAL_UART_CLEAR_FEFLAG(&uart_obj.handle);
        }
        if (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_PE) != RESET)
        {
            __HAL_UART_CLEAR_PEFLAG(&uart_obj.handle);
        }
        if (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_CTS) != RESET)
        {
            UART_INSTANCE_CLEAR_FUNCTION(&uart_obj.handle, UART_FLAG_CTS);
        }
        if (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_TXE) != RESET)
        {
            UART_INSTANCE_CLEAR_FUNCTION(&uart_obj.handle, UART_FLAG_TXE);
        }
        if (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_TC) != RESET)
        {
            UART_INSTANCE_CLEAR_FUNCTION(&uart_obj.handle, UART_FLAG_TC);
        }
        if (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_RXNE) != RESET)
        {
            UART_INSTANCE_CLEAR_FUNCTION(&uart_obj.handle, UART_FLAG_RXNE);
        }
    }
}

void Uart::rxDmaIrqHandler()
{
    HAL_DMA_IRQHandler(&uart_obj.rx_dma_handle);    
}

int Uart::available()
{
	return rxBuffer.available();
}

int Uart::peek()
{
	return rxBuffer.peek();
}

int Uart::read()
{
	return rxBuffer.read_char();
}

size_t Uart::write(const uint8_t data)
{
    uint32_t level;

    HAL_UART_LOCK_DEF();

    HAL_UART_LOCK(&uart_obj.handle);

    level = HAL_DisableInterrupt();
    UART_INSTANCE_CLEAR_FUNCTION(&uart_obj.handle, UART_FLAG_TC);
    __HAL_UART_PUTC(&uart_obj.handle, data);
    while (__HAL_UART_GET_FLAG(&uart_obj.handle, UART_FLAG_TC) == RESET);
    HAL_EnableInterrupt(level);

    HAL_UART_UNLOCK(&uart_obj.handle);

	return 1;
}


size_t Uart::write(const char * data)
{

	size_t writed = 0;
	while(*data != '\0')
	{
		writed += Uart::write(*data);
		++data;
	}
	
	return writed;
}

void Uart::_setConfig(unsigned long baudrate, uint16_t config)
{
    bool is_parity;

    uart_obj.handle.Instance          = uart_obj.uart_config->Instance;
    uart_obj.handle.Init.BaudRate     = baudrate;
    uart_obj.handle.Init.HwFlowCtl    = 0;
    uart_obj.handle.Init.Mode         = UART_MODE_TX_RX;
    uart_obj.handle.Init.OverSampling = UART_OVERSAMPLING_16;

    if (SERIAL_PARITY_NONE != (config & SERIAL_PARITY_MASK))
    {
        is_parity = true;
    }
    else
    {
        is_parity = false;
    }

    switch (config & SERIAL_DATA_MASK)
    {
    case SERIAL_DATA_6:
        if (is_parity)
        {
            uart_obj.handle.Init.WordLength = UART_WORDLENGTH_7B;
        }
        else
        {
            uart_obj.handle.Init.WordLength = UART_WORDLENGTH_6B;
        }
        break;
    case SERIAL_DATA_7:
        if (is_parity)
        {
            uart_obj.handle.Init.WordLength = UART_WORDLENGTH_8B;
        }
        else
        {
            uart_obj.handle.Init.WordLength = UART_WORDLENGTH_7B;
        }
        break;
    case SERIAL_DATA_8:
        if (is_parity)
        {
            uart_obj.handle.Init.WordLength = UART_WORDLENGTH_9B;
        }
        else
        {
            uart_obj.handle.Init.WordLength = UART_WORDLENGTH_8B;
        }
        break;
    default:
        if (is_parity)
        {
            uart_obj.handle.Init.WordLength = UART_WORDLENGTH_9B;
        }
        else
        {
            uart_obj.handle.Init.WordLength = UART_WORDLENGTH_8B;
        }
        break;
    }

    switch (config & SERIAL_STOP_BIT_MASK)
    {
    case SERIAL_STOP_BIT_1:
        uart_obj.handle.Init.StopBits   = UART_STOPBITS_1;
        break;
    case SERIAL_STOP_BIT_1_5:
        uart_obj.handle.Init.StopBits   = UART_STOPBITS_1_5;
        break;
    case SERIAL_STOP_BIT_2:
        uart_obj.handle.Init.StopBits   = UART_STOPBITS_2;
        break;
    default:
        uart_obj.handle.Init.StopBits   = UART_STOPBITS_1;
        break;
    }

    switch (config & SERIAL_PARITY_MASK)
    {
    case SERIAL_PARITY_NONE:
        uart_obj.handle.Init.Parity     = UART_PARITY_NONE;
        break;
    case SERIAL_PARITY_ODD:
        uart_obj.handle.Init.Parity     = UART_PARITY_ODD;
        break;
    case SERIAL_PARITY_EVEN:
        uart_obj.handle.Init.Parity     = UART_PARITY_EVEN;
        break;
    default:
        uart_obj.handle.Init.Parity     = UART_PARITY_NONE;
        break;
    }

    return;
}

#ifdef __cplusplus
extern "C" {
#endif


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart_handle)
{
    size_t recv_len;
    size_t recv_total_index;
    uint32_t level;
    uart_obj_t *uart;

    uart = (uart_obj_t *)uart_handle;

    level = HAL_DisableInterrupt();
    recv_total_index = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&uart->rx_dma_handle);
    recv_len = recv_total_index - uart->last_rx_pos;
    uart->last_rx_pos = recv_total_index;
    HAL_EnableInterrupt(level);

    if (recv_len)
    {
        uart->serial->handleEvent(UART_EVENT_RX_DMADONE | (recv_len << 8));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart_handle)
{
    HAL_UART_RxHalfCpltCallback(uart_handle);
}

#ifdef USART1_BASE
void USART1_IRQHandler(){
	serial[0]->irqHandler();
}

void UART1_DMA_RX_IRQHandler(void)
{
    serial[0]->rxDmaIrqHandler();
}

#endif /* USART1_BASE */

#ifdef USART2_BASE
void USART2_IRQHandler(){
	serial[1]->irqHandler();
}

void UART2_DMA_RX_IRQHandler(void)
{
    serial[1]->rxDmaIrqHandler();
}
#endif /* USART2_BASE */

#ifdef USART3_BASE
void USART3_IRQHandler(){
	serial[2]->irqHandler();
}

void UART3_DMA_RX_IRQHandler(void)
{
    serial[2]->rxDmaIrqHandler();
}

#endif /* USART3_BASE */

#ifdef USART4_BASE
void USART4_IRQHandler(){
	serial[3]->irqHandler();
}

void UART4_DMA_RX_IRQHandler(void)
{
    serial[3]->rxDmaIrqHandler();
}

#endif /* USART4_BASE */


#ifdef USART5_BASE
void USART5_IRQHandler(){
	serial[4]->irqHandler();
}

void UART5_DMA_RX_IRQHandler(void)
{
    serial[4]->rxDmaIrqHandler();
}

#endif /* USART5_BASE */

#ifdef USART6_BASE
void USART5_IRQHandler(){
	serial[5]->irqHandler();
}

void UART6_DMA_RX_IRQHandler(void)
{
    serial[5]->rxDmaIrqHandler();
}

#endif /* USART6_BASE */

#ifdef __cplusplus
}
#endif
