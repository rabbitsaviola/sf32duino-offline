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

#pragma once

#include "api/HardwareSerial.h"
#include "api/RingBuffer.h"
#include "register.h"
// #include <cstddef>

// typedef struct dma_config
// {
//     DMA_Channel_TypeDef *Instance;    /*!< DMA HW low layer handle */
//     uint32_t dma_rcc;            /*!< DMA RCC handle */
//     IRQn_Type dma_irq;              /*!< DMA interrupt number */
//     uint32_t request;            /*!< DMA request number */
// } dma_config_t;


#define UART_RX_BUF_SIZE   (512)

typedef struct
{
    USART_TypeDef      *Instance;
    IRQn_Type           irq_type;
    struct dma_config   dma_rx;
    // struct dma_config   dma_tx;
} uart_config_t;

class Uart;

typedef struct uart_obj_tag
{
    UART_HandleTypeDef handle;
    DMA_HandleTypeDef rx_dma_handle;
    size_t last_rx_pos;
    const uart_config_t *uart_config;
    Uart *serial;
} uart_obj_t;

typedef void (*UartReceiveCallback)(arduino::HardwareSerial *serial);

class Uart : public arduino::HardwareSerial
{
    public:
        Uart(uint8_t _inst, int16_t _pinRX, int16_t _pinTX);
        void begin(unsigned long baudRate);
        void begin(unsigned long baudrate, uint16_t config);
        void end();
        int available();
        int peek();
        int read();
        void flush();
        size_t write(const uint8_t data);
        size_t write(const char * data);

        operator bool() { return true; }

        void irqHandler();
        void rxDmaIrqHandler();
        void onReceive(UartReceiveCallback cb);
        void handleEvent(int event);
        DMA_HandleTypeDef *getRxDmaHandle(void) {return &uart_obj.rx_dma_handle;}

    private:
        arduino::RingBufferN<UART_RX_BUF_SIZE> rxBuffer;
        uart_obj_t uart_obj;
        UartReceiveCallback rx_cb;
        void _setConfig(unsigned long baudrate, uint16_t config);


    int16_t pinRX;
    int16_t pinTX;
    uint8_t instance;    

};

