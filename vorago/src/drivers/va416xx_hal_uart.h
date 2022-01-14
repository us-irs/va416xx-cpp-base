/***************************************************************************************
 * @file     va416xx_hal_uart.h
 * @version  V0.4.1
 * @date     06 February 2019
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2020 VORAGO Technologies.
 *
 * @par
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND BY
 * ALL THE TERMS AND CONDITIONS OF THE VORAGO TECHNOLOGIES END USER LICENSE AGREEMENT.
 * THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VORAGO TECHNOLOGIES
 * SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************************/

#ifndef __HAL_UART_H
#define __HAL_UART_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "event_handler_index.h"
#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define HAL_UART_VERSION (0x00000401)  // 0.4.1

#define UART_CALC_CLOCKSCALE(_scc, _baud)            \
  ((_scc / (_baud * 16)) << UART_CLKSCALE_INT_Pos) | \
      (((((_scc % (_baud * 16)) * 64 + (_baud * 8)) / (_baud * 16))) << UART_CLKSCALE_FRAC_Pos)

#define UART0_BANK (0)
#define UART1_BANK (1)
#define UART2_BANK (2)

/** Common UART config macros */
#define UART_CFG_230K_8N1                                                                    \
  (stc_uart_cfg_t) {                                                                         \
    .baudRate = 230400, .irq = en_uart_irq_rx, .wordSize = 8, .parity = en_uart_parity_none, \
    .stopBits = 1, false                                                                     \
  }

#define UART_CFG_115K_8N1                                                                    \
  (stc_uart_cfg_t) {                                                                         \
    .baudRate = 115200, .irq = en_uart_irq_rx, .wordSize = 8, .parity = en_uart_parity_none, \
    .stopBits = 1, false                                                                     \
  }

#define UART_CFG_9600_8N1                                                                  \
  (stc_uart_cfg_t) {                                                                       \
    .baudRate = 9600, .irq = en_uart_irq_rx, .wordSize = 8, .parity = en_uart_parity_none, \
    .stopBits = 1, false                                                                   \
  }

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/** UART parity setting */
typedef enum { en_uart_parity_none = 0, en_uart_parity_even, en_uart_parity_odd } en_uart_parity_t;

/** UART flow control setting */
typedef enum {
  en_uart_flowctrl_none = 0,
  en_uart_flowctrl_autoCtsRts,
} en_uart_flowctrl_t;

/** UART interrupt setting - these can be ORed together */
typedef enum {
  en_uart_irq_none = 0x0,       // Polling, must periodically check RX fifo
                                // and tx will block until fully transmitted
  en_uart_irq_rx = 0x1,         // Irq when RX FIFO count >= RXFIFOIRQTRG
  en_uart_irq_rxStatus = 0x2,   // Receiver irq enable for status conditions
  en_uart_irq_rxTo = 0x4,       // Receiver irq enable for timeout conditions
  en_uart_irq_rxAddr9 = 0x8,    // Receiver irq enable for matched address in 9-bit mode
  en_uart_irq_tx = 0x10,        // Irq when TX FIFO count < TXFIFOIRQTRG
  en_uart_irq_txStatus = 0x20,  // Transmitter irq enable for status conditions (WRLOST)
  en_uart_irq_txEmpty = 0x40,   // Generates an irq when tx FIFO is empty and TXBUSY is 0
  en_uart_irq_txCts = 0x80      // Transmitter irq enable when CTSn changes value
} en_uart_irq_t;

/** UART init settings structure */
typedef struct {
  uint32_t baudRate;            // data rate
  en_uart_parity_t parity;      // parity type (none, even, odd)
  en_uart_flowctrl_t flowctrl;  // flow control (none or use CTS/RTS)
  en_uart_irq_t irq;            // interrupt settings (multiple flags can be ORed)
  uint8_t wordSize;             // data bits: 5 through 8 bits
  uint8_t stopBits;             // 1 or 2 stop bits
  bool loopback;                // loopback enable

} stc_uart_cfg_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern hal_status_t HAL_Uart_Init(VOR_UART_Type* const uart, const stc_uart_cfg_t initCfg);
extern hal_status_t HAL_Uart_DeInit(VOR_UART_Type* const uart);  // disables uart to save power
extern hal_status_t HAL_Uart_TxByte(VOR_UART_Type* const uart, const uint8_t txByte);
extern hal_status_t HAL_Uart_TxStr(VOR_UART_Type* const uart, const char* pStr);
extern hal_status_t HAL_Uart_TxBreak(VOR_UART_Type* const uart, const uint32_t breakCount);
extern hal_status_t HAL_Uart_RxByte(VOR_UART_Type* const uart, uint8_t* const pRxByte);
extern hal_status_t HAL_Uart_GetRxLen(VOR_UART_Type* const uart, uint32_t* const pRxLen);
extern hal_status_t HAL_Uart_RxHndlr(
    void);  // RX fifo handler, polling (for all initialized uarts w/o interrupt)

extern hal_status_t HAL_Uart_SetTXIntCallback(VOR_UART_Type* const uart, eventHandlerIdx_t idx);
extern hal_status_t HAL_Uart_SetRXIntCallback(VOR_UART_Type* const uart, eventHandlerIdx_t idx);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_UART_H */
