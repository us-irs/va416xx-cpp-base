/***************************************************************************************
 * @file     va416xx_hal_debug.c
 * @version  V0.1
 * @date     13 February 2019
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2019 VORAGO Technologies.
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

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_debug.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "va416xx.h"

#ifndef DEBUG_NO_HAL
#include "va416xx_hal_uart.h"
#endif

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

//#define DEBUG_BUFFER_LEN (128)

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

// static char debugBuf[DEBUG_BUFFER_LEN];

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static en_stdio_t ioOut = en_stdio_none;
#ifdef ENABLE_RTT
static uint8_t log_buff[100];
#endif

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

void VOR_printf(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);

  if (ioOut == en_stdio_rtt) {
#ifdef ENABLE_RTT
    vsnprintf((char *)(log_buff), 100, fmt, args);
    SEGGER_RTT_WriteString(0, (const char *)log_buff);
#endif
  } else {
    vfprintf(stdout, fmt, args);
    fflush(stdout);
  }

  va_end(args);
}

void DBG_printf(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);

  if (ioOut == en_stdio_rtt) {
#ifdef ENABLE_RTT
    vsnprintf((char *)(log_buff), 100, fmt, args);
    SEGGER_RTT_WriteString(0, "DEBUG: ");
    SEGGER_RTT_WriteString(0, (const char *)log_buff);
#endif
  } else {
    fprintf(stderr, "DEBUG: ");
    vfprintf(stderr, fmt, args);
    fflush(stderr);
  }

  va_end(args);
}

void DBG_println(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);

  if (ioOut == en_stdio_rtt) {
#ifdef ENABLE_RTT
    vsnprintf((char *)(log_buff), 100, fmt, args);
    SEGGER_RTT_WriteString(0, "DEBUG: ");
    SEGGER_RTT_WriteString(0, (const char *)log_buff);
    SEGGER_RTT_WriteString(0, "\r\n");
#endif
  } else {
    fprintf(stderr, "DEBUG: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\r\n");
    fflush(stderr);
  }

  va_end(args);
}

void DBG_SetStdioOutput(en_stdio_t io) {
  ioOut = io;

  switch (io) {
    case en_stdio_porta:
      // Configure BANK0 as outputs
      VOR_GPIO->BANK[0].DIR |= 0x000000FFU;
      // Configure BANK0 bit 7 as pulse mode
      VOR_GPIO->BANK[0].PULSE |= 0x00000080U;
      break;
    case en_stdio_portb:
      // Configure BANK1 as outputs
      VOR_GPIO->BANK[1].DIR |= 0x000000FFU;
      // Configure BANK1 bit 7 as pulse mode
      VOR_GPIO->BANK[1].PULSE |= 0x00000080U;
      break;
    case en_stdio_portc:
      // Configure BANK2 as outputs
      VOR_GPIO->BANK[2].DIR |= 0x000000FFU;
      // Configure BANK2 bit 7 as pulse mode
      VOR_GPIO->BANK[2].PULSE |= 0x00000080U;
      break;
    case en_stdio_portd:
      // Configure BANK3 as outputs
      VOR_GPIO->BANK[3].DIR |= 0x000000FFU;
      // Configure BANK3 bit 7 as pulse mode
      VOR_GPIO->BANK[3].PULSE |= 0x00000080U;
      break;
    case en_stdio_rtt:
#ifdef ENABLE_RTT
      SEGGER_RTT_Init();
      SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#endif
      break;
    default:
      break;
  }
}

int fputc(int c, FILE *fPointer) {
  uint32_t dm;

  switch (ioOut) {
    case en_stdio_none:
      break;
    case en_stdio_uart0:
#ifndef DEBUG_NO_HAL
      HAL_Uart_TxByte(VOR_UART0, (uint8_t)c);
#endif
      break;
    case en_stdio_uart1:
#ifndef DEBUG_NO_HAL
      HAL_Uart_TxByte(VOR_UART1, (uint8_t)c);
#endif
      break;
    case en_stdio_uart2:
#ifndef DEBUG_NO_HAL
      HAL_Uart_TxByte(VOR_UART2, (uint8_t)c);
#endif
      break;
    case en_stdio_porta:
      dm = VOR_GPIO->BANK[0].DATAMASK;
      VOR_GPIO->BANK[0].DATAMASK = 0xff;     // Write DataMask
      VOR_GPIO->BANK[0].DATAOUT = 0x80 | c;  // Pulse bit 7
      VOR_GPIO->BANK[0].DATAMASK = dm;
      break;
    case en_stdio_portb:
      dm = VOR_GPIO->BANK[1].DATAMASK;
      VOR_GPIO->BANK[1].DATAMASK = 0xff;     // Write DataMask
      VOR_GPIO->BANK[1].DATAOUT = 0x80 | c;  // Pulse bit 7
      VOR_GPIO->BANK[1].DATAMASK = dm;
      break;
    case en_stdio_portc:
      dm = VOR_GPIO->BANK[2].DATAMASK;
      VOR_GPIO->BANK[2].DATAMASK = 0xff;     // Write DataMask
      VOR_GPIO->BANK[2].DATAOUT = 0x80 | c;  // Pulse bit 7
      VOR_GPIO->BANK[2].DATAMASK = dm;
      break;
    case en_stdio_portd:
      dm = VOR_GPIO->BANK[3].DATAMASK;
      VOR_GPIO->BANK[3].DATAMASK = 0xff;     // Write DataMask
      VOR_GPIO->BANK[3].DATAOUT = 0x80 | c;  // Pulse bit 7
      VOR_GPIO->BANK[3].DATAMASK = dm;
      break;
    case en_stdio_rtt:
#ifdef ENABLE_RTT
      SEGGER_RTT_PutChar(0, (uint8_t)c);
#endif
      break;
  }
  return c;
}

en_stdio_t DBG_GetStdioOutput(void) { return ioOut; }

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
