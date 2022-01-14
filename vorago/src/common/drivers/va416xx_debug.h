/***************************************************************************************
 * @file     va416xx_debug.h
 * @version  V0.1
 * @date     12 February 2019
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
 
#ifndef __VA416_DEBUG_H
#define __VA416_DEBUG_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "VORConfig.h"

#ifdef ENABLE_RTT
#include "segger_rtt.h"
#endif

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** Runtime assert */
#ifdef USE_ASSERT
#define c_assert(e) assert(e)
#else
#define c_assert(e) ((void)0)
#endif
  
/** Compile time assert */
#ifdef USE_ASSERT
#define COMPILE_TIME_ASSERT(pred) \
  switch(0){case 0:case pred:;}
#else
#define COMPILE_TIME_ASSERT(pred) ((void)0)
#endif
  
/** Debug print */
#ifdef DEBUG_PRINTS
#define dbgprintf(...) DBG_printf(__VA_ARGS__)
#define dbgprintln(...) DBG_println(__VA_ARGS__)
#else
#define dbgprintf(...) ((void)0)
#define dbgprintln(...) ((void)0)
#endif

#ifdef PRINTF_REDEFINE
#define printf(...) VOR_printf(__VA_ARGS__)
#endif

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/
  
typedef enum{
  en_stdio_none,
  en_stdio_uart0,
  en_stdio_uart1,
  en_stdio_uart2,
  en_stdio_porta,
  en_stdio_portb,
  en_stdio_portc,
  en_stdio_portd,
  en_stdio_rtt
} en_stdio_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern void VOR_printf(const char *fmt, ...);
extern void DBG_printf(const char *fmt, ...);  // prints "DEBUG: " before the msg
extern void DBG_println(const char *fmt, ...); // prints "DEBUG: " before the msg
extern void DBG_SetStdioOutput(en_stdio_t io);
extern en_stdio_t DBG_GetStdioOutput(void);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __VA416_DEBUG_H */
