/***************************************************************************************
 * @file     va416xx_hal.h
 * @version  V0.1
 * @date     06 February 2019
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
 
#ifndef __VA416XX_HAL_H
#define __VA416XX_HAL_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "va416xx.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** NULL pointer */
#ifndef NULL
#define NULL ( (void *) 0)
#endif
    
#define HAL_NVIC_MAX_PRIO  (15)  // highest priority number (lowest priority)

#define APB1_CLK  (SystemCoreClock/2) // APB1 peripherals 0x4001xxxx
#define APB2_CLK  (SystemCoreClock/4) // APB2 peripherals 0x4002xxxx

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/** HAL status (driver call return value) */
typedef enum
{
  hal_status_ok             = 0, 
  hal_status_initError      = 1,
  hal_status_badParam       = 2,
  hal_status_notInitialized = 3,
  hal_status_badPeriphID    = 4,
  hal_status_timeout        = 5,
  hal_status_rxError        = 6,
  hal_status_txError        = 7,
  hal_status_bufEmpty       = 8,
  hal_status_bufFull        = 9,
  hal_status_nak            = 10,
  hal_status_arblost        = 11,
  hal_status_busy           = 12,
  hal_status_notImplemented = 13,
  hal_status_alignmentErr   = 14,
  hal_status_periphErr      = 15,
  hal_status_end            = 16  // end of list indicator
} hal_status_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/** System millisecond counter */
extern volatile uint64_t HAL_time_ms;
extern volatile bool newSysTick;

extern const char * HALStatusStrArr[(uint32_t)hal_status_end+1];

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern hal_status_t HAL_Init(void);
extern hal_status_t HAL_SysTick_Init(void);
extern const char * HAL_StatusToString(hal_status_t stat);

// define this in your project somewhere
extern void OnSystemClockChanged(void);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __VA416XX_HAL_H */
