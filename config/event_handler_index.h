/***************************************************************************************
 * @file     event_handler_index.h
 * @version  V0.1
 * @date     15 February 2019
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

#ifndef __EVENT_HNDLR_INDEX_H
#define __EVENT_HNDLR_INDEX_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal.h"

#ifdef __cplusplus
extern "C"
#endif

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define EVENTHANDLER_LEN (size_t)(eventHandlerIdx_end)

    /*****************************************************************************/
    /* Global type definitions ('typedef')                                       */
    /*****************************************************************************/

    /** ordering of the labels here need to match the function order in the array */
    typedef enum {
      eventHandlerIdx_null = 0,
      eventHandlerIdx_u0ExampleCall = 1,
      eventHandlerIdx_end = 2
    } eventHandlerIdx_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/** List of events/callbacks that driver interrupts can call.
    Defined at compile time to avoid use of non-constant function pointers */
extern void (*const apEventHList[EVENTHANDLER_LEN])(void);

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern void U0_ExampleCallback(void);

#ifdef __cplusplus
}
#endif

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __EVENT_HNDLR_INDEX_H */
