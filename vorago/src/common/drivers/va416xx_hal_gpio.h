/***************************************************************************************
 * @file     va416xx_hal_gpio.h
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

#ifndef __VA416XX_HAL_GPIO_H
#define __VA416XX_HAL_GPIO_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "va416xx.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define GPIO_OUT(port, pin) (port->DIR |= (1UL << pin))
#define GPIO_IN(port, pin) (port->DIR &= ~(1UL << pin))
#define GPIO_SET(port, pin) (port->SETOUT = (1UL << pin))
#define GPIO_CLR(port, pin) (port->CLROUT = (1UL << pin))
#define GPIO_TOG(port, pin) (port->TOGOUT = (1UL << pin))
#define GPIO_RD(port, pin) ((port->DATAIN & (1UL << pin)) >> pin)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __VA416XX_HAL_GPIO_H */
