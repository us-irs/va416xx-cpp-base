/***************************************************************************************
 * @file     va416xx_hal_ioconfig.h
 * @version  V0.4
 * @date     30 January 2019
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

#ifndef __HAL_IOCFG_H
#define __HAL_IOCFG_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define HAL_IOCFG_VERSION (0x00000401)  // 0.4.1

/** Function Select */
#define FUNSEL0 (0)
#define FUNSEL1 (1)
#define FUNSEL2 (2)
#define FUNSEL3 (3)

/** define the available GPIO pins on device */
#define USER_PORTA_PINS (0x0000ffff)
#define USER_PORTB_PINS (0x0000ffff)
#define USER_PORTC_PINS (0x0000ffff)
#define USER_PORTD_PINS (0x0000ffff)
#define USER_PORTE_PINS (0x0000ffff)
#define USER_PORTF_PINS (0x0000ffff)
#define USER_PORTG_PINS (0x000000ff)

/** number of GPIO pins on ports */
#define PORTA_F_NUM_PINS (16)
#define PORTG_NUM_PINS (8)

/** common IO Configutration register values */
#define IOCFG_REG_DEFAULT \
  (un_iocfg_reg_t) {      \
    { 0 }                 \
  }  // reset value (0x00000000)
#define IOCFG_REG_PULLUP      \
  (un_iocfg_reg_t) {          \
    { .plevel = 1, .pen = 1 } \
  }
#define IOCFG_REG_PULLDN      \
  (un_iocfg_reg_t) {          \
    { .plevel = 0, .pen = 1 } \
  }
#define IOCFG_REG_FUN0 \
  (un_iocfg_reg_t) {   \
    { .funsel = 0 }    \
  }
#define IOCFG_REG_FUN1 \
  (un_iocfg_reg_t) {   \
    { .funsel = 1 }    \
  }
#define IOCFG_REG_FUN2 \
  (un_iocfg_reg_t) {   \
    { .funsel = 2 }    \
  }
#define IOCFG_REG_FUN3 \
  (un_iocfg_reg_t) {   \
    { .funsel = 3 }    \
  }

// Used to define end of IOcfg structure array
#define IOCFG_PINCFG_END \
  { .port = 0 }

// IOCONFIG peripheral clock enable/disable macros
#define IOCFG_ENABLE_CLOCK() VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_IOCONFIG
#define IOCFG_DISABLE_CLOCK() VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_IOCONFIG

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

typedef uint32_t gpio_pin_number_t;

/** port pin direction input or output */
typedef enum en_iocfg_dir {
  en_iocfg_dir_input = 0,  // double underscore intentional (for text alignment reasons)
  en_iocfg_dir_output = 1,
  en_iocfg_dir_dncare = 2  // this will map to input (used for pins that are not function-0)
} en_iocfg_dir_t;

/** port pin function select */
typedef enum en_iocfg_funsel {
  en_iocfg_funsel_0 = FUNSEL0,
  en_iocfg_funsel_1 = FUNSEL1,
  en_iocfg_funsel_2 = FUNSEL2,
  en_iocfg_funsel_3 = FUNSEL3
} en_iocfg_funsel_t;

/** IO Configuration register */
typedef struct stc_iocfg_reg {
  uint32_t flttype : 3;  // Filter type (default 0 sync to sysclk only)
  uint32_t fltclk : 3;   // IO filter clocks select 0-7 (default 0 sysclk)
  uint32_t invinp : 1;   // Enable input inversion (1 = invert)
  uint32_t iewo : 1;     // Enable input even when in putput mode (1 = enable)
  uint32_t opendrn : 1;  // Enable open drain mode (1 = enabled)
  uint32_t invout : 1;   // Enable output inversion (1 = invert)
  uint32_t plevel : 1;   // Direction of PEN pull (high/low, 1 = pullup)
  uint32_t pen : 1;      // Enable internal pullup/pulldown (1 = enabled)
  uint32_t pwoa : 1;     // Enable pullup/pulldown even when output is active (1 = enabled)
  uint32_t funsel : 2;   // Sets the function select
  uint32_t reservd : 1;  // Reserved
  uint32_t iodis : 1;    // Disable the IO buffer. Turns off both input and output. In reads 0.
  uint32_t : 15;         // Reserved reads as 0
} stc_iocfg_reg_t;

/** Union wrapper for IO configuration register struct */
typedef union un_iocfg_reg {
  stc_iocfg_reg_t regStc;  // the structure with bitfields
  uint32_t regRaw;         // the raw 32bit value
} un_iocfg_reg_t;

/** A pin config struct */
typedef struct stc_iocfg_pin_cfg {
  VOR_GPIO_Type* port;
  gpio_pin_number_t pinNum;
  en_iocfg_dir_t dir;
  un_iocfg_reg_t reg;
} stc_iocfg_pin_cfg_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/** Alias for GPIO Ports. */
extern VOR_GPIO_Type* const PORTA;
extern VOR_GPIO_Type* const PORTB;
extern VOR_GPIO_Type* const PORTC;
extern VOR_GPIO_Type* const PORTD;
extern VOR_GPIO_Type* const PORTE;
extern VOR_GPIO_Type* const PORTF;
extern VOR_GPIO_Type* const PORTG;

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern hal_status_t HAL_Iocfg_Init(
    const stc_iocfg_pin_cfg_t*
        pinConfig);  // init pins to sw default, then setup specified pins in pinConfig array
extern hal_status_t HAL_Iocfg_DeInit(void);  // declocks and resets IOCONFIG peripheral
extern hal_status_t HAL_Iocfg_SetupPin(VOR_GPIO_Type* port, gpio_pin_number_t pin,
                                       en_iocfg_dir_t dir, un_iocfg_reg_t reg);  // setup a pin
extern hal_status_t HAL_Iocfg_SetupPins(const stc_iocfg_pin_cfg_t* pinConfig);
extern hal_status_t HAL_Iocfg_PinMux(VOR_GPIO_Type* port, gpio_pin_number_t pin, uint32_t funsel);
extern hal_status_t HAL_Iocfg_SetClkDiv(uint32_t clkNum, uint32_t divVal);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_IOCFG_H */
