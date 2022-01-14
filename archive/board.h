/***************************************************************************************
 * @file     board.h
 * @version  V1.11
 * @date     04 Dec 2020
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
 
#ifndef __BOARD_H
#define __BOARD_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "va416xx.h"
#include "va416xx_hal_ioconfig.h"
#include "va416xx_hal_clkgen.h"

/** BSP selection */
#include "va416xx_evk.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** Hardware version (define for VA416x0 RevB) */
#define __MCU_HW_VER_REVB

/** Software Version */
#define SOFTWARE_VERSION_STR  "2020_12_04_1v11"
#define SOFTWARE_VERSION (0x00010011) // 1.11

// assert enable/disable (comment out to disable)
//#define USE_ASSERT

// debug print enable/disable (comment out to disable)
#define DEBUG_PRINTS

// Enable Segger RTT (comment out to disable)
#define ENABLE_RTT

// turn calls to printf() into VOR_printf()
#define PRINTF_REDEFINE

// enable watchdog (turn off if using breakpoints/debug)
//#define ENABLE_WATCHDOG

/** Default pin IOCONFIG register. type: un_iocfg_reg_t - see va416xx_hal_ioconfig.h */
/** A pin's IOCONFIG is set to this by HAL_Iocfg_Init() if that pin is not in the cfg array */
#define DEFAULT_PIN_IOCFG   (IOCFG_REG_PULLDN) // internal pulldown enabled for input pin

/** Default pin direction (input/output) type: en_iocfg_dir_t - see va416xx_hal_ioconfig.h */
/** A pin's DIR is set to this by HAL_Iocfg_Init() if that pin is not in the cfg array */
#define DEFAULT_PIN_DIR     (en_iocfg_dir__input) // default pin input

/* Override external clocks (project specific) */
#undef   EXTCLK
#undef   XTAL
#undef   HBO
#define  EXTCLK          (40000000UL)      /* XTAL minus frequency */
#define  XTAL            (10000000UL)      /* Oscillator frequency */
#define  HBO             (18500000UL)      /* Internal HBO frequency (18-22mhz) */

/* Expected VREF voltage */
#define ADC_VREF         (3.3f)

/* DAC channel to use for ADC calibration */
#define ADC_CAL_DAC      (VOR_DAC0)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

// default IO pin configuration array (all pins)
extern const stc_iocfg_pin_cfg_t ioPinCfgArr[];

// EBI pin configuration array (EBI pins only)
extern const stc_iocfg_pin_cfg_t ebiPinCfgArr[];

// Ethernet pin configuration array (ETH pins only)
extern const stc_iocfg_pin_cfg_t ethPinCfgArr[];

// Current external clock source type (none, crystal, or external clock)
extern hal_xtalsel_t gCurrentXtalsel; // defined in main.c

// system seconds counter, defined in main.c
extern volatile uint32_t gSecondsCounter;

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __BOARD_H */
