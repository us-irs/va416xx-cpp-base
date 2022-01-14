/***************************************************************************************
 * @file     va416xx_hal.c
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
 
/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx.h"
#include "va416xx_hal.h"
#include "va416xx_hal_ioconfig.h"
#include "va416xx_hal_irqrouter.h"
#include "va416xx_debug.h"
#include "VORConfig.h"

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

#define SYSCONFIG_PROCID (0x040057E3)
#define SYSCONFIG_PERID  (0x028007E9)

// default SysTick interval (can override/redefine in board.h)
#ifndef SYSTICK_INTERVAL_MS
#define SYSTICK_INTERVAL_MS (10)
#endif

/*****************************************************************************/ 
/* Global variable definitions (declared in header file with 'extern')       */ 
/*****************************************************************************/

volatile uint64_t HAL_time_ms = 0;  // millisecond tick counter (64-bit)
volatile bool newSysTick = false;

const char * HALStatusStrArr[(uint32_t)hal_status_end+1] = 
{
  "ok",             //hal_status_ok             = 0, 
  "initError",      //hal_status_initError      = 1,
  "badParam",       //hal_status_badParam       = 2,
  "notInitialized", //hal_status_notInitialized = 3,
  "badPeriphID",    //hal_status_badPeriphID    = 4,
  "timeout",        //hal_status_timeout        = 5,
  "rxError",        //hal_status_rxError        = 6,
  "txError",        //hal_status_txError        = 7,
  "bufEmpty",       //hal_status_bufEmpty       = 8,
  "bufFull",        //hal_status_bufFull        = 9,
  "nak",            //hal_status_nak            = 10,
  "arblost",        //hal_status_arblost        = 11,
  "busy",           //hal_status_busy           = 12,
  "notImplemented", //hal_status_notImplemented = 13,
  "alignmentErr",   //hal_status_alignmentErr   = 14,
  "periphErr",      //hal_status_periphErr      = 15,
  "",               //all other values
};

/*****************************************************************************/ 
/* Local type definitions ('typedef')                                        */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local variable definitions ('static')                                     */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local function prototypes ('static')                                      */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Initialize HAL framework
 **
 ******************************************************************************/
hal_status_t HAL_Init(void)
{
  hal_status_t status;
  
  // Check sysconfig peripheral
  c_assert(SYSCONFIG_PROCID == VOR_SYSCONFIG->PROCID);
  c_assert(SYSCONFIG_PERID == VOR_SYSCONFIG->PERID);
  
  // Enable clock gating to GPIO and critical periphs
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE =
        (CLK_ENABLE_PORTA | CLK_ENABLE_PORTB | CLK_ENABLE_PORTC |
        CLK_ENABLE_PORTD | CLK_ENABLE_PORTE | CLK_ENABLE_PORTF |
        CLK_ENABLE_PORTG | CLK_ENABLE_IOCONFIG | CLK_ENABLE_CLKGEN);
  
  // IRQ router
  status = HAL_Irqrouter_Init();
  c_assert(status == hal_status_ok);
  if(status != hal_status_ok){
    return hal_status_initError;
  }
  
  // SysTick 10ms interval
  status = HAL_SysTick_Init();
  c_assert(status == hal_status_ok);
  if(status != hal_status_ok){
    return hal_status_initError;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Initialize SysTick counter
 **
 ******************************************************************************/
hal_status_t HAL_SysTick_Init(void)
{
  SysTick->LOAD = ((SystemCoreClock/1000)*SYSTICK_INTERVAL_MS)-1;
  NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
  SysTick->VAL  = 0UL;
  SysTick->CTRL = 0x7;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Returns a string version of the hal_status_t status
 **
 ******************************************************************************/
const char * HAL_StatusToString(hal_status_t stat)
{
  if(stat > hal_status_end){ stat = hal_status_end; }
  return HALStatusStrArr[(uint32_t)stat];
}

/*******************************************************************************
 **
 ** @brief  SysTick handler. Increments HAL_time_ms
 **
 ******************************************************************************/
void SysTick_Handler(void)
{
  HAL_time_ms += SYSTICK_INTERVAL_MS;
  newSysTick = true;
}

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
