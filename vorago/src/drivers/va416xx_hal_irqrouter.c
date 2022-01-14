/***************************************************************************************
 * @file     va416xx_hal_irqrouter.c
 * @version  V0.1
 * @date     07 February 2019
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

#include "va416xx_hal_irqrouter.h"

#include "va416xx_debug.h"
#include "va416xx_hal_timer.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define IRQROUTER_PERID (0x028107E9)

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

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
 ** @brief  Reset and enable IRQ_ROUTER
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Irqrouter_Init(void) {
  // reset
  HAL_Irqrouter_Reset();

  // check PERID
  if (IRQROUTER_PERID != VOR_IRQ_ROUTER->PERID) {
    return hal_status_badPeriphID;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set DMA Trigger Select Register
 **
 ** @param  channel dma channel 0-3 (uint32_t)
 ** @param  selCode source of DMA trigger (en_irqr_dmasel_t)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Irqrouter_SetDmaSel(uint32_t channel, en_irqr_dmasel_t selCode) {
  switch (channel) {
    case 0:
      VOR_IRQ_ROUTER->DMASEL0 = (uint32_t)selCode;
      break;
    case 1:
      VOR_IRQ_ROUTER->DMASEL1 = (uint32_t)selCode;
      break;
    case 2:
      VOR_IRQ_ROUTER->DMASEL2 = (uint32_t)selCode;
      break;
    case 3:
      VOR_IRQ_ROUTER->DMASEL3 = (uint32_t)selCode;
      break;
    default:
      return hal_status_badParam;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set DMA Trigger Type Select Register
 **
 ** @param  channel dma channel 0-3 (uint32_t)
 ** @param  selCode single request (1) or normal (0) (en_irqr_dmattsel_t)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Irqrouter_SetDmaTtsel(uint32_t channel, en_irqr_dmattsel_t selCode) {
  if (channel > 3) {
    return hal_status_badParam;
  }
  VOR_IRQ_ROUTER->DMATTSEL = (uint32_t)selCode << channel;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set ADC Trigger Select Register
 **
 ** @param  timerNum - selects status output of timer[timerNum] to trigger ADC
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Irqrouter_SetAdcSel(uint32_t timerNum) {
  if (timerNum >= HAL_NUM_TIMERS) return hal_status_badParam;
  VOR_IRQ_ROUTER->ADCSEL = timerNum;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set DAC Trigger Select Register
 **
 ** @param  channel  - selects DAC0 or DAC1 (0 or 1)
 ** @param  timerNum - selects status output of timer[timerNum] to trigger DAC
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Irqrouter_SetDacSel(uint32_t channel, uint32_t timerNum) {
  if (timerNum >= HAL_NUM_TIMERS) return hal_status_badParam;
  switch (channel) {
    case 0:
      VOR_IRQ_ROUTER->DACSEL0 = timerNum;
      break;
    case 1:
      VOR_IRQ_ROUTER->DACSEL1 = timerNum;
      break;
    default:
      return hal_status_badParam;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Enable clock and reset IRQ_ROUTER
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Irqrouter_Reset(void) {
  IRQROUTER_ENABLE_CLOCK();
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_IRQ_Msk;
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_IRQ_Msk;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Reset and then declock IRQ_ROUTER
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Irqrouter_DeInit(void) {
  HAL_Irqrouter_Reset();
  IRQROUTER_DISABLE_CLOCK();
  return hal_status_ok;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
