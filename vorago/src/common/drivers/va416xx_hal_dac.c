/***************************************************************************************
 * @file     va416xx_hal_dac.c
 * @version  V0.3
 * @date     19 July 2019
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

#include "va416xx_hal_dac.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

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
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Enable clock and reset BOTH DACs
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DAC_Reset(void) {
  /***
   *  Enable clock and reset
   *  There is only one CLK_ENABLE and reset for both DACs
   */
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_DAC;
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~RESET_DAC;
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= RESET_DAC;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Initialize DAC peripheral
 **
 ** @param  dac - VOR_DAC0 or VOR_DAC1
 **
 ** @return hal_status_t status of the driver call
 **
 ** @Note   The DAC gets loaded with a default settling 4
 **
 ******************************************************************************/
hal_status_t HAL_DAC_Init(VOR_DAC_Type* const dac) {
  /***
   *  Enable clock
   *  There is only one CLK_ENABLE for both DACs
   */
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_DAC;
  dac->CTRL1 |= (1UL << DAC_CTRL1_DAC_EN_Pos) | (4UL << DAC_CTRL1_DAC_SETTLING_Pos);
  dac->FIFO_CLR = 1;
  dac->IRQ_CLR = 0xf;

  if ((dac->IRQ_RAW & 0x3) == 0x3) {
    // error condition (both full and empty)
    return hal_status_periphErr;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  De-initialize DAC peripheral
 **
 ** @param  dac - VOR_DAC0 or VOR_DAC1
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DAC_DeInit(VOR_DAC_Type* const dac) {
  dac->CTRL1 = 0;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set Transmit FIFO Interrupt Trigger Value
 **
 ** @param  dac - VOR_DAC0 or VOR_DAC1
 **
 ** @return result
 **
 ******************************************************************************/
hal_status_t HAL_DAC_FIFO_SetTrigLevel(VOR_DAC_Type* const dac, uint32_t level) {
  dac->TXFIFOIRQTRG = level;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Manual Trigger DAC, set output (simple)
 **
 ** @param  dac - VOR_DAC0 or VOR_DAC1
 **
 ** @param  value - 0x0 - 0xfff - DAC output value
 **
 ** @return hal_status_t - result of driver call
 **
 ******************************************************************************/
hal_status_t HAL_DAC_ManualTrigger(VOR_DAC_Type* const dac, uint16_t value) {
  if ((dac != VOR_DAC0) && (dac != VOR_DAC1)) {
    return hal_status_badParam;
  }
  if (value > 0xfff) {
    return hal_status_badParam;
  }

  dac->FIFO_CLR = 1;
  dac->FIFO_DATA = value;

  dac->CTRL0 |= DAC_CTRL0_MAN_TRIG_EN_Msk;  // manual trigger conversion
  __NOP();
  __NOP();
  while (dac->STATUS & DAC_STATUS_DAC_BUSY_Msk)
    ;  // wait until not busy

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  DAC settling time
 **
 ** @param  dac - VOR_DAC0 or VOR_DAC1
 **
 ** @return result
 **
 ******************************************************************************/
hal_status_t HAL_DAC_SetSettlingTime(VOR_DAC_Type* const dac, uint32_t time) {
  dac->CTRL1 |= !(time << DAC_CTRL1_DAC_EN_Pos);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  There is one single bit clear the FIFO
 **
 ** @param  dac - VOR_DAC0 or VOR_DAC1
 **
 ** @return result
 **
 ******************************************************************************/
hal_status_t HAL_DAC_FIFO_Clear(VOR_DAC_Type* const dac) {
  dac->FIFO_CLR = (1UL << DAC_FIFO_CLR_FIFO_CLR_Pos);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Read DAC peripheral ID
 **
 ** @param  dac - VOR_DAC0 or VOR_DAC1
 **
 ** @return DAC peripheral ID
 **
 ******************************************************************************/
uint32_t HAL_DAC_ReadPerid(VOR_DAC_Type* const dac) { return dac->PERID; }

/*******************************************************************************
 **
 ** @brief  Read DAC Cal bits
 **
 ** @param  dac - VOR_DAC0 or VOR_DAC1
 **
 ** @return result
 **
 ******************************************************************************/
uint32_t HAL_DAC_ReadCal(VOR_DAC_Type* const dac) {
  if (dac == VOR_DAC0) {
    return VOR_SYSCONFIG->DAC0_CAL;
  } else {
    return VOR_SYSCONFIG->DAC1_CAL;
  }
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
