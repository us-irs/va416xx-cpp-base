/***************************************************************************************
 * @file     va416xx_hal_adc.c
 * @version  V1.0.1
 * @date     19 July 2019
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

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal_adc.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define ADC_PERID (0x001907E9)

#define ADC_CLK_MIN (2000000UL)
#define ADC_CLK_MAX (12500000UL)

// number of NOPs between a manual trigger and checking that busy bit is set
#define ADC_NOP_CNT (3)

// resetting the ADC should not be necessary
//#define INSERT_PERIPHERAL_RESET

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
 ** @brief  Initialize ADC peripheral
 **
 ** @return hal_status_t status of the driver call
 **
 ** @Note   Reference VA416x0 Programmers Guide, ADC Section 18
 **
 ******************************************************************************/
hal_status_t HAL_ADC_Init(void) {
  // Enable clock
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_ADC;

#ifdef INSERT_PERIPHERAL_RESET
  uint32_t cnt;

  // reset ADC
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_ADC_Msk;
  for (cnt = 0; cnt < 7; cnt++)
    ;
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_ADC_Msk;
  for (cnt = 0; cnt < 7; cnt++)
    ;
#endif

  // check peripheral
  if (VOR_ADC->PERID != ADC_PERID) {
    return hal_status_initError;
  }

  // ADC clock (must be 2-12.5 MHz)
  // NOTE: Not using divide by 1 or /2 ratio in REVA silicon because of triggering issue
  // For this reason, keep SYSCLK above 8MHz to have the ADC /4 ratio in range)
  VOR_CLKGEN->CTRL1 &= ~CLKGEN_CTRL1_ADC_CLK_DIV_SEL_Msk;  // default div by 8
  if (SystemCoreClock <= (ADC_CLK_MAX * 4)) {
    VOR_CLKGEN->CTRL1 |= 0x1 << CLKGEN_CTRL1_ADC_CLK_DIV_SEL_Pos;  // div 4
  }

  VOR_ADC->CTRL = 0;
  VOR_ADC->FIFO_CLR = ADC_FIFO_CLR_FIFO_CLR_Msk;  // clear ADC FIFO

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  De-initialize ADC peripheral
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_ADC_DeInit(void) {
#ifdef INSERT_PERIPHERAL_RESET
  uint32_t cnt;

  VOR_SYSCONFIG->PERIPHERAL_RESET &=
      ~SYSCONFIG_PERIPHERAL_RESET_ADC_Msk;  // reset ADC clear the bit
  for (cnt = 0; cnt < 7; cnt++)
    ;
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_ADC_Msk;
  for (cnt = 0; cnt < 7; cnt++)
    ;
#endif
  // disable the clock
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_ADC;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Clear the FIFO
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_ADC_FIFO_Clear(void) {
  VOR_ADC->FIFO_CLR = (1UL << ADC_FIFO_CLR_FIFO_CLR_Pos);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Read ADC Cal bits
 **
 ** @return ADC calibration
 **
 ******************************************************************************/
uint32_t HAL_ADC_ReadCal(void) { return VOR_SYSCONFIG->ADC_CAL; }

/*******************************************************************************
 **
 ** @brief  Set FIFO Trigger Level
 **
 ** @param  Trigger level (0-31)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_ADC_FIFO_SetTrigLevel(uint32_t level) {
  VOR_ADC->RXFIFOIRQTRG = ((level << ADC_RXFIFOIRQTRG_LEVEL_Pos) & ADC_RXFIFOIRQTRG_LEVEL_Msk);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set (enable/disable) ADC external trigger mode
 **
 ** @param  extTrig - true = enabled, false = disabled
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_ADC_SetExtTrigEnable(bool extTrig) {
  if (extTrig) {
    VOR_ADC->CTRL |= ADC_CTRL_EXT_TRIG_EN_Msk;
  } else {
    VOR_ADC->CTRL &= ~ADC_CTRL_EXT_TRIG_EN_Msk;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Manually trigger a Temperature sensor reading
 **
 ** @param  pointer to a single conversion result
 **
 ** @return hal_status_t status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_ADC_ReadTempSensorManualTrigger(uint32_t *result) {
  if (result == NULL) return hal_status_badParam;

  // set CTRL to 1 conversion, CONV_CNT=0 (n+1), CHAN_TAG_EN=0
  VOR_ADC->CTRL = (0 << ADC_CTRL_CONV_CNT_Pos) | ADC_TEMP_SENSE;
  VOR_ADC->FIFO_CLR = ADC_FIFO_CLR_FIFO_CLR_Msk;
  VOR_ADC->CTRL |= ADC_CTRL_MANUAL_TRIG_Msk;  // start manual convert
  for (volatile uint32_t i = 0; i < ADC_NOP_CNT; i++) {
    __NOP();
    // wait
  }

  if (VOR_ADC->STATUS & ADC_STATUS_ADC_BUSY_Msk) {
    while (VOR_ADC->STATUS & ADC_STATUS_ADC_BUSY_Msk)
      ;  // wait on BUSY
    *result = VOR_ADC->FIFO_DATA;
  } else {
    // converter malfunction, should never get here
    *result = VOR_ADC->FIFO_DATA;  // try to read the fifo anyways (for diagnostic)
    return hal_status_periphErr;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Manually triggers a single ADC channel reading (simple mode)
 **
 ** @param  chanelNum - channel number (0 to 15)
 ** @param  result - pointer to conversion result buffer (uint16_t[])
 **
 ** @return hal_status_t status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_ADC_ReadSingle(uint8_t channelNum, uint16_t *result) {
  if (channelNum >= ADC_NUM_CHANNELS) return hal_status_badParam;
  if (result == NULL) return hal_status_badParam;

  // set CTRL to 1 conversion, CONV_CNT=0 (n+1), CHAN_TAG_EN=0
  VOR_ADC->CTRL = (0 << ADC_CTRL_CONV_CNT_Pos);
  VOR_ADC->CTRL |= (0x1UL << channelNum) << ADC_CTRL_CHAN_EN_Pos;  // select channel
  VOR_ADC->FIFO_CLR = ADC_FIFO_CLR_FIFO_CLR_Msk;
  VOR_ADC->CTRL |= ADC_CTRL_MANUAL_TRIG_Msk;  // start manual convert
  for (volatile uint32_t i = 0; i < ADC_NOP_CNT; i++) {
    __NOP();
    // wait
  }

  if (VOR_ADC->STATUS & ADC_STATUS_ADC_BUSY_Msk) {
    while (VOR_ADC->STATUS & ADC_STATUS_ADC_BUSY_Msk)
      ;  // wait on BUSY
    *result = (uint16_t)(VOR_ADC->FIFO_DATA);
  } else {
    // converter malfunction, should never get here
    *result = (uint16_t)(VOR_ADC->FIFO_DATA);  // try to read the fifo anyways (for diagnostic)
    return hal_status_periphErr;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Manually triggers an ADC channel reading (more options)
 **
 ** @param  ctrl - ADC_CTRL register struct (conversion settings)
 ** @param  result - pointer to conversion result buffer (uint16_t[])
 **
 ** @return hal_status_t status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_ADC_ManualTrigger(stc_adc_ctrl_t ctrl, uint16_t *result) {
  if (result == NULL) return hal_status_badParam;

  ctrl.ext_trig_en = 0;
  ctrl.manual_trig = 1;
  VOR_ADC->CTRL = *((uint32_t *)&ctrl);
  for (volatile uint32_t i = 0; i < ADC_NOP_CNT; i++) {
    __NOP();  // wait
  }

  if (VOR_ADC->STATUS & ADC_STATUS_ADC_BUSY_Msk) {
    while (VOR_ADC->STATUS & ADC_STATUS_ADC_BUSY_Msk)
      ;  // wait for BUSY to clear
    while (VOR_ADC->STATUS & ADC_STATUS_FIFO_ENTRY_CNT_Msk) {
      *result++ = (uint16_t)(VOR_ADC->FIFO_DATA);
    }
  } else {
    // converter malfunction, should never get here
    while (VOR_ADC->STATUS & ADC_STATUS_FIFO_ENTRY_CNT_Msk) {
      *result++ = (uint16_t)(VOR_ADC->FIFO_DATA);  // try to read anyways
    }
    return hal_status_periphErr;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Manually triggers an ADC channel reading
 **
 ** @param  chan - channel bitmask (see ADC Channel bit definitions in hal_adc.h)
 ** @param  result - [out] conversion value (raw counts), includes channel tag
 **
 ** @return hal_status_t status of driver call
 **
 ** @note   DEPRECATED 02/13/2020 - USE HAL_ADC_ManualTrigger() OR HAL_ADC_ReadSingle()
 **
 ******************************************************************************/
hal_status_t HAL_ADC_ReadChanManualTrig(uint32_t chan, uint32_t *result) {
  volatile uint32_t index;

  // set the default to 1 conversion, CONV_CNT=0 (n+1), CHAN_TAG_EN 1
  VOR_ADC->CTRL = 0x010000;
  VOR_ADC->CTRL |= (chan & ADC_CTRL_CHAN_EN_Msk);  // select channel
  VOR_ADC->FIFO_CLR = ADC_FIFO_CLR_FIFO_CLR_Msk;
  VOR_ADC->CTRL |= ADC_CTRL_MANUAL_TRIG_Msk;  // start manual convert
  for (index = 0; index < ADC_NOP_CNT; index++) {
    __NOP();
    // wait
  }

  if (VOR_ADC->STATUS & ADC_STATUS_ADC_BUSY_Msk) {
    while (VOR_ADC->STATUS & ADC_STATUS_ADC_BUSY_Msk)
      ;  // wait on BUSY
    *result = VOR_ADC->FIFO_DATA;
  } else {
    // converter malfunction, should never get here
    *result = VOR_ADC->FIFO_DATA;  // try to read the fifo anyways (for diagnostic)
    return hal_status_periphErr;
  }

  return hal_status_ok;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
