/***************************************************************************************
 * @file     va416xx_hal_adc_swcal.c
 * @version  V0.8
 * @date     30 March 2020
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

#include "va416xx_hal_adc_swcal.h"

#include "va416xx_hal_adc.h"
#include "va416xx_hal_dac.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

// DAC correction values
#define DAC_GAIN_CORR (0.9742f)
#define DAC_OFFSET_CORR (0.0014f)

// The two DAC setpoints.
#define DAC_POINT0 (0x100)
#define DAC_POINT1 (0xe00)

// Number of ADC samples to average. Min 1 max 7
#define ADCCAL_NUM_AVG (4)

#ifndef ADC_CAL_DAC
#error "must define ADC_CAL_DAC in board.h as either VOR_DAC0 or VOR_DAC1"
#endif

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

// software calibration values (defaults to 1.0x gain, 0 offset)
uint32_t adc_gain_corr = 0x10000;  // upper 16 bits integer, lower 16 bits fractional
int32_t adc_offset_corr = 0;       // offset in ADC counts

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Perform a 2-point software calibration of the ADC using either DAC0
 **         or DAC1 (define ADC_CAL_DAC in board.h)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_ADC_SoftwareCal(void) {
  hal_status_t stat = HAL_ADC_Init();
  if (stat != hal_status_ok) {
    return stat;
  }
  stat = HAL_DAC_Init(ADC_CAL_DAC);
  if (stat != hal_status_ok) {
    return stat;
  }

  uint16_t adcVals[ADCCAL_NUM_AVG + 1];
  uint16_t adc_point0, adc_point1;
  stc_adc_ctrl_t adcCtrl;
  uint32_t acc = 0;

  if (ADC_CAL_DAC == VOR_DAC0) {
    adcCtrl.chan_en = ADC_DAC0;
  } else {
    adcCtrl.chan_en = ADC_DAC1;
  }
  adcCtrl.chan_tag_en = 0;
  adcCtrl.conv_cnt_m1 = ADCCAL_NUM_AVG;  // avg+1 conversions
  adcCtrl.sweep_en = 0;

  // read first point
  stat = HAL_DAC_ManualTrigger(ADC_CAL_DAC, DAC_POINT0);
  if (stat != hal_status_ok) {
    return stat;
  }
  for (uint32_t i = 0; i < 1000; i++)
    ;
  stat = HAL_ADC_ManualTrigger(adcCtrl, adcVals);
  if (stat != hal_status_ok) {
    return stat;
  }
  for (uint32_t i = 1; i <= ADCCAL_NUM_AVG; i++) {
    acc += adcVals[i];  // discard sample 0, start at 1
  }
  adc_point0 = (uint16_t)(acc / ADCCAL_NUM_AVG);

  // read second point
  stat = HAL_DAC_ManualTrigger(ADC_CAL_DAC, DAC_POINT1);
  if (stat != hal_status_ok) {
    return stat;
  }
  for (uint32_t i = 0; i < 1000; i++)
    ;
  stat = HAL_ADC_ManualTrigger(adcCtrl, adcVals);
  if (stat != hal_status_ok) {
    return stat;
  }
  acc = 0;
  for (uint32_t i = 1; i <= ADCCAL_NUM_AVG; i++) {
    acc += adcVals[i];  // discard sample 0, start at 1
  }
  adc_point1 = (uint16_t)(acc / ADCCAL_NUM_AVG);

  float exp_point0 = ((float)(DAC_POINT0)*DAC_GAIN_CORR) + ((DAC_OFFSET_CORR / ADC_VREF) * 4095.0f);
  float exp_point1 = ((float)(DAC_POINT1)*DAC_GAIN_CORR) + ((DAC_OFFSET_CORR / ADC_VREF) * 4095.0f);

  float adcGainCorr = (exp_point1 - exp_point0) / (float)(adc_point1 - adc_point0);
  float adcOffsetCorr = exp_point1 - (adcGainCorr * (float)(adc_point1));

  adc_gain_corr = (uint32_t)(adcGainCorr * 65536);  // 16 bits integer, 16 bits fractional
  adc_offset_corr =
      (int32_t)(adcOffsetCorr + (adcOffsetCorr ? 0.5f : -0.5f));  // rounding for integer conversion

  HAL_DAC_ManualTrigger(ADC_CAL_DAC, 0);  // DAC back to 0V

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Applies the gain/offset correction to a RAW adc reading (0-4095)
 **
 ** @param  rawCount RAW adc reading (0-4095), must have channel tag information removed
 **
 ** @return uint16_t the corrected value
 **
 ** @note   if better performance is necessary, best to copy and inline this function
 **
 ******************************************************************************/
uint16_t HAL_ADC_ApplyCorr(uint16_t rawCount) {
  int32_t tmp = ((((int32_t)(rawCount)*adc_gain_corr) >> 16) + adc_offset_corr);
  // bound result to 0 - 4095 (12-bit)
  if (tmp > 4095) {
    tmp = 4095;
  }
  if (tmp < 0) {
    tmp = 0;
  }
  return (uint16_t)tmp;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
