/***************************************************************************************
 * @file     va416xx_hal_adc_swcal.h
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

#ifndef __HAL_ADC_SWCAL_H
#define __HAL_ADC_SWCAL_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "VORConfig.h"
#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/** Apply SW gain and offset correction to an input raw ADC value x*/
/** NOTE: may overflow or underflow. Best to use HAL_ADC_ApplyCorr() instead */
#define ADC_APPLY_CORR(x) ((((uint32_t)(x)*adc_gain_corr) >> 16) + adc_offset_corr)

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern uint32_t adc_gain_corr;   // upper 16 bits integer, lower 16 bits frac
extern int32_t adc_offset_corr;  // offset in ADC counts

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern hal_status_t HAL_ADC_SoftwareCal(void);
extern uint16_t HAL_ADC_ApplyCorr(uint16_t rawCount);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_ADC_SWCAL_H */
