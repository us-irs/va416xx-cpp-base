/***************************************************************************************
 * @file     va108xx_hal_dac.h
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

#ifndef __HAL_DAC_H
#define __HAL_DAC_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/** Version */
#define HAL_DAC_VERSION (0x00000101)  // 0.1.1 (05 Dec 2019)

/** DAC Peripheral clock source */
#define DAC_CLK (APB2_CLK)

// DAC External Outputs
#define AN_OUT0 0x0
#define AN_OUT1 0x1

// ADC Internal inputs
#define ADC_DAC0 0x0100
#define ADC_DAC1 0x0200
#define ADC_TEMP_SENSE 0x0400
#define ADC_BG_1P0 0x0800
#define ADC_BG_1P5 0x1000
#define ADC_AVDD15 0x2000
#define ADC_DVDD15 0x4000
#define ADC_VREFP5 0x8000

#define DAC_MANUAL_TRIG_YES 0x1
#define DAC_MANUAL_TRIG_NO 0x0
#define DAC_EXT_TRIG_YES 0x1
#define DAC_EXT_TRIG_NO 0x0

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/** DAC trigger types */
typedef enum { en_dac_manual, en_dac_ext, en_dac_none } en_dac_trig_t;

/** DAC speed enum */
typedef enum { en_dac_min_2M, en_dac_max_12p5M } en_dac_spd_t;

/** DAC init structure / device handle */
typedef struct stc_dac_handle {
  VOR_DAC_Type* dac;  // Trigger
} stc_dac_handle_t;

typedef struct stc_dac_ctrl0_handle {
  uint32_t dac_manual_trig;
  uint32_t dac_ext_trig_en;
  bool isClockOkay;
  bool isInitialized;
} stc_dac_ctrl0_t;

typedef struct stc_dac_ctrl1_handle {
  uint32_t dac_en;
  uint32_t dac_settling;
  bool isClockOkay;
} stc_dac_ctrl1_t;

/** DAC IRQ  struct */
typedef struct stc_dac_irq_handle {
  uint32_t en_dac_irq_fifo_depthtrig;
  uint32_t en_dac_irq_trig_error;
  uint32_t en_dac_irq_dac_done;
  uint32_t en_dac_irq_fifo_uflow;
  uint32_t en_dac_irq_fifo_oflow;
  uint32_t en_dac_irq_fifo_full;
  uint32_t en_dac_irq_fifo_empty;
} stc_dac_irq_t;

/** DAC DATA  struct */
typedef struct stc_dac_data_handle {
  uint32_t data;
} stc_dac_data_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

// Init / de-init
extern hal_status_t HAL_DAC_Reset(void);
extern hal_status_t HAL_DAC_Init(VOR_DAC_Type* const dac);
extern hal_status_t HAL_DAC_DeInit(VOR_DAC_Type* const dac);
extern uint32_t HAL_DAC_ReadPerid(VOR_DAC_Type* const);
extern uint32_t HAL_DAC_ReadCal(VOR_DAC_Type* const dac);
extern hal_status_t HAL_DAC_FIFO_SetTrigLevel(VOR_DAC_Type* const dac, uint32_t level);
extern hal_status_t HAL_DAC_SetSettlingTime(VOR_DAC_Type* const dac, uint32_t time);
extern hal_status_t HAL_DAC_IRQ_Enable(VOR_DAC_Type* const dac, stc_dac_irq_t* const dacHandlePtr);
extern hal_status_t HAL_DAC_IRQ_Clear(VOR_DAC_Type* const dac, stc_dac_irq_t* const dacHandlePtr);
extern hal_status_t HAL_DAC_IRQ_Clear_All(VOR_DAC_Type* const dac);
extern hal_status_t HAL_DAC_FIFO_Clear(VOR_DAC_Type* const dac);
extern hal_status_t HAL_DAC_ManualTrigger(VOR_DAC_Type* const dac, uint16_t dacVal);
extern stc_dac_data_t HAL_DAC_WriteData(void);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_DAC_H */
