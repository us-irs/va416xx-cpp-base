/***************************************************************************************
 * @file     va416xx_hal_irqrouter.h
 * @version  V0.2
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

#ifndef __HAL_IRQROUTER_H
#define __HAL_IRQROUTER_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/** Version */
#define HAL_IRQROUTER_VERSION (0x00000200)  // 0.2.0 (23 September 2019)

// IRQ Router peripheral clock enable/disable macros
#define IRQROUTER_ENABLE_CLOCK() VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_IRQ
#define IRQROUTER_DISABLE_CLOCK() VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_IRQ

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/** DMASELx register DMA selection codes */
typedef enum {
  en_irqr_dmasel_none = 14,
  en_irqr_dmasel_spi0_tx = 0,
  en_irqr_dmasel_spi0_rx = 1,
  en_irqr_dmasel_spi1_tx = 2,
  en_irqr_dmasel_spi1_rx = 3,
  en_irqr_dmasel_spi2_tx = 4,
  en_irqr_dmasel_spi2_rx = 5,
  en_irqr_dmasel_spi3_tx = 6,
  en_irqr_dmasel_spi3_rx = 7,
  en_irqr_dmasel_uart0_tx = 8,
  en_irqr_dmasel_uart0_rx = 9,
  en_irqr_dmasel_uart1_tx = 10,
  en_irqr_dmasel_uart1_rx = 11,
  en_irqr_dmasel_uart2_tx = 12,
  en_irqr_dmasel_uart2_rx = 13,
  en_irqr_dmasel_spw = 22,
  en_irqr_dmasel_dac0 = 24,
  en_irqr_dmasel_dac1 = 25,
  en_irqr_dmasel_adc = 28,
  en_irqr_dmasel_wdog = 31,
  en_irqr_dmasel_tim0_done = 32,
  en_irqr_dmasel_tim1_done = 33,
  en_irqr_dmasel_tim2_done = 34,
  en_irqr_dmasel_tim3_done = 35,
  en_irqr_dmasel_tim4_done = 36,
  en_irqr_dmasel_tim5_done = 37,
  en_irqr_dmasel_tim6_done = 38,
  en_irqr_dmasel_tim7_done = 39,
  en_irqr_dmasel_tim8_done = 40,
  en_irqr_dmasel_tim9_done = 41,
  en_irqr_dmasel_tim10_done = 42,
  en_irqr_dmasel_tim11_done = 43,
  en_irqr_dmasel_tim12_done = 44,
  en_irqr_dmasel_tim13_done = 45,
  en_irqr_dmasel_tim14_done = 46,
  en_irqr_dmasel_tim15_done = 47,
  en_irqr_dmasel_tim16_done = 48,
  en_irqr_dmasel_tim17_done = 49,
  en_irqr_dmasel_tim18_done = 50,
  en_irqr_dmasel_tim19_done = 51,
  en_irqr_dmasel_tim20_done = 52,
  en_irqr_dmasel_tim21_done = 53,
  en_irqr_dmasel_tim22_done = 54,
  en_irqr_dmasel_tim23_done = 55,
  en_irqr_dmasel_can0 = 56,
  en_irqr_dmasel_can1 = 58,
  en_irqr_dmasel_i2c0_ms_rx = 60,
  en_irqr_dmasel_i2c0_ms_tx = 61,
  en_irqr_dmasel_i2c0_sl_rx = 62,
  en_irqr_dmasel_i2c0_sl_tx = 63,
  en_irqr_dmasel_i2c1_ms_rx = 64,
  en_irqr_dmasel_i2c1_ms_tx = 65,
  en_irqr_dmasel_i2c1_sl_rx = 66,
  en_irqr_dmasel_i2c1_sl_tx = 67,
  en_irqr_dmasel_i2c2_ms_rx = 68,
  en_irqr_dmasel_i2c2_ms_tx = 69,
  en_irqr_dmasel_i2c2_sl_rx = 70,
  en_irqr_dmasel_i2c2_sl_tx = 71
} en_irqr_dmasel_t;

typedef enum { en_irqr_dmattsel_dmareq = 0, en_irqr_dmattsel_dmasreq = 1 } en_irqr_dmattsel_t;

typedef enum {
  en_irqr_adcdacsel_tim0_stat = 0,
  en_irqr_adcdacsel_tim1_stat = 1,
  en_irqr_adcdacsel_tim2_stat = 2,
  en_irqr_adcdacsel_tim3_stat = 3,
  en_irqr_adcdacsel_tim4_stat = 4,
  en_irqr_adcdacsel_tim5_stat = 5,
  en_irqr_adcdacsel_tim6_stat = 6,
  en_irqr_adcdacsel_tim7_stat = 7,
  en_irqr_adcdacsel_tim8_stat = 8,
  en_irqr_adcdacsel_tim9_stat = 9,
  en_irqr_adcdacsel_tim10_stat = 10,
  en_irqr_adcdacsel_tim11_stat = 11,
  en_irqr_adcdacsel_tim12_stat = 12,
  en_irqr_adcdacsel_tim13_stat = 13,
  en_irqr_adcdacsel_tim14_stat = 14,
  en_irqr_adcdacsel_tim15_stat = 15,
  en_irqr_adcdacsel_tim16_stat = 16,
  en_irqr_adcdacsel_tim17_stat = 17,
  en_irqr_adcdacsel_tim18_stat = 18,
  en_irqr_adcdacsel_tim19_stat = 19,
  en_irqr_adcdacsel_tim20_stat = 20,
  en_irqr_adcdacsel_tim21_stat = 21,
  en_irqr_adcdacsel_tim22_stat = 22,
  en_irqr_adcdacsel_tim23_stat = 23,
} en_irqr_adcdacsel_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern hal_status_t HAL_Irqrouter_Init(void);
extern hal_status_t HAL_Irqrouter_SetDmaSel(uint32_t channel, en_irqr_dmasel_t selCode);
extern hal_status_t HAL_Irqrouter_SetDmaTtsel(uint32_t channel, en_irqr_dmattsel_t selCode);
extern hal_status_t HAL_Irqrouter_SetAdcSel(uint32_t timerNum);
extern hal_status_t HAL_Irqrouter_SetDacSel(uint32_t channel, uint32_t timerNum);
extern hal_status_t HAL_Irqrouter_Reset(void);
extern hal_status_t HAL_Irqrouter_DeInit(void);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_IRQROUTER_H */
