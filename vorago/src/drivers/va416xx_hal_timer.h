/***************************************************************************************
 * @file     va416xx_hal_timer.h
 * @version  V0.3
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

#ifndef __HAL_TIMER_H
#define __HAL_TIMER_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/** Version */
#define HAL_TIMER_VERSION (0x00000302)  // 0.3.2 (23 April 2019)

/** Number of timers */
#define HAL_NUM_TIMERS (24)

/** Macro for determining timer clock rate (APB1 sys/2 or APB2 sys/4) */
#define TIMER_CLK(timerNum) (((timerNum) < 16) ? (SystemCoreClock / 2) : (SystemCoreClock / 4))

/** Enable/disable timer macros */
#define ENABLE_TIMER(timerNum)          \
  if ((timerNum) < HAL_NUM_TIMERS) {    \
    VOR_TIM_BANK[timerNum]->ENABLE = 1; \
  }
#define DISABLE_TIMER(timerNum)         \
  if ((timerNum) < HAL_NUM_TIMERS) {    \
    VOR_TIM_BANK[timerNum]->ENABLE = 0; \
  }

/** PWM duty cycle (percent times 1000) */
#define TIMER_PWM_DUTY_MIN (0)
#define TIMER_PWM_DUTY_MAX (100000)

/** Not need from headerfile */
#undef VOR_TIM_BANK

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#elif defined(__ICCARM__)
#pragma language = extended
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
#pragma warning 586
#else
#warning Not supported compiler type
#endif

// Timer CASCADEx register selection codes
typedef enum {
  en_tim_cas_src_pa0 = 0,
  en_tim_cas_src_pa1 = 1,
  en_tim_cas_src_pa2 = 2,
  en_tim_cas_src_pa3 = 3,
  en_tim_cas_src_pa4 = 4,
  en_tim_cas_src_pa5 = 5,
  en_tim_cas_src_pa6 = 6,
  en_tim_cas_src_pa7 = 7,
  en_tim_cas_src_pa8 = 8,
  en_tim_cas_src_pa9 = 9,
  en_tim_cas_src_pa10 = 10,
  en_tim_cas_src_pa11 = 11,
  en_tim_cas_src_pa12 = 12,
  en_tim_cas_src_pa13 = 13,
  en_tim_cas_src_pa14 = 14,
  en_tim_cas_src_pa15 = 15,
  en_tim_cas_src_pb0 = 16,
  en_tim_cas_src_pb1 = 17,
  en_tim_cas_src_pb2 = 18,
  en_tim_cas_src_pb3 = 19,
  en_tim_cas_src_pb4 = 20,
  en_tim_cas_src_pb5 = 21,
  en_tim_cas_src_pb6 = 22,
  en_tim_cas_src_pb7 = 23,
  en_tim_cas_src_pb8 = 24,
  en_tim_cas_src_pb9 = 25,
  en_tim_cas_src_pb10 = 26,
  en_tim_cas_src_pb11 = 27,
  en_tim_cas_src_pb12 = 28,
  en_tim_cas_src_pb13 = 29,
  en_tim_cas_src_pb14 = 30,
  en_tim_cas_src_pb15 = 31,
  en_tim_cas_src_pc0 = 32,
  en_tim_cas_src_pc1 = 33,
  en_tim_cas_src_pc2 = 34,
  en_tim_cas_src_pc3 = 35,
  en_tim_cas_src_pc4 = 36,
  en_tim_cas_src_pc5 = 37,
  en_tim_cas_src_pc6 = 38,
  en_tim_cas_src_pc7 = 39,
  en_tim_cas_src_pc8 = 40,
  en_tim_cas_src_pc9 = 41,
  en_tim_cas_src_pc10 = 42,
  en_tim_cas_src_pc11 = 43,
  en_tim_cas_src_pc12 = 44,
  en_tim_cas_src_pc13 = 45,
  en_tim_cas_src_pc14 = 46,
  en_tim_cas_src_pc15 = 47,
  en_tim_cas_src_pd0 = 48,
  en_tim_cas_src_pd1 = 49,
  en_tim_cas_src_pd2 = 50,
  en_tim_cas_src_pd3 = 51,
  en_tim_cas_src_pd4 = 52,
  en_tim_cas_src_pd5 = 53,
  en_tim_cas_src_pd6 = 54,
  en_tim_cas_src_pd7 = 55,
  en_tim_cas_src_pd8 = 56,
  en_tim_cas_src_pd9 = 57,
  en_tim_cas_src_pd10 = 58,
  en_tim_cas_src_pd11 = 59,
  en_tim_cas_src_pd12 = 60,
  en_tim_cas_src_pd13 = 61,
  en_tim_cas_src_pd14 = 62,
  en_tim_cas_src_pd15 = 63,
  en_tim_cas_src_pe0 = 64,
  en_tim_cas_src_pe1 = 65,
  en_tim_cas_src_pe2 = 66,
  en_tim_cas_src_pe3 = 67,
  en_tim_cas_src_pe4 = 68,
  en_tim_cas_src_pe5 = 69,
  en_tim_cas_src_pe6 = 70,
  en_tim_cas_src_pe7 = 71,
  en_tim_cas_src_pe8 = 72,
  en_tim_cas_src_pe9 = 73,
  en_tim_cas_src_pe10 = 74,
  en_tim_cas_src_pe11 = 75,
  en_tim_cas_src_pe12 = 76,
  en_tim_cas_src_pe13 = 77,
  en_tim_cas_src_pe14 = 78,
  en_tim_cas_src_pe15 = 79,
  en_tim_cas_src_tim0 = 80,
  en_tim_cas_src_tim1 = 81,
  en_tim_cas_src_tim2 = 82,
  en_tim_cas_src_tim3 = 83,
  en_tim_cas_src_tim4 = 84,
  en_tim_cas_src_tim5 = 85,
  en_tim_cas_src_tim6 = 86,
  en_tim_cas_src_tim7 = 87,
  en_tim_cas_src_tim8 = 88,
  en_tim_cas_src_tim9 = 89,
  en_tim_cas_src_tim10 = 90,
  en_tim_cas_src_tim11 = 91,
  en_tim_cas_src_tim12 = 92,
  en_tim_cas_src_tim13 = 93,
  en_tim_cas_src_tim14 = 94,
  en_tim_cas_src_tim15 = 95,
  en_tim_cas_src_tim16 = 96,
  en_tim_cas_src_tim17 = 97,
  en_tim_cas_src_tim18 = 98,
  en_tim_cas_src_tim19 = 99,
  en_tim_cas_src_tim20 = 100,
  en_tim_cas_src_tim21 = 101,
  en_tim_cas_src_tim22 = 102,
  en_tim_cas_src_tim23 = 103,
  en_tim_cas_src_txev = 104,
  en_tim_cas_src_adc_irq = 105,
  en_tim_cas_src_rom_sbe = 106,
  en_tim_cas_src_rom_mbe = 107,
  en_tim_cas_src_ram0_sbe = 108,
  en_tim_cas_src_ram0_mbe = 109,
  en_tim_cas_src_ram1_sbe = 110,
  en_tim_cas_src_ram1_mbe = 111,
  en_tim_cas_src_wdog_irq = 112
} en_tim_cas_src_t;

typedef enum {
  en_tim_status_sel_pulse = 0,
  en_tim_status_sel_activebit = 1,
  en_tim_status_sel_toggle = 2,
  en_tim_status_sel_pwma = 3,
  en_tim_status_sel_pwmb = 4,
  en_tim_status_sel_enabled = 5,
  en_tim_status_sel_pwma_active = 6
} en_tim_status_sel_t;

/** Timer configuration struct, for HAL_Timer_InitTimer() */
typedef struct {
  en_tim_status_sel_t status_sel;  // timer status select bits CTRL 7:5
  bool status_inv;                 // true = invert the value selected by status_sel
  bool irq_en;                     // true = enable interrupt on count reaching zero
                                   // lets TIM generate int, still needs enable in NVIC
  bool auto_deactivate;            // true = auto-deactivate when count reaches zero
  bool auto_disable;               // true = auto-disable when count reaches zero
  uint32_t rst_value;              // timer reset value
  uint32_t cnt_value;              // initial timer count value
  uint32_t pwma_value;             // timer PWMA value
  uint32_t pwmb_value;             // timer PWMB value
} stc_tim_cfg_t;

/** CSD_CTRL register bitfields */
typedef struct {
  uint32_t csden0 : 1;    // 1 = counter only counts when selected cascade0 signal is active
  uint32_t csdinv0 : 1;   // 1 = invert cascade0 signal (active low). 0 = active high
  uint32_t csden1 : 1;    // 1 = counter only counts when selected cascade1 signal is active
  uint32_t csdinv1 : 1;   // 1 = invert cascade1 signal (active low). 0 = active high
  uint32_t dcasop : 1;    // Dual cascade, 0 = logical AND, 1 = logical OR of cascade0/1 signals
  uint32_t reserved : 1;  // Bit 5 reserved, reads as 0
  uint32_t csdtrg0 : 1;   // 1 = Cascade0 trigger mode. Once running, cascade control is ignored
  uint32_t csdtrg1 : 1;   // 1 = Cascade1 trigger mode. Once running, cascade control is ignored
  uint32_t csden2 : 1;    // 1 = counter wil stop when selected cascade2 signal is active
  uint32_t csdinv2 : 1;   // 1 = invert cascade2 signal (active low). 0 = active high
  uint32_t csdtrg2 : 1;   // 1 = Cascade2 trigger mode. Timer will stop when when counter reaches 0,
                          // and the level-sensitive cascade2 signal is active.
  uint32_t : 21;          // Reserved reads as 0
} stc_tim_cascade_ctrl_t;

/** Cascade settings struct, for HAL_Timer_SetupCascade() */
typedef struct {
  union {
    stc_tim_cascade_ctrl_t ctrl;
    uint32_t ctrl_raw;
  };
  en_tim_cas_src_t cascade0;
  en_tim_cas_src_t cascade1;
  en_tim_cas_src_t cascade2;
} stc_tim_cascade_cfg_t;

/** Percent type for PWM duty cycle */
typedef uint32_t percent_1000_t;  // percent times 1000 - 100% == 100000

/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
#pragma pop
#elif defined(__ICCARM__)
/* leave anonymous unions enabled */
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
#pragma warning restore
#else
#warning Not supported compiler type
#endif

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

// Resolves APB1 / APB2 banking issues
extern VOR_TIM_Type* const VOR_TIM_BANK[24];

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern hal_status_t HAL_Timer_ResetTimer(uint32_t timerNum);
extern hal_status_t HAL_Timer_InitTimer(uint32_t timerNum, stc_tim_cfg_t cfg, bool enableAfterInit);
extern hal_status_t HAL_Timer_SetupCascade(uint32_t timerNum, stc_tim_cascade_cfg_t cascade);
extern hal_status_t HAL_Timer_SetupPeriodicIrqMs(uint32_t timerNum, uint32_t periodMsec,
                                                 uint32_t priority);
extern hal_status_t HAL_Timer_SetupPeriodicIrqUs(uint32_t timerNum, uint32_t periodUsec,
                                                 uint32_t priority);
extern hal_status_t HAL_Timer_SetupTimeoutMs(uint32_t timerNum, uint32_t timeoutMsec);
extern hal_status_t HAL_Timer_SetDelayTimer(uint32_t timerNum);
extern hal_status_t HAL_Timer_DelayMs(uint32_t delayMsec);
extern hal_status_t HAL_Timer_SetupPWMA(uint32_t timerNum, uint32_t freqHz,
                                        percent_1000_t dutyCycle);
extern hal_status_t HAL_Timer_SetPWMFreq(uint32_t timerNum, uint32_t freqHz);
extern hal_status_t HAL_Timer_SetPWMADutyCycle(uint32_t timerNum, percent_1000_t dutyCycle);
extern hal_status_t HAL_Timer_EnableIrq(uint32_t timerNum, uint32_t priority);
extern hal_status_t HAL_Timer_DisableIrq(uint32_t timerNum);
extern hal_status_t HAL_Timer_DeInitTimer(uint32_t timerNum);

extern bool HAL_Timer_IsTimerActive(uint32_t timerNum);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_TIMER_H */
