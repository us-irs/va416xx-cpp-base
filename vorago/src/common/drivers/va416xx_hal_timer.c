/***************************************************************************************
 * @file     va416xx_hal_timer.c
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

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal_timer.h"

#include "va416xx_debug.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define TIMER_PERID (0x021107E9)

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

#undef VOR_TIM_BANK
VOR_TIM_Type* const VOR_TIM_BANK[24] = {
    (VOR_TIM_Type*)VOR_TIM0_BASE,  (VOR_TIM_Type*)VOR_TIM1_BASE,  (VOR_TIM_Type*)VOR_TIM2_BASE,
    (VOR_TIM_Type*)VOR_TIM3_BASE,  (VOR_TIM_Type*)VOR_TIM4_BASE,  (VOR_TIM_Type*)VOR_TIM5_BASE,
    (VOR_TIM_Type*)VOR_TIM6_BASE,  (VOR_TIM_Type*)VOR_TIM7_BASE,  (VOR_TIM_Type*)VOR_TIM8_BASE,
    (VOR_TIM_Type*)VOR_TIM9_BASE,  (VOR_TIM_Type*)VOR_TIM10_BASE, (VOR_TIM_Type*)VOR_TIM11_BASE,
    (VOR_TIM_Type*)VOR_TIM12_BASE, (VOR_TIM_Type*)VOR_TIM13_BASE, (VOR_TIM_Type*)VOR_TIM14_BASE,
    (VOR_TIM_Type*)VOR_TIM15_BASE, (VOR_TIM_Type*)VOR_TIM16_BASE, (VOR_TIM_Type*)VOR_TIM17_BASE,
    (VOR_TIM_Type*)VOR_TIM18_BASE, (VOR_TIM_Type*)VOR_TIM19_BASE, (VOR_TIM_Type*)VOR_TIM20_BASE,
    (VOR_TIM_Type*)VOR_TIM21_BASE, (VOR_TIM_Type*)VOR_TIM22_BASE, (VOR_TIM_Type*)VOR_TIM23_BASE};

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/** Delay timer bank - for DelayMs() and DelayUs() */
static int32_t delayTimerBank = -1;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Enable clock gate and reset a timer's registers to default (not running)
 **
 ** @param  timerNum - the timer to reset
 **
 ******************************************************************************/
hal_status_t HAL_Timer_ResetTimer(uint32_t timerNum) {
  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    return hal_status_badParam;
  }

  HAL_Timer_DisableIrq(timerNum);
  VOR_SYSCONFIG->TIM_CLK_ENABLE |= (1UL << timerNum);

  c_assert(VOR_TIM_BANK[timerNum]->PERID == TIMER_PERID);
  if (TIMER_PERID != VOR_TIM_BANK[timerNum]->PERID) {
    // something is wrong
    return hal_status_badPeriphID;
  }

  // reset the timer
  VOR_SYSCONFIG->TIM_RESET &= ~(1UL << timerNum);
  __NOP();
  __NOP();
  VOR_SYSCONFIG->TIM_RESET |= (1UL << timerNum);

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Enables clock gate, resets, and then initializes a timer
 **
 ** @param  timerNum - the timer to init
 **
 ** @param  cfg - timer configuration structure
 **
 ** @param  enableAfterInit - if true, enabes the timer ENABLE bit after setup completion
 **
 ******************************************************************************/
hal_status_t HAL_Timer_InitTimer(uint32_t timerNum, stc_tim_cfg_t cfg, bool enableAfterInit) {
  hal_status_t status = HAL_Timer_ResetTimer(timerNum);
  c_assert(hal_status_ok == status);
  if (hal_status_ok != status) {
    return status;
  }

  // setup the timer
  c_assert(timerNum < HAL_NUM_TIMERS);

  VOR_TIM_BANK[timerNum]->CTRL = ((uint32_t)cfg.status_sel) << TIM_CTRL_STATUS_SEL_Pos;
  VOR_TIM_BANK[timerNum]->CTRL |= ((cfg.status_inv) ? (TIM_CTRL_STATUS_INV_Msk) : 0);
  VOR_TIM_BANK[timerNum]->CTRL |= ((cfg.irq_en) ? (TIM_CTRL_IRQ_ENB_Msk) : 0);
  VOR_TIM_BANK[timerNum]->CTRL |= ((cfg.auto_deactivate) ? (TIM_CTRL_AUTO_DEACTIVATE_Msk) : 0);
  VOR_TIM_BANK[timerNum]->CTRL |= ((cfg.auto_disable) ? (TIM_CTRL_AUTO_DISABLE_Msk) : 0);
  VOR_TIM_BANK[timerNum]->RST_VALUE = cfg.rst_value;
  VOR_TIM_BANK[timerNum]->CNT_VALUE = cfg.cnt_value;
  VOR_TIM_BANK[timerNum]->PWMA_VALUE = cfg.pwma_value;
  VOR_TIM_BANK[timerNum]->PWMB_VALUE = cfg.pwmb_value;

  // enable
  if (enableAfterInit) {
    VOR_TIM_BANK[timerNum]->ENABLE = 1;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets up a timer's cascade registers
 **
 ** @param  timerNum - the timer number to set up cascade (0-23)
 **
 ** @param  cascade - the timer cascade settings struct
 **
 ** @note   Must call HAL_Timer_InitTimer() or HAL_Timer_ResetTimer() first before this,
 **           to ensure timer's clock is enabled. A call to init() or reset() will reset the
 **           cascade registers. To init a timer but not have it run until the cascade is
 **           set up, call init() with enableAfterInit false, set up cascade, then enable.
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetupCascade(uint32_t timerNum, stc_tim_cascade_cfg_t cascade) {
  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    return hal_status_badParam;
  }

  // ensure clock is enabled
  c_assert((VOR_SYSCONFIG->TIM_CLK_ENABLE & (1UL << timerNum)) > 0);
  if ((VOR_SYSCONFIG->TIM_CLK_ENABLE & (1UL << timerNum)) == 0) {
    // timer not clocked - call HAL_Timer_InitTimer() or HAL_Timer_ResetTimer() first
    return hal_status_initError;
  }

  VOR_TIM_BANK[timerNum]->CASCADE0 = (uint32_t)cascade.cascade0;
  VOR_TIM_BANK[timerNum]->CASCADE1 = (uint32_t)cascade.cascade1;
  VOR_TIM_BANK[timerNum]->CASCADE2 = (uint32_t)cascade.cascade2;
  VOR_TIM_BANK[timerNum]->CSD_CTRL = cascade.ctrl_raw;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets up a periodic interrupt
 **
 ** @param  timerNum - the timer to use (0-23)
 **
 ** @param  periodMsec - interrupt period (in milliseconds)
 **
 ** @param  priority - NVIC IRQ priority
 **
 ** @return hal_status_t - the status of the driver call
 **
 ** @note   Must define the timer ISR in user application
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetupPeriodicIrqMs(uint32_t timerNum, uint32_t periodMsec,
                                          uint32_t priority) {
  hal_status_t status = HAL_Timer_InitTimer(
      timerNum,
      (stc_tim_cfg_t){
          .irq_en = true, .rst_value = (TIMER_CLK(timerNum) / 1000) * periodMsec, .cnt_value = 0},
      false);

  // check init
  c_assert(hal_status_ok == status);
  if (hal_status_ok != status) {
    return status;
  }

  // enable irq in NVIC and enable timer
  HAL_Timer_EnableIrq(timerNum, priority);
  VOR_TIM_BANK[timerNum]->ENABLE = 1;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets up a periodic interrupt
 **
 ** @param  timerNum - the timer to use (0-23)
 **
 ** @param  periodUsec - interrupt period (in microseconds)
 **
 ** @param  priority - NVIC IRQ priority
 **
 ** @return hal_status_t - the status of the driver call
 **
 ** @note   Must define the timer ISR in user application
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetupPeriodicIrqUs(uint32_t timerNum, uint32_t periodUsec,
                                          uint32_t priority) {
  hal_status_t status =
      HAL_Timer_InitTimer(timerNum,
                          (stc_tim_cfg_t){.irq_en = true,
                                          .rst_value = (TIMER_CLK(timerNum) / 1000000) * periodUsec,
                                          .cnt_value = 0},
                          false);

  // check init
  c_assert(hal_status_ok == status);
  if (hal_status_ok != status) {
    return status;
  }

  // enable irq in NVIC and enable timer
  HAL_Timer_EnableIrq(timerNum, priority);
  VOR_TIM_BANK[timerNum]->ENABLE = 1;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets up a countdown - the timer will count down and reach zero (and
 **           then become inactive) after a specified number of milliseconds.
 **           Used with HAL_Timer_IsTimerActive() to test if the timer is still active
 **
 ** @param  timerNum - the timer to use (0-23)
 **
 ** @param  timeoutMsec - the time for the timer to reach zero and stop, in milliseconds
 **
 ** @return hal_status_t - the status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetupTimeoutMs(uint32_t timerNum, uint32_t timeoutMsec) {
  hal_status_t status =
      HAL_Timer_InitTimer(timerNum,
                          (stc_tim_cfg_t){.auto_disable = true,
                                          .rst_value = (TIMER_CLK(timerNum) / 1000) * timeoutMsec,
                                          .cnt_value = 0},
                          true);

  // check init
  c_assert(hal_status_ok == status);
  return status;
}

/*******************************************************************************
 **
 ** @brief  Sets up a countdown - the timer will count down and reach zero (and
 **           then become inactive) after a specified number of microseconds.
 **           Used with HAL_Timer_IsTimerActive() to test if the timer is still active
 **
 ** @param  timerNum - the timer to use (0-23)
 **
 ** @param  timeoutUsec - the time for the timer to reach zero and stop, in microseconds
 **
 ** @return hal_status_t - the status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetupTimeoutUs(uint32_t timerNum, uint32_t timeoutUsec) {
  hal_status_t status = HAL_Timer_InitTimer(
      timerNum,
      (stc_tim_cfg_t){.auto_disable = true,
                      .rst_value = (TIMER_CLK(timerNum) / 1000000) * timeoutUsec,
                      .cnt_value = 0},
      true);

  // check init
  c_assert(hal_status_ok == status);
  return status;
}

/*******************************************************************************
 **
 ** @brief  Sets the delay timer bank
 **
 ** @param  timerNum - the timer to use for DelayMs() (0-23)
 **
 ** @return hal_status_t - the status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetDelayTimer(uint32_t timerNum) {
  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    delayTimerBank = -1;
    return hal_status_badParam;
  }
  delayTimerBank = timerNum;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Waits for a specified number of milliseconds
 **
 ** @param  delayMsec - number of milliseconds to wait
 **
 ** @return hal_status_t - the status of the driver call
 **
 ** @note   Requires HAL_Timer_SetDelayTimer() to be called first to set which timer to use
 **
 ******************************************************************************/
hal_status_t HAL_Timer_DelayMs(uint32_t delayMsec) {
  hal_status_t status;

  c_assert((delayTimerBank >= 0) && (delayTimerBank < HAL_NUM_TIMERS));
  if ((delayTimerBank >= 0) && (delayTimerBank < HAL_NUM_TIMERS)) {
    status = HAL_Timer_SetupTimeoutMs((uint32_t)delayTimerBank, delayMsec);
    c_assert(status == hal_status_ok);
    if (status == hal_status_ok) {
      while (true == HAL_Timer_IsTimerActive((uint32_t)delayTimerBank)) {
        // TODO: feed watchdog
      }
    }
    return status;
  }
  return hal_status_notInitialized;  // need to call HAL_Timer_SetDelayTimer() first
}

/*******************************************************************************
 **
 ** @brief  Setup a PWM using a specified timer - GPIO must be selected to the timer alt function
 **
 ** @param  timerNum - the timer to use - must match timer for pin alt function (0-23)
 **
 ** @param  freqHz - PWM frequency
 **
 ** @param  dutyCycle - Initial PWM duty cycle (percent times 1000) - 0% = 0, 100% == 100000
 **
 ** @return hal_status_t - the status of the driver call
 **
 ** @note   To change the duty cycle later, use HAL_Timer_SetPWMADutyCycle().
 **         To change the frequency later, call HAL_Timer_SetPWMFreq() and then
 **           HAL_Timer_SetPWMADutyCycle().
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetupPWMA(uint32_t timerNum, uint32_t freqHz, percent_1000_t dutyCycle) {
  hal_status_t status;
  uint64_t pwma_val;

  // check frequency (must be nonzero)
  c_assert(0 != freqHz);
  if (0 == freqHz) {
    return hal_status_badParam;
  }
  pwma_val = (uint64_t)(TIMER_CLK(timerNum) / freqHz) * (uint64_t)(TIMER_PWM_DUTY_MAX - dutyCycle);
  pwma_val /= (uint64_t)TIMER_PWM_DUTY_MAX;
  status = HAL_Timer_InitTimer(timerNum,
                               (stc_tim_cfg_t){.status_sel = en_tim_status_sel_pwma,
                                               .rst_value = (TIMER_CLK(timerNum) / freqHz),
                                               .cnt_value = 0,
                                               .pwma_value = (uint32_t)pwma_val},
                               true);

  // check init
  c_assert(hal_status_ok == status);
  return status;
}

/*******************************************************************************
 **
 ** @brief  Sets a timer's PWM frequency when in PWM mode
 **
 ** @param  timerNum - the timer to set PWM freq (0-23)
 **
 ** @param  freqHz - PWM frequency
 **
 ** @return hal_status_t - the status of the driver call
 **
 ** @note   Must call HAL_Timer_InitTimer() or HAL_Timer_SetupPWMA() first before this,
 **           to ensure timer's clock is enabled. This function will not readjust the
 **           PWMA register, so call HAL_Timer_SetPWMADutyCycle() after changing the
 **           freq to ensure the desired duty cycle is still correct.
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetPWMFreq(uint32_t timerNum, uint32_t freqHz) {
  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    return hal_status_badParam;
  }

  // ensure clock is enabled
  c_assert((VOR_SYSCONFIG->TIM_CLK_ENABLE & (1UL << timerNum)) > 0);
  if ((VOR_SYSCONFIG->TIM_CLK_ENABLE & (1UL << timerNum)) == 0) {
    // timer not clocked - call HAL_Timer_InitTimer() or HAL_Timer_SetupPWMA() first
    return hal_status_initError;
  }

  // check frequency (must be nonzero)
  c_assert(0 != freqHz);
  if (0 == freqHz) {
    return hal_status_badParam;
  }

  // set frequency
  VOR_TIM_BANK[timerNum]->RST_VALUE = (TIMER_CLK(timerNum) / freqHz);

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets a timer's PWM duty cycle when in PWMA mode
 **
 ** @param  timerNum - the timer to set duty cycle (0-23)
 **
 ** @param  dutyCycle - PWM duty cycle (percent times 1000) - 0% = 0, 100% == 100000
 **
 ** @return hal_status_t - the status of the driver call
 **
 ** @note   Must call HAL_Timer_InitTimer() or HAL_Timer_SetupPWMA() first before this,
 **           to ensure timer's clock is enabled. If changing a timer's PWM frequency,
 **           call this afterwards to correct the PWMA value after freq change.
 **
 ******************************************************************************/
hal_status_t HAL_Timer_SetPWMADutyCycle(uint32_t timerNum, percent_1000_t dutyCycle) {
  uint64_t pwma_val;

  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    return hal_status_badParam;
  }

  // ensure clock is enabled
  c_assert((VOR_SYSCONFIG->TIM_CLK_ENABLE & (1UL << timerNum)) > 0);
  if ((VOR_SYSCONFIG->TIM_CLK_ENABLE & (1UL << timerNum)) == 0) {
    // timer not clocked - call HAL_Timer_InitTimer() or HAL_Timer_SetupPWMA() first
    return hal_status_initError;
  }

  // calculate and set PWMA_VALUE
  pwma_val =
      (uint64_t)(VOR_TIM_BANK[timerNum]->RST_VALUE) * (uint64_t)(TIMER_PWM_DUTY_MAX - dutyCycle);
  pwma_val /= (uint64_t)TIMER_PWM_DUTY_MAX;
  VOR_TIM_BANK[timerNum]->PWMA_VALUE = (uint32_t)pwma_val;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Enable an IRQ for a specified timer
 **
 ** @param  timerNum - the timer to enable the IRQ for (0-23)
 **
 ** @param  priority - the interrupt priority for this IRQ
 **
 ** @return hal_status_t - the status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Timer_EnableIrq(uint32_t timerNum, uint32_t priority) {
  IRQn_Type irq;

  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    return hal_status_badParam;
  }
  c_assert(priority <= HAL_NVIC_MAX_PRIO);
  if (false == (priority <= HAL_NVIC_MAX_PRIO)) {
    return hal_status_badParam;
  }
  irq = (IRQn_Type)(TIM0_IRQn + timerNum);
  NVIC_SetPriority(irq, priority);
  NVIC_EnableIRQ(irq);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Disable an IRQ for a specified timer
 **
 ** @param  timerNum - the timer to disable the IRQ for (0-23)
 **
 ** @return hal_status_t - the status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Timer_DisableIrq(uint32_t timerNum) {
  IRQn_Type irq;

  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    return hal_status_badParam;
  }
  irq = (IRQn_Type)(TIM0_IRQn + timerNum);
  NVIC_DisableIRQ(irq);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Disable and declock a timer. Also disables the IRQ
 **
 ** @param  timerNum - the timer to de-initialize (0-23)
 **
 ** @return hal_status_t - the status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_Timer_DeInitTimer(uint32_t timerNum) {
  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    return hal_status_badParam;
  }

  // check perid
  c_assert(VOR_TIM_BANK[timerNum]->PERID == TIMER_PERID);

  // de-init
  VOR_TIM_BANK[timerNum]->ENABLE = 0;
  VOR_SYSCONFIG->TIM_CLK_ENABLE &= ~(1UL << timerNum);
  HAL_Timer_DisableIrq(timerNum);

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Returns true if the the specified timer is running (counting), else false
 **
 ** @param  timerNum - the timer to check active status (0-23)
 **
 ** @return bool - timer is active (true) or timer is not active (false)
 **
 ******************************************************************************/
bool HAL_Timer_IsTimerActive(uint32_t timerNum) {
  // check timer number
  c_assert(timerNum < HAL_NUM_TIMERS);
  if (false == (timerNum < HAL_NUM_TIMERS)) {
    // index out of range
    return false;
  }

  // check clock enable
  c_assert((VOR_SYSCONFIG->TIM_CLK_ENABLE & (1UL << timerNum)) > 0);
  if ((VOR_SYSCONFIG->TIM_CLK_ENABLE & (1UL << timerNum)) == 0) {
    // if clock is not enabled, the timer is not running
    return false;
  }

  // check active flag
  if (VOR_TIM_BANK[timerNum]->CTRL & TIM_CTRL_ACTIVE_Msk) {
    return true;
  }
  return false;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
