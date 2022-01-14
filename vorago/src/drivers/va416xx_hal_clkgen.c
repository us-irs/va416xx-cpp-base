/***************************************************************************************
 * @file     va416xx_hal_clkgen.c
 * @version  V0.4.1
 * @date     24 Nov 2020
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

#include "va416xx_hal_clkgen.h"

#include <math.h>
#include <stdlib.h>

#include "va416xx_debug.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define XTAL_OSC_TSTART_MS (15)
#define ONE_MHZ (1000000)

// for test purposes - enables output SYSCLK/4 over SPI3_SCK (ROM_SCK)
//#define __SPI3_SYSCLK_4

// if defined, output detailed PLL config information
//#define __PLL_DEBUG_LOG

#ifndef __PLL_DEBUG_LOG
#undef dbgprintf
#undef dbgprintln
#define dbgprintf(...) ((void)0)
#define dbgprintln(...) ((void)0)
#endif

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static void rearm_sysclk_lost(void);
#if 0
static void rearm_pll_lock_lost(void);
#endif
static void hbo_clk_wait_ms(uint32_t ms);
static void delay(void);
static int calcPLL(uint32_t inFreqHz, uint32_t outFreqHz);

#ifdef __SPI3_SYSCLK_4
static void spi3_maxclock(bool en);
#endif

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Enable and rearm system clock lost detection
 **
 ** @return none
 **
 ******************************************************************************/
static void rearm_sysclk_lost(void) {
  VOR_CLKGEN->CTRL0 |= CLKGEN_CTRL0_SYS_CLK_LOST_DET_EN_Msk;
  VOR_CLKGEN->CTRL1 |= CLKGEN_CTRL1_SYS_CLK_LOST_DET_REARM_Msk;
  VOR_CLKGEN->CTRL1 &= ~CLKGEN_CTRL1_SYS_CLK_LOST_DET_REARM_Msk;
}

// Enable and rearm PLL lock lost detection
// *** not working on VA416x0_REVA ***
#ifdef __MCU_HW_VER_REVB
static void rearm_pll_lock_lost(void) {
  VOR_CLKGEN->CTRL1 |= CLKGEN_CTRL1_PLL_LOST_LOCK_DET_EN_Msk;
  VOR_CLKGEN->CTRL1 |= CLKGEN_CTRL1_PLL_LCK_DET_REARM_Msk;
  VOR_CLKGEN->CTRL1 &= ~CLKGEN_CTRL1_PLL_LCK_DET_REARM_Msk;
}
#endif

/*******************************************************************************
 **
 ** @brief  Wait a specified number of milliseconds (approx on HBO clk)
 **
 ** @param  ms - number of milliseconds to wait
 **
 ** @return none
 **
 ******************************************************************************/
static void hbo_clk_wait_ms(uint32_t ms) {
  uint32_t i;
  while (ms) {
    for (i = 0; i < 10000; i++) {
      __NOP();
    }
    ms--;
    VOR_WATCH_DOG->WDOGINTCLR = 1;
  }
}

/*******************************************************************************
 **
 ** @brief  Wait for at least 500 SYSCLK cycles
 **
 ** @return none
 **
 ******************************************************************************/
static void delay(void) {
  uint32_t i;
  for (i = 0; i < 500; i++) {
    __NOP();
  }
}

/*******************************************************************************
 **
 ** @brief  Initializes the system clock source
 **
 ** @param  clkConfig - clock setup struct
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Clkgen_Init(hal_clkgen_init_t clkConfig) {
  uint32_t pllInFreq;
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= (CLK_ENABLE_IRQ | CLK_ENABLE_CLKGEN);
  VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_CLKSEL_SYS_Msk;  // force to HBO clk
  delay();
  VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_CLK_DIV_SEL_Msk;  // select 1x divide

  // setup oscillator, PLL input clk
  c_assert((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_CLKSEL_SYS_Msk) == 0);  // ensure on HBO
  switch (clkConfig.xtalsel) {
    case hal_xtalsel_none:
      VOR_CLKGEN->CTRL1 &= ~CLKGEN_CTRL1_XTAL_EN_Msk;
      VOR_CLKGEN->CTRL1 &= ~CLKGEN_CTRL1_XTAL_N_EN_Msk;
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_REF_CLK_SEL_Msk;
      delay();
      break;
    case hal_xtalsel_xtal_en:
      VOR_CLKGEN->CTRL1 &= ~CLKGEN_CTRL1_XTAL_N_EN_Msk;
      VOR_CLKGEN->CTRL1 |= CLKGEN_CTRL1_XTAL_EN_Msk;
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_REF_CLK_SEL_Msk;
      VOR_CLKGEN->CTRL0 |= 1UL << CLKGEN_CTRL0_REF_CLK_SEL_Pos;
      hbo_clk_wait_ms(XTAL_OSC_TSTART_MS);
      break;
    case hal_xtalsel_xtal_n_en:
      VOR_CLKGEN->CTRL1 &= ~CLKGEN_CTRL1_XTAL_EN_Msk;
      VOR_CLKGEN->CTRL1 |= CLKGEN_CTRL1_XTAL_N_EN_Msk;
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_REF_CLK_SEL_Msk;
      VOR_CLKGEN->CTRL0 |= 2UL << CLKGEN_CTRL0_REF_CLK_SEL_Pos;
      delay();
      break;
  }

  // setup PLL
  switch (clkConfig.pllcfg) {
    case hal_pllcfg_pwrdn:
      VOR_CLKGEN->CTRL0 |= CLKGEN_CTRL0_PLL_PWDN_Msk;      // power down PLL
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_REF_CLK_SEL_Msk;  // no CLK to PLL
      break;
    case hal_pllcfg_enabled:
      // set up PLL multiplier
      c_assert(clkConfig.xtalsel != hal_xtalsel_none);
      if (clkConfig.xtalsel == hal_xtalsel_none) {
        return hal_status_badParam;
      }
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_PLL_PWDN_Msk;  // power up PLL

      __NOP();
      __NOP();

      if (clkConfig.pll_out_mhz < PLLOUT_MIN_MHZ) {
        clkConfig.pll_out_mhz = PLLOUT_MIN_MHZ;
      }
      if (clkConfig.pll_out_mhz > PLLOUT_MAX_MHZ) {
        clkConfig.pll_out_mhz = PLLOUT_MAX_MHZ;
      }
      if (clkConfig.xtalsel == hal_xtalsel_xtal_en) {
        pllInFreq = VOR_XTAL;
      } else {
        pllInFreq = VOR_EXTCLK;
      }
      if (calcPLL(pllInFreq, clkConfig.pll_out_mhz * ONE_MHZ) != 0) {
        return hal_status_initError;
      }

      // setup other control bits (not in testmode, no bypass, internal FB)
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_PLL_TEST_Msk;
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_PLL_BYPASS_Msk;
      VOR_CLKGEN->CTRL0 |= CLKGEN_CTRL0_PLL_INTFB_Msk;  // always set this

      // reset PLL
      VOR_CLKGEN->CTRL0 |= CLKGEN_CTRL0_PLL_RESET_Msk;
      delay();
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_PLL_RESET_Msk;
      delay();

      // check for lock
      if (VOR_CLKGEN->STAT & (CLKGEN_STAT_FBSLIP_Msk | CLKGEN_STAT_RFSLIP_Msk)) {
        // not locked - delay, and check again
        delay();
        if (VOR_CLKGEN->STAT & (CLKGEN_STAT_FBSLIP_Msk | CLKGEN_STAT_RFSLIP_Msk)) {
          // there is a problem - stay on HBO clk and report an error condition
          return hal_status_initError;
        }
      }
      break;
    case hal_pllcfg_bypass:
      // setup PLL in bypass
      c_assert(clkConfig.xtalsel != hal_xtalsel_none);
      if (clkConfig.xtalsel == hal_xtalsel_none) {
        return hal_status_badParam;
      }
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_PLL_PWDN_Msk;  // power up PLL
      __NOP();
      __NOP();

      // setup other control bits (not in testmode, no bypass, internal FB)
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_PLL_TEST_Msk;
      VOR_CLKGEN->CTRL0 |= CLKGEN_CTRL0_PLL_BYPASS_Msk;  // bypass
      VOR_CLKGEN->CTRL0 |= CLKGEN_CTRL0_PLL_INTFB_Msk;   // always set this

      // reset PLL
      VOR_CLKGEN->CTRL0 |= CLKGEN_CTRL0_PLL_RESET_Msk;
      delay();
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_PLL_RESET_Msk;
      delay();

      break;
  }

  // setup loss of clock detection
  if (true == clkConfig.lost_det_en) {
    rearm_sysclk_lost();
#ifdef __MCU_HW_VER_REVB
    if (clkConfig.pllcfg == hal_pllcfg_enabled) {
      rearm_pll_lock_lost();
    }
#endif
  }

  delay();

  // setup CLK_DIV
  switch (clkConfig.clk_div_sel) {
    case hal_clk_div_1x:
      VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_CLK_DIV_SEL_Msk;
      break;
    case hal_clk_div_2x:
      VOR_CLKGEN->CTRL0 |= 1 << CLKGEN_CTRL0_CLK_DIV_SEL_Pos;
      VOR_CLKGEN->CTRL0 &= ~(2 << CLKGEN_CTRL0_CLK_DIV_SEL_Pos);
      break;
    case hal_clk_div_4x:
      VOR_CLKGEN->CTRL0 |= 2 << CLKGEN_CTRL0_CLK_DIV_SEL_Pos;
      VOR_CLKGEN->CTRL0 &= ~(1 << CLKGEN_CTRL0_CLK_DIV_SEL_Pos);
      break;
    case hal_clk_div_8x:
      VOR_CLKGEN->CTRL0 |= CLKGEN_CTRL0_CLK_DIV_SEL_Msk;
      break;
  }

  delay();

  // setup CLKSEL
  switch (clkConfig.clksel) {
    case hal_clksel_sys_hbo:
      // already on HBO
      break;
    case hal_clksel_sys_xtal_n:
      c_assert(clkConfig.xtalsel == hal_xtalsel_xtal_n_en);
      if (clkConfig.xtalsel != hal_xtalsel_xtal_n_en) {
        return hal_status_badParam;
      }
      VOR_CLKGEN->CTRL0 |= 1UL << CLKGEN_CTRL0_CLKSEL_SYS_Pos;  // select XTAL minus to sysclk
      break;
    case hal_clksel_sys_pll:
      c_assert(clkConfig.pllcfg != hal_pllcfg_pwrdn);
      if (clkConfig.pllcfg == hal_pllcfg_pwrdn) {
        return hal_status_badParam;
      }
      VOR_CLKGEN->CTRL0 |= 2UL << CLKGEN_CTRL0_CLKSEL_SYS_Pos;  // select PLL to sysclk
      break;
    case hal_clksel_sys_xtal_osc:
      c_assert(clkConfig.xtalsel == hal_xtalsel_xtal_en);
      if (clkConfig.xtalsel != hal_xtalsel_xtal_en) {
        return hal_status_badParam;
      }
      VOR_CLKGEN->CTRL0 |= 3UL << CLKGEN_CTRL0_CLKSEL_SYS_Pos;  // select XTAL osc to sysclk
      break;
  }

  SystemCoreClockUpdate();

  // ADC clock (must be 2-12.5 MHz)
  // NOTE: Not using divide by 1 or /2 ratio in REVA silicon because of triggering issue
  // For this reason, keep SYSCLK above 8MHz to have the ADC /4 ratio in range)
  VOR_CLKGEN->CTRL1 &= ~CLKGEN_CTRL1_ADC_CLK_DIV_SEL_Msk;  // default div by 8
  if (SystemCoreClock <= 50000000) {
    VOR_CLKGEN->CTRL1 |= 0x1 << CLKGEN_CTRL1_ADC_CLK_DIV_SEL_Pos;  // div 4
  }

#ifdef __SPI3_SYSCLK_4
  hbo_clk_wait_ms(5);
  spi3_maxclock(true);
#endif

  return hal_status_ok;
}

#ifdef __SPI3_SYSCLK_4
// test feature - enable SPI3 (ROM_SPI) clock constantly tx all 0s forever
// expect SYSCLK / 4 on ROM_SCK
static void spi3_maxclock(bool en) {
  if (en) {
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= (CLK_ENABLE_SPI3);
    NVIC_DisableIRQ(SPI3_RX_IRQn);
    VOR_SPI->BANK[3].CLKPRESCALE = 0x0;
    VOR_SPI->BANK[3].CTRL0 = 0xf;
    VOR_SPI->BANK[3].CTRL1 = 0x382 | (1 << SPI_CTRL1_SS_Pos);
    VOR_SPI->BANK[3].TXFIFOIRQTRG = 8;
    VOR_SPI->BANK[3].IRQ_ENB = SPI3_IRQ_ENB_TXIM_Msk;
    NVIC_EnableIRQ(SPI3_TX_IRQn);
  } else {
    NVIC_DisableIRQ(SPI3_RX_IRQn);
    NVIC_DisableIRQ(SPI3_TX_IRQn);
    VOR_SPI->BANK[3].CTRL0 = 0;
    VOR_SPI->BANK[3].CTRL1 = 0;
    VOR_SPI->BANK[3].IRQ_ENB = 0;
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~(CLK_ENABLE_SPI3);
  }
}

void SPI3_TX_IRQHandler(void) {
  int maxload = 16;
  while ((VOR_SPI3->STATUS & SPI3_STATUS_TNF_Msk) && (--maxload)) {
    VOR_SPI3->DATA = 0;
  }
}
#endif

////
////
////
//// Begin TrueCircuits PLL calculation script section
////
////
////

/*
 *        True Circuits, Inc. General-Purpose PLL for TSMC CL013LP 130nm
 *
 *               Frequency Programming Calculation Script
 *
 *                             Revision 1.1
 *
 *               *** True Circuits, Inc. Confidential ***
 *
 *  Copyright (C) 2000-2017 True Circuits, Inc.  All rights reserved.
 */

#define VCO_MIN (1.1e+08)
#define VCO_MAX (5.5e+08)
#define REF_MIN (4.29688e+06)
#define REF_MAX (5.5e+08)
#define NR_MIN (1)
#define NR_MAX (16)
#define NF_MIN (1)
#define NF_MAX (64)
#define NO_MIN (1)
#define NO_MAX (16)
#define NB_MIN (1)
#define NB_MAX (64)
#define MAX_VCO (1)
#define REF_RNG (1)

int calcPLL(uint32_t inFreqHz, uint32_t outFreqHz) {
  int nr, nrx, nf, nfi, no, noe, nout, nor, nore, nb, first, firstx, found;
  long long nfx;
  double fin, fout, fvco;
  double val, nval, err, merr, terr;
  int x_nrx, x_no, x_nb;
  long long x_nfx;
  double x_fvco, x_err;

  // VORAGO edits: new input handling (no argc, argv[])
  fin = (double)inFreqHz;
  fout = (double)outFreqHz;
  val = fout / fin;
  terr = -2;  // choose lowest error
  // End VORAGO edits

  dbgprintf("Begin PLL settings calculation\n");

  if (fout * NO_MIN > VCO_MAX) {
    dbgprintf("Error:  Maximum frequency exceeded.\n");
    return -1;
  }
  if (fout * NO_MAX < VCO_MIN) {
    dbgprintf("Error:  Minimum frequency exceeded.\n");
    return -1;
  }

  first = firstx = 1;
  if (terr == -1) {
    dbgprintf("NR\tNF\tOD\tNB\tFvco\t\terror\n");
    dbgprintf("------------------------------------------------------\n");
  } else if (terr != -2) {
    first = 0;
    if (terr == 0) terr = 1e-16;
    merr = fabs(terr);
  }
  found = 0;
  for (nfi = (int)val; nfi < NF_MAX; ++nfi) {
    nr = (int)(rint(((double)nfi) / val));
    if (nr == 0) continue;
    if ((REF_RNG) && (nr < NR_MIN)) continue;
    if (fin / ((double)nr) > REF_MAX) continue;
    nrx = nr;
    nf = nfx = nfi;
    nval = ((double)nfx) / ((double)nr);
    if (nf == 0) nf = 1;
    err = 1 - nval / val;

    if ((first) || (fabs(err) < merr * (1 + 1e-6)) || (fabs(err) < 1e-16)) {
      nout = (int)(floor(VCO_MAX / fout));
      for (no = (nout > NO_MAX) ? NO_MAX : nout; no > NO_MIN; --no) {
        if ((REF_RNG) && ((nr / no) < NR_MIN)) continue;
        if ((nr % no) == 0) break;
      }
      if ((nr % no) != 0) continue;
      nor = ((nout > NO_MAX) ? NO_MAX : nout) / no;
      nore = NF_MAX / nf;
      if (nor > nore) nor = nore;
      noe = (int)(ceil(VCO_MIN / fout));
      if (!MAX_VCO) {
        nore = (noe - 1) / no + 1;
        nor = nore;
        nout = 0; /* force next if to fail */
      }
      if ((((no * nor) < (nout >> 1)) || ((no * nor) < noe)) && ((no * nor) < (NF_MAX / nf))) {
        no = NF_MAX / nf;
        if (no > NO_MAX) no = NO_MAX;
        if (no > nout) no = nout;
        nfx *= no;
        nf *= no;
        if ((no > 1) && (!firstx)) continue;
        /* wait for larger nf in later iterations */
      } else {
        nrx /= no;
        nfx *= nor;
        nf *= nor;
        no *= nor;
        if (no > NO_MAX) continue;
        if ((nor > 1) && (!firstx)) continue;
        /* wait for larger nf in later iterations */
      }

      nb = nfx;
      if (nb < NB_MIN) nb = NB_MIN;
      if (nb > NB_MAX) continue;

      fvco = fin / ((double)nrx) * ((double)nfx);
      if (fvco < VCO_MIN) continue;
      if (fvco > VCO_MAX) continue;
      if (nf < NF_MIN) continue;
      if ((REF_RNG) && (fin / ((double)nrx) < REF_MIN)) continue;
      if ((REF_RNG) && (nrx > NR_MAX)) continue;
      if (!(((firstx) && (terr < 0)) || (fabs(err) < merr * (1 - 1e-6)) ||
            ((MAX_VCO) && (no > x_no))))
        continue;
      if ((!firstx) && (terr >= 0) && (nrx > x_nrx)) continue;

      found = 1;
      x_no = no;
      x_nrx = nrx;
      x_nfx = nfx;
      x_nb = nb;
      x_fvco = fvco;
      x_err = err;
      first = firstx = 0;
      merr = fabs(err);
      if (terr != -1) continue;
      dbgprintf("%d\t%lld\t%d\t%d\t%e\t%#+g\n", nrx, nfx, no, nb, fvco, err);
    }
  }
  if (!found) {
    dbgprintf("Error:  No workable settings found.\n");
    return -1;
  }
  if (terr != -1) {
    nrx = x_nrx;
    nfx = x_nfx;
    no = x_no;
    nb = x_nb;
    fvco = x_fvco;
    err = x_err;
    if ((terr != -2) && (fabs(err) >= terr * (1 - 1e-6))) {
      dbgprintf("Error:  No appropriate ratio found.\n");
      return -1;
    }

    dbgprintf("NR = %d\n", nrx);
    dbgprintf("NF = %lld\n", nfx);
    dbgprintf("OD = %d\n", no);
    dbgprintf("NB = %d\n", nb);

    dbgprintf("\n");
    dbgprintf("Fin  = %g\n", fin);
    dbgprintf("Fvco = %g\n", fvco);
    dbgprintf("Fout = %g\n", fvco / no);
    dbgprintf("error = %+g\n", err);

    dbgprintf("\n");
    dbgprintf("CLKR[3:0] = 0x%x\n", nrx - 1);
    dbgprintf("CLKF[5:0] = 0x%x\n", (int)nfx - 1);
    dbgprintf("CLKOD[3:0] = 0x%x\n", no - 1);
    dbgprintf("BWADJ[5:0] = 0x%x\n", nb - 1);

    // VORAGO edits: write values to register
    VOR_CLKGEN->CTRL0 &= ~(CLKGEN_CTRL0_PLL_CLKR_Msk | CLKGEN_CTRL0_PLL_CLKF_Msk |
                           CLKGEN_CTRL0_PLL_CLKOD_Msk | CLKGEN_CTRL0_PLL_BWADJ_Msk);

    VOR_CLKGEN->CTRL0 |= (nrx - 1) << CLKGEN_CTRL0_PLL_CLKR_Pos;
    VOR_CLKGEN->CTRL0 |= (nfx - 1) << CLKGEN_CTRL0_PLL_CLKF_Pos;
    VOR_CLKGEN->CTRL0 |= (no - 1) << CLKGEN_CTRL0_PLL_CLKOD_Pos;
    VOR_CLKGEN->CTRL0 |= (nb - 1) << CLKGEN_CTRL0_PLL_BWADJ_Pos;
    // end VORAGO edits
  }
  return 0;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
