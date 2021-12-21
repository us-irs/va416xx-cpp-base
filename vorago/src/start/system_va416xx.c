/**************************************************************************//**
 * @file     system_ARMCM4.c
 * @brief    CMSIS Device System Source File for
 *           ARMCM4 Device Series
 * @version  V5.00
 * @date     10. January 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
 /*
 From: C:\Keil\ARM\Device\ARM\ARMCM4\Source
 */

#include "va416xx.h"
//#include "board.h"

/*----------------------------------------------------------------------------
  Define clocks (can override XTAL and EXTCLK in board.h)
 *----------------------------------------------------------------------------*/

#ifndef XTAL
#define  XTAL            (10000000UL)      /* Oscillator default frequency */
#endif

#ifndef HBO
#define  HBO             (20000000UL)      /* Internal HBO frequency */
#endif

#ifndef EXTCLK
#define  EXTCLK          (10000000UL)      /* XTAL minus default frequency */
#endif

/*----------------------------------------------------------------------------
  Externals
 *----------------------------------------------------------------------------*/
#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
  extern uint32_t __Vectors;
#endif

/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = HBO; // default HBO CLK


/*----------------------------------------------------------------------------
  System Core Clock update function
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)
{
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_CLKGEN; // ensure CLKGEN is enabled
  switch((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_CLKSEL_SYS_Msk) >> CLKGEN_CTRL0_CLKSEL_SYS_Pos)
  {
    case 0:
      // 00b - Internal HB Oscillator
      SystemCoreClock = HBO;
      break;
    case 1:
      // 01b - XTAL minus
      SystemCoreClock = EXTCLK;
      break;
    case 2:
      // 10b - PLL output
      if((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_REF_CLK_SEL_Msk) == 0x1){
        SystemCoreClock = XTAL;
      }
      if((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_REF_CLK_SEL_Msk) == 0x2){
        SystemCoreClock = EXTCLK;
      }
      if((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_PLL_BYPASS_Msk) == 0){
        SystemCoreClock /= (((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_PLL_CLKR_Msk) >> 
                                                 CLKGEN_CTRL0_PLL_CLKR_Pos) + 1);
        SystemCoreClock *= (((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_PLL_CLKF_Msk) >> 
                                                 CLKGEN_CTRL0_PLL_CLKF_Pos) + 1);
        SystemCoreClock /= (((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_PLL_CLKOD_Msk) >> 
                                                 CLKGEN_CTRL0_PLL_CLKOD_Pos) + 1);
      }
      break;
    case 3:
      // 11b - XTAL Oscillator
      SystemCoreClock = XTAL;
      break;
  }
  
  // CLK_DIV_SEL
  switch((VOR_CLKGEN->CTRL0 & CLKGEN_CTRL0_CLK_DIV_SEL_Msk) >> CLKGEN_CTRL0_CLK_DIV_SEL_Pos)
  {
    case 0:
      // divide by 1 (no divide)
      break;
    case 1:
      // divide by 2
      SystemCoreClock /= 2;
      break;
    case 2:
      // divide by 4
      SystemCoreClock /= 4;
      break;
    case 3:
      // divide by 8
      SystemCoreClock /= 8;
      break;
  }
}

/*----------------------------------------------------------------------------
  System initialization function
 *----------------------------------------------------------------------------*/
void SystemInit (void)
{

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
  SCB->VTOR = (uint32_t) &__Vectors;
#endif

#if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
  SCB->CPACR |= ((3U << 10U*2U) |           /* set CP10 Full Access */
                 (3U << 11U*2U)  );         /* set CP11 Full Access */
#endif

#ifdef UNALIGNED_SUPPORT_DISABLE
  SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif
  
  SystemCoreClock = HBO; // default HBO CLK
}
