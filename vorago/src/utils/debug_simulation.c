/***************************************************************************************
 * @file     debug_simulation.c
 * @version  V1.1
 * @date     20 April 2020
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2016 VORAGO Technologies.
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
#ifndef __DEBUG_SIMULATIONS_C
#define __DEBUG_SIMULATIONS_C

#include "debug_simulation.h"

#include "va416xx.h"

void delay_100(void) {
  uint32_t i;

  // roughly 100us at 80mHz
  for (i = 0; i < 1000; i++) {
    // Sit'n'spin
  }
}

void delay_500(void) {
  uint32_t i;

  // roughly 500us at 80mHz
  for (i = 0; i < 5000; i++) {
    // Sit'n'spin
  }
}

void finish_test(uint32_t test_info, uint32_t return_code) {
  // VORAGO update 5/7/19: Added forcing function 0
  // PA[0-15] funsel 0 (in case set to something else)
  for (uint8_t i = 0; i < 16; i++) {
    VOR_IOCONFIG->PORTA[i] = 0;
  }

  // Confirm direction still good
#ifdef EBI_ETH_BRD
  VOR_GPIO->BANK[0].DIR |= ((uint32_t)0x000000FF);  // for EBI/ETH board
#else
  VOR_GPIO->BANK[0].DIR |= ((uint32_t)0x0000FFFF);
#endif

  if (return_code == 0) {
    // PASS
    VOR_GPIO->BANK[0].DATAOUT = ((uint32_t)0x0000500D);
  } else {
    // FAIL
    VOR_GPIO->BANK[0].DATAOUT = ((uint32_t)0x0000BAD0);
  }
  VOR_GPIO->BANK[0].DATAOUT = ((uint32_t)test_info);

  // End of test
  VOR_GPIO->BANK[0].DATAOUT = ((uint32_t)0x0000BEEF);

  // Loop
  while (1)
    ;
}

void test_progress(uint32_t test_info, uint32_t errors) {
#ifdef EBI_ETH_BRD
  VOR_GPIO->BANK[0].DIR |= ((uint32_t)0x000000FF);  // for EBI/ETH board
#else
  VOR_GPIO->BANK[0].DIR |= ((uint32_t)0x0000FFFF);
#endif
  VOR_GPIO->BANK[0].DATAOUT = ((uint32_t)test_info);
  VOR_GPIO->BANK[0].DATAOUT = ((uint32_t)errors);
}

#endif
