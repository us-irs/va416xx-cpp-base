/***************************************************************************************
 * @file     debug_simulation.h
 * @version  V1.0
 * @date     31. March 2016
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
#ifndef __DEBUG_SIMULATION_H
#define __DEBUG_SIMULATION_H

#include "va416xx.h"

//#define EBI_ETH_BRD

extern void finish_test(uint32_t test_info, uint32_t return_code);
extern void test_progress(uint32_t test_info, uint32_t errors);
extern void delay_500(void);
extern void delay_100(void);

#endif
