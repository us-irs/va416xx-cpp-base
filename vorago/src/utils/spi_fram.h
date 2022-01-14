/***************************************************************************************
 * @file     spi_fram.h
 * @version  V1.01
 * @date     08 November 2019
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

#ifndef __SPI_FRAM_H
#define __SPI_FRAM_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define FRAM_LEN (0x40000)
//#define USE_HAL_DRIVER

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern hal_status_t FRAM_Init(uint8_t spiBank, uint8_t csNum);
extern hal_status_t FRAM_Write(uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len);
extern hal_status_t FRAM_Read(uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len);
extern hal_status_t FRAM_UnInit(uint8_t spiBank);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __SPI_FRAM_H */
