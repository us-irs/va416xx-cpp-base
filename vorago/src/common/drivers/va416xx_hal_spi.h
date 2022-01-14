/***************************************************************************************
 * @file     va416xx_hal_spi.h
 * @version  V0.4
 * @date     11 December 2020
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

#ifndef __HAL_SPI_H
#define __HAL_SPI_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "event_handler_index.h"
#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define HAL_SPI_VERSION (0x00000400)  // 0.4.0

#define NUM_SPI_BANKS (4)

#define SPI_MASTER_MSK (0xF)  // SPI 0-3 support master mode
#define SPI_SLAVE_MSK (0x7)   // SPI 0-2 support slave mode

#define SPI0_BANK (0)
#define SPI1_BANK (1)
#define SPI2_BANK (2)
#define SPI3_BANK (3)

#define SPI_CLK (SystemCoreClock / 2)  // SPI peripherals reside on APB1

#define SPI_MIN_WORDLEN (4)
#define SPI_MAX_WORDLEN (16)

// missing from header
/* ----------------------------------  SPI_DATA  --------------------------- */
#ifndef SPI_DATA_VALUE_Pos
#define SPI_DATA_VALUE_Pos \
  0 /*!< SPIB DATA: VALUE Position 							*/
#endif
#ifndef SPI_DATA_VALUE_Msk
#define SPI_DATA_VALUE_Msk                                          \
  (0x000000ffffUL << SPI_DATA_VALUE_Pos) /*!< SPIB DATA: VALUE Mask \
                                          */
#endif
#ifndef SPI_DATA_BMSKIPDATA_Pos
#define SPI_DATA_BMSKIPDATA_Pos \
  30 /*!< SPIB DATA: BMSKIPDATA Position 					*/
#endif
#ifndef SPI_DATA_BMSKIPDATA_Msk
#define SPI_DATA_BMSKIPDATA_Msk                                       \
  (0x01UL << SPI_DATA_BMSKIPDATA_Pos) /*!< SPIB DATA: BMSKIPDATA Mask \
                                       */
#endif
#ifndef SPI_DATA_BMSTART_BMSTOP_Pos
#define SPI_DATA_BMSTART_BMSTOP_Pos 31 /*!< SPIB DATA: BMSTART/BMSTOP Position 			*/
#endif
#ifndef SPI_DATA_BMSTART_BMSTOP_Msk
#define SPI_DATA_BMSTART_BMSTOP_Msk                                           \
  (0x01UL << SPI_DATA_BMSTART_BMSTOP_Pos) /*!< SPIB DATA: BMSTART/BMSTOP Mask \
                                           */
#endif

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/** SPI clocking mode */
typedef enum {
  en_spi_clkmode_0 = 0,
  en_spi_clkmode_1 = 1,
  en_spi_clkmode_2 = 2,
  en_spi_clkmode_3 = 3
} en_spi_clkmode_t;

/** SPI master or slave configuration */
typedef enum { en_spi_ms_master = 0, en_spi_ms_slave = 1 } en_spi_ms_t;

/** The SPI initialization struct */
typedef struct {
  en_spi_ms_t ms;
  en_spi_clkmode_t mode;
  uint16_t clkDiv;
  uint8_t wordLen;
  uint8_t slaveSelect;
  bool loopback;
  bool blockmode;
  bool bmstall;
  bool mdlycap;
} hal_spi_init_t;

/** The SPI transfer struct */
typedef struct {
  void* txbuf;           // transmit buffer address, or NULL for rx only (send pump)
  void* rxbuf;           // receive buffer address, or NULL for tx only
  uint16_t len;          // transfer length (must be nonzero)
  uint8_t ss;            // slave select number
  uint8_t bufInc16 : 1;  // buffer increment: 8 bit (0) or 16 bit (1)
  uint8_t bmstop : 1;    // 1: eassert CSn on last word of the xfer if blockmode (when master)
  eventHandlerIdx_t callbackIdx;  // index in event handler list of function to call when complete
  volatile bool* complete;        // address of bool to set true when xfer complete (NULL to ignore)
} spi_xfer_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

// Init and deinit
extern hal_status_t HAL_Spi_Init(VOR_SPI_Type* const spi, const hal_spi_init_t init);
extern hal_status_t HAL_Spi_DeInit(VOR_SPI_Type* const spi);

// Simplified init for some use cases
extern hal_status_t HAL_Spi_InitMaster(VOR_SPI_Type* const spi, en_spi_clkmode_t mode,
                                       uint8_t wordLen, uint16_t clkDiv);
extern hal_status_t HAL_Spi_InitSlave(VOR_SPI_Type* const spi, en_spi_clkmode_t mode,
                                      uint8_t wordLen, uint8_t ss);

// Transfers using interrupts (nonblocking and blocking)
extern hal_status_t HAL_Spi_Xfer(VOR_SPI_Type* const spi, spi_xfer_t* xfer);
extern hal_status_t HAL_Spi_XferB16(VOR_SPI_Type* const spi, uint16_t* txBuf, uint16_t* rxBuf,
                                    uint16_t len, bool bmstop, uint8_t ss);
extern hal_status_t HAL_Spi_XferB8(VOR_SPI_Type* const spi, uint8_t* txBuf, uint8_t* rxBuf,
                                   uint16_t len, bool bmstop, uint8_t ss);

// TODO: Transfers using DMA
extern hal_status_t HAL_Spi_XferDma(VOR_SPI_Type* const spi, spi_xfer_t* xfer);

// Check for busy
extern hal_status_t HAL_Spi_IsBusy(VOR_SPI_Type* const spi, bool* busy);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_SPI_H */
