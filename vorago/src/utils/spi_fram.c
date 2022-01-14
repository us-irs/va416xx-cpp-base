/***************************************************************************************
 * @file     spi_fram.c
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

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "spi_fram.h"

#include "VORConfig.h"
#ifdef VOR_FRAM_USE_HAL_DRIVER
#include "va416xx_hal_spi.h"
#endif

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/* Commands */
#define FRAM_WREN 0x06
#define FRAM_WRDI 0x04
#define FRAM_RDSR 0x05
#define FRAM_WRSR 0x01
#define FRAM_READ 0x03
#define FRAM_WRITE 0x02
#define FRAM_RDID 0x9F

/* Address Masks */
#define ADDR_MSB_MASK (uint32_t)0xFF0000
#define ADDR_MID_MASK (uint32_t)0x00FF00
#define ADDR_LSB_MASK (uint32_t)0x0000FF
#define MSB_ADDR_BYTE(addr) ((uint8_t)((addr & ADDR_MSB_MASK) >> 16))
#define MID_ADDR_BYTE(addr) ((uint8_t)((addr & ADDR_MID_MASK) >> 8))
#define LSB_ADDR_BYTE(addr) ((uint8_t)(addr & ADDR_LSB_MASK))

#ifndef VOR_FRAM_USE_HAL_DRIVER
#define HAL_SPI_VERSION (0x00000300)  // 0.3.0

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
#endif

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static bool isSpiInit[NUM_SPI_BANKS] = {0};

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static void wait_idle(uint8_t spiBank);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Wait for the SPI to be idle, then clear the FIFOs
 **
 ******************************************************************************/
static void wait_idle(uint8_t spiBank) {
  if (spiBank >= NUM_SPI_BANKS) {
    return;
  }
  while (!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_TFE_Msk)) {
  };  // Wait until TxBuf sends all
  while (VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_BUSY_Msk) {
  };  // Wait here until bytes are fully transmitted.
  VOR_SPI->BANK[spiBank].FIFO_CLR =
      SPI_FIFO_CLR_RXFIFO_Msk | SPI_FIFO_CLR_TXFIFO_Msk;  // Clear Tx & RX fifo
}

/*******************************************************************************
 **
 ** @brief  Init a SPI periph for F-ram access (SPI in simple polling mode)
 **
 ******************************************************************************/
hal_status_t FRAM_Init(uint8_t spiBank, uint8_t csNum) {
  if (spiBank >= NUM_SPI_BANKS) {
    return hal_status_badParam;
  }
  if (csNum > 7) {
    return hal_status_badParam;
  }

#ifdef VOR_FRAM_USE_HAL_DRIVER
  // using HAL driver
  hal_spi_init_t init;
  init.blockmode = true;
  init.bmstall = true;
  init.clkDiv = 4;
  init.loopback = false;
  init.mdlycap = false;
  init.mode = en_spi_clkmode_0;
  init.ms = en_spi_ms_master;
  init.slaveSelect = csNum;
  init.wordLen = 8;

  hal_status_t stat = HAL_Spi_Init(&VOR_SPI->BANK[spiBank], init);
  if (stat != hal_status_ok) {
    return stat;
  }  // abort if driver init issue

  // Turn off HAL SPI driver interrupts (using polling mode)
  NVIC_DisableIRQ((IRQn_Type)(SPI0_RX_IRQn + (2 * spiBank)));
  VOR_SPI->BANK[spiBank].IRQ_ENB = 0;
#else
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= (CLK_ENABLE_SPI0 << spiBank);
  VOR_SPI->BANK[spiBank].CLKPRESCALE = 0x4;
  VOR_SPI->BANK[spiBank].CTRL0 = 0x7;
  VOR_SPI->BANK[spiBank].CTRL1 = 0x382 | (csNum << SPI_CTRL1_SS_Pos);
#endif  // HAL driver

  // Clear Block Protection bits to enable programming
  // Does not set SRWD, so WP_n pin has no effect
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA =
      (uint32_t)FRAM_WREN | SPI_DATA_BMSTART_BMSTOP_Msk;  // Set Write Enable Latch(WEL) bit
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_WRSR;  // Write single-byte Status Register message
  VOR_SPI->BANK[spiBank].DATA =
      (uint32_t)0x00 | SPI_DATA_BMSTART_BMSTOP_Msk;  // Clear the BP1/BP0 protection
  wait_idle(spiBank);

  isSpiInit[spiBank] = true;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Write to F-ram on spi[spiBank]
 **
 ******************************************************************************/
hal_status_t FRAM_Write(uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len) {
  if (spiBank >= NUM_SPI_BANKS) {
    return hal_status_badParam;
  }
  if (isSpiInit[spiBank] == false) {
    return hal_status_notInitialized;
  }

  uint32_t volatile voidRead = 0;
  (void)voidRead;
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA =
      (uint32_t)FRAM_WREN | SPI_DATA_BMSTART_BMSTOP_Msk;  // Set Write Enable Latch(WEL) bit
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_WRITE;           // Write command
  VOR_SPI->BANK[spiBank].DATA = MSB_ADDR_BYTE(addr);  // Address high byte
  VOR_SPI->BANK[spiBank].DATA = MID_ADDR_BYTE(addr);  // Address mid byte
  VOR_SPI->BANK[spiBank].DATA = LSB_ADDR_BYTE(addr);  // Address low byte

  while (len - 1) {
    while (!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_TNF_Msk)) {
    };
    VOR_SPI->BANK[spiBank].DATA = *buf++;
    voidRead = VOR_SPI->BANK[spiBank].DATA;
    --len;
  }
  while (!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_TNF_Msk)) {
  }
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)(*buf) | SPI_DATA_BMSTART_BMSTOP_Msk;
  wait_idle(spiBank);

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Read from F-ram on spi[spiBank]
 **
 ******************************************************************************/
hal_status_t FRAM_Read(uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len) {
  if (spiBank >= NUM_SPI_BANKS) {
    return hal_status_badParam;
  }
  if (isSpiInit[spiBank] == false) {
    return hal_status_notInitialized;
  }

  uint32_t volatile voidRead = 0;
  (void)voidRead;
  uint32_t i;

  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_READ;            // Read command
  VOR_SPI->BANK[spiBank].DATA = MSB_ADDR_BYTE(addr);  // Address high byte
  VOR_SPI->BANK[spiBank].DATA = MID_ADDR_BYTE(addr);  // Address mid byte
  VOR_SPI->BANK[spiBank].DATA = LSB_ADDR_BYTE(addr);  // Address low byte

  for (i = 0; i < 4; i++) {
    VOR_SPI->BANK[spiBank].DATA = 0x00;  // Pump the SPI
    while (!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk)) {
    };
    voidRead = VOR_SPI->BANK[spiBank].DATA;  // Void read
  }

  for (i = 0; i < len; i++) {
    VOR_SPI->BANK[spiBank].DATA = 0x00;  // Pump the SPI
    while (!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk)) {
    }
    *buf = VOR_SPI->BANK[spiBank].DATA;
    buf++;
  }
  VOR_SPI->BANK[spiBank].DATA = SPI_DATA_BMSTART_BMSTOP_Msk;  // Terminate Block Transfer
  wait_idle(spiBank);

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Un-init the F-ram and SPI
 **
 ******************************************************************************/
hal_status_t FRAM_UnInit(uint8_t spiBank) {
  if (spiBank >= NUM_SPI_BANKS) {
    return hal_status_badParam;
  }

  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA =
      (uint32_t)FRAM_WREN | SPI_DATA_BMSTART_BMSTOP_Msk;  // Set Write Enable Latch(WEL) bit
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_WRSR;  // Write single-byte Status Register message
  VOR_SPI->BANK[spiBank].DATA =
      (uint32_t)0xfd | SPI_DATA_BMSTART_BMSTOP_Msk;  // Set the BP1/BP0 protection
  wait_idle(spiBank);

  hal_status_t stat = hal_status_ok;

#ifdef VOR_FRAM_USE_HAL_DRIVER
  stat = HAL_Spi_DeInit(&VOR_SPI->BANK[spiBank]);
#else
  VOR_SPI->BANK[spiBank].CTRL1 = 0;
  VOR_SPI->BANK[spiBank].CTRL0 = 0;
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~(CLK_ENABLE_SPI0 << spiBank);
#endif
  isSpiInit[spiBank] = false;
  return stat;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
