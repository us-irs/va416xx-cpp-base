/***************************************************************************************
 * @file     va416xx_hal_spi.c
 * @version  V0.3
 * @date     22 February 2019
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

#include "va416xx_hal_spi.h"

#include "circular_buffer.h"
#include "va416xx_debug.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define SPI_PERID (0x021307E9)
#define SPI_FIFO_SZ (16)

// maximum number of xfers in the xfer queue
#define SPI_VCB_MAX_ITEMS (4)

#define SPI_INVALID_BANKNUM (0xFF)
#define SPI_MAX_BANKNUM (NUM_SPI_BANKS - 1)

#define TXTIMEOUT (100000)

#define SPI_PUMPWORD (0x0)

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

typedef enum { en_spi_state_idle = 0, en_spi_state_pending, en_spi_state_finished } en_spi_state_t;

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static const uint8_t spiNumSlaveSelects[NUM_SPI_BANKS] = {4, 8, 7, 1};
static bool aIsSpiInitialized[NUM_SPI_BANKS] = {false, false, false, false};

// circular buffer (transaction queue)
static VOR_CircularBuffer_t vcbSpiTxns[NUM_SPI_BANKS];
static spi_xfer_t vcbSpiQueue_data[NUM_SPI_BANKS][SPI_VCB_MAX_ITEMS];

// state machine variables *** DO NOT MESS WITH THESE ***
static volatile en_spi_state_t states[NUM_SPI_BANKS] = {en_spi_state_idle, en_spi_state_idle,
                                                        en_spi_state_idle, en_spi_state_idle};
static spi_xfer_t currentXfer[NUM_SPI_BANKS];
static uint16_t sent[NUM_SPI_BANKS];
static uint16_t received[NUM_SPI_BANKS];
// end state machine variables

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

__STATIC_INLINE uint32_t GetBankNum(VOR_SPI_Type* const spi);
__STATIC_INLINE void EnableRxIrq(uint32_t bank);
//__STATIC_INLINE void EnableTxIrq(uint32_t bank); // disabled to remove not referenced warning
__STATIC_INLINE bool DisableRxIrq(uint32_t bank);
__STATIC_INLINE bool DisableTxIrq(uint32_t bank);
__STATIC_INLINE void SpiFifo_Clear(uint32_t bank);
__STATIC_INLINE void InitCircularBuffer(uint32_t bank);
__STATIC_INLINE void LoadTxFifo(uint32_t bank);
__STATIC_INLINE void SPI_StateMachine(uint32_t bank);

static hal_status_t SetClkDiv(VOR_SPI_Type* const spi, uint16_t div);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Takes a pointer to a SPI peripheral and returns its bank number
 **
 ** @param  spi - pointer to SPI peripheral
 **
 ** @return uint32_t the bank number for this SPI
 **
 ******************************************************************************/
__STATIC_INLINE uint32_t GetBankNum(VOR_SPI_Type* const spi) {
  if (VOR_SPI0 == spi) {
    return SPI0_BANK;
  } else if (VOR_SPI1 == spi) {
    return SPI1_BANK;
  } else if (VOR_SPI2 == spi) {
    return SPI2_BANK;
  } else if (VOR_SPI3 == spi) {
    return SPI3_BANK;
  } else {
    // invalid bank
    return SPI_INVALID_BANKNUM;
  }
}

/*******************************************************************************
 **
 ** @brief  Enables a SPI's RX IRQ in the NVIC
 **
 ** @param  bank - the SPI bank number (0-3)
 **
 ******************************************************************************/
__STATIC_INLINE void EnableRxIrq(uint32_t bank) {
  switch (bank) {
    case SPI0_BANK:
      NVIC_EnableIRQ(SPI0_RX_IRQn);
      break;
    case SPI1_BANK:
      NVIC_EnableIRQ(SPI1_RX_IRQn);
      break;
    case SPI2_BANK:
      NVIC_EnableIRQ(SPI2_RX_IRQn);
      break;
    case SPI3_BANK:
      NVIC_EnableIRQ(SPI3_RX_IRQn);
      break;
  }
}

/*******************************************************************************
 **
 ** @brief  Enables a SPI's TX IRQ in the NVIC
 **
 ** @param  bank - the SPI bank number (0-3)
 **
 ******************************************************************************/
// Currently disabled because it is not used - suppresses not referenced warning
#if 0
__STATIC_INLINE void EnableTxIrq(uint32_t bank)
{
  switch(bank){
    case SPI0_BANK:
      NVIC_EnableIRQ(SPI0_TX_IRQn);
      break;
    case SPI1_BANK:
      NVIC_EnableIRQ(SPI1_TX_IRQn);
      break;
    case SPI2_BANK:
      NVIC_EnableIRQ(SPI2_TX_IRQn);
      break;
    case SPI3_BANK:
      NVIC_EnableIRQ(SPI3_TX_IRQn);
      break;
  }
}
#endif

/*******************************************************************************
 **
 ** @brief  Disables a SPI's RX IRQ in the NVIC
 **
 ** @param  bank - the SPI bank number (0-3)
 **
 ** @return bool - true if the RX IRQ for that bank was previously enabled, else false
 **
 ******************************************************************************/
__STATIC_INLINE bool DisableRxIrq(uint32_t bank) {
  switch (bank) {
    case SPI0_BANK:
      if (NVIC_GetEnableIRQ(SPI0_RX_IRQn)) {
        NVIC_DisableIRQ(SPI0_RX_IRQn);
        return true;
      }
      break;
    case SPI1_BANK:
      if (NVIC_GetEnableIRQ(SPI1_RX_IRQn)) {
        NVIC_DisableIRQ(SPI1_RX_IRQn);
        return true;
      }
      break;
    case SPI2_BANK:
      if (NVIC_GetEnableIRQ(SPI2_RX_IRQn)) {
        NVIC_DisableIRQ(SPI2_RX_IRQn);
        return true;
      }
      break;
    case SPI3_BANK:
      if (NVIC_GetEnableIRQ(SPI3_RX_IRQn)) {
        NVIC_DisableIRQ(SPI3_RX_IRQn);
        return true;
      }
      break;
  }
  return false;
}

/*******************************************************************************
 **
 ** @brief  Disables a SPI's TX IRQ in the NVIC
 **
 ** @param  bank - the SPI bank number (0-3)
 **
 ** @return bool - true if the TX IRQ for that bank was previously enabled, else false
 **
 ******************************************************************************/
__STATIC_INLINE bool DisableTxIrq(uint32_t bank) {
  switch (bank) {
    case SPI0_BANK:
      if (NVIC_GetEnableIRQ(SPI0_TX_IRQn)) {
        NVIC_DisableIRQ(SPI0_TX_IRQn);
        return true;
      }
      break;
    case SPI1_BANK:
      if (NVIC_GetEnableIRQ(SPI1_TX_IRQn)) {
        NVIC_DisableIRQ(SPI1_TX_IRQn);
        return true;
      }
      break;
    case SPI2_BANK:
      if (NVIC_GetEnableIRQ(SPI2_TX_IRQn)) {
        NVIC_DisableIRQ(SPI2_TX_IRQn);
        return true;
      }
      break;
    case SPI3_BANK:
      if (NVIC_GetEnableIRQ(SPI3_TX_IRQn)) {
        NVIC_DisableIRQ(SPI3_TX_IRQn);
        return true;
      }
      break;
  }
  return false;
}

/*******************************************************************************
 **
 ** @brief  Reset spi FIFOs
 **
 ******************************************************************************/
__STATIC_INLINE void SpiFifo_Clear(uint32_t bank) {
  // Clear Tx & RX fifo
  VOR_SPI->BANK[bank].FIFO_CLR = SPI_FIFO_CLR_RXFIFO_Msk | SPI_FIFO_CLR_TXFIFO_Msk;
}

/*******************************************************************************
 **
 ** @brief  Reset/Init spi circular buffer
 **
 ******************************************************************************/

__STATIC_INLINE void InitCircularBuffer(uint32_t bank) {
  VOR_CircularBuffer_Initialize(&vcbSpiTxns[bank], &vcbSpiQueue_data[bank][0], sizeof(spi_xfer_t),
                                SPI_VCB_MAX_ITEMS);
}

/*******************************************************************************
 **
 ** @brief  Resets and initializes a SPI peripheral
 **
 ** @param  spi - pointer tp SPI peripheral to initialize (SPI0-3)
 **
 ** @param  initCfg - the SPI peripheral configuration struct (init settings)
 **
 ******************************************************************************/
hal_status_t HAL_Spi_Init(VOR_SPI_Type* const spi, const hal_spi_init_t init) {
  uint32_t bank = GetBankNum(spi);
  hal_status_t status;

  // Enable clock and reset SPI
  switch (bank) {
    case SPI0_BANK:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_SPI0;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk;
      break;

    case SPI1_BANK:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_SPI1;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI1_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPI1_Msk;
      break;

    case SPI2_BANK:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_SPI2;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI2_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPI2_Msk;
      break;

    case SPI3_BANK:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_SPI3;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI3_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPI3_Msk;
      break;

    default:
      // invalid spi bank
      return hal_status_badParam;
  }
  aIsSpiInitialized[bank] = false;

  // check peripheral ID
  c_assert(spi->PERID == SPI_PERID);
  if (false == (spi->PERID == SPI_PERID)) {
    return hal_status_badPeriphID;  // not running on correct hw, or a register issue
  }

  // Clear FIFOs
  SpiFifo_Clear(bank);

  // SPI master or slave operation
  switch (init.ms) {
    case en_spi_ms_master:
      c_assert(((1UL << bank) & SPI_MASTER_MSK) > 0);
      spi->CTRL1 &= ~SPI_CTRL1_MS_Msk;
      break;
    case en_spi_ms_slave:
      c_assert(((1UL << bank) & SPI_SLAVE_MSK) > 0);
      spi->CTRL1 |= SPI_CTRL1_MS_Msk;
      break;
    default:
      return hal_status_badParam;
  }

  // Init circular buffer and state machine
  InitCircularBuffer(bank);
  states[bank] = en_spi_state_idle;

  // SPI clocking mode
  switch (init.mode) {
    case en_spi_clkmode_0:
      spi->CTRL0 &= ~(SPI_CTRL0_SPO_Msk | SPI_CTRL0_SPH_Msk);
      break;
    case en_spi_clkmode_1:
      spi->CTRL0 &= ~(SPI_CTRL0_SPO_Msk);
      spi->CTRL0 |= (SPI_CTRL0_SPH_Msk);
      break;
    case en_spi_clkmode_2:
      spi->CTRL0 &= ~(SPI_CTRL0_SPH_Msk);
      spi->CTRL0 |= (SPI_CTRL0_SPO_Msk);
      break;
    case en_spi_clkmode_3:
      spi->CTRL0 |= (SPI_CTRL0_SPO_Msk | SPI_CTRL0_SPH_Msk);
      break;
    default:
      return hal_status_badParam;
  }

  // Clock divider
  status = SetClkDiv(spi, init.clkDiv);
  if (status != hal_status_ok) {
    return status;
  }

  // Word length
  c_assert(init.wordLen >= SPI_MIN_WORDLEN);
  c_assert(init.wordLen <= SPI_MAX_WORDLEN);
  if ((init.wordLen < SPI_MIN_WORDLEN) || (init.wordLen > SPI_MAX_WORDLEN)) {
    return hal_status_badParam;
  }
  spi->CTRL0 &= ~SPI_CTRL0_SIZE_Msk;
  spi->CTRL0 |= ((init.wordLen - 1) << SPI_CTRL0_SIZE_Pos);

  // Slave select pin
  c_assert(init.slaveSelect < spiNumSlaveSelects[bank]);
  if (init.slaveSelect >= spiNumSlaveSelects[bank]) {
    return hal_status_badParam;
  }
  spi->CTRL1 &= ~SPI_CTRL1_SS_Msk;
  spi->CTRL1 |= (init.slaveSelect << SPI_CTRL1_SS_Pos);

  // Loopback
  if (true == init.loopback) {
    spi->CTRL1 |= SPI_CTRL1_LBM_Msk;
  } else {
    spi->CTRL1 &= ~SPI_CTRL1_LBM_Msk;
  }

  // Blockmode
  if (true == init.blockmode) {
    spi->CTRL1 |= SPI_CTRL1_BLOCKMODE_Msk;
  } else {
    spi->CTRL1 &= ~SPI_CTRL1_BLOCKMODE_Msk;
  }

  // BMStart (always true/enabled for now)
  spi->CTRL1 |= SPI_CTRL1_BMSTART_Msk;

  // BMStall
  if (true == init.bmstall) {
    spi->CTRL1 |= SPI_CTRL1_BMSTALL_Msk;
  } else {
    spi->CTRL1 &= ~SPI_CTRL1_BMSTALL_Msk;
  }

  // Master delayed capture (MDLYCAP)
  if (true == init.mdlycap) {
    spi->CTRL1 |= SPI_CTRL1_MDLYCAP_Msk;
  } else {
    spi->CTRL1 &= ~SPI_CTRL1_MDLYCAP_Msk;
  }

  // Enable SPI and interrupts
  spi->RXFIFOIRQTRG = 1;
  spi->IRQ_ENB |= (SPI_IRQ_ENB_RORIM_Msk |  // SPI_IRQ_ENB_RTIM_Msk |
                   SPI_IRQ_ENB_RXIM_Msk);
  EnableRxIrq(bank);
  spi->CTRL1 |= SPI_CTRL1_ENABLE_Msk;
  aIsSpiInitialized[bank] = true;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Setup a SPI bank as master, simplified params
 **
 ** @param  spi - pointer to a SPI peripheral
 **
 ** @param  mode - SPI clocking mode, en_spi_clkmode_0, 1, 2, or 3
 **
 ** @param  wordLen - SPI word length, valid range 4-16 bits
 **
 ** @param  clkDiv - clock divider, spiclk = periphclk / clkDiv - must be even
 **
 ******************************************************************************/
hal_status_t HAL_Spi_InitMaster(VOR_SPI_Type* const spi, en_spi_clkmode_t mode, uint8_t wordLen,
                                uint16_t clkDiv) {
  hal_spi_init_t init;
  init.blockmode = true;
  init.bmstall = false;
  init.clkDiv = clkDiv;
  init.loopback = false;
  init.mdlycap = false;
  init.mode = mode;
  init.ms = en_spi_ms_master;
  init.wordLen = wordLen;

  return HAL_Spi_Init(spi, init);
}

/*******************************************************************************
 **
 ** @brief  Setup a SPI bank as slave, simplified params
 **
 ** @param  spi - pointer to a SPI peripheral
 **
 ** @param  mode - SPI clocking mode, en_spi_clkmode_0, 1, 2, or 3
 **
 ** @param  wordLen - SPI word length, valid range 4-16 bits
 **
 ** @param  ss - slave select index
 **
 ******************************************************************************/
hal_status_t HAL_Spi_InitSlave(VOR_SPI_Type* const spi, en_spi_clkmode_t mode, uint8_t wordLen,
                               uint8_t ss) {
  hal_spi_init_t init;
  init.blockmode = false;
  init.bmstall = false;
  init.clkDiv = 12;
  init.loopback = false;
  init.mdlycap = false;
  init.mode = mode;
  init.ms = en_spi_ms_slave;
  init.slaveSelect = ss;
  init.wordLen = wordLen;

  return HAL_Spi_Init(spi, init);
}

/*******************************************************************************
 **
 ** @brief  Adds a transaction to the SPI transaction queue for the specified SPI
 **
 ** @param  spi - pointer to a SPI peripheral
 **
 ** @param  xfer - pointer to a SPI transaction struct
 **
 ******************************************************************************/
hal_status_t HAL_Spi_Xfer(VOR_SPI_Type* const spi, spi_xfer_t* xfer) {
  uint32_t bank = GetBankNum(spi);

  // SPI check
  c_assert(bank <= SPI_MAX_BANKNUM);
  if (false == (bank <= SPI_MAX_BANKNUM)) {
    return hal_status_badParam;  // invalid uart bank
  }
  c_assert(aIsSpiInitialized[bank] == true);
  if (aIsSpiInitialized[bank] == false) {
    return hal_status_notInitialized;
  }

  // transfer NULL check
  c_assert(xfer != NULL);
  if (xfer == NULL) {
    return hal_status_badParam;
  }

  // transfer length check (cannot be zero)
  c_assert(xfer->len > 0);
  if (xfer->len == 0) {
    return hal_status_badParam;
  }

  // Slave select check
  c_assert(xfer->ss < spiNumSlaveSelects[bank]);
  if (xfer->ss >= spiNumSlaveSelects[bank]) {
    return hal_status_badParam;
  }

  // Transfer complete flag to false
  if (xfer->complete != NULL) {
    *xfer->complete = false;
  }

  // Add transfer to queue
  DisableRxIrq(bank);
  if (VOR_CircularBuffer_Write(&vcbSpiTxns[bank], xfer) != 0) {
    EnableRxIrq(bank);
    return hal_status_bufFull;
  }

  // If the SPI is idle, put into finished/begin state, then run state machine
  if (states[bank] == en_spi_state_idle) {
    states[bank] = en_spi_state_finished;
    SPI_StateMachine(bank);  // kick off the xfer
  }
  EnableRxIrq(bank);

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Starts a SPI transfer, blocking until it completes (16 bit buffers)
 **
 ** @param  spi - pointer to SPI peripheral (in)
 **
 ** @param  txBuf - pointer to transmit buffer (in) (or NULL for rx only, send pump)
 **
 ** @param  rxBuf - pointer to receive buffer (out) (or NULL for tx only)
 **
 ** @param  len - number of words to send/receive (in)
 **
 ** @param  bmstop - if blockmode and bmstall enabled, deassert CSn at end of txn
 **
 ** @param  ss  - slave select index to use
 **
 ******************************************************************************/
hal_status_t HAL_Spi_XferB16(VOR_SPI_Type* const spi, uint16_t* txBuf, uint16_t* rxBuf,
                             uint16_t len, bool bmstop, uint8_t ss) {
  hal_status_t status;
  spi_xfer_t xfer;
  volatile bool xferComplete = false;

  xfer.bmstop = bmstop;
  xfer.callbackIdx = eventHandlerIdx_null;
  xfer.complete = &xferComplete;
  xfer.len = len;
  xfer.rxbuf = (void*)rxBuf;
  xfer.txbuf = (void*)txBuf;
  xfer.bufInc16 = 1;
  xfer.ss = ss;

  status = HAL_Spi_Xfer(spi, &xfer);
  if (status != hal_status_ok) {
    return status;
  }
  while (!xferComplete) {
    // TODO: timeout?
    __WFI();
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Starts a SPI transfer, blocking until it completes (8 bit buffers)
 **
 ** @param  spi - pointer to SPI peripheral (in)
 **
 ** @param  txBuf - pointer to transmit buffer (in) (or NULL for rx only, send pump)
 **
 ** @param  rxBuf - pointer to receive buffer (out) (or NULL for tx only)
 **
 ** @param  len - number of words to send/receive (in)
 **
 ** @param  bmstop - if blockmode and bmstall enabled, deassert CSn at end of txn
 **
 ** @param  ss  - slave select index to use
 **
 ******************************************************************************/
hal_status_t HAL_Spi_XferB8(VOR_SPI_Type* const spi, uint8_t* txBuf, uint8_t* rxBuf, uint16_t len,
                            bool bmstop, uint8_t ss) {
  hal_status_t status;
  spi_xfer_t xfer;
  volatile bool xferComplete = false;

  xfer.bmstop = bmstop;
  xfer.callbackIdx = eventHandlerIdx_null;
  xfer.complete = &xferComplete;
  xfer.len = len;
  xfer.rxbuf = (void*)rxBuf;
  xfer.txbuf = (void*)txBuf;
  xfer.bufInc16 = 0;
  xfer.ss = ss;

  status = HAL_Spi_Xfer(spi, &xfer);
  if (status != hal_status_ok) {
    return status;
  }
  while (!xferComplete) {
    // TODO: timeout?
    __WFI();
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Adds a transaction to the SPI transaction queue for the specified SPI.
 **          This transfer will use DMA.
 **
 ** @param  spi - pointer to a SPI peripheral
 **
 ** @param  xfer - pointer to a SPI transaction struct
 **
 ******************************************************************************/
hal_status_t HAL_Spi_XferDma(VOR_SPI_Type* const spi, spi_xfer_t* xfer) {
  // CURRENTLY NOT IMPLEMENTED
  return hal_status_notImplemented;
}

/*******************************************************************************
 **
 ** @brief  De-initializes a SPI peripheral. Declocks and puts the SPI into reset
 **
 ** @param  spi - pointer to SPI peripheral to de-init (SPI0-3)
 **
 ******************************************************************************/
hal_status_t HAL_Spi_DeInit(VOR_SPI_Type* const spi) {
  uint32_t bank = GetBankNum(spi);

  c_assert(bank <= SPI_MAX_BANKNUM);
  if (false == (bank <= SPI_MAX_BANKNUM)) {
    return hal_status_badParam;  // invalid spi bank
  }

  c_assert(aIsSpiInitialized[bank] == true);
  if (aIsSpiInitialized[bank] == false) {
    return hal_status_notInitialized;  // attempt to de-init a periph that's already not init
  }

  DisableRxIrq(bank);
  DisableTxIrq(bank);

  // Put SPI into reset and then deassert clock
  switch (bank) {
    case SPI0_BANK:
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_SPI0;
      break;

    case SPI1_BANK:
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI1_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_SPI1;
      break;

    case SPI2_BANK:
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI2_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_SPI2;
      break;

    case SPI3_BANK:
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI3_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_SPI3;
      break;

    default:
      // invalid spi bank
      return hal_status_badParam;
  }
  aIsSpiInitialized[bank] = false;
  states[bank] = en_spi_state_idle;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Checks if a SPI bank is busy or not. Not busy means no xfer is active,
 **           and that there is nothing in the xfer queue. Can be used to time a
 **           re-init or de-init of the SPI until all xfers are finished
 **
 ** @param  spi [in] - pointer to SPI peripheral to check
 **
 ** @param  busy [out] - set to true if the SPI is busy, else set to false
 **
 ******************************************************************************/
hal_status_t HAL_Spi_IsBusy(VOR_SPI_Type* const spi, bool* busy) {
  uint32_t bank = GetBankNum(spi);

  c_assert(bank <= SPI_MAX_BANKNUM);
  if (false == (bank <= SPI_MAX_BANKNUM)) {
    return hal_status_badParam;  // invalid spi bank
  }

  // null check
  c_assert(busy != NULL);
  if (busy == NULL) {
    return hal_status_badParam;  // invalid busy flag pointer
  }

  // check that SPI state is in the idle state
  if (states[bank] == en_spi_state_idle) {
    *busy = false;
  } else {
    *busy = true;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set CLKPRESCALE and SCRDV bits according to div value (1-65024)
 **
 ** @param  div - clk divide value, div must be even
 **
 ******************************************************************************/
static hal_status_t SetClkDiv(VOR_SPI_Type* const spi, uint16_t div) {
  uint32_t scrdv_val = 0, prescale_val = 0;
  uint32_t i;

  c_assert(div != 0);
  if (div == 0) {
    // illegal clkdiv
    return hal_status_badParam;
  }

  // find largest (even) prescale value that divides into div
  for (i = 2; i <= 0xfe; i += 2) {
    if ((div % i) == 0) {
      prescale_val = i;
    }
  }
  if (prescale_val == 0) {
    // divide value must be even
    return hal_status_badParam;
  }
  div /= prescale_val;
  // dbgprintln("SPI clkdiv prescale val: %d", prescale_val);

  // scrdv
  if (div <= 256) {
    scrdv_val = div - 1;
  } else {
    // dbgprintln("invalid SPI clkdiv value: %d", div);
    return hal_status_badParam;
  }

  // set registers
  spi->CLKPRESCALE = prescale_val;
  spi->CTRL0 &= ~(SPI_CTRL0_SCRDV_Msk);
  spi->CTRL0 |= (scrdv_val << SPI_CTRL0_SCRDV_Pos);

  return hal_status_ok;
}

//
// State Machine / Interrupt Handlers
//

__STATIC_INLINE void LoadTxFifo(uint32_t bank) {
  uint32_t wrWord;
  while (VOR_SPI->BANK[bank].STATUS & SPI_STATUS_TNF_Msk) {
    if (sent[bank] == currentXfer[bank].len) {
      break;
    }
    if (currentXfer[bank].txbuf != NULL) {
      if (currentXfer[bank].bufInc16) {
        wrWord = *(uint16_t*)currentXfer[bank].txbuf;
        currentXfer[bank].txbuf = (void*)(((uint32_t)currentXfer[bank].txbuf) + 2);
      } else {
        wrWord = *(uint8_t*)currentXfer[bank].txbuf;
        currentXfer[bank].txbuf = (void*)(((uint32_t)currentXfer[bank].txbuf) + 1);
      }
    } else {
      wrWord = SPI_PUMPWORD;
    }
    if ((sent[bank] == (currentXfer[bank].len - 1)) && (currentXfer[bank].bmstop)) {
      wrWord |= SPI_DATA_BMSTART_BMSTOP_Msk;
    }
    VOR_SPI->BANK[bank].DATA = wrWord;
    sent[bank]++;
  }
}

__STATIC_INLINE void SPI_StateMachine(uint32_t bank) {
  static bool voidReadNext[NUM_SPI_BANKS] = {false, false, false, false};
  volatile uint32_t voidRead = 0;
  (void)voidRead;
  c_assert(bank <= SPI_MAX_BANKNUM);
  switch (states[bank]) {
    case en_spi_state_idle:
      // master: nothing to do

      // if slave, and received something when idle (no rxbuf), throw it away
      while (VOR_SPI->BANK[bank].STATUS & SPI_STATUS_RNE_Msk) {
        voidRead = VOR_SPI->BANK[bank].DATA;
      }
      break;
    case en_spi_state_pending:
      // load tx fifo
      LoadTxFifo(bank);

      // pull data off of the rx fifo
      while ((VOR_SPI->BANK[bank].STATUS & SPI_STATUS_RNE_Msk) &&
             (received[bank] < currentXfer[bank].len)) {
        if ((currentXfer[bank].rxbuf != NULL) && (!voidReadNext[bank])) {
          if (currentXfer[bank].bufInc16) {
            *(uint16_t*)currentXfer[bank].rxbuf = VOR_SPI->BANK[bank].DATA;
            currentXfer[bank].rxbuf = (void*)(((uint32_t)currentXfer[bank].rxbuf) + 2);
          } else {
            *(uint8_t*)currentXfer[bank].rxbuf = VOR_SPI->BANK[bank].DATA;
            currentXfer[bank].rxbuf = (void*)(((uint32_t)currentXfer[bank].rxbuf) + 1);
          }
          received[bank]++;
        } else {
          voidRead = VOR_SPI->BANK[bank].DATA;
          if (voidReadNext[bank]) {
            voidReadNext[bank] = false;
          } else {
            received[bank]++;
          }
        }
      }

      // if notdone
      if (received[bank] < currentXfer[bank].len - 1) {
        break;
      }
      if (received[bank] == currentXfer[bank].len - 1) {
        if ((currentXfer[bank].bmstop == false) &&
            (VOR_SPI->BANK[bank].CTRL1 & SPI_CTRL1_BMSTALL_Msk)) {
          // if BMSTOP=false and BMSTALL=true, will only receive len-1 words due to HW bug
          voidReadNext[bank] = true;  // will rx an extra word at beginning of next txn
        } else {
          break;
        }
      }
      // else
      // mark current xfer as done
      if (currentXfer[bank].complete != NULL) {
        *currentXfer[bank].complete = true;
      }
      states[bank] = en_spi_state_finished;
      // execute callback
      if (currentXfer[bank].callbackIdx != eventHandlerIdx_null) {
        apEventHList[currentXfer[bank].callbackIdx]();
      }
      // fall through to the next case (run FINISHED)
      // no break
    case en_spi_state_finished:
      // look for an xfer on the queue, pull it off of the queue, fill the txfifo
      if (VOR_CircularBuffer_IsEmpty(&vcbSpiTxns[bank])) {
        states[bank] = en_spi_state_idle;
      } else {
        // read new xfer
        VOR_CircularBuffer_Read(&vcbSpiTxns[bank], &currentXfer[bank]);
        sent[bank] = 0;
        received[bank] = 0;

        // set slave select (if it needs to change)
        if (((VOR_SPI->BANK[bank].CTRL1 & SPI_CTRL1_SS_Msk) >> SPI_CTRL1_SS_Pos) !=
            currentXfer[bank].ss) {
          VOR_SPI->BANK[bank].CTRL1 &= ~SPI_CTRL1_SS_Msk;
          VOR_SPI->BANK[bank].CTRL1 |= (currentXfer[bank].ss << SPI_CTRL1_SS_Pos);
        }

        // load tx fifo (wait to tx until loaded)
        VOR_SPI->BANK[bank].CTRL1 |= SPI_CTRL1_MTXPAUSE_Msk;
        LoadTxFifo(bank);
        VOR_SPI->BANK[bank].CTRL1 &= ~SPI_CTRL1_MTXPAUSE_Msk;

        states[bank] = en_spi_state_pending;
      }
      break;
    default:
      // error - abort
      states[bank] = en_spi_state_idle;
      VOR_SPI->BANK[bank].FIFO_CLR = SPI_FIFO_CLR_RXFIFO_Msk | SPI_FIFO_CLR_TXFIFO_Msk;
      break;
  }
}

void VORIRQ_SPI0_RX_IRQHandler(void) { SPI_StateMachine(SPI0_BANK); }
void VORIRQ_SPI1_RX_IRQHandler(void) { SPI_StateMachine(SPI1_BANK); }
void VORIRQ_SPI2_RX_IRQHandler(void) { SPI_StateMachine(SPI2_BANK); }
void VORIRQ_SPI3_RX_IRQHandler(void) { SPI_StateMachine(SPI3_BANK); }

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
