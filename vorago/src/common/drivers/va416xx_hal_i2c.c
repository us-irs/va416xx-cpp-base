/***************************************************************************************
 * @file     va416xx_hal_i2c.c
 * @version  V0.5
 * @date     19 July 2019
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

#include "va416xx_hal_i2c.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define CLK_100K (100000)
#define CLK_400K (400000)

#define I2C_PERID (0x021407E9)
#define NUM_I2CS (3)
#define I2C_MAX_BANKNUM (NUM_I2CS - 1)  // I2C0-1
#define I2C_FIFO_SZ (16)
#define I2C_INVALID_BANKNUM (0xff)  // bad index

/** I2C Software timeout (not CLKLOTO) */
#ifndef I2C_TIMEOUT_MS
#define I2C_TIMEOUT_MS (100)
#endif

/** Analog glitch filter */
#ifndef I2CM_ALGFILT
#define I2CM_ALGFILT 0
#endif

/** Digital glitch filter */
#ifndef I2CM_DLGFILT
#define I2CM_DLGFILT 1
#endif

/** Master mode callbacks */
#ifndef HAL_I2C0_XFER_COMPLETE_CALLBACK
#define HAL_I2C0_XFER_COMPLETE_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C1_XFER_COMPLETE_CALLBACK
#define HAL_I2C1_XFER_COMPLETE_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C2_XFER_COMPLETE_CALLBACK
#define HAL_I2C2_XFER_COMPLETE_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C0_XFER_TIMEOUT_CALLBACK
#define HAL_I2C0_XFER_TIMEOUT_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C1_XFER_TIMEOUT_CALLBACK
#define HAL_I2C1_XFER_TIMEOUT_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C2_XFER_TIMEOUT_CALLBACK
#define HAL_I2C2_XFER_TIMEOUT_CALLBACK() ((void)0)
#endif

/** Slave mode callbacks */
#ifndef HAL_I2C0_S0_RX_THRESH_CALLBACK
#define HAL_I2C0_S0_RX_THRESH_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C1_S0_RX_THRESH_CALLBACK
#define HAL_I2C1_S0_RX_THRESH_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C2_S0_RX_THRESH_CALLBACK
#define HAL_I2C2_S0_RX_THRESH_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C0_S0_TXCOMPLETE_CALLBACK
#define HAL_I2C0_S0_TXCOMPLETE_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C1_S0_TXCOMPLETE_CALLBACK
#define HAL_I2C1_S0_TXCOMPLETE_CALLBACK() ((void)0)
#endif

#ifndef HAL_I2C2_S0_TXCOMPLETE_CALLBACK
#define HAL_I2C2_S0_TXCOMPLETE_CALLBACK() ((void)0)
#endif

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

typedef struct {
  uint32_t startTimeMs;
  char *wrBuf;
  char *rdBuf;
  uint16_t wrLen;
  uint16_t rdLen;
} i2c_mst_txn_t;

typedef struct {
  char *txBuf;
  char *rxBuf;
  uint16_t txLen;
  uint16_t rxLen;
  uint16_t rxThresh;
  uint16_t rxCount;
} i2c_sl_txn_t;

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static i2c_mst_txn_t aI2C[NUM_I2CS];
static i2c_sl_txn_t aI2C_S[NUM_I2CS];

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

__STATIC_INLINE uint8_t GetBankNum(VOR_I2C_Type *const i2c);
__STATIC_INLINE void CheckForBusFault(VOR_I2C_Type *const i2c);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Takes a pointer to a I2C peripheral and returns its bank number
 **
 ** @param  i2c - pointer to I2C peripheral
 **
 ** @return uint32_t the bank number for this I2C
 **
 ******************************************************************************/
__STATIC_INLINE uint8_t GetBankNum(VOR_I2C_Type *const i2c) {
  if (VOR_I2C0 == i2c) {
    return 0;
  } else if (VOR_I2C1 == i2c) {
    return 1;
  } else if (VOR_I2C2 == i2c) {
    return 2;
  } else {
    // invalid i2c bank
    return I2C_INVALID_BANKNUM;
  }
}

/*******************************************************************************
 **
 ** @brief  Checks for SDA low and attempts to free the bus if stuck low (master only)
 **
 ** @param  i2c - pointer to I2C peripheral
 **
 ** @return none
 **
 ******************************************************************************/
__STATIC_INLINE void CheckForBusFault(VOR_I2C_Type *const i2c) {
  uint16_t i, retry = 5;
  if (!(i2c->CTRL & I2C_CTRL_ENABLE_Msk)) {
    return;
  }  // master not enabled
  while (((i2c->STATUS & I2C_STATUS_RAW_SDA_Msk) == 0) && retry--) {
    // bus fault condition - try and free the bus
    i2c->CMD = 0x4;  // cancel
    i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
    i2c->WORDS = 1;
    i2c->ADDRESS = 0x01;  // read
    i2c->CMD = 0x3;       // start with stop
    for (i = 0xffff; i; i--) {
      if (i2c->STATUS & I2C_STATUS_IDLE_Msk) {
        break;
      }
    }
  }
  i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
}

/*******************************************************************************
 **
 ** @brief  Enable the clock and reset an I2C peripheral to default state
 **
 ** @param  i2c pointer to I2C peripheral
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2C_Reset(VOR_I2C_Type *const i2c) {
  uint8_t bank = GetBankNum(i2c);

  if (bank == I2C_INVALID_BANKNUM) {
    return hal_status_badParam;
  }
  NVIC_DisableIRQ((IRQn_Type)(I2C0_MS_IRQn + 2 * bank));
  NVIC_DisableIRQ((IRQn_Type)(I2C0_SL_IRQn + 2 * bank));

  // Enable clock and reset peripheral
  switch ((uint32_t)i2c) {
    case VOR_I2C0_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_I2C0;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~(CLK_ENABLE_I2C0);
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= CLK_ENABLE_I2C0;
      break;
    case VOR_I2C1_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_I2C1;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~(CLK_ENABLE_I2C1);
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= CLK_ENABLE_I2C1;
      break;
    case VOR_I2C2_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_I2C2;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~(CLK_ENABLE_I2C2);
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= CLK_ENABLE_I2C2;
      break;
    default:
      return hal_status_badParam;
  }

  if (i2c->PERID != I2C_PERID) {
    return hal_status_badPeriphID;  // not running on correct hw, or a register issue
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Initialize I2C master
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  init I2C master init structure
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_Init(VOR_I2C_Type *const i2c, stc_i2c_masterInit_t const init) {
  hal_status_t status;
  uint8_t bank = GetBankNum(i2c);

  // ensure clocked, and can read perid
  status = HAL_I2C_Enclock(i2c);
  if (status != hal_status_ok) {
    return status;
  }
  if (i2c->PERID != I2C_PERID) {
    return hal_status_badPeriphID;  // not running on correct hw, or a register issue
  }

  // set speed
  switch (init.speed) {
    case en_i2c_100k:
      // set standard mode 20x clock
      i2c->CLKSCALE = (I2C_CLK / CLK_100K / 20) - 1;
      break;
    case en_i2c_400k:
      // set fast mode 25x clock
      if (I2C_CLK < 10000000) {
        return hal_status_initError;
      }  // cannot do fast mode below 10mhz bus clk
      i2c->CLKSCALE = ((I2C_CLK / CLK_400K / 25) - 1) | 0x80000000;
      break;
    case en_i2c_max:
      // set fast mode 25x clock (max rate div 1)
      i2c->CLKSCALE = 0x80000000;
      break;
    default:
      return hal_status_badParam;
  }

  i2c->CTRL = 0;
  i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
  i2c->RXFIFOIRQTRG = 0x8;  // irq RX half full
  i2c->TXFIFOIRQTRG = 0x8;  // irq TX half empty

  i2c->IRQ_ENB = 0;
  NVIC_EnableIRQ((IRQn_Type)(I2C0_MS_IRQn + 2 * bank));

  if (init.txfemd) {
    i2c->CTRL |= I2C_CTRL_TXFEMD_Msk;
  }
  if (init.rxffmd) {
    i2c->CTRL |= I2C_CTRL_RXFFMD_Msk;
  }
  if (I2CM_ALGFILT) {
    i2c->CTRL |= I2C_CTRL_ALGFILTER_Msk;
  }
  if (I2CM_DLGFILT) {
    i2c->CTRL |= I2C_CTRL_DLGFILTER_Msk;
  }
  i2c->CTRL |= I2C_CTRL_ENABLE_Msk;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Initialize I2C slave
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  init I2C slave init structure
 **
 ** @return hal_status_t status of the driver call
 **
 ** @note   this func and init struct currently does not use the addressB feature
 **
 ******************************************************************************/

hal_status_t HAL_I2CS_Init(VOR_I2C_Type *const i2c, stc_i2c_slaveInit_t const init) {
  hal_status_t status;
  uint8_t bank = GetBankNum(i2c);

  // ensure clocked, and can read perid
  status = HAL_I2C_Enclock(i2c);
  if (status != hal_status_ok) {
    return status;
  }
  if (i2c->PERID != I2C_PERID) {
    return hal_status_badPeriphID;  // not running on correct hw, or a register issue
  }

  i2c->S0_CTRL = 0;
  i2c->S0_FIFO_CLR = I2C_S0_FIFO_CLR_RXFIFO_Msk | I2C_S0_FIFO_CLR_TXFIFO_Msk;
  i2c->S0_RXFIFOIRQTRG = 0x8;  // irq RX half full
  i2c->S0_TXFIFOIRQTRG = 0x8;  // irq TX half empty

  i2c->S0_ADDRESS = (init.slaveAddr & 0x7ff) | 0x1;  // preshifted, set r/w bit
  if (init.tenBitMode) {
    i2c->S0_ADDRESS |= I2C_S0_ADDRESS_A10MODE_Msk;
  }  // TODO: needs headerfile update
  if (init.addrMaskEnable) {
    i2c->S0_ADDRESSMASK = (init.addrMask & 0x7fe);
  } else {
    i2c->S0_ADDRESSMASK = 0x7fe;
  }

  i2c->S0_IRQ_ENB = 0;
  NVIC_EnableIRQ((IRQn_Type)(I2C0_SL_IRQn + 2 * bank));

  if (init.txfemd) {
    i2c->S0_CTRL |= I2C_S0_CTRL_TXFEMD_Msk;
  }
  if (init.rxffmd) {
    i2c->S0_CTRL |= I2C_S0_CTRL_RXFFMD_Msk;
  }
  i2c->S0_CTRL |= I2C_S0_CTRL_ENABLE_Msk;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Disable the clock to an I2C peripheral to save power
 **
 ** @param  i2c pointer to I2C peripheral
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2C_Declock(VOR_I2C_Type *const i2c) {
  switch ((uint32_t)i2c) {
    case VOR_I2C0_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_I2C0;
      break;
    case VOR_I2C1_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_I2C1;
      break;
    case VOR_I2C2_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_I2C2;
      break;
    default:
      return hal_status_badParam;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Enable the clock to an I2C peripheral but not reset (preserve state)
 **
 ** @param  i2c pointer to I2C peripheral
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2C_Enclock(VOR_I2C_Type *const i2c) {
  switch ((uint32_t)i2c) {
    case VOR_I2C0_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_I2C0;
      break;
    case VOR_I2C1_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_I2C1;
      break;
    case VOR_I2C2_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_I2C2;
      break;
    default:
      return hal_status_badParam;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Perform a blocking write in master mode
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  dataPtr pointer to write buffer
 ** @param  len number of bytes to write
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_WriteB(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *dataPtr,
                             uint32_t len) {
  hal_status_t status;
  status = HAL_I2CM_Write(i2c, i2cAddr, dataPtr, len);
  if (status == hal_status_ok) {
    do {
      status = HAL_I2CM_GetXferStatus(i2c);
    } while (status == hal_status_busy);
  }
  return status;
}

/*******************************************************************************
 **
 ** @brief  Perform a blocking write in master mode without using interrupts
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  dataPtr pointer to write buffer
 ** @param  len number of bytes to write
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_WriteBNoInt(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *dataPtr,
                                  uint32_t len) {
  uint32_t tStart = (uint32_t)HAL_time_ms;

  // check for possible errors
  if (HAL_I2C_Enclock(i2c) != hal_status_ok) {
    return hal_status_badParam;
  }
  if (!len) {
    return hal_status_badParam;
  }
  if (dataPtr == NULL) return hal_status_badParam;
  if ((i2c->STATUS & (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) !=
      (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) {
    return hal_status_busy;  // a transaction is already in progress
  }

  i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
  i2c->WORDS = len;
  i2c->ADDRESS = i2cAddr & 0xFFFFFFFE;
  while (len) {
    i2c->DATA = *dataPtr++;
    len--;
    if ((i2c->STATUS & I2C_STATUS_TXNFULL_Msk) == 0) {
      break;
    }
  }
  i2c->CMD = 0x3;  // start with stop
  while (i2c->STATUS & I2C_STATUS_IDLE_Msk) {
  }  // wait for xfer to start
  while ((i2c->STATUS & I2C_STATUS_IDLE_Msk) == 0) {
    if ((len > 0) && ((i2c->STATUS & I2C_STATUS_TXNFULL_Msk) != 0)) {
      i2c->DATA = *dataPtr++;
      len--;
    }
    if ((uint32_t)HAL_time_ms == (tStart + I2C_TIMEOUT_MS)) {
      // timeout condition
      i2c->CMD = 0x4;  // cancel
      CheckForBusFault(i2c);
      return hal_status_timeout;
    }
  }
  if ((i2c->STATUS & (I2C_STATUS_NACKADDR_Msk | I2C_STATUS_NACKDATA_Msk)) != 0) {
    // NACK condition
    i2c->FIFO_CLR = I2C_S0_FIFO_CLR_TXFIFO_Msk;
    return hal_status_nak;
  }
  if ((i2c->STATUS & I2C_STATUS_ARBLOST_Msk) != 0) {
    // Arbitration lost
    i2c->FIFO_CLR = I2C_S0_FIFO_CLR_TXFIFO_Msk;
    return hal_status_arblost;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Perform a blocking read in master mode
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  dataPtr pointer to read buffer
 ** @param  len number of bytes to read
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_ReadB(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *dataPtr,
                            uint32_t len) {
  hal_status_t status;
  status = HAL_I2CM_Read(i2c, i2cAddr, dataPtr, len);
  if (status == hal_status_ok) {
    do {
      status = HAL_I2CM_GetXferStatus(i2c);
    } while (status == hal_status_busy);
  }
  return status;
}

/*******************************************************************************
 **
 ** @brief  Perform a blocking read in master mode without using interrupts
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  dataPtr pointer to read buffer
 ** @param  len number of bytes to read
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_ReadBNoInt(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *dataPtr,
                                 uint32_t len) {
  uint32_t tStart = (uint32_t)HAL_time_ms;

  // check for possible errors
  if (HAL_I2C_Enclock(i2c) != hal_status_ok) {
    return hal_status_badParam;
  }
  if (!len) {
    return hal_status_badParam;
  }
  if (dataPtr == NULL) return hal_status_badParam;
  if ((i2c->STATUS & (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) !=
      (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) {
    return hal_status_busy;  // a transaction is already in progress
  }

  i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
  i2c->WORDS = len;
  i2c->ADDRESS = i2cAddr | 0x1;  // OR in read bit
  i2c->CMD = 0x3;                // start with stop

  while ((i2c->STATUS & I2C_STATUS_IDLE_Msk) == 0) {  // wait for IDLE condition
    if ((len > 0) && ((i2c->STATUS & I2C_STATUS_RXNEMPTY_Msk) != 0)) {
      *dataPtr++ = i2c->DATA;
      len--;
    }
    if ((uint32_t)HAL_time_ms == (tStart + I2C_TIMEOUT_MS)) {
      // timeout condition
      i2c->CMD = 0x4;  // cancel
      CheckForBusFault(i2c);
      return hal_status_timeout;
    }
  }
  if ((i2c->STATUS & I2C_STATUS_NACKADDR_Msk) != 0) {
    // NACK condition
    i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk;
    return hal_status_nak;
  }
  while (len) {
    if ((i2c->STATUS & I2C_STATUS_RXNEMPTY_Msk) != 0) {
      *dataPtr++ = i2c->DATA;
      len--;
    } else {
      // RX fifo empty earlier than expected
      return hal_status_rxError;
    }
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Perform a blocking write-read in master mode
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  wrDataPtr pointer to write buffer (register address)
 ** @param  wrLen number of bytes to write (typically 1 or 2)
 ** @param  rdDataPtr pointer to read buffer
 ** @param  rdLen number of bytes to read
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_WrReadB(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *wrDataPtr,
                              uint32_t wrLen, char *rdDataPtr, uint32_t rdLen) {
  hal_status_t status;
  status = HAL_I2CM_WrRead(i2c, i2cAddr, wrDataPtr, wrLen, rdDataPtr, rdLen);
  if (status == hal_status_ok) {
    do {
      status = HAL_I2CM_GetXferStatus(i2c);
    } while (status == hal_status_busy);
  }
  return status;
}

/*******************************************************************************
 **
 ** @brief  Perform a blocking write-read in master mode without using interrupts
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  wrDataPtr pointer to write buffer (register address)
 ** @param  wrLen number of bytes to write (typically 1 or 2)
 ** @param  rdDataPtr pointer to read buffer
 ** @param  rdLen number of bytes to read
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_WrReadBNoInt(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr,
                                   char *wrDataPtr, uint32_t wrLen, char *rdDataPtr,
                                   uint32_t rdLen) {
  uint32_t tStart = (uint32_t)HAL_time_ms;

  // check for possible errors
  if (HAL_I2C_Enclock(i2c) != hal_status_ok) {
    return hal_status_badParam;
  }
  if (!wrLen) {
    return hal_status_badParam;
  }
  if (!rdLen) {
    return hal_status_badParam;
  }
  if ((wrDataPtr == NULL) || (rdDataPtr == NULL)) return hal_status_badParam;
  if ((i2c->STATUS & (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) !=
      (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) {
    return hal_status_busy;  // a transaction is already in progress
  }

  i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
  i2c->WORDS = wrLen;
  i2c->ADDRESS = i2cAddr & 0xFFFFFFFE;
  while (wrLen) {
    i2c->DATA = *wrDataPtr++;
    wrLen--;
    if ((i2c->STATUS & I2C_STATUS_TXNFULL_Msk) == 0) {
      break;
    }
  }
  i2c->CMD = 0x1;  // start without stop
  while ((i2c->STATUS & (I2C_STATUS_WAITING_Msk | I2C_STATUS_IDLE_Msk)) == 0) {
    if ((wrLen > 0) && ((i2c->STATUS & I2C_STATUS_TXNFULL_Msk) != 0)) {
      i2c->DATA = *wrDataPtr++;
      wrLen--;
    }
    if ((uint32_t)HAL_time_ms == (tStart + I2C_TIMEOUT_MS + 1)) {
      // timeout condition
      i2c->CMD = 0x4;  // cancel
      CheckForBusFault(i2c);
      return hal_status_timeout;
    }
  }
  if ((i2c->STATUS & (I2C_STATUS_NACKADDR_Msk | I2C_STATUS_NACKDATA_Msk)) != 0) {
    // NACK condition
    i2c->FIFO_CLR = I2C_S0_FIFO_CLR_TXFIFO_Msk;
    i2c->CMD = 0x2;  // stop
    return hal_status_nak;
  }
  if ((i2c->STATUS & I2C_STATUS_ARBLOST_Msk) != 0) {
    // Arbitration lost
    i2c->FIFO_CLR = I2C_S0_FIFO_CLR_TXFIFO_Msk;
    return hal_status_arblost;
  }
  i2c->WORDS = rdLen;
  i2c->ADDRESS = i2cAddr | 0x1;                       // OR in read bit
  i2c->CMD = 0x3;                                     // (repeated) start with stop
  while ((i2c->STATUS & I2C_STATUS_IDLE_Msk) == 0) {  // wait for IDLE condition
    if ((rdLen > 0) && ((i2c->STATUS & I2C_STATUS_RXNEMPTY_Msk) != 0)) {
      *rdDataPtr++ = i2c->DATA;
      rdLen--;
    }
    if ((uint32_t)HAL_time_ms == (tStart + I2C_TIMEOUT_MS + 1)) {
      // timeout condition
      i2c->CMD = 0x4;  // cancel
      CheckForBusFault(i2c);
      return hal_status_timeout;
    }
  }
  while (rdLen) {
    if ((i2c->STATUS & I2C_STATUS_RXNEMPTY_Msk) != 0) {
      *rdDataPtr++ = i2c->DATA;
      rdLen--;
    } else {
      // RX fifo empty earlier than expected
      return hal_status_rxError;
    }
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Perform a non-blocking write in master mode
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  dataPtr pointer to write buffer
 ** @param  len number of bytes to write
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_Write(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *dataPtr,
                            uint32_t len) {
  uint8_t bank = GetBankNum(i2c);

  // check for possible errors
  if (HAL_I2C_Enclock(i2c) != hal_status_ok) {
    return hal_status_badParam;
  }
  if (!len) {
    return hal_status_badParam;
  }
  if (dataPtr == NULL) return hal_status_badParam;
  if ((i2c->CTRL & I2C_CTRL_ENABLED_Msk) == 0) {
    return hal_status_notInitialized;
  }
  if ((i2c->STATUS & (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) !=
      (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) {
    return hal_status_busy;  // the I2C bus is busy
  }

  i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
  i2c->WORDS = len;
  i2c->ADDRESS = i2cAddr & 0xFFFFFFFE;
  while (len) {
    i2c->DATA = *dataPtr++;
    len--;
    if ((i2c->STATUS & I2C_STATUS_TXNFULL_Msk) == 0) {
      break;
    }
  }

  aI2C[bank].rdBuf = NULL;
  aI2C[bank].rdLen = 0;
  aI2C[bank].wrBuf = dataPtr;
  aI2C[bank].wrLen = len;
  aI2C[bank].startTimeMs = (uint32_t)HAL_time_ms;

  i2c->CMD = 0x3;  // start with stop
  while (i2c->STATUS & I2C_STATUS_IDLE_Msk)
    ;  // wait for xfer to start
  i2c->IRQ_CLR = 0xffffffff;
  i2c->IRQ_ENB = I2C_IRQ_ENB_IDLE_Msk | I2C_IRQ_ENB_CLKLOTO_Msk;
  if (len) {
    i2c->IRQ_ENB |= I2C_IRQ_ENB_TXREADY_Msk;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Perform a non-blocking read in master mode
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  dataPtr pointer to read buffer
 ** @param  len number of bytes to read
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_Read(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *dataPtr,
                           uint32_t len) {
  uint8_t bank = GetBankNum(i2c);

  // check for possible errors
  if (HAL_I2C_Enclock(i2c) != hal_status_ok) {
    return hal_status_badParam;
  }
  if (!len) {
    return hal_status_badParam;
  }
  if (dataPtr == NULL) return hal_status_badParam;
  if ((i2c->CTRL & I2C_CTRL_ENABLED_Msk) == 0) {
    return hal_status_notInitialized;
  }
  if ((i2c->STATUS & (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) !=
      (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) {
    return hal_status_busy;  // the I2C bus is busy
  }

  i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
  i2c->WORDS = len;
  i2c->ADDRESS = i2cAddr | 0x1;

  aI2C[bank].rdBuf = dataPtr;
  aI2C[bank].rdLen = len;
  aI2C[bank].wrBuf = NULL;
  aI2C[bank].wrLen = 0;
  aI2C[bank].startTimeMs = (uint32_t)HAL_time_ms;

  i2c->CMD = 0x3;  // start with stop
  while (i2c->STATUS & I2C_STATUS_IDLE_Msk)
    ;  // wait for xfer to start
  i2c->IRQ_CLR = 0xffffffff;
  i2c->IRQ_ENB = I2C_IRQ_ENB_IDLE_Msk | I2C_IRQ_ENB_RXREADY_Msk | I2C_IRQ_ENB_CLKLOTO_Msk;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Perform a non-blocking write-read in master mode
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  i2cAddr I2C slave address (pre-shifted)
 ** @param  wrDataPtr pointer to write buffer (register address)
 ** @param  wrLen number of bytes to write (typically 1 or 2)
 ** @param  rdDataPtr pointer to read buffer
 ** @param  rdLen number of bytes to read
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_WrRead(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *wrDataPtr,
                             uint32_t wrLen, char *rdDataPtr, uint32_t rdLen) {
  uint8_t bank = GetBankNum(i2c);

  // check for possible errors
  if (HAL_I2C_Enclock(i2c) != hal_status_ok) {
    return hal_status_badParam;
  }
  if (!wrLen) {
    return hal_status_badParam;
  }
  if (!rdLen) {
    return hal_status_badParam;
  }
  if ((wrDataPtr == NULL) || (rdDataPtr == NULL)) return hal_status_badParam;
  if ((i2c->CTRL & I2C_CTRL_ENABLED_Msk) == 0) {
    return hal_status_notInitialized;
  }
  if ((i2c->STATUS & (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) !=
      (I2C_STATUS_IDLE_Msk | I2C_STATUS_I2CIDLE_Msk)) {
    return hal_status_busy;  // the I2C bus is busy
  }

  i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
  i2c->WORDS = wrLen;
  i2c->ADDRESS = i2cAddr & 0xFFFFFFFE;
  while (wrLen) {
    i2c->DATA = *wrDataPtr++;
    wrLen--;
    if ((i2c->STATUS & I2C_STATUS_TXNFULL_Msk) == 0) {
      break;
    }
  }

  aI2C[bank].rdBuf = rdDataPtr;
  aI2C[bank].rdLen = rdLen;
  aI2C[bank].wrBuf = wrDataPtr;
  aI2C[bank].wrLen = wrLen;
  aI2C[bank].startTimeMs = (uint32_t)HAL_time_ms;

  i2c->CMD = 0x1;  // start without stop
  while (i2c->STATUS & I2C_STATUS_IDLE_Msk)
    ;  // wait for xfer to start
  i2c->IRQ_CLR = 0xffffffff;
  i2c->IRQ_ENB = I2C_IRQ_ENB_RXREADY_Msk | I2C_IRQ_ENB_CLKLOTO_Msk | I2C_IRQ_ENB_WAITING_Msk |
                 I2C_IRQ_ENB_IDLE_Msk;
  if (wrLen) {
    i2c->IRQ_ENB |= I2C_IRQ_ENB_TXREADY_Msk;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Get transfer status of an I2C peripheral
 **
 ** @param  i2c pointer to I2C peripheral
 **
 ** @return hal_status_t I2C status (ok if idle/complete)
 **
 ******************************************************************************/
hal_status_t HAL_I2CM_GetXferStatus(VOR_I2C_Type *const i2c) {
  uint8_t bank = GetBankNum(i2c);

  if ((i2c->CTRL & I2C_CTRL_ENABLED_Msk) == 0) {
    return hal_status_notInitialized;
  }
  if ((i2c->STATUS & I2C_STATUS_IDLE_Msk) == 0) {
    if ((uint32_t)HAL_time_ms == (aI2C[bank].startTimeMs + I2C_TIMEOUT_MS + 1)) {
      i2c->CMD = 0x4;  // cancel
      CheckForBusFault(i2c);
      return hal_status_timeout;  // I2C transfer timed out (bus fault)
    }
    return hal_status_busy;  // transfer in progress
  }
  if ((i2c->STATUS & I2C_STATUS_NACKADDR_Msk) != 0) {
    i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
    return hal_status_nak;  // non acknowledge (slave busy or not present or fault)
  }
  if (((i2c->STATUS & I2C_STATUS_NACKDATA_Msk) != 0) && ((i2c->ADDRESS & 0x1) == 0)) {
    // NACK data during write (a NACKDATA during a read is normal for the last byte)
    i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
    return hal_status_nak;  // non acknowledge (slave busy or not present or fault)
  }
  if ((i2c->STATUS & I2C_STATUS_ARBLOST_Msk) != 0) {
    i2c->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
    return hal_status_arblost;  // arbitration lost
  }
  return hal_status_ok;  // transfer complete, bus is idle
}

/*******************************************************************************
 **
 ** @brief  I2C0 master mode IRQ handler
 **
 ******************************************************************************/
void I2C0_MS_IRQHandler(void) __attribute__((weak));
void I2C0_MS_IRQHandler(void) {
  if (VOR_I2C0->IRQ_END & I2C_IRQ_END_IDLE_Msk) {
    // handle IDLE - transfer done

    // empty RX fifo even if below trigger level
    if (aI2C[0].rdBuf != NULL) {
      while (VOR_I2C0->STATUS & I2C_STATUS_RXNEMPTY_Msk) {
        if (aI2C[0].rdLen) {
          *aI2C[0].rdBuf++ = VOR_I2C0->DATA;
          aI2C[0].rdLen--;
        } else
          break;
      }
    }
    VOR_I2C0->IRQ_ENB = 0;
    HAL_I2C0_XFER_COMPLETE_CALLBACK();
  }
  if (VOR_I2C0->IRQ_END & I2C_IRQ_END_WAITING_Msk) {
    // handle WAITING - likely need to send repeated START for write-read xfer
    if (aI2C[0].rdLen && (aI2C[0].rdBuf != NULL)) {
      VOR_I2C0->WORDS = aI2C[0].rdLen;
      VOR_I2C0->ADDRESS |= 0x1;  // OR in read bit
      VOR_I2C0->CMD = 0x3;       // (repeated) start with stop
      VOR_I2C0->IRQ_ENB &=
          ~(I2C_IRQ_ENB_TXREADY_Msk | I2C_IRQ_ENB_WAITING_Msk);  // turn off TXREADY interrupt
      VOR_I2C0->IRQ_ENB |= I2C_IRQ_ENB_IDLE_Msk;
    } else {
      VOR_I2C0->CMD = 0x2;  // stop
    }
  }
  if (VOR_I2C0->IRQ_END & I2C_IRQ_END_CLKLOTO_Msk) {
    // handle SCL low timeout
    VOR_I2C0->CMD = 0x4;  // cancel
    VOR_I2C0->FIFO_CLR = I2C_FIFO_CLR_RXFIFO_Msk | I2C_FIFO_CLR_TXFIFO_Msk;
    VOR_I2C0->IRQ_ENB = 0;
    HAL_I2C0_XFER_TIMEOUT_CALLBACK();
  }
  if (VOR_I2C0->IRQ_END & I2C_IRQ_END_TXREADY_Msk) {
    // TX ready (below trigger level) - fill TX fifo if more to fill
    if ((aI2C[0].wrBuf != NULL) && aI2C[0].wrLen) {
      // there are things to write to the fifo
      while (aI2C[0].wrLen && (VOR_I2C0->STATUS & I2C_STATUS_TXNFULL_Msk)) {
        VOR_I2C0->DATA = *aI2C[0].wrBuf++;
        aI2C[0].wrLen--;
      }
    } else {
      // no more to write - kill this interrupt source
      VOR_I2C0->IRQ_ENB &= ~I2C_IRQ_ENB_TXREADY_Msk;
    }
  }
  if (VOR_I2C0->IRQ_END & I2C_IRQ_END_RXREADY_Msk) {
    // RX ready (above trigger level) - empty RX fifo
    if (aI2C[0].rdBuf != NULL) {
      while (aI2C[0].rdLen && (VOR_I2C0->STATUS & I2C_STATUS_RXNEMPTY_Msk)) {
        *aI2C[0].rdBuf++ = VOR_I2C0->DATA;
        aI2C[0].rdLen--;
      }
    }
  }
}
void I2C0_MS_RX_IRQHandler(void) { I2C0_MS_IRQHandler(); }
void I2C0_MS_TX_IRQHandler(void) { I2C0_MS_IRQHandler(); }

/*******************************************************************************
 **
 ** @brief  I2C1 master mode IRQ handler
 **
 ******************************************************************************/
void I2C1_MS_IRQHandler(void) __attribute__((weak));
void I2C1_MS_IRQHandler(void) {
  if (VOR_I2C1->IRQ_END & I2C_IRQ_END_IDLE_Msk) {
    // handle IDLE - transfer done

    // empty RX fifo even if below trigger level
    if (aI2C[1].rdBuf != NULL) {
      while (VOR_I2C1->STATUS & I2C_STATUS_RXNEMPTY_Msk) {
        if (aI2C[1].rdLen) {
          *aI2C[1].rdBuf++ = VOR_I2C1->DATA;
          aI2C[1].rdLen--;
        } else
          break;
      }
    }
    VOR_I2C1->IRQ_ENB = 0;
    HAL_I2C1_XFER_COMPLETE_CALLBACK();
  }
  if (VOR_I2C1->IRQ_END & I2C_IRQ_END_WAITING_Msk) {
    // handle WAITING - likely need to send repeated START for write-read xfer
    if (aI2C[1].rdLen && (aI2C[1].rdBuf != NULL)) {
      VOR_I2C1->WORDS = aI2C[1].rdLen;
      VOR_I2C1->ADDRESS |= 0x1;  // OR in read bit
      VOR_I2C1->CMD = 0x3;       // (repeated) start with stop
      VOR_I2C1->IRQ_ENB &=
          ~(I2C_IRQ_ENB_TXREADY_Msk | I2C_IRQ_ENB_WAITING_Msk);  // turn off TXREADY interrupt
      VOR_I2C1->IRQ_ENB |= I2C_IRQ_ENB_IDLE_Msk;
    } else {
      VOR_I2C1->CMD = 0x2;  // stop
    }
  }
  if (VOR_I2C1->IRQ_END & I2C_IRQ_END_CLKLOTO_Msk) {
    // handle SCL low timeout
    VOR_I2C1->CMD = 0x4;  // cancel
    VOR_I2C1->IRQ_ENB = 0;
    HAL_I2C1_XFER_TIMEOUT_CALLBACK();
  }
  if (VOR_I2C1->IRQ_END & I2C_IRQ_END_TXREADY_Msk) {
    // TX ready (below trigger level) - fill TX fifo if more to fill
    if ((aI2C[1].wrBuf != NULL) && aI2C[1].wrLen) {
      // there are things to write to the fifo
      while (aI2C[1].wrLen && (VOR_I2C1->STATUS & I2C_STATUS_TXNFULL_Msk)) {
        VOR_I2C1->DATA = *aI2C[1].wrBuf++;
        aI2C[1].wrLen--;
      }
    } else {
      // no more to write - kill this interrupt source
      VOR_I2C1->IRQ_ENB &= ~I2C_IRQ_ENB_TXREADY_Msk;
    }
  }
  if (VOR_I2C1->IRQ_END & I2C_IRQ_END_RXREADY_Msk) {
    // RX ready (above trigger level) - empty RX fifo
    if (aI2C[1].rdBuf != NULL) {
      while (aI2C[1].rdLen && (VOR_I2C1->STATUS & I2C_STATUS_RXNEMPTY_Msk)) {
        *aI2C[1].rdBuf++ = VOR_I2C1->DATA;
        aI2C[1].rdLen--;
      }
    }
  }
}
void I2C1_MS_RX_IRQHandler(void) { I2C1_MS_IRQHandler(); }
void I2C1_MS_TX_IRQHandler(void) { I2C1_MS_IRQHandler(); }

/*******************************************************************************
 **
 ** @brief  I2C2 master mode IRQ handler
 **
 ******************************************************************************/
void I2C2_MS_IRQHandler(void) {
  if (VOR_I2C2->IRQ_END & I2C_IRQ_END_IDLE_Msk) {
    // handle IDLE - transfer done

    // empty RX fifo even if below trigger level
    if (aI2C[2].rdBuf != NULL) {
      while (VOR_I2C2->STATUS & I2C_STATUS_RXNEMPTY_Msk) {
        if (aI2C[2].rdLen) {
          *aI2C[2].rdBuf++ = VOR_I2C2->DATA;
          aI2C[2].rdLen--;
        } else
          break;
      }
    }
    VOR_I2C2->IRQ_ENB = 0;
    HAL_I2C2_XFER_COMPLETE_CALLBACK();
  }
  if (VOR_I2C2->IRQ_END & I2C_IRQ_END_WAITING_Msk) {
    // handle WAITING - likely need to send repeated START for write-read xfer
    if (aI2C[2].rdLen && (aI2C[2].rdBuf != NULL)) {
      VOR_I2C2->WORDS = aI2C[2].rdLen;
      VOR_I2C2->ADDRESS |= 0x1;  // OR in read bit
      VOR_I2C2->CMD = 0x3;       // (repeated) start with stop
      VOR_I2C2->IRQ_ENB &=
          ~(I2C_IRQ_ENB_TXREADY_Msk | I2C_IRQ_ENB_WAITING_Msk);  // turn off TXREADY interrupt
      VOR_I2C2->IRQ_ENB |= I2C_IRQ_ENB_IDLE_Msk;
    } else {
      VOR_I2C2->CMD = 0x2;  // stop
    }
  }
  if (VOR_I2C2->IRQ_END & I2C_IRQ_END_CLKLOTO_Msk) {
    // handle SCL low timeout
    VOR_I2C2->CMD = 0x4;  // cancel
    VOR_I2C2->IRQ_ENB = 0;
    HAL_I2C2_XFER_TIMEOUT_CALLBACK();
  }
  if (VOR_I2C2->IRQ_END & I2C_IRQ_END_TXREADY_Msk) {
    // TX ready (below trigger level) - fill TX fifo if more to fill
    if ((aI2C[2].wrBuf != NULL) && aI2C[2].wrLen) {
      // there are things to write to the fifo
      while (aI2C[2].wrLen && (VOR_I2C2->STATUS & I2C_STATUS_TXNFULL_Msk)) {
        VOR_I2C2->DATA = *aI2C[2].wrBuf++;
        aI2C[2].wrLen--;
      }
    } else {
      // no more to write - kill this interrupt source
      VOR_I2C2->IRQ_ENB &= ~I2C_IRQ_ENB_TXREADY_Msk;
    }
  }
  if (VOR_I2C2->IRQ_END & I2C_IRQ_END_RXREADY_Msk) {
    // RX ready (above trigger level) - empty RX fifo
    if (aI2C[2].rdBuf != NULL) {
      while (aI2C[2].rdLen && (VOR_I2C2->STATUS & I2C_STATUS_RXNEMPTY_Msk)) {
        *aI2C[2].rdBuf++ = VOR_I2C2->DATA;
        aI2C[2].rdLen--;
      }
    }
  }
}
void I2C2_MS_RX_IRQHandler(void) { I2C2_MS_IRQHandler(); }
void I2C2_MS_TX_IRQHandler(void) { I2C2_MS_IRQHandler(); }

/*******************************************************************************
 **
 ** @brief  Begin listening to I2C slave. Callback when desired number of bytes received
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  dataPtr pointer to an rx buffer
 ** @param  maxLen length of rx buffer
 ** @param  threshLen initiate RX callback when bytes received == threshLen
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_I2CS_Listen(VOR_I2C_Type *const i2c, char *dataPtr, uint16_t maxLen,
                             uint16_t threshLen) {
  uint8_t bank = GetBankNum(i2c);

  // check for possible errors
  if (HAL_I2C_Enclock(i2c) != hal_status_ok) {
    return hal_status_badParam;
  }
  if (!maxLen) {
    return hal_status_badParam;
  }
  if (!threshLen) {
    return hal_status_badParam;
  }
  if (dataPtr == NULL) {
    return hal_status_badParam;
  }
  if ((i2c->S0_CTRL & I2C_S0_CTRL_ENABLED_Msk) == 0) {
    return hal_status_notInitialized;
  }
  if ((i2c->S0_STATUS & I2C_S0_STATUS_IDLE_Msk) == 0) {
    return hal_status_busy;
  }

  i2c->S0_IRQ_ENB = 0;
  i2c->S0_MAXWORDS = maxLen;

  aI2C_S[bank].rxBuf = dataPtr;
  aI2C_S[bank].rxLen = maxLen;
  aI2C_S[bank].rxThresh = threshLen;
  aI2C_S[bank].rxCount = 0;
  aI2C_S[bank].txBuf = NULL;
  aI2C_S[bank].txLen = 0;

  // clear fifos
  i2c->S0_FIFO_CLR = I2C_S0_FIFO_CLR_RXFIFO_Msk | I2C_S0_FIFO_CLR_TXFIFO_Msk;

  // clear IRQ flags
  i2c->S0_IRQ_CLR = I2C_S0_IRQ_CLR_COMPLETED_Msk;

  // enable interrupts
  i2c->S0_IRQ_ENB = I2C_S0_IRQ_ENB_COMPLETED_Msk | I2C_S0_IRQ_ENB_RXREADY_Msk;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Load a response into the I2C slave TX FIFO for the master to read
 **
 ** @param  i2c pointer to I2C peripheral
 ** @param  dataPtr pointer to a tx buffer
 ** @param  len length of buffer (num bytes to send)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
extern hal_status_t HAL_I2CS_Respond(VOR_I2C_Type *const i2c, char *dataPtr, uint16_t len) {
  uint8_t bank = GetBankNum(i2c);

  // check for possible errors
  if (HAL_I2C_Enclock(i2c) != hal_status_ok) {
    return hal_status_badParam;
  }
  if (!len) {
    return hal_status_badParam;
  }
  if (dataPtr == NULL) {
    return hal_status_badParam;
  }

  i2c->S0_IRQ_ENB = 0;

  aI2C_S[bank].rxBuf = NULL;
  aI2C_S[bank].rxLen = 0;
  aI2C_S[bank].rxThresh = 0;
  aI2C_S[bank].rxCount = 0;
  aI2C_S[bank].txBuf = dataPtr;
  aI2C_S[bank].txLen = len;

  // clear fifos
  i2c->S0_FIFO_CLR = I2C_S0_FIFO_CLR_RXFIFO_Msk | I2C_S0_FIFO_CLR_TXFIFO_Msk;

  // clear IRQ flags
  i2c->S0_IRQ_CLR = I2C_S0_IRQ_CLR_COMPLETED_Msk;

  // load TX fifo
  while (aI2C_S[bank].txLen && (i2c->S0_STATUS & I2C_S0_STATUS_TXNFULL_Msk)) {
    i2c->S0_DATA = *aI2C_S[bank].txBuf++;
    aI2C_S[bank].txLen--;
  }

  // enable interrupts
  i2c->S0_IRQ_ENB = I2C_S0_IRQ_ENB_COMPLETED_Msk | I2C_S0_IRQ_ENB_TXREADY_Msk;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  I2C0 slave mode IRQ handler
 **
 ******************************************************************************/
void I2C0_SL_IRQHandler(void) __attribute__((weak));
void I2C0_SL_IRQHandler(void) {
  volatile uint8_t junk = 0;
  (void)junk;

  if (VOR_I2C0->S0_IRQ_END & I2C_S0_IRQ_END_TXREADY_Msk) {
    // TX ready (below trigger level) - load TX fifo
    if (aI2C_S[0].txBuf != NULL) {
      while (aI2C_S[0].txLen && (VOR_I2C0->S0_STATUS & I2C_S0_STATUS_TXNFULL_Msk)) {
        VOR_I2C0->S0_DATA = *aI2C_S[0].txBuf++;
        aI2C_S[0].txLen--;
      }
    }
  }

  if (VOR_I2C0->S0_IRQ_END & I2C_S0_IRQ_END_RXREADY_Msk) {
    // RX ready (above trigger level) - empty RX fifo
    if (aI2C_S[0].rxBuf != NULL) {
      while (aI2C_S[0].rxLen && (VOR_I2C0->S0_STATUS & I2C_S0_STATUS_RXNEMPTY_Msk)) {
        *aI2C_S[0].rxBuf++ = VOR_I2C0->S0_DATA;
        aI2C_S[0].rxLen--;
        aI2C_S[0].rxCount++;
        if (aI2C_S[0].rxCount == aI2C_S[0].rxThresh) {
          HAL_I2C0_S0_RX_THRESH_CALLBACK();
        }
      }
    }
    while (VOR_I2C0->S0_STATUS & I2C_STATUS_RXNEMPTY_Msk) {
      // if no RX buffer, or RX buf is full, empty remaining rx bytes
      junk = VOR_I2C0->S0_DATA;
    }
  }

  if (VOR_I2C0->S0_IRQ_END & I2C_S0_IRQ_END_COMPLETED_Msk) {
    // completed - empty rx fifo
    VOR_I2C0->S0_IRQ_ENB = 0;
    if ((aI2C_S[0].txBuf != NULL) && (aI2C_S[0].txLen == 0) &&
        (VOR_I2C0->S0_STATUS & I2C_S0_STATUS_TXEMPTY_Msk)) {
      // TX completed
      HAL_I2C0_S0_TXCOMPLETE_CALLBACK();
    }
    if (aI2C_S[0].rxBuf != NULL) {
      while (aI2C_S[0].rxLen && (VOR_I2C0->S0_STATUS & I2C_S0_STATUS_RXNEMPTY_Msk)) {
        *aI2C_S[0].rxBuf++ = VOR_I2C0->S0_DATA;
        aI2C_S[0].rxLen--;
        aI2C_S[0].rxCount++;
        if (aI2C_S[0].rxCount == aI2C_S[0].rxThresh) {
          HAL_I2C0_S0_RX_THRESH_CALLBACK();
        }
      }
    }
    while (VOR_I2C0->S0_STATUS & I2C_STATUS_RXNEMPTY_Msk) {
      // if no RX buffer, or RX buf is full, empty remaining rx bytes
      junk = VOR_I2C0->S0_DATA;
    }
  }
}
void I2C0_SL_RX_IRQHandler(void) { I2C0_SL_IRQHandler(); }
void I2C0_SL_TX_IRQHandler(void) { I2C0_SL_IRQHandler(); }

/*******************************************************************************
 **
 ** @brief  I2C1 slave mode IRQ handler
 **
 ******************************************************************************/
void I2C1_SL_IRQHandler(void) __attribute__((weak));
void I2C1_SL_IRQHandler(void) {
  volatile uint8_t junk = 0;
  (void)junk;

  if (VOR_I2C1->S0_IRQ_END & I2C_S0_IRQ_END_TXREADY_Msk) {
    // TX ready (below trigger level) - load TX fifo
    if (aI2C_S[1].txBuf != NULL) {
      while (aI2C_S[1].txLen && (VOR_I2C1->S0_STATUS & I2C_S0_STATUS_TXNFULL_Msk)) {
        VOR_I2C1->S0_DATA = *aI2C_S[1].txBuf++;
        aI2C_S[1].txLen--;
      }
    }
  }

  if (VOR_I2C1->S0_IRQ_END & I2C_S0_IRQ_END_RXREADY_Msk) {
    // RX ready (above trigger level) - empty RX fifo
    if (aI2C_S[1].rxBuf != NULL) {
      while (aI2C_S[1].rxLen && (VOR_I2C1->S0_STATUS & I2C_S0_STATUS_RXNEMPTY_Msk)) {
        *aI2C_S[1].rxBuf++ = VOR_I2C1->S0_DATA;
        aI2C_S[1].rxLen--;
        aI2C_S[1].rxCount++;
        if (aI2C_S[1].rxCount == aI2C_S[1].rxThresh) {
          HAL_I2C1_S0_RX_THRESH_CALLBACK();
        }
      }
    }
    while (VOR_I2C1->S0_STATUS & I2C_STATUS_RXNEMPTY_Msk) {
      // if no RX buffer, or RX buf is full, empty remaining rx bytes
      junk = VOR_I2C1->S0_DATA;
    }
  }

  if (VOR_I2C1->S0_IRQ_END & I2C_S0_IRQ_END_COMPLETED_Msk) {
    // completed - empty rx fifo
    VOR_I2C1->S0_IRQ_ENB = 0;
    if ((aI2C_S[1].txBuf != NULL) && (aI2C_S[1].txLen == 0) &&
        (VOR_I2C1->S0_STATUS & I2C_S0_STATUS_TXEMPTY_Msk)) {
      // TX completed
      HAL_I2C1_S0_TXCOMPLETE_CALLBACK();
    }
    if (aI2C_S[1].rxBuf != NULL) {
      while (aI2C_S[1].rxLen && (VOR_I2C1->S0_STATUS & I2C_S0_STATUS_RXNEMPTY_Msk)) {
        *aI2C_S[1].rxBuf++ = VOR_I2C1->S0_DATA;
        aI2C_S[1].rxLen--;
        aI2C_S[1].rxCount++;
        if (aI2C_S[1].rxCount == aI2C_S[1].rxThresh) {
          HAL_I2C1_S0_RX_THRESH_CALLBACK();
        }
      }
    }
    while (VOR_I2C1->S0_STATUS & I2C_STATUS_RXNEMPTY_Msk) {
      // if no RX buffer, or RX buf is full, empty remaining rx bytes
      junk = VOR_I2C1->S0_DATA;
    }
  }
}

void I2C1_SL_RX_IRQHandler(void) __attribute__((weak));
void I2C1_SL_RX_IRQHandler(void) { I2C1_SL_IRQHandler(); }

void I2C1_SL_TX_IRQHandler(void) __attribute__((weak));
void I2C1_SL_TX_IRQHandler(void) { I2C1_SL_IRQHandler(); }

/*******************************************************************************
 **
 ** @brief  I2C2 slave mode IRQ handler
 **
 ******************************************************************************/
void I2C2_SL_IRQHandler(void) __attribute__((weak));
void I2C2_SL_IRQHandler(void) {
  volatile uint8_t junk = 0;
  (void)junk;

  if (VOR_I2C2->S0_IRQ_END & I2C_S0_IRQ_END_TXREADY_Msk) {
    // TX ready (below trigger level) - load TX fifo
    if (aI2C_S[2].txBuf != NULL) {
      while (aI2C_S[2].txLen && (VOR_I2C2->S0_STATUS & I2C_S0_STATUS_TXNFULL_Msk)) {
        VOR_I2C2->S0_DATA = *aI2C_S[2].txBuf++;
        aI2C_S[2].txLen--;
      }
    }
  }

  if (VOR_I2C2->S0_IRQ_END & I2C_S0_IRQ_END_RXREADY_Msk) {
    // RX ready (above trigger level) - empty RX fifo
    if (aI2C_S[2].rxBuf != NULL) {
      while (aI2C_S[2].rxLen && (VOR_I2C2->S0_STATUS & I2C_S0_STATUS_RXNEMPTY_Msk)) {
        *aI2C_S[2].rxBuf++ = VOR_I2C2->S0_DATA;
        aI2C_S[2].rxLen--;
        aI2C_S[2].rxCount++;
        if (aI2C_S[2].rxCount == aI2C_S[2].rxThresh) {
          HAL_I2C2_S0_RX_THRESH_CALLBACK();
        }
      }
    }
    while (VOR_I2C2->S0_STATUS & I2C_STATUS_RXNEMPTY_Msk) {
      // if no RX buffer, or RX buf is full, empty remaining rx bytes
      junk = VOR_I2C2->S0_DATA;
    }
  }

  if (VOR_I2C2->S0_IRQ_END & I2C_S0_IRQ_END_COMPLETED_Msk) {
    // completed - empty rx fifo
    VOR_I2C2->S0_IRQ_ENB = 0;
    if ((aI2C_S[2].txBuf != NULL) && (aI2C_S[2].txLen == 0) &&
        (VOR_I2C2->S0_STATUS & I2C_S0_STATUS_TXEMPTY_Msk)) {
      // TX completed
      HAL_I2C2_S0_TXCOMPLETE_CALLBACK();
    }
    if (aI2C_S[2].rxBuf != NULL) {
      while (aI2C_S[2].rxLen && (VOR_I2C2->S0_STATUS & I2C_S0_STATUS_RXNEMPTY_Msk)) {
        *aI2C_S[2].rxBuf++ = VOR_I2C2->S0_DATA;
        aI2C_S[2].rxLen--;
        aI2C_S[2].rxCount++;
        if (aI2C_S[2].rxCount == aI2C_S[2].rxThresh) {
          HAL_I2C2_S0_RX_THRESH_CALLBACK();
        }
      }
    }
    while (VOR_I2C2->S0_STATUS & I2C_STATUS_RXNEMPTY_Msk) {
      // if no RX buffer, or RX buf is full, empty remaining rx bytes
      junk = VOR_I2C2->S0_DATA;
    }
  }
}

void I2C2_SL_RX_IRQHandler(void) __attribute__((weak));
void I2C2_SL_RX_IRQHandler(void) { I2C2_SL_IRQHandler(); }

void I2C2_SL_TX_IRQHandler(void) __attribute__((weak));
void I2C2_SL_TX_IRQHandler(void) { I2C2_SL_IRQHandler(); }

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
