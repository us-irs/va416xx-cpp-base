/***************************************************************************************
 * @file     va416xx_hal_i2c.h
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

#ifndef __HAL_I2C_H
#define __HAL_I2C_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "VORConfig.h"
#include "va416xx_debug.h"
#include "va416xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/** Version */
#define HAL_I2C_VERSION (0x00000501)  // 0.5.1 (19 September 2019)

/** I2C Peripheral clock source */
#define I2C_CLK (APB1_CLK)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

// "shifted" means bit[0] is already the placeholder for the Read/Write_N bit
// do not put a '1' in Read/Write_N bit
// Write does not strip it, but Read ORs it in
typedef uint16_t i2caddr_shifted_t;

/** I2C speed enum */
typedef enum { en_i2c_100k = 0, en_i2c_400k = 1, en_i2c_max = 2 } en_i2c_spd_t;

/** I2C master init structure */
typedef struct stc_i2c_masterInit {
  uint8_t txfemd : 1;    // 0: stall on tx empty, 1: end txn
  uint8_t rxffmd : 1;    // 0: stall on rx full, 1: nack
  uint8_t loopback : 1;  // 0: normal operation, 1: conn ms to sl
  en_i2c_spd_t speed;    // 100khz, 400khz, or max speed
} stc_i2c_masterInit_t;

/** I2C slave init structure */
typedef struct stc_i2c_slaveInit {
  i2caddr_shifted_t slaveAddr;  // The slave address (7 or 10 bit + rw bit (0))
  i2caddr_shifted_t addrMask;   // Slave address mask
  uint8_t addrMaskEnable : 1;   // 1: use addrMask 0: mask = 0x7fe (exact match)
  uint8_t tenBitMode : 1;       // 0 = 7 bit addr mode, 1 = 10bit mode
  uint8_t txfemd : 1;           // 0: stall on tx empty, 1: nack address
  uint8_t rxffmd : 1;           // 0: stall on rx full, 1: nack
} stc_i2c_slaveInit_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

/** Init / de-init */
extern hal_status_t HAL_I2C_Reset(VOR_I2C_Type *const i2c);
extern hal_status_t HAL_I2CM_Init(VOR_I2C_Type *const i2c, stc_i2c_masterInit_t const init);
extern hal_status_t HAL_I2CS_Init(VOR_I2C_Type *const i2c, stc_i2c_slaveInit_t const init);
extern hal_status_t HAL_I2C_Declock(VOR_I2C_Type *const i2c);
extern hal_status_t HAL_I2C_Enclock(VOR_I2C_Type *const i2c);

/** Blocking master transfers (returns when xfer completed or timeout) */
extern hal_status_t HAL_I2CM_WriteB(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr,
                                    char *dataPtr, uint32_t len);
extern hal_status_t HAL_I2CM_ReadB(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr,
                                   char *dataPtr, uint32_t len);
extern hal_status_t HAL_I2CM_WrReadB(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr,
                                     char *wrDataPtr, uint32_t wrLen, char *rdDataPtr,
                                     uint32_t rdLen);

/** Non blocking master transfers (returns immediately, have to poll/check for
    completion or wait for callback) */
extern hal_status_t HAL_I2CM_Write(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr,
                                   char *dataPtr, uint32_t len);
extern hal_status_t HAL_I2CM_Read(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr, char *dataPtr,
                                  uint32_t len);
extern hal_status_t HAL_I2CM_WrRead(VOR_I2C_Type *const i2c, i2caddr_shifted_t i2cAddr,
                                    char *wrDataPtr, uint32_t wrLen, char *rdDataPtr,
                                    uint32_t rdLen);
extern hal_status_t HAL_I2CM_GetXferStatus(VOR_I2C_Type *const i2c);

/** Slave mode calls. Non blocking, return immediately */
extern hal_status_t HAL_I2CS_Listen(VOR_I2C_Type *const i2c, char *dataPtr, uint16_t maxLen,
                                    uint16_t threshLen);
extern hal_status_t HAL_I2CS_Respond(VOR_I2C_Type *const i2c, char *dataPtr, uint16_t len);

// Default IRQ handlers. Still need to be called by user IRQ definition
extern void VORIRQ_I2C0_SL_IRQHandler(void);
extern void VORIRQ_I2C0_SL_RX_IRQHandler(void);
extern void VORIRQ_I2C0_SL_TX_IRQHandler(void);
extern void VORIRQ_I2C1_SL_IRQHandler(void);
extern void VORIRQ_I2C1_SL_RX_IRQHandler(void);
extern void VORIRQ_I2C1_SL_TX_IRQHandler(void);

#ifdef __cplusplus
}
#endif

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_I2C_H */
