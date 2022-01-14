/***************************************************************************************
 * @file     va416xx_bitband.h
 * @version  V0.1
 * @date     18 March 2019
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

#ifndef __VA416XX_BITBAND_H
#define __VA416XX_BITBAND_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

// Convert SRAM address and bit pos to bitband alias address
#define BITBAND_SRAM_REF (0x20000000)
#define BITBAND_SRAM_BASE (0x22000000)
#define BITBAND_SRAM(addr, b) \
  ((BITBAND_SRAM_BASE + ((uint32_t)addr - BITBAND_SRAM_REF) * 32 + (b * 4)))

// Convert PERIPHERAL address and bit pos to bitband alias address
#define BITBAND_PERI_REF (0x40000000)
#define BITBAND_PERI_BASE (0x42000000)
#define BITBAND_PERI(addr, b) \
  ((BITBAND_PERI_BASE + ((uint32_t)addr - BITBAND_PERI_REF) * 32 + (b * 4)))

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

/*****************************************************************************
 * @brief
 *   Perform bit-band write operation on peripheral memory location.
 *
 * @details
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the Cortex-M4 reference manual for further
 *   details about bit-banding.
 *
 * @param[in] addr Peripheral address location to modify bit in.
 *
 * @param[in] bit Bit position to modify, 0-31.
 *
 * @param[in] val Value to set bit to, 0 or 1.
 ******************************************************************************/
__STATIC_INLINE void Bitband_Peripheral(volatile uint32_t *addr, uint32_t bit, uint32_t val) {
  uint32_t tmp = BITBAND_PERI(addr, bit);

  *((volatile uint32_t *)tmp) = (uint32_t)val;
}

/*****************************************************************************
 * @brief
 *   Perform a read operation on the peripheral bit-band memory location.
 *
 * @details
 *   This function reads a single bit from the peripheral bit-band alias region.
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the Cortex-M4 reference manual for further
 *   details about bit-banding.
 *
 * @param[in] addr   Peripheral address location to read.
 *
 * @param[in] bit    Bit position to read, 0-31.
 *
 * @return           Value of the requested bit.
 ******************************************************************************/
__STATIC_INLINE uint32_t Bitband_PeripheralRead(volatile uint32_t *addr, uint32_t bit) {
  uint32_t tmp = BITBAND_PERI(addr, bit);

  return *((volatile uint32_t *)tmp);
}

/*****************************************************************************
 * @brief
 *   Perform bit-band write operation on SRAM memory location.
 *
 * @details
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the Cortex-M4 reference manual for further
 *   details about bit-banding.
 *
 * @param[in] addr SRAM address location to modify bit in.
 *
 * @param[in] bit Bit position to modify, 0-31.
 *
 * @param[in] val Value to set bit to, 0 or 1.
 ******************************************************************************/
__STATIC_INLINE void Bitband_SRAM(uint32_t *addr, uint32_t bit, uint32_t val) {
  uint32_t tmp = BITBAND_SRAM(addr, bit);

  *((volatile uint32_t *)tmp) = (uint32_t)val;
}

/*****************************************************************************
 * @brief
 *   Read a single bit from the SRAM bit-band alias region.
 *
 * @details
 *   This function reads a single bit from the SRAM bit-band alias region.
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the Cortex-M4 reference manual for further
 *   details about bit-banding.
 *
 * @param[in] addr    SRAM address location to modify bit in.
 *
 * @param[in] bit     Bit position to modify, 0-31.
 *
 * @return            Value of the requested bit.
 ******************************************************************************/
__STATIC_INLINE uint32_t Bitband_SRAMRead(uint32_t *addr, uint32_t bit) {
  uint32_t tmp = BITBAND_SRAM(addr, bit);

  return *((volatile uint32_t *)tmp);
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __VA416XX_BITBAND_H */
