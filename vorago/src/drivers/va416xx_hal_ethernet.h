/***************************************************************************************
 * @file     va416xx_hal_ethernet.h
 * @version  V1.0
 * @date      April 2019
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

#ifndef __VA416XX_HAL_ETHERNET_H
#define __VA416XX_HAL_ETHERNET_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include <stdlib.h>
#include <string.h>

#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

// PHY definitions
#define GMII_WAIT 10000  // used for timeout MDIO
#define PHYREGSUSED 16   // number of PHY registers used

#define PHY100MBPS 1
#define PHY10MBPS 0

#define PHYAUTONEGEN 1
#define PHYAUTONEGDIS 0

#define PHYLOOPBACK 1
#define PHYDISLOOPBAK 0

#define PHYFULLDUPLEX 1
#define PHYHALFDUPLEX 0

// ****************************************************************************
// MII Registers OFFSETS and their layouts. PHY register map is standard
// Addresses shifted for position into ( GMII Register Address [10:6] )
//
#define PHY_CONTROL_REG 0x0000            // Control Register
#define PHY_STATUS_REG 0x0040             // Status Register
#define PHY_ID_HI_REG 0x0080              // PHY Identifier High Register
#define PHY_ID_LOW_REG 0x00c0             // PHY Identifier High Register
#define PHY_AN_ADV_REG 0x0100             // Auto-Negotiation Advertisement Register
#define PHY_LNK_PART_ABl_REG 0x0140       // Link Partner Ability Register (Base Page)
#define PHY_AN_EXP_REG 0x0180             // Auto-Negotiation Expansion Register
#define PHY_AN_NXT_PAGE_TX_REG 0x01c0     // Next Page Transmit Register
#define PHY_LNK_PART_NXT_PAGE_REG 0x0200  // Link Partner Next Page Register

#define PHY_MII_CTRL 0x0500        // MII (Phy specific control)
#define PHY_RX_ERR_COUNTER 0x0540  // Receive Error Counter

#define PHY_INTR_CTRL_STATUS 0x06c0    // Interrupt Control/Status
#define PHY_LINKMD_CTRL_STATUS 0x0740  // LinkMD Control/Status
#define PHY_CONTROL_ONE 0x0780         // PHY Control 1
#define PHY_CONTROL_TWO 0x07c0         // PHY Control 2

// *********************************************************************************
// PHY Control register  (KSZ8041TL)  Physical is 16bit
#define MIIADDR_PHY_LAYR_ADDR 0x0800    // Physical Layer Address (should be strapped to 1)
#define MIIADDR_PHY_CLR_ADDR_RW 0xf83d  // clear PHY register address and R/W
#define MIIADDR_PHY_WRITE 0x0002        // or for a write
#define MIIADDR_PHY_BUSY 0x0001         // GMII busy
#define MIIADDR_PHY_LOOPBACK 0x4000     // PHY Loopback
#define MIIADDR_PHY_SPEED \
  0x2000  // set phy speed bit				* Auto-negotiation must be run -
#define MIIADDR_PHY_AN \
  0x1000  // set auto-negotiation bit	*		inorder to make these bit -
#define MIIADDR_PHY_DUPLEX \
  0x0100                          // set Duplex mode bit			*		take effect on wire
#define MIIADDR_PHY_RESET 0x8000  // reset the PHY

// PHY Control register one (KSZ8041TL)  Physical is 16bit
#define PHY_LED_MODE 0x4000  // LED control = LED1: Speed; LED0: Link/Activity

// GMII (PHY) status register (KSZ8041TL)  Physical is 16bit
#define MIISTATUS_PHY_LINK 0x0004  // Link status 1=up

// GMII CSR clock range - VA416xx will only operate between 20MHZ and 100MHZ
#define PHY_MACMII_CR_DIV42 0x0  // clock is divide by 60 - 100 MHZ
#define PHY_MACMII_CR_DIV26 0x3  // clock is divide by 35 - 60 MHZ
#define PHY_MACMII_CR_DIV16 0x2  // clock is divide by 20 - 35 MHZ

#define PHYSICAL_DATA_Msk 0xffff

#define PHY_INTER_ENABLE_ALL 0xff00

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

hal_status_t HAL_ReadPhyReg(uint16_t, uint16_t *);
hal_status_t HAL_WritePhyReg(uint16_t, uint16_t);

hal_status_t HAL_SetPhyDuplex(uint32_t);
hal_status_t HAL_SetPhyLoopBack(uint32_t);
hal_status_t HAL_SetPhyAutoNegotiate(uint32_t);
hal_status_t HAL_SetPhySpeed(uint32_t);
hal_status_t HAL_ResetPHY(void);
hal_status_t HAL_CoreReset(void);

hal_status_t HAL_SetPhyLeds(uint32_t);

void HAL_GetMacAddr(uint8_t *);
void HAL_SetMacAddr(uint8_t *);
void HAL_setBuffer(uint32_t *, uint32_t, uint32_t);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __VA416XX_HAL_ETHERNET_H */
