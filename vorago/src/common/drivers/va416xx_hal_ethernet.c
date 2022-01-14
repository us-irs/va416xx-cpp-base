/***************************************************************************************
 * @file     va416xx_hal_ethernet.c
 * @version  V1.0
 * @date     26 February 2020
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

#include "va416xx_hal_ethernet.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/******************************************************************************
 * @brief  Function to read the Phy register. The access to phy register
 *         is a slow process as the data is moved accross MDI/MDO interface
 * @param[in] Register offset is the index of one of the 15 phy register.
 * @param[out] uint16_t data read from the respective phy register (only valid if return value is
 * 0). \return Returns 0 on success else return the error status.
 */
hal_status_t HAL_ReadPhyReg(uint16_t phyRegAddr, uint16_t *data) {
  uint32_t count;

  // check if busy is set
  if (VOR_ETH->MAC_GMII_ADDR & ETH_MAC_GMII_ADDR_GB_Msk) {
    // busy is set , wait for it to clear
    count = GMII_WAIT;
    while ((VOR_ETH->MAC_GMII_ADDR & ETH_MAC_GMII_ADDR_GB_Msk) && (count)) {
      count--;  // timout if busy not cleared
    }

    if (!count)  // Error.. timed out waiting for busy to clear
    {
      return hal_status_timeout;  // REFACTOR MULTIPLY TRIES BEFORE CALLING QUIT'S
    }
  }

  // clear PHY register address and R/W bit
  VOR_ETH->MAC_GMII_ADDR &= MIIADDR_PHY_CLR_ADDR_RW;
  // PHY register address in GmiiGmiiAddr register of synopGMAC ip
  VOR_ETH->MAC_GMII_ADDR |= phyRegAddr;
  // start the read cycle
  VOR_ETH->MAC_GMII_ADDR |= ETH_MAC_GMII_ADDR_GB_Msk;
  // Wait until the busy bit gets cleared with in a certain amount of time
  count = GMII_WAIT;
  while (!(VOR_ETH->MAC_GMII_ADDR & ETH_MAC_GMII_ADDR_GB_Msk) && (count)) {
    count--;
  }

  if (count)  // did not time out
  {
    *data = (VOR_ETH->MAC_GMII_DATA & PHYSICAL_DATA_Msk);  // write out the Register data
  } else  // REFACTOR MULTIPLY TRIES BEFORE CALLING QUIT'S
  {
    return hal_status_timeout;  // error, timed out waiting
  }
  return hal_status_ok;
}

/******************************************************************************
 * Function to write to the Phy register. The access to phy register
 * is a slow process as the data is moved accross MDI/MDO interface
 * @param[in] Register offset is the index of one of the 32 phy register.
 * @param[in] data to be written to the respective phy register.
 * \return Returns hal_status_ok on success, else the error status.
 */
hal_status_t HAL_WritePhyReg(uint16_t phyRegAddr, uint16_t data) {
  uint32_t count;

  // check if busy is set, can't write if interface is busy
  if (VOR_ETH->MAC_GMII_ADDR & ETH_MAC_GMII_ADDR_GB_Msk) {
    // busy is set , wait for it to clear
    count = GMII_WAIT;
    while ((VOR_ETH->MAC_GMII_ADDR & ETH_MAC_GMII_ADDR_GB_Msk) && (count)) {
      count--;  // timout if not cleared
    }

    if (count == 0)  // Error.. timed out waiting for busy to clear
    {
      return hal_status_timeout;  // REFACTOR MULTIPLY TRIES BEFORE CALLING QUIT'S
    }
  }

  // write the data to PHY
  VOR_ETH->MAC_GMII_DATA = (uint32_t)data;

  // clear PHY register address and R/W bit
  VOR_ETH->MAC_GMII_ADDR &= MIIADDR_PHY_CLR_ADDR_RW;

  // write the address from where the data to be read
  // in GmiiAddr register
  VOR_ETH->MAC_GMII_ADDR |= (phyRegAddr | ETH_MAC_GMII_ADDR_GW_Msk);
  VOR_ETH->MAC_GMII_ADDR |= ETH_MAC_GMII_ADDR_GB_Msk;

  // Wait until the busy bit gets cleared with in a certain amount of time
  count = GMII_WAIT;
  while (!(VOR_ETH->MAC_GMII_ADDR & ETH_MAC_GMII_ADDR_GB_Msk) && (count)) {
    count--;
  }

  if (count == 0) {
    return hal_status_timeout;  // REFACTOR MULTIPLY TRIES BEFORE CALLING QUIT'S
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  sets PHY led config
 ** @param  entry
 ** @return result of PHY register read/write
 **
 ******************************************************************************/
hal_status_t HAL_SetPhyLeds(uint32_t speed) {
  hal_status_t status;
  uint16_t temp;

  status = HAL_ReadPhyReg(PHY_CONTROL_ONE, &temp);  // read phy control resister
  if (status != hal_status_ok) {
    return status;
  }

  status = HAL_ReadPhyReg(PHY_CONTROL_ONE, &temp);  // read again, fixes buffer issue
  if (status != hal_status_ok) {
    return status;
  }

  if (speed) {
    temp |= PHY_LED_MODE;  // LED1: Activity; LED0: Link
  } else {
    temp &= ~(PHY_LED_MODE);  // LED1: Speed; LED0: Link/Activity
  }

  return (HAL_WritePhyReg(PHY_CONTROL_REG, temp));
}
/*******************************************************************************
 ** @brief  sets Physical Duplex state
 ** @param  entry
 ** @return result
 **
 ** //jpwi this is the control bit, read connection status in PHY Control 2
 ** //jpwi must set here and trigger new AN cycle
 ******************************************************************************/
hal_status_t HAL_SetPhyDuplex(uint32_t dupx) {
  hal_status_t status;
  uint16_t temp;

  status = HAL_ReadPhyReg(PHY_CONTROL_REG, &temp);  // read phy control resister
  if (status != hal_status_ok) {
    return status;  // REFACTOR MULTIPLY TRIES BEFORE CALLING QUIT'S
  }

  status = HAL_ReadPhyReg(PHY_CONTROL_REG, &temp);  // fixes buffer issue in Synopsis IP
  if (status != hal_status_ok) {
    return status;  // REFACTOR MULTIPLY TRIES BEFORE CALLING QUIT'S
  }

  if (dupx) {
    temp |= MIIADDR_PHY_DUPLEX;  // set PHY into FULL DUPLEX mode
  } else {
    temp &= ~(MIIADDR_PHY_DUPLEX);  // set PHY into Half Duplex mode
  }

  return (HAL_WritePhyReg(PHY_CONTROL_REG, temp));  // REFACTOR MULTIPLY TRIES BEFORE CALLING QUIT'S
}

/*******************************************************************************
 **
 ** @brief  sets PHY loopback state
 ** @param  entry
 ** @return result of PHY register read/write
 **
 ******************************************************************************/
hal_status_t HAL_SetPhyLoopBack(uint32_t loopback) {
  hal_status_t status;
  uint16_t temp;

  status = HAL_ReadPhyReg(PHY_CONTROL_REG, &temp);  // read phy control resister
  if (status != hal_status_ok) {
    return status;
  }

  status = HAL_ReadPhyReg(PHY_CONTROL_REG, &temp);  // read again, fixes buffer issue
  if (status != hal_status_ok) {
    return status;
  }

  if (loopback) {
    temp |= MIIADDR_PHY_LOOPBACK;  // PHY Loopback mode  ON
  } else {
    temp &= ~(MIIADDR_PHY_LOOPBACK);  // PHY Loopback mode  OFF
  }

  return (HAL_WritePhyReg(PHY_CONTROL_REG, temp));
}

/*******************************************************************************
 **
 ** @brief  PHY Auto-negotiation mode
 ** @param  entry
 ** @return result
 **
 ******************************************************************************/
hal_status_t HAL_SetPhyAutoNegotiate(uint32_t negotiate) {
  hal_status_t status;
  uint16_t temp;

  status = HAL_ReadPhyReg(PHY_CONTROL_REG, &temp);  // read phy control resister
  if (status != hal_status_ok) {
    return status;
  }

  status = HAL_ReadPhyReg(PHY_CONTROL_REG, &temp);  // fixes buffer issue
  if (status != hal_status_ok) {
    return status;
  }

  if (negotiate) {
    temp |= MIIADDR_PHY_AN;  // enable PHY Auto-Negotiation mode
  } else {
    temp &= ~(MIIADDR_PHY_AN);  // disable PHY Loopback mode
  }

  return (HAL_WritePhyReg(PHY_CONTROL_REG, temp));
}

/*******************************************************************************
 **
 ** @brief  Sets Speed at 100Mbps
 **
 ** @return result
 **
 ** @note This bit is ignored if Auto-negotiation bit is set
 ** //jpwi this is the control bit read connection status in PHY Control 2
 ** //jpwi must set here and trigger new AN cycle
 ******************************************************************************/
hal_status_t HAL_SetPhySpeed(uint32_t speed) {
  hal_status_t status;
  uint16_t temp;

  status = HAL_ReadPhyReg(PHY_CONTROL_REG, &temp);  // read phy control resister
  if (status != hal_status_ok) {
    return status;
  }

  status = HAL_ReadPhyReg(PHY_CONTROL_REG, &temp);  // 2 reads fixes buffer issue
  if (status != hal_status_ok) {
    return status;
  }

  // NOTE: this setting will be ignored if auto-negotiate is enabled
  // TODO: TEST FOR AN
  if (speed) {
    temp |= MIIADDR_PHY_SPEED;  // set 100Mbps
  } else {
    temp &= ~(MIIADDR_PHY_SPEED);  // set 10Mbps
  }

  return (HAL_WritePhyReg(PHY_CONTROL_REG, temp));
}

/*******************************************************************************
 **
 ** @brief
 **
 ** @param  entry: nothing
 **
 ** @return PHY is reset, this bit self clears
 **
 ******************************************************************************/
hal_status_t HAL_ResetPHY(void) {
  uint16_t temp;

  if ((HAL_ReadPhyReg(PHY_CONTROL_REG, &temp) == 0))  // read phy control resister
  {
    if ((HAL_ReadPhyReg(PHY_CONTROL_REG, &temp) == 0))  // read phy control resister
    {
      temp |= MIIADDR_PHY_RESET;  // reset PHY

      if ((HAL_WritePhyReg(PHY_CONTROL_REG, temp) != 0)) {
        return hal_status_timeout;
      }
    } else {
      return hal_status_timeout;
    }
  } else {
    return hal_status_timeout;
  }
  return hal_status_ok;
}

/*******************************************************************************
 ** @brief  Get PHY interrupt Control/Status
 ** @param  entry
 ** @return result
 **
 ******************************************************************************/
uint16_t HAL_GetPhyIntrStatusl(void) {
  uint16_t tmp;

  // read phy Interrupt Control/Status register
  if ((HAL_ReadPhyReg(PHY_INTR_CTRL_STATUS, &tmp) == 0)) {
    // read again, phy Interrupt Control/Status register
    if ((HAL_ReadPhyReg(PHY_INTR_CTRL_STATUS, &tmp) == 0)) {
      return tmp;
    }
  }
  return 0xffff;  // ERROR
}

// ****************************************************************************
// End of PHY functions
// ****************************************************************************

/*******************************************************************************
 ** @brief  This resets the DMA and MAC core.
 **         After reset all the registers holds their respective reset value
 ** @param  none
 ** @return hal_status_ok on success else return the error status.
 ******************************************************************************/
hal_status_t HAL_CoreReset(void) {
  // check AHB status (DMA), verify bus idle
  if (VOR_ETH->DMA_AHB_STATUS & ETH_DMA_AHB_STATUS_AHBMASTRSTS_Msk) {
    return hal_status_busy;  // AHB is not available
  }

  // reset Ethernet core
  if (VOR_ETH->DMA_BUS_MODE & ETH_DMA_BUS_MODE_SWR_Msk) {
    return hal_status_initError;  // error, already in reset
  } else {
    VOR_ETH->DMA_BUS_MODE |= ETH_DMA_BUS_MODE_SWR_Msk;
    while (VOR_ETH->DMA_BUS_MODE & ETH_DMA_BUS_MODE_SWR_Msk)
      ;
  }
  return hal_status_ok;
}

/*******************************************************************************
 ** @brief  Retreive the MAC address from MAC Address register's
 ** @entry Pointer to buffer to return MAC address
 ** @return none
 ******************************************************************************/
void HAL_GetMacAddr(uint8_t *macAddr) {
  macAddr[5] = (VOR_ETH->MAC_ADDR_L & 0xff);
  macAddr[4] = (VOR_ETH->MAC_ADDR_L >> 8) & 0xff;
  macAddr[3] = (VOR_ETH->MAC_ADDR_L >> 16) & 0xff;
  macAddr[2] = (VOR_ETH->MAC_ADDR_L >> 24) & 0xff;
  macAddr[1] = (VOR_ETH->MAC_ADDR_H & 0xff);
  macAddr[0] = (VOR_ETH->MAC_ADDR_H >> 8) & 0xff;
}

/*******************************************************************************
 ** @brief  Sets the MAC address into MAC Address registers
 ** @param  pointer buffer containing MAC address
 ** @return none
 ******************************************************************************/
void HAL_SetMacAddr(uint8_t *macaddr) {
  VOR_ETH->MAC_ADDR_H = (macaddr[5] << 8) | macaddr[4];
  VOR_ETH->MAC_ADDR_L = (macaddr[3] << 24) | (macaddr[2] << 16) | (macaddr[1] << 8) | macaddr[0];
}

/*******************************************************************************
 **
 ** @brief  set's the memory buffer of dword
 **
 ** @param  entry
 ** @param
 **
 ** @return result
 **
 ******************************************************************************/
void HAL_setBuffer(uint32_t *bufr, uint32_t val, uint32_t size) {
  for (uint32_t cnt = 0; cnt < size; cnt++) {
    *bufr = val;
  }
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
