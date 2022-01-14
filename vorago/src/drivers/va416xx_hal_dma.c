/***************************************************************************************
 * @file     va416xx_hal_dma.c
 * @version  V0.3
 * @date     13 July 2020
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

#include "va416xx_hal_dma.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define DMA_PERID (0x000BB230)

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

#if defined(__CC_ARM)

/** Default DMA control block, 0x20000000 - 0x2000007f (ARM compiler 5) */
static stc_dma_control_blk_t dma_blk __attribute__((section(".ARM.__at_0x20000000"), zero_init));

#elif defined(__ICCARM__)

/** Default DMA control block, 0x20000000 - 0x2000007f (IAR) */
static stc_dma_control_blk_t dma_blk __attribute__((section(".ARM.__at_0x20000000")));

#elif defined(__GNUC__) /* GNU Compiler */

// Explicit placement using linker script
static stc_dma_control_blk_t dma_blk __attribute__((section(".dma_blk")));

#endif

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief Initialize the DMA block
 **
 ** @param ctrlBasePtr - pointer to the DMA control block - if NULL, uses default
 ** @param cacheable - indicates cacheable access
 ** @param bufferable - indicates bufferable access
 ** @param privileged - indicates privileged access
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_Init(stc_dma_control_blk_t* ctrlBasePtr, bool cacheable, bool bufferable,
                          bool privileged) {
  hal_status_t status;

  dbgprintln("DMA Init");

  status = HAL_DMA_Reset();
  if (status != hal_status_ok) {
    return status;
  }

  if (ctrlBasePtr == NULL) {
    ctrlBasePtr = &dma_blk;
  }
  status = HAL_DMA_SetCtrlBasePtr(ctrlBasePtr);
  if (status != hal_status_ok) {
    return status;
  }

  VOR_DMA->CFG = 0;
  if (cacheable) {
    VOR_DMA->CFG |= 0x4 << DMA_CFG_CHNL_PROT_CTRL_Pos;
  }
  if (bufferable) {
    VOR_DMA->CFG |= 0x2 << DMA_CFG_CHNL_PROT_CTRL_Pos;
  }
  if (privileged) {
    VOR_DMA->CFG |= 0x1 << DMA_CFG_CHNL_PROT_CTRL_Pos;
  }
  VOR_DMA->CFG |= (0x1);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Clocks and resets the DMA block and checks peripheral ID registers
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_Reset(void) {
  uint32_t perid;

  NVIC_DisableIRQ(DMA_ERROR_IRQn);
  NVIC_DisableIRQ(DMA_ACTIVE0_IRQn);
  NVIC_DisableIRQ(DMA_ACTIVE1_IRQn);
  NVIC_DisableIRQ(DMA_ACTIVE2_IRQn);
  NVIC_DisableIRQ(DMA_ACTIVE3_IRQn);
  NVIC_DisableIRQ(DMA_DONE0_IRQn);
  NVIC_DisableIRQ(DMA_DONE1_IRQn);
  NVIC_DisableIRQ(DMA_DONE2_IRQn);
  NVIC_DisableIRQ(DMA_DONE3_IRQn);

  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_DMA;
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~CLK_ENABLE_DMA;
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= CLK_ENABLE_DMA;

  perid = VOR_DMA->PERIPH_ID_0;
  perid |= (VOR_DMA->PERIPH_ID_1 << 8);
  perid |= (VOR_DMA->PERIPH_ID_2 << 16);
  perid |= (VOR_DMA->PERIPH_ID_3 << 24);
  if (perid != DMA_PERID) return hal_status_badPeriphID;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Enable DMA clock
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_Enclock(void) {
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_DMA;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Disable DMA clock
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_Declock(void) {
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_DMA;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Clear DMA bus error
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_ClearBusError(void) {
  VOR_DMA->ERR_CLR |= 1;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set DMA control structure base pointer
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SetCtrlBasePtr(stc_dma_control_blk_t* ctrlBasePtr) {
  if (ctrlBasePtr == NULL) {
    return hal_status_badParam;
  }
  if (((uint32_t)ctrlBasePtr & (~DMA_CTRL_BASE_PTR_CTRL_BASE_PTR_Msk)) > 0) {
    return hal_status_alignmentErr;  // lower 7 bits of address should be zero
  }
  VOR_DMA->CTRL_BASE_PTR = (uint32_t)ctrlBasePtr;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Trigger a SW request to a specified DMA channel
 **
 ** @param  ch - DMA channel number to SW Request (0-3)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SwRequest(uint32_t ch) {
  if (ch >= DMA_NUM_CHNLS) {
    return hal_status_badParam;
  }
  VOR_DMA->CHNL_SW_REQUEST = 1UL << ch;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set/clr allowing DMA_SREQ[] to generate a request
 **
 ** @param  ch - DMA channel number to set useburst enable
 ** @param  useburst - true enable DMA_SREQ[], false disable DMA_SREQ[]
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SetUseburst(uint32_t ch, bool useburst) {
  if (ch >= DMA_NUM_CHNLS) {
    return hal_status_badParam;
  }
  if (useburst) {
    VOR_DMA->CHNL_USEBURST_SET = 1UL << ch;
  } else {
    VOR_DMA->CHNL_USEBURST_CLR = 1UL << ch;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set/clr enable of DMA_REQ[] or DMA_SREQ[] to generate a request
 **
 ** @param  ch - DMA channel number to control request enable
 ** @param  enable - true enable DMA requests, false disable DMA requests
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SetRequestEnable(uint32_t ch, bool enable) {
  if (ch >= DMA_NUM_CHNLS) {
    return hal_status_badParam;
  }
  if (enable) {
    VOR_DMA->CHNL_REQ_MASK_SET = 1UL << ch;
  } else {
    VOR_DMA->CHNL_REQ_MASK_CLR = 1UL << ch;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Enables/disables a DMA channel
 **
 ** @param  ch - DMA channel number to set enable status (0-3)
 ** @param  enable - true enable channel, false disable channel
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SetChannelEnable(uint32_t ch, bool enable) {
  if (ch >= DMA_NUM_CHNLS) {
    return hal_status_badParam;
  }
  if (enable) {
    VOR_DMA->CHNL_ENABLE_SET = 1UL << ch;
  } else {
    VOR_DMA->CHNL_ENABLE_CLR = 1UL << ch;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets a DMA channel to use primary or alternate data structure
 **
 ** @param  ch - DMA channel number to disable (0-3)
 ** @param  primary - true use primary structure, false use alternate
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SetPriAlt(uint32_t ch, bool primary) {
  if (ch >= DMA_NUM_CHNLS) {
    return hal_status_badParam;
  }
  if (primary) {
    VOR_DMA->CHNL_PRI_ALT_CLR = 1UL << ch;
  }  // select primary
  else {
    VOR_DMA->CHNL_PRI_ALT_SET = 1UL << ch;
  }  // select alternate
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets a DMA channel's priority, high or low
 **
 ** @param  ch - DMA channel number to set priority level
 ** @param  high - true high priority, false low priority
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SetChannelPriority(uint32_t ch, bool high) {
  if (ch >= DMA_NUM_CHNLS) {
    return hal_status_badParam;
  }
  if (high) {
    VOR_DMA->CHNL_PRIORITY_SET = 1UL << ch;
  }  // select high priority
  else {
    VOR_DMA->CHNL_PRIORITY_CLR = 1UL << ch;
  }  // select low priority
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Copy a channel control struct to the DMA control block
 **
 ** @param  ch - DMA channel number to set priority level
 ** @param  primary - true copy to primary struct, false copy to alternate
 ** @param  pCtrl - pointer to a DMA channel control struct (primary or alternate)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_CopyChannelCtrl(uint32_t ch, bool primary, stc_dma_chnl_t* pCtrl) {
  stc_dma_control_blk_t* pDmaCtrlBlk;

  dbgprintln("DMA CopyChannelCtrl(%d, %d, %x)", ch, (uint32_t)primary, (uint32_t)pCtrl);

  if (ch >= DMA_NUM_CHNLS) {
    return hal_status_badParam;
  }
  if (pCtrl == NULL) {
    return hal_status_badParam;
  }
  if (POINTS_TO_SRAM(VOR_DMA->CTRL_BASE_PTR) == false) {
    return hal_status_notInitialized;
  }

  pDmaCtrlBlk = (stc_dma_control_blk_t*)VOR_DMA->CTRL_BASE_PTR;

  if (primary) {
    pDmaCtrlBlk->pri[ch].ctrl_raw = pCtrl->ctrl_raw;
    pDmaCtrlBlk->pri[ch].dst = pCtrl->dst;
    pDmaCtrlBlk->pri[ch].src = pCtrl->src;
  } else {
    pDmaCtrlBlk->alt[ch].ctrl_raw = pCtrl->ctrl_raw;
    pDmaCtrlBlk->alt[ch].dst = pCtrl->dst;
    pDmaCtrlBlk->alt[ch].src = pCtrl->src;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Gets a DMA channel's waitonreq status (1 or 0)
 **
 ** @param  ch - DMA channel number to set priority level
 ** @param  status [out] - waitonreq status high (1) or low (0)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_GetWaitOnReqStatus(uint32_t ch, uint8_t* status) {
  if (ch >= DMA_NUM_CHNLS) {
    return hal_status_badParam;
  }
  if (VOR_DMA->WAITONREQ_STATUS & (1UL << ch)) {
    *status = 1;
  } else {
    *status = 0;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  An example usage - set up a 32-bit copy from a buffer using primary block
 **
 ** @param  source - pointer to source buffer
 ** @param  dest - pointer to dest buffer, or register address (eg. a peripheral FIFO)
 ** @param  len - source buffer length (number of DMA transfers in cycle)
 ** @param  ch - DMA channel to use (0-3)
 ** @param  dst_inc - dest address increment - valid values: 2 (word increment), 3 (no increment,
 *for FIFO)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SetupPriAutoWordinc(uint32_t* source, uint32_t* dest, uint32_t len, uint8_t ch,
                                         uint8_t dst_inc) {
  if (source == NULL) return hal_status_badParam;
  if (dest == NULL) return hal_status_badParam;
  if (len == 0) return hal_status_badParam;
  if (ch >= DMA_NUM_CHNLS) return hal_status_badParam;
  if ((dst_inc > DMA_CHNL_CFG_INC_NONE) || (dst_inc < DMA_CHNL_CFG_INC_WORD))
    return hal_status_badParam;

  len--;
  uint32_t src = (uint32_t)source;
  uint32_t dst = (uint32_t)dest;
  if (dst_inc == 0) {
    dst += len;  // byte inc (not supported for 32-bit xfer)
  }
  if (dst_inc == 1) {
    dst += (len << 1);  // halfword inc (not supported for 32-bit xfer)
  }
  if (dst_inc == 2) {
    dst += (len << 2);  // word inc
  }
  src += (len << 2);

  dma_blk.pri[ch].ctrl_raw = 0;  // zero out ctrl
  dma_blk.pri[ch].src = src;
  dma_blk.pri[ch].dst = dst;
  dma_blk.pri[ch].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_AUTO;  // auto
  dma_blk.pri[ch].ctrl.n_minus_1 = len;                            // len already subtracted by one
  dma_blk.pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_WORD;          // word
  dma_blk.pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_WORD;            // word inc
  dma_blk.pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_WORD;          // word
  dma_blk.pri[ch].ctrl.dst_inc = dst_inc;                  // dest ptr increment (word inc or none)
  dma_blk.pri[ch].ctrl.r_power = DMA_CHNL_CFG_R_POWER_16;  // rearbitrate every 16 transfers

  // enable IRQ when DMA cycle is done
  IRQn_Type irq = (IRQn_Type)(DMA_DONE0_IRQn + ch);
  NVIC_SetPriority(irq, 0);
  NVIC_EnableIRQ(irq);

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  An example usage - set up a mem-mem copy from a buffer using primary block
 **
 ** @param  source - pointer to source buffer
 ** @param  dest - pointer to dest buffer
 ** @param  len - source buffer length (number of DMA transfers in cycle)
 ** @param  ch - DMA channel to use (0-3)
 ** @param  src_size - source buffer width (also used as increment) - 0 (byte), 1 (halfword), 2
 *(word)
 ** @param  dst_inc - dest buffer width (also used as increment) - 0 (byte), 1 (halfword), 2 (word)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_MemToMem(uint32_t* source, uint32_t* dest, uint32_t len, uint8_t ch,
                              uint8_t src_size, uint8_t dst_size) {
  if (source == NULL) return hal_status_badParam;
  if (dest == NULL) return hal_status_badParam;
  if (len == 0) return hal_status_badParam;
  if (ch >= DMA_NUM_CHNLS) return hal_status_badParam;
  if ((dst_size > 2) || (src_size > 2)) return hal_status_badParam;

  len--;
  uint32_t src = (uint32_t)source;
  uint32_t dst = (uint32_t)dest;
  if (dst_size < 3) {
    dst += (len << dst_size);
  }
  if (src_size < 3) {
    src += (len << src_size);
  }

  dma_blk.pri[ch].ctrl_raw = 0;  // zero out ctrl
  dma_blk.pri[ch].src = src;
  dma_blk.pri[ch].dst = dst;
  dma_blk.pri[ch].ctrl.cycle_ctrl =
      DMA_CHNL_CFG_CYCLE_CTRL_AUTO;                       // auto (run to completion once triggered)
  dma_blk.pri[ch].ctrl.n_minus_1 = len;                   // len already subtracted by one
  dma_blk.pri[ch].ctrl.src_size = src_size;               // source width
  dma_blk.pri[ch].ctrl.src_inc = src_size;                // source ptr increment
  dma_blk.pri[ch].ctrl.dst_size = dst_size;               // dest width (must match source)
  dma_blk.pri[ch].ctrl.dst_inc = dst_size;                // dest ptr increment
  dma_blk.pri[ch].ctrl.r_power = DMA_CHNL_CFG_R_POWER_4;  // how often to rearbitrate (num xfers)

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  An example usage - copy an SRAM block to a peripheral FIFO register
 **
 ** @param  source - pointer to source buffer (8 bits wide)
 ** @param  dest - pointer to dest (a peripheral FIFO addr)
 ** @param  len - source buffer length (number of DMA transfers in cycle)
 ** @param  ch - DMA channel to use (0-3)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SRAMtoPeriph8(uint8_t* source, uint8_t* dest, uint32_t len, uint8_t ch) {
  if (source == NULL) return hal_status_badParam;
  if (dest == NULL) return hal_status_badParam;
  if (len == 0) return hal_status_badParam;
  if (ch >= DMA_NUM_CHNLS) return hal_status_badParam;

  len--;
  uint32_t src = (uint32_t)source;
  uint32_t dst = (uint32_t)dest;
  src += len;

  dma_blk.pri[ch].ctrl_raw = 0;  // zero out ctrl
  dma_blk.pri[ch].src = src;
  dma_blk.pri[ch].dst = dst;
  dma_blk.pri[ch].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_BASIC;
  dma_blk.pri[ch].ctrl.n_minus_1 = len;  // len already subtracted by one
  dma_blk.pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_BYTE;
  dma_blk.pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_BYTE;  // source ptr increment
  dma_blk.pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_BYTE;
  dma_blk.pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_NONE;    // dest ptr increment
  dma_blk.pri[ch].ctrl.r_power = DMA_CHNL_CFG_R_POWER_16;  // rearbitrate every 16 transfers

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  An example usage - copy an SRAM block to a peripheral FIFO register
 **
 ** @param  source - pointer to source buffer (16 bits wide)
 ** @param  dest - pointer to dest (a peripheral FIFO addr)
 ** @param  len - source buffer length (number of DMA transfers in cycle)
 ** @param  ch - DMA channel to use (0-3)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SRAMtoPeriph16(uint16_t* source, uint16_t* dest, uint32_t len, uint8_t ch) {
  if (source == NULL) return hal_status_badParam;
  if (dest == NULL) return hal_status_badParam;
  if (len == 0) return hal_status_badParam;
  if (ch >= DMA_NUM_CHNLS) return hal_status_badParam;

  len--;
  uint32_t src = (uint32_t)source;
  uint32_t dst = (uint32_t)dest;
  src += (len << 1);

  dma_blk.pri[ch].ctrl_raw = 0;  // zero out ctrl
  dma_blk.pri[ch].src = src;
  dma_blk.pri[ch].dst = dst;
  dma_blk.pri[ch].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_BASIC;
  dma_blk.pri[ch].ctrl.n_minus_1 = len;  // len already subtracted by one
  dma_blk.pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_HALFWORD;
  dma_blk.pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_HALFWORD;  // source ptr increment
  dma_blk.pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_HALFWORD;
  dma_blk.pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_NONE;    // dest ptr increment
  dma_blk.pri[ch].ctrl.r_power = DMA_CHNL_CFG_R_POWER_16;  // rearbitrate every 16 transfers

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  An example usage - copy an SRAM block to a peripheral FIFO register
 **
 ** @param  source - pointer to source buffer (32 bits wide)
 ** @param  dest - pointer to dest (a peripheral FIFO addr)
 ** @param  len - source buffer length (number of DMA transfers in cycle)
 ** @param  ch - DMA channel to use (0-3)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_SRAMtoPeriph32(uint32_t* source, uint32_t* dest, uint32_t len, uint8_t ch) {
  if (source == NULL) return hal_status_badParam;
  if (dest == NULL) return hal_status_badParam;
  if (len == 0) return hal_status_badParam;
  if (ch >= DMA_NUM_CHNLS) return hal_status_badParam;

  len--;
  uint32_t src = (uint32_t)source;
  uint32_t dst = (uint32_t)dest;
  src += (len << 2);

  dma_blk.pri[ch].ctrl_raw = 0;  // zero out ctrl
  dma_blk.pri[ch].src = src;
  dma_blk.pri[ch].dst = dst;
  dma_blk.pri[ch].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_BASIC;
  dma_blk.pri[ch].ctrl.n_minus_1 = len;  // len already subtracted by one
  dma_blk.pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_WORD;
  dma_blk.pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_WORD;  // source ptr increment
  dma_blk.pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_WORD;
  dma_blk.pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_NONE;    // dest ptr increment
  dma_blk.pri[ch].ctrl.r_power = DMA_CHNL_CFG_R_POWER_16;  // rearbitrate every 16 transfers

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  An example usage - copy peripheral RX FIFO to SRAM buffer
 **
 ** @param  source - pointer to source (a peripheral FIFO addr)
 ** @param  dest - pointer to dest buffer (8 bits wide)
 ** @param  len - dest buffer length (number of DMA transfers in cycle)
 ** @param  ch - DMA channel to use (0-3)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_PeriphToSRAM8(uint8_t* source, uint8_t* dest, uint32_t len, uint8_t ch) {
  if (source == NULL) return hal_status_badParam;
  if (dest == NULL) return hal_status_badParam;
  if (len == 0) return hal_status_badParam;
  if (ch >= DMA_NUM_CHNLS) return hal_status_badParam;

  len--;
  uint32_t src = (uint32_t)source;
  uint32_t dst = (uint32_t)dest;
  dst += len;

  dma_blk.pri[ch].ctrl_raw = 0;  // zero out ctrl
  dma_blk.pri[ch].src = src;
  dma_blk.pri[ch].dst = dst;
  dma_blk.pri[ch].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_BASIC;
  dma_blk.pri[ch].ctrl.n_minus_1 = len;  // len already subtracted by one
  dma_blk.pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_BYTE;
  dma_blk.pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_NONE;  // source ptr increment
  dma_blk.pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_BYTE;
  dma_blk.pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_BYTE;      // dest ptr increment
  dma_blk.pri[ch].ctrl.r_power = DMA_CHNL_CFG_R_POWER_EACH;  // rearbitrate each

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  An example usage - copy peripheral RX FIFO to SRAM buffer
 **
 ** @param  source - pointer to source (a peripheral FIFO addr)
 ** @param  dest - pointer to dest buffer (16 bits wide)
 ** @param  len - dest buffer length (number of DMA transfers in cycle)
 ** @param  ch - DMA channel to use (0-3)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_PeriphToSRAM16(uint16_t* source, uint16_t* dest, uint32_t len, uint8_t ch) {
  if (source == NULL) return hal_status_badParam;
  if (dest == NULL) return hal_status_badParam;
  if (len == 0) return hal_status_badParam;
  if (ch >= DMA_NUM_CHNLS) return hal_status_badParam;

  len--;
  uint32_t src = (uint32_t)source;
  uint32_t dst = (uint32_t)dest;
  dst += (len << 1);

  dma_blk.pri[ch].ctrl_raw = 0;  // zero out ctrl
  dma_blk.pri[ch].src = src;
  dma_blk.pri[ch].dst = dst;
  dma_blk.pri[ch].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_BASIC;
  dma_blk.pri[ch].ctrl.n_minus_1 = len;  // len already subtracted by one
  dma_blk.pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_HALFWORD;
  dma_blk.pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_NONE;  // source ptr increment
  dma_blk.pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_HALFWORD;
  dma_blk.pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_HALFWORD;  // dest ptr increment
  dma_blk.pri[ch].ctrl.r_power = DMA_CHNL_CFG_R_POWER_EACH;  // rearbitrate each

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  An example usage - copy peripheral RX FIFO to SRAM buffer
 **
 ** @param  source - pointer to source (a peripheral FIFO addr)
 ** @param  dest - pointer to dest buffer (32 bits wide)
 ** @param  len - dest buffer length (number of DMA transfers in cycle)
 ** @param  ch - DMA channel to use (0-3)
 **
 ** @return hal_status_t status of the driver call
 **
 ******************************************************************************/
hal_status_t HAL_DMA_PeriphToSRAM32(uint32_t* source, uint32_t* dest, uint32_t len, uint8_t ch) {
  if (source == NULL) return hal_status_badParam;
  if (dest == NULL) return hal_status_badParam;
  if (len == 0) return hal_status_badParam;
  if (ch >= DMA_NUM_CHNLS) return hal_status_badParam;

  len--;
  uint32_t src = (uint32_t)source;
  uint32_t dst = (uint32_t)dest;
  dst += (len << 2);

  dma_blk.pri[ch].ctrl_raw = 0;  // zero out ctrl
  dma_blk.pri[ch].src = src;
  dma_blk.pri[ch].dst = dst;
  dma_blk.pri[ch].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_BASIC;
  dma_blk.pri[ch].ctrl.n_minus_1 = len;  // len already subtracted by one
  dma_blk.pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_WORD;
  dma_blk.pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_NONE;  // source ptr increment
  dma_blk.pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_WORD;
  dma_blk.pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_WORD;      // dest ptr increment
  dma_blk.pri[ch].ctrl.r_power = DMA_CHNL_CFG_R_POWER_EACH;  // rearbitrate each

  return hal_status_ok;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
