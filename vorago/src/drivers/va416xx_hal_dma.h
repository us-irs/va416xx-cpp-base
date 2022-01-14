/***************************************************************************************
 * @file     va416xx_hal_dma.h
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

#ifndef __HAL_DMA_H
#define __HAL_DMA_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_debug.h"
#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/** Version */
#define HAL_DMA_VERSION (0x00000200)  // 0.2.0 (15 May 2020)

#define DMA_NUM_CHNLS (4)

/** Pointer checks */
#define POINTS_TO_SRAM0(p) (((p >= 0x1fff8000) && (p < 0x20000000)) ? true : false)
#define POINTS_TO_SRAM1(p) (((p >= 0x20000000) && (p < 0x20008000)) ? true : false)
#define POINTS_TO_SRAM(p) (((p >= 0x1fff8000) && (p < 0x20008000)) ? true : false)
#define POINTS_TO_EXTMEM(p) (((p >= 0x60000000) && (p < 0x61000000)) ? true : false)
#define POINTS_TO_PERIPH(p) (((p >= 0x40000000) && (p < 0x40030000)) ? true : false)
#define POINTS_TO_CODE(p) (((p >= 0x00000000) && (p < 0x00040000)) ? true : false)

/** DMA channel configuration bitfield defines */

// Source, destination address increment (src_inc, dst_inc)
#define DMA_CHNL_CFG_INC_BYTE (0)
#define DMA_CHNL_CFG_INC_HALFWORD (1)
#define DMA_CHNL_CFG_INC_WORD (2)
#define DMA_CHNL_CFG_INC_NONE (3)

// Source, destination data size (src_size, dst_size). Source/dest size must match
#define DMA_CHNL_CFG_SIZE_BYTE (0)
#define DMA_CHNL_CFG_SIZE_HALFWORD (1)
#define DMA_CHNL_CFG_SIZE_WORD (2)

// Source, destination prot control flags (can be ORed together)
#define DMA_CHNL_CFG_PROT_CACHEABLE (4)
#define DMA_CHNL_CFG_PROT_BUFFERABLE (2)
#define DMA_CHNL_CFG_PROT_PRIVILEGED (1)
#define DMA_CHNL_CFG_PROT_NONE (0)

// r_power - number of cycles before controller rearbitrates (2^R cycles)
#define DMA_CHNL_CFG_R_POWER_EACH (0)
#define DMA_CHNL_CFG_R_POWER_2 (1)
#define DMA_CHNL_CFG_R_POWER_4 (2)
#define DMA_CHNL_CFG_R_POWER_8 (3)
#define DMA_CHNL_CFG_R_POWER_16 (4)
#define DMA_CHNL_CFG_R_POWER_32 (5)
#define DMA_CHNL_CFG_R_POWER_64 (6)
#define DMA_CHNL_CFG_R_POWER_128 (7)
#define DMA_CHNL_CFG_R_POWER_256 (8)
#define DMA_CHNL_CFG_R_POWER_512 (9)
#define DMA_CHNL_CFG_R_POWER_1024 (10)

// next_useburst - set useburst when in scatter-gather and completing alt cycle
#define DMA_CHNL_CFG_NEXT_USEBURST_NO (0)
#define DMA_CHNL_CFG_NEXT_USEBURST_YES (1)

// cycle_ctrl - DMA channel operating mode
#define DMA_CHNL_CFG_CYCLE_CTRL_STOP (0)
#define DMA_CHNL_CFG_CYCLE_CTRL_BASIC (1)
#define DMA_CHNL_CFG_CYCLE_CTRL_AUTO (2)
#define DMA_CHNL_CFG_CYCLE_CTRL_PINGPONG (3)
#define DMA_CHNL_CFG_CYCLE_CTRL_MEM_SG_PRI (4)
#define DMA_CHNL_CFG_CYCLE_CTRL_MEM_SG_ALT (5)
#define DMA_CHNL_CFG_CYCLE_CTRL_PER_SG_PRI (6)
#define DMA_CHNL_CFG_CYCLE_CTRL_PER_SG_ALT (7)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#elif defined(__ICCARM__)
#pragma language = extended
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
#pragma warning 586
#else
#warning Not supported compiler type
#endif

/** DMA channel config struct */
typedef struct stc_dma_chnl_cfg {
  uint32_t cycle_ctrl : 3;
  uint32_t next_useburst : 1;
  uint32_t n_minus_1 : 10;
  uint32_t r_power : 4;
  uint32_t src_prot_ctrl : 3;
  uint32_t dst_prot_ctrl : 3;
  uint32_t src_size : 2;
  uint32_t src_inc : 2;
  uint32_t dst_size : 2;
  uint32_t dst_inc : 2;
} stc_dma_chnl_cfg_t;

/** DMA channel struct */
typedef struct stc_dma_chnl {
  /* Note: This DMA engine counts from high address -> low (counts down) */
  /* So, source, dest pointers need to point to the LAST element, not the first */
  uint32_t src;  // source *END* pointer - addr of last elememt in source
  uint32_t dst;  // destination *END* pointer - addr of last element in dest
  union {
    uint32_t ctrl_raw;        // Control register raw val
    stc_dma_chnl_cfg_t ctrl;  // Control register bitfield
  };
  uint32_t padding;  // unused
} stc_dma_chnl_t;

/** DMA control block data structure */
typedef struct stc_dma_control_blk {
  stc_dma_chnl_t pri[DMA_NUM_CHNLS];  // primary ch   0x00 to 0x3f
  stc_dma_chnl_t alt[DMA_NUM_CHNLS];  // alternate ch 0x40 to 0x7f
} stc_dma_control_blk_t;

/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
#pragma pop
#elif defined(__ICCARM__)
/* leave anonymous unions enabled */
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
#pragma warning restore
#else
#warning Not supported compiler type
#endif

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

// DMA master controls
extern hal_status_t HAL_DMA_Init(stc_dma_control_blk_t* ctrlBasePtr, bool cacheable,
                                 bool bufferable, bool privileged);
extern hal_status_t HAL_DMA_Reset(void);
extern hal_status_t HAL_DMA_Enclock(void);
extern hal_status_t HAL_DMA_Declock(void);
extern hal_status_t HAL_DMA_ClearBusError(void);
extern hal_status_t HAL_DMA_SetCtrlBasePtr(stc_dma_control_blk_t* ctrlBasePtr);

// Channel specific
extern hal_status_t HAL_DMA_SwRequest(uint32_t ch);
extern hal_status_t HAL_DMA_SetUseburst(uint32_t ch, bool useburst);
extern hal_status_t HAL_DMA_SetRequestEnable(uint32_t ch, bool enable);
extern hal_status_t HAL_DMA_SetChannelEnable(uint32_t ch, bool enable);
extern hal_status_t HAL_DMA_SetPriAlt(uint32_t ch, bool primary);
extern hal_status_t HAL_DMA_SetChannelPriority(uint32_t ch, bool high);
extern hal_status_t HAL_DMA_CopyChannelCtrl(uint32_t ch, bool primary, stc_dma_chnl_t* pCtrl);
extern hal_status_t HAL_DMA_GetWaitOnReqStatus(uint32_t ch, uint8_t* status);

// example usage
extern hal_status_t HAL_DMA_SetupPriAutoWordinc(uint32_t* source, uint32_t* dest, uint32_t len,
                                                uint8_t ch, uint8_t dst_inc);
extern hal_status_t HAL_DMA_MemToMem(uint32_t* source, uint32_t* dest, uint32_t len, uint8_t ch,
                                     uint8_t src_size, uint8_t dst_size);
extern hal_status_t HAL_DMA_SRAMtoPeriph8(uint8_t* source, uint8_t* dest, uint32_t len, uint8_t ch);
extern hal_status_t HAL_DMA_SRAMtoPeriph16(uint16_t* source, uint16_t* dest, uint32_t len,
                                           uint8_t ch);
extern hal_status_t HAL_DMA_SRAMtoPeriph32(uint32_t* source, uint32_t* dest, uint32_t len,
                                           uint8_t ch);
extern hal_status_t HAL_DMA_PeriphToSRAM8(uint8_t* source, uint8_t* dest, uint32_t len, uint8_t ch);
extern hal_status_t HAL_DMA_PeriphToSRAM16(uint16_t* source, uint16_t* dest, uint32_t len,
                                           uint8_t ch);
extern hal_status_t HAL_DMA_PeriphToSRAM32(uint32_t* source, uint32_t* dest, uint32_t len,
                                           uint8_t ch);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __HAL_DMA_H */
