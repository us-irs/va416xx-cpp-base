/***************************************************************************************
 * @file     va416xx_hal_canbus.c
 * @version  V0.2
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "va416xx.h"
//#include <string.h>
//#include <setjmp.h>

#include "va416xx_hal_canbus.h"

#define CAN_DEBUG

#define MY_ID (0xdeafbeef)

/*******************************************************************************
 **
 ** @brief  can0_test
 ** copied from can_ahb.log
 **
 ******************************************************************************/

//  Below here is old test

/*
//    VOR_CAN->BANK[CANPortNum].GMSKX = (0x0000fffe);


//    VOR_CAN->BANK[CANPortNum].GMSKB = (0x0000ffe7);
    VOR_CAN->BANK[CANPortNum].GMSKB =
                    (7UL)<<CAN_GMSKB_GM0_Pos
//      | CAN_GMSKB_IDE_Msk
//      | CAN_GMSKB_RTR_Msk
        | (0x7FFUL)<<CAN_GMSKB_GM1_Pos;


//    VOR_CAN->BANK[CANPortNum].BMSKX = (0x0000fffe);
    VOR_CAN->BANK[CANPortNum].BMSKX =
//		CAN0_BMSKX_XRTR_Msk
                  (0x7FFFUL)<<CAN_BMSKX_BM_Pos;

//    VOR_CAN->BANK[CANPortNum].BMSKB = (0x0000ffe7);
                VOR_CAN->BANK[CANPortNum].BMSKB =
                  (0x7UL)<<CAN_BMSKB_BM0_Pos
//			| CAN_BMSKB_IDE_Msk
//			| CAN_BMSKB_RTR_Msk
                        | (0x7FFUL)<<CAN_BMSKB_BM1_Pos;

    //The IDx_CMB Registers are used for transmit or receive data
    VOR_CAN->BANK[CANPortNum].ID0_CMB14 = (0x00000000);
    VOR_CAN->BANK[CANPortNum].ID1_CMB14 = (0x0000aaa0);



    VOR_CAN->BANK[CANPortNum].DATA3_CMB14 = (0x00000000);
    VOR_CAN->BANK[CANPortNum].DATA2_CMB14 = (0x00000000);
    VOR_CAN->BANK[CANPortNum].DATA1_CMB14 = (0x00000000);
    VOR_CAN->BANK[CANPortNum].DATA0_CMB14 = (0x00000000);

// VOR_CAN->BANK[CANPortNum].CNSTAT_CMB14 = (0x00000002);
    VOR_CAN->BANK[CANPortNum].CNSTAT_CMB14 =
       (0x2UL)<<CAN_CNSTAT_CMB0_ST_Pos
                         |(0x0UL)<<CAN_CNSTAT_CMB0_PRI_Pos
                         |(0x0UL)<<CAN_CNSTAT_CMB0_DLC_Pos;


//    VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0 = (0x00008008);
    VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0 =
        (0x8UL)<<CAN_CNSTAT_CMB0_ST_Pos
                         |(0x0UL)<<CAN_CNSTAT_CMB0_PRI_Pos
                         |(0x8UL)<<CAN_CNSTAT_CMB0_DLC_Pos;



    VOR_CAN->BANK[CANPortNum].ID0_CMB0 = (0x00000000);
    VOR_CAN->BANK[CANPortNum].ID1_CMB0 = (0x0000aaa0);
    VOR_CAN->BANK[CANPortNum].DATA3_CMB0 = (0x0000cdef);
    VOR_CAN->BANK[CANPortNum].DATA2_CMB0 = (0x000089ab);
    VOR_CAN->BANK[CANPortNum].DATA1_CMB0 = (0x00004567);
    VOR_CAN->BANK[CANPortNum].DATA0_CMB0 = (0x00000123);
    VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0 = (0x0000800c);

//The CIEN register enables the transmit and receive interrupts of message buffers 0 through 14 and
the CAN error interrupt.
//    VOR_CAN->BANK[CANPortNum].CIEN = (0x0000ffff);
    VOR_CAN->BANK[CANPortNum].CIEN =
                    0x7FFF<<CAN_CIEN_IEN_Pos
                    |CAN_CIEN_EIEN_Msk;

 //The CICEN register determines whether the interrupt pending flag in IPND should be translated
into the Interrupt Code field of the STPND register.
 //   VOR_CAN->BANK[CANPortNum].CICEN = (0x0000ffff);
     VOR_CAN->BANK[CANPortNum].CICEN =
                     0x7FFF<<CAN_CICEN_ICEN_Pos
                     |CAN_CICEN_EICEN_Msk;

//    VOR_CAN->BANK[CANPortNum].CGCR = (0x00000001);

//these status reg reads do nothing
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;

// now the data will change on the next read
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;

                //this does not appear in Prog Guide, is this a bitband addr for something in the
NVIC? VOR_CAN->BANK[CANPortNum].CICLR = (0x00000001); VOR_CAN->BANK[CANPortNum].DATA3_CMB0 =
(0x00003d7f); VOR_CAN->BANK[CANPortNum].DATA2_CMB0 = (0x00000f98);
    VOR_CAN->BANK[CANPortNum].DATA1_CMB0 = (0x0000208c);
    VOR_CAN->BANK[CANPortNum].DATA0_CMB0 = (0x00001ce8);
    VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0 = (0x0000800c);

    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;

// now the data will change on the next read
    tmpregdata= VOR_CAN->BANK[CANPortNum].CNSTAT_CMB0;

    tmpregdata= VOR_CAN->BANK[CANPortNum].CIPND;
// write out to GPIO for tester
    portef_out(tmpregdata);        // expected value = 0x00000001

    VOR_CAN->BANK[CANPortNum].CICLR = (0x00000001);
}
*/
void HAL_Can_Disable(uint32_t CANPortNum) {
  uint32_t CAN_IRQn;
  if (CANPortNum == 0) {
    CAN_IRQn = CAN0_IRQn;
  } else {
    CAN_IRQn = CAN1_IRQn;
  }
  NVIC_DisableIRQ((IRQn_Type)(CAN_IRQn));  // Disable IRQ
  // just 0 would be ok too
  VOR_CAN->BANK[CANPortNum].CGCR = CAN_CGCR_BUFFLOCK_Msk | CAN_CGCR_TSTPEN_Msk | CAN_CGCR_DDIR_Msk |
                                   CAN_CGCR_LO_Msk | CAN_CGCR_LOOPBACK_Msk | CAN_CGCR_INTERNAL_Msk |
                                   CAN_CGCR_DIAGEN_Msk | CAN_CGCR_EIT_Msk;
  /*  	  CAN_CGCR_CANEN_Msk     \
  //		| CAN_CGCR_CRX_Msk       \
  //		| CAN_CGCR_CTX_Msk       \
  //		| CAN_CGCR_IGNACK_Msk
  */
  //  VOR_UART->BANK[ch].IRQ_ENB = 0x00;
  //	VOR_UART->BANK[ch].ENABLE = 0x00;
}

void HAL_Can_ClearBuffer(can_cmb_t *can_cmb, const uint32_t count) {
  uint32_t i;
  for (i = 0; i < count; i++) {
    can_cmb[i].CNSTAT = en_can_cmb_cnstat_st_RX_NOT_ACTIVE;
  }
}

uint32_t HAL_Can_Setup(VOR_CAN_Type *myCAN, const can_config_t *myCANCfg,
                       const IRQn_Type CAN_IRQn) {
  // All clks assumed enabled so we don't need	VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE
  NVIC_SetPriority(CAN_IRQn, 0);
  NVIC_EnableIRQ(CAN_IRQn);  // Enable IRQ

  myCAN->CGCR = myCANCfg->CGCR;

  myCAN->CTIM = myCANCfg->CTIM_TSEG2 << CAN_CTIM_TSEG2_Pos    // 6 time quanta
                | myCANCfg->CTIM_TSEG1 << CAN_CTIM_TSEG1_Pos  // 4 time quanta
                | myCANCfg->CTIM_SJW << CAN_CTIM_SJW_Pos      // 4 time quanta
                | myCANCfg->CTIM_PSC << CAN_CTIM_PSC_Pos;     // CAN PreScalar=2

  myCAN->CICEN = myCANCfg->CICEN;
  return 0;
}

void HAL_Can_Setup_gMask(VOR_CAN_Type *myCAN, const hal_can_mskBX_t gMaskBX) {
  myCAN->GMSKX = gMaskBX & 0xffff;  // GM[14:0],XRTR
  myCAN->GMSKB = gMaskBX >> 16;
}

void HAL_Can_Setup_bMask(VOR_CAN_Type *myCAN, const hal_can_mskBX_t bMaskBX) {
  myCAN->BMSKX = bMaskBX & 0xffff;  // GM[28:18],RTR,IDE,GM[17:15]
  myCAN->BMSKB = bMaskBX >> 16;
}

// We cannot dontCare the IDE bit, but it affects whether the RTR or XRTR bit is the one we care
// about not sure if it's ever functional to don't-care RTR bit, if not then they can be taken off
// paramlist and fixed at 0
hal_can_mskBX_t HAL_Can_Make_maskBX(const hal_can_id29_t dontCareID, const bool dontCareRTR_SRR,
                                    const bool dontCareXRTR) {
  // builds a concatenated MSKB:MSKX
  return (BITMASK_AND_SHIFTR(dontCareID, 28, 18, 5)  // upper 16 bit word B starts here
          | (dontCareRTR_SRR << (CAN_GMSKB_RTR_Pos +
                                 16))  // RTR must always be 0=care for IDE=0 for 1 for IDE=0
                                       //    |(CAN_GMSKB_IDE_Pos+16) // IDE must always be 0=care
          | BITMASK_AND_SHIFTR(dontCareID, 17, 15, 0) |
          BITMASK_AND_SHIFTL(dontCareID, 14, 0, 1)    // lower 16 bit word  X starts here
          | (dontCareXRTR << (CAN_GMSKX_XRTR_Pos)));  // must be 0 for ext pkt
}

static hal_can_mskBX_t make_hal_can_xMaskBX(const hal_can_id29_t dontCareID, const bool isIDE,
                                            const bool dontCareRTR) {
  // builds a concatenated MSKB:MSKX
  if (isIDE) {
    return HAL_Can_Make_maskBX(
        dontCareID, 1,
        dontCareRTR);  // ID, RTR is now SRR bit is always 1=don't care, dontCareXRTR=dontCareRTR
  } else {
    return HAL_Can_Make_maskBX(dontCareID, dontCareRTR,
                               0);  // ID, dontCareRTR=dontCareRTR, XRTR=0 don't care
  }
}

static uint32_t set_hal_can_id10_bits_from_pkt(const hal_can_id29_or_11_t id,
                                               const en_can_cmb_msgtype_t msgType,
                                               can_cmb_t *can_cmb) {
  if ((can_cmb->CNSTAT != en_can_cmb_cnstat_st_RX_NOT_ACTIVE) &&
      (can_cmb->CNSTAT != en_can_cmb_cnstat_st_TX_NOT_ACTIVE)) {
    // buffer was not in a receptive state
    // we could force it, but it means something is wrong
    return 1;
  }
  switch (msgType) {
    case (en_can_cmb_msgtype_STD11): {
      can_cmb->ID1 = BITMASK_AND_SHIFTL(id, 10, 0, 5);
      //| CAN_CMB_ID1_STD_RTR_Msk; RTR is zero for ext not remote
      return 0;
    }
    case (en_can_cmb_msgtype_STD11_REM): {
      can_cmb->ID1 = BITMASK_AND_SHIFTL(id, 10, 0, 5) | CAN_CMB_ID1_STD_RTR_Msk;
      return 0;
    }
    case (en_can_cmb_msgtype_EXT29): {
      can_cmb->ID1 = BITMASK_AND_SHIFTR(id, 28, 18, 5) | BITMASK_AND_SHIFTR(id, 17, 15, 0) |
                     CAN_CMB_ID1_IDE_Msk;  // definitely extended frame
      can_cmb->ID0 = BITMASK_AND_SHIFTL(id, 14, 0, 1);
      //| CAN_CMB_ID1_EXT_RTR_Msk; RTR is zero for ext not remote
      return 0;
    }
    case (en_can_cmb_msgtype_EXT29_REM): {
      can_cmb->ID1 = BITMASK_AND_SHIFTR(id, 28, 18, 5) | BITMASK_AND_SHIFTR(id, 17, 15, 0) |
                     CAN_CMB_ID1_IDE_Msk;  // extended frame
      can_cmb->ID0 = BITMASK_AND_SHIFTL(id, 14, 0, 1) | CAN_CMB_ID0_EXT_RTR_Msk;
      return 0;
    }
    default: {
      return 2;  // unknown msg type
    }
  }
}

void HAL_Can_Setup_gMaskIDType(VOR_CAN_Type *myCAN, const hal_can_id29_t dontCareID,
                               const bool dontCareRTR, const bool isIDE) {
  HAL_Can_Setup_gMask(myCAN, make_hal_can_xMaskBX(dontCareID, isIDE, dontCareRTR));
}

void HAL_Can_Setup_bMaskIDType(VOR_CAN_Type *myCAN, const hal_can_id29_t dontCareID,
                               const bool dontCareRTR, const bool isIDE) {
  HAL_Can_Setup_bMask(myCAN, make_hal_can_xMaskBX(dontCareID, isIDE, dontCareRTR));
}

uint32_t HAL_Can_ConfigCMB_Rx(const hal_can_id29_or_11_t id, const en_can_cmb_msgtype_t msgType,
                              can_cmb_t *can_cmb) {
  can_cmb->CNSTAT = en_can_cmb_cnstat_st_RX_NOT_ACTIVE;
  set_hal_can_id10_bits_from_pkt(id, msgType, can_cmb);  // set ID and RTR/IDE bits
  can_cmb->CNSTAT = en_can_cmb_cnstat_st_RX_READY;       // wait for pkt match
  return 0;
}

uint32_t HAL_Can_getCanPkt(can_cmb_t *can_cmb, can_pkt_t *myPkt) {
  if ((can_cmb->CNSTAT & CAN_CNSTAT_CMB0_ST_Msk) >= EN_CAN_CMB_CNSTAT_ST_TX) {
    return 1;  // buffer not confix for rx
  }

  while (((can_cmb->CNSTAT & CAN_CNSTAT_CMB0_ST_Msk) != en_can_cmb_cnstat_st_RX_FULL) &&
         ((can_cmb->CNSTAT & CAN_CNSTAT_CMB0_ST_Msk) != en_can_cmb_cnstat_st_RX_OVERRUN)) {
    // some sort of timeout would be good here instead of a hang
  }

  myPkt->dataLengthBytes = (can_cmb->CNSTAT) >> CAN0_CNSTAT_CMB0_DLC_Pos;  // 12
  if (myPkt->dataLengthBytes > 8) {
    return 2;  // illegal data length??
  }

  // excess bytes beyond dataLengthBytes is junk but not cleared here
  myPkt->data16[0] = can_cmb->DATA0;
  myPkt->data16[1] = can_cmb->DATA1;
  myPkt->data16[2] = can_cmb->DATA2;
  myPkt->data16[3] = can_cmb->DATA3;

  myPkt->timestamp16 = can_cmb->TSTP;

  can_cmb->CNSTAT = 0;  // clr state to RX_NOT_ACTIVE, wipes DLC/PRI too
  return 0;
}

uint32_t HAL_Can_sendCanPkt(can_cmb_t *can_cmb, const can_pkt_t *myPkt) {
  uint32_t dataLengthBytes = myPkt->dataLengthBytes;
  if (dataLengthBytes > 8) {
    dataLengthBytes = 8;
  }

  // all std frames IDE=0 always 11 identifer bits
  // all Ext frames IDE=1
  // in Ext frames, SRR=0 is 11 ID bits   ??or is it?? does this case exist?
  // in Ext frames, SRR=1 is 29 ID bits
  // RTR=1 is remote frame for any of these, but it's ID1[4] in std and ID0[0] in ext

  if ((can_cmb->CNSTAT & CAN_CNSTAT_CMB0_ST_Msk) == en_can_cmb_cnstat_st_RX_NOT_ACTIVE) {
    // if buf configured as rx and idle, make it a tx
    can_cmb->CNSTAT = en_can_cmb_cnstat_st_TX_NOT_ACTIVE;
  }

  if ((can_cmb->CNSTAT & CAN_CNSTAT_CMB0_ST_Msk) != en_can_cmb_cnstat_st_TX_NOT_ACTIVE) {
    // buf was active as a tx or rx so not free to tx
    return 1;
  }
  // no timestamp on outgoing msg
  // afaik no effect from putting in more data bytes than used
  can_cmb->DATA0 = myPkt->data16[0];
  can_cmb->DATA1 = myPkt->data16[1];
  can_cmb->DATA2 = myPkt->data16[2];
  can_cmb->DATA3 = myPkt->data16[3];

  set_hal_can_id10_bits_from_pkt(myPkt->id, myPkt->msgType, can_cmb);

  // txType must only be
  // en_can_cmb_cnstat_st_TX_RTR/en_can_cmb_cnstat_st_TX_ONCE/en_can_cmb_cnstat_st_TX_ONCE_RTR
  // otherwise return 3?
  if ((myPkt->msgType == en_can_cmb_msgtype_EXT29_REM) |
      (myPkt->msgType == en_can_cmb_msgtype_STD11_REM)) {
    can_cmb->CNSTAT = (dataLengthBytes << CAN_CNSTAT_CMB0_DLC_Pos) |
                      (uint32_t)en_can_cmb_cnstat_st_TX_ONCE_RTR |
                      (myPkt->txPriorityCode << CAN_CNSTAT_CMB0_PRI_Pos);
    // this will tx then auto-transition to RX_READY and should rx the response
    //?CMB must care about RTR or XRTR? RTR or XRTR will already be set in the ID10 so maybe it
    // won't care so ultimately code needs to come back later and look for CNSTAT.ST=RX_FULL it must
    // not be possible for it to match another CMB, not sure what would happen
  } else {
    can_cmb->CNSTAT = (dataLengthBytes << CAN_CNSTAT_CMB0_DLC_Pos) |
                      (uint32_t)en_can_cmb_cnstat_st_TX_ONCE |
                      (myPkt->txPriorityCode << CAN_CNSTAT_CMB0_PRI_Pos);
  }
  return 0;
}
