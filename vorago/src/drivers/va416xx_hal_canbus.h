#ifndef __VA416XX_HAL_CANBUS_H
#define __VA416XX_HAL_CANBUS_H

#include "event_handler_index.h"
#include "va416xx_hal.h"

// can_ug.pdf declares as PRI, but our PG labels as "PR"
#define CAN_CNSTAT_CMB0_PR_Pos CAN_CNSTAT_CMB0_PRI_Pos

#define CAN_CMB_ID1_IDE_Pos (3UL) /*!< CAN                                 */
#define CAN_CMB_ID1_IDE_Msk (0x1UL << CAN_CMB_ID1_IDE_Pos) /*!< CAN                    */

// extended frame only
#define CAN_CMB_ID1_SRR_Pos (4UL) /*!< CAN                                 */
#define CAN_CMB_ID1_SRR_Msk (0x1UL << CAN_CMB_ID1_SRR_Pos) /*!< CAN                    */

// RTR is in this pos for std frame only, in ext this is SRR and MUST be set to 1
#define CAN_CMB_ID1_STD_RTR_Pos (4UL)
#define CAN_CMB_ID1_STD_RTR_Msk (1UL << CAN_CMB_ID1_STD_RTR_Pos)

// ext this is SRR and MUST be set to 1
#define CAN_CMB_ID1_EXT_SRR_Pos (4UL)
#define CAN_CMB_ID1_EXT_SRR_Msk (1UL << CAN_CMB_ID1_EXT_SRR_Pos)

// ext frame only, 11 or 29 bit
#define CAN_CMB_ID0_EXT_RTR_Pos (0UL)
#define CAN_CMB_ID0_EXT_RTR_Msk (1UL << CAN_CMB_ID0_EXT_RTR_Pos)

#define HAL_CAN_VERSION (0x00000101)

#define NUM_CAN_BANKS (2)
#define CAN_INVALID_BANKNUM (0xFF)
#define CAN_MAX_BANKNUM (NUM_CAN_BANKS - 1)

// I was going to call this FU_ALL but I see an issue
#define ALLF (0xffffffffUL)

// BITMASK(val,msb,lsb) take in 32 bit val, mask out from lsb to msb, inclusive
#define BITMASK(val, msb, lsb) ((val & ((ALLF << (31 - msb)) >> (31 - (msb - lsb)))) << lsb)
//[16:14] is <<15 YES then >>29 YES then <<14 YES

// BITMASK_AND_SHIFTR(val,msb,lsb,resultLsbPos)
// take in 32 bit val, mask out from lsb to msb, inclusive
// then shift the indicated bits so lsb is resultLsbPos
// do not feed a resultLsbPos that results in a negative shift
// zero shifts are acceptable
// C does not support signed shifts, even during precompiler constant folding
#define BITMASK_AND_SHIFTR(val, msb, lsb, resultLsbPos) \
  (BITMASK(val, msb, lsb) >> (lsb - resultLsbPos))
#define BITMASK_AND_SHIFTL(val, msb, lsb, resultLsbPos) \
  (BITMASK(val, msb, lsb) << (resultLsbPos - lsb))

typedef uint32_t hal_can_id29_t;
typedef uint32_t hal_can_id11_t;
typedef uint32_t hal_can_id29_or_11_t;  // std frame only 11 bits, ext is 29 bits
typedef uint32_t hal_can_irq_num_t;
typedef uint32_t hal_can_mskBX_t;
typedef uint32_t hal_can_id10_32bits_t;  // id1[15:0]:id0[15:0]
typedef uint32_t hal_can_id_plus_rtr_srr_t;

// this is dictated by reg def order, just getting restructured
// there are 15 per CAN module
typedef struct {
  __IO uint32_t CNSTAT; /*!< Buffer Status / Control Register */
  __IO uint32_t TSTP;  /*!< CAN Frame Timestamp                                                   */
  __IO uint32_t DATA3; /*!< CAN Frame Data Word 3[15:0]                                           */
  __IO uint32_t DATA2; /*!< CAN Frame Data Word 2[15:0]                                           */
  __IO uint32_t DATA1; /*!< CAN Frame Data Word 1[15:0]                                           */
  __IO uint32_t DATA0; /*!< CAN Frame Data Word 0[15:0]                                           */
  __IO uint32_t ID0;   /*!< CAN Frame Identifier Word 0                                           */
  __IO uint32_t ID1;   /*!< CAN Frame Identifier Word 1                                           */
} can_cmb_t;

typedef struct {
  uint32_t CGCR;        // many flags
  uint32_t CTIM_TSEG2;  //+1 for actual
  uint32_t CTIM_TSEG1;  //+1 for actual
  uint32_t CTIM_SJW;    //+1 for actual
  uint32_t CTIM_PSC;    //+2 for actual
  uint32_t CICEN;       // EICEN|ICEN[14:0]
} can_config_t;

typedef enum {
  en_can_cmb_msgtype_NULL = 0,
  en_can_cmb_msgtype_STD11 = 1,
  en_can_cmb_msgtype_STD11_REM = 2,  // set as tx buf *without* data and data will be filled
  en_can_cmb_msgtype_STD11_REM_RESP =
      3,                         // set as rx buf *with* data and will autoresp back when matched
  en_can_cmb_msgtype_EXT29 = 4,  // set as tx buf *without* data and data will be filled
  en_can_cmb_msgtype_EXT29_REM = 5,  // set as tx buf *without* data and data will be filled
  en_can_cmb_msgtype_EXT29_REM_RESP =
      6  // set as rx buf *with* data and will autoresp back when matched
} en_can_cmb_msgtype_t;

// could use a status
typedef struct {
  // uint32_t  CNSTAT;                       /*!< Buffer Status / Control Register               */
  en_can_cmb_msgtype_t msgType;
  uint32_t dataLengthBytes;  // 0 t0 8, is 0 legit?
  uint16_t timestamp16;      /*!< CAN Frame Timestamp rx only                    */
  uint16_t data16[4];        /*!< CAN Frame Data 16 bit Word [3:0]               */
  hal_can_id29_t id;         // 29 or 11 bits
  //  uint32_t  ID0;                          /*!< CAN Frame Identifier Word 0                    */
  //  uint32_t  ID1;                          /*!< CAN Frame Identifier Word 1                    */
  uint32_t txPriorityCode;  // 4 bits is 0 highest or lowest? tx frame only
} can_pkt_t;

typedef enum {
  en_can_cmb_cnstat_st_RX_NOT_ACTIVE =
      0x0,  // CNSTAT_ST must be set to this to enable writes to rx ID filters
  en_can_cmb_cnstat_st_RX_READY = 0x2,  // code will set CNSTAT_ST to this to enable rx
  en_can_cmb_cnstat_st_RX_BUSY0 = 0x3,
  en_can_cmb_cnstat_st_RX_FULL = 0x4,  // rx buf is valid
  en_can_cmb_cnstat_st_RX_BUSY1 = 0x5,
  en_can_cmb_cnstat_st_RX_OVERRUN =
      0x6,  // rx buf is valid & latest but at least one earlier rx was tossed, NOT used if BUFLOCK
  en_can_cmb_cnstat_st_RX_BUSY2 = 0x7,
  en_can_cmb_cnstat_st_TX_NOT_ACTIVE =
      0x8,  // code can set CNSTAT_ST to this to load tx or abort a stuck tx
  en_can_cmb_cnstat_st_TX_RTR =
      0xa,  // code sets CNSTAT_ST to this for automated CMB *response* to a matching RTR
  en_can_cmb_cnstat_st_TX_ONCE = 0xc,  // code sets CNSTAT_ST to this to transmit RTR or non-RTR
  en_can_cmb_cnstat_st_TX_BUSY0 = 0xd,
  en_can_cmb_cnstat_st_TX_ONCE_RTR = 0xe,
  en_can_cmb_cnstat_st_TX_BUSY2 = 0xf
} en_can_cmb_cnstat_st_t;

#define EN_CAN_CMB_CNSTAT_ST_TX (en_can_cmb_cnstat_st_TX_NOT_ACTIVE)

extern bool HAL_Can_isRxResp(can_cmb_t *can_cmb);
extern uint32_t HAL_Can_getRxResp(can_cmb_t *can_cmb, can_pkt_t *myPkt);
extern uint32_t HAL_Can_getCanPkt(can_cmb_t *can_cmb, can_pkt_t *myPkt);
extern uint32_t HAL_Can_sendCanPkt(can_cmb_t *can_cmb, const can_pkt_t *myPkt);
extern void HAL_Can_SetupLoopback(uint32_t CANPortNum);
extern void HAL_Can_Disable(uint32_t CANPortNum);
extern void HAL_Can_ClearBuffer(can_cmb_t *can_cmb,
                                const uint32_t count);  // no bounds checking here
extern uint32_t HAL_Can_Setup(VOR_CAN_Type *myCAN, const can_config_t *myCANCfg,
                              const IRQn_Type CAN_IRQn);
extern uint32_t HAL_Can_ConfigCMB_Rx(const hal_can_id29_or_11_t id,
                                     const en_can_cmb_msgtype_t msgType, can_cmb_t *can_cmb);
// gMask applies to Buffer[0..13]
extern void HAL_Can_Setup_gMask(VOR_CAN_Type *myCAN, const hal_can_mskBX_t gMaskBX);
// bMask applies only to Buffer[14]
extern void HAL_Can_Setup_bMask(VOR_CAN_Type *myCAN, const hal_can_mskBX_t bMaskBX);
// We cannot dontCare the IDE bit, but it affects whether the RTR or XRTR bit is the one we care
// about not sure if it's ever functional to don't-care RTR bit, if not then they can be taken off
// paramlist and fixed at 0
extern hal_can_mskBX_t HAL_Can_Make_maskBX(const hal_can_id29_t dontCareID,
                                           const bool dontCareRTR_SRR, const bool dontCareXRTR);

#endif
