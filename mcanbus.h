
#ifndef __MCANBUS_H__
#define __MCANBUS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "driverlib.h"
#include "device.h"
#include "inc/stw_types.h"
#include "inc/stw_dataTypes.h"
#include <string.h>    // for memset

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h> /* strtoimax */
#include <errno.h>
#include <stdint.h>

#include "version.h"
#include "inventev_msgs.h"


// Defines

// :: WARNING!! :: DO NOT use this #define name, it breaks MCAN RX interrupts.
// Number of messages to cache in an array for RX
//#define NUM_OF_MSG                      (1U)

// Standard filter list identifier
#define MCAN_STD_ID_FILTER_NUM          (1U)
#define MCAN_EXT_ID_FILTER_NUM          (1U)  // change from 0
#define MCAN_FIFO_0_NUM                 (10U)
#define MCAN_FIFO_0_ELEM_SIZE           (MCAN_ELEM_SIZE_8BYTES)  // (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_FIFO_1_NUM                 (10U)
#define MCAN_FIFO_1_ELEM_SIZE           (MCAN_ELEM_SIZE_8BYTES)  // (MCAN_ELEM_SIZE_64BYTES)

#define MCAN_RX_BUFF_NUM                (10U)
#define MCAN_RX_BUFF_ELEM_SIZE          (MCAN_ELEM_SIZE_8BYTES)  // (MCAN_ELEM_SIZE_64BYTES)

#define MCAN_TX_BUFF_SIZE               (5U)
#define MCAN_TX_BUFF_ELEM_SIZE          (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_TX_EVENT_SIZE              (10U)

//#define MCAN_EXT_ID_AND_MASK            (0x1FFFFFFFU)
//#define MCAN_MSG_INT                    (0x81200)

#define MCAN_CUSTOM_CANID_BITMASK       (0x1FFFC000U) // refer to MCANSS_STD_ID_FILTER_SFID1_MASK in mcan.c, which is also wrong ..
#define MCAN_CUSTOM_CANID_SHIFT         (18U)

//  Defining Starting Addresses for Message RAM Sections,
//  (Calculated from Macros based on User defined configuration above)

// Start searching for CAN messages starting with this address and above
#define MCAN_STD_ID_FILT_START_ADDR     (0x0U)
#define MCAN_EXT_ID_FILT_START_ADDR     (MCAN_STD_ID_FILT_START_ADDR + ((MCAN_STD_ID_FILTER_NUM * MCANSS_STD_ID_FILTER_SIZE_WORDS * 4U)))
#define MCAN_FIFO_0_START_ADDR          (MCAN_EXT_ID_FILT_START_ADDR + ((MCAN_EXT_ID_FILTER_NUM * MCANSS_EXT_ID_FILTER_SIZE_WORDS * 4U)))
#define MCAN_FIFO_1_START_ADDR          (MCAN_FIFO_0_START_ADDR + (MCAN_getMsgObjSize(MCAN_FIFO_0_ELEM_SIZE) * 4U * MCAN_FIFO_0_NUM))
#define MCAN_RX_BUFF_START_ADDR         (MCAN_FIFO_1_START_ADDR + (MCAN_getMsgObjSize(MCAN_FIFO_1_ELEM_SIZE) * 4U * MCAN_FIFO_1_NUM))
#define MCAN_TX_BUFF_START_ADDR         (MCAN_RX_BUFF_START_ADDR + (MCAN_getMsgObjSize(MCAN_RX_BUFF_ELEM_SIZE) * 4U * MCAN_RX_BUFF_NUM))
#define MCAN_TX_EVENT_START_ADDR        (MCAN_TX_BUFF_START_ADDR + (MCAN_getMsgObjSize(MCAN_TX_BUFF_ELEM_SIZE) * 4U * MCAN_TX_BUFF_SIZE))

// Added
#define MCAN_TX_FQ_SIZE             (0U)

// Globals, see inventev_inverter_cont.h
extern volatile uint16_t cpuTimer0IntCount;
//extern InventevReceivedObj SessionRcvMsgs;

typedef struct MCANInterruptCounters
{
    uint8_t RXFIFO0NewMsg ;  // MCAN_INTR_SRC_RX_FIFO0_NEW_MSG
    uint8_t RXFIFO0Watermark ;  // MCAN_INTR_SRC_RX_FIFO0_WATERMARK
    uint8_t RXFIFO0Full; // MCAN_INTR_SRC_RX_FIFO0_FULL
    uint8_t RXFIFO0MsgLost; // MCAN_INTR_SRC_RX_FIFO0_MSG_LOST
    uint8_t RXFIFO1NewMsg; //MCAN_INTR_SRC_RX_FIFO1_NEW_MSG
    uint8_t RXFIFO1Watermark;//MCAN_INTR_SRC_RX_FIFO1_WATERMARK
    uint8_t RXFIFO1Full; // MCAN_INTR_SRC_RX_FIFO1_FULL
    uint8_t RXFIFO1MsgLost;  //MCAN_INTR_SRC_RX_FIFO1_MSG_LOST
    uint8_t HighPriority; // RXMCAN_INTR_SRC_HIGH_PRIO_MSG
    uint8_t TransmissionComplete; // MCAN_INTR_SRC_TRANS_COMPLETE
    uint8_t TransmissionCancellationFinished; // MCAN_INTR_SRC_TRANS_CANCEL_FINISH
    uint8_t TXFIFOEmpty; //MCAN_INTR_SRC_TX_FIFO_EMPTY
    uint8_t TXFIFONewEntry; // MCAN_INTR_SRC_TX_EVT_FIFO_NEW_ENTRY
    uint8_t TXFIFOWatermark; //MCAN_INTR_SRC_TX_EVT_FIFO_WATERMARK
    uint8_t TXFIFOFull; // MCAN_INTR_SRC_TX_EVT_FIFO_FULL
    uint8_t TXFIFOElementLost; // MCAN_INTR_SRC_TX_EVT_FIFO_ELEM_LOST
    uint8_t TimestampWraparound; //MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND
    uint8_t MsgRamAccessFailure; // CAN_INTR_SRC_MSG_RAM_ACCESS_FAILURE
    uint8_t Timeout; // MCAN_INTR_SRC_TIMEOUT
    uint8_t DedicatedRXBufferMsg; // MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG
    uint8_t BitErrorCorrected; // MCAN_INTR_SRC_BIT_ERR_CORRECTED
    uint8_t BitErrorUncorrected; // MCAN_INTR_SRC_BIT_ERR_UNCORRECTED
    uint8_t ErrorLogOverflow; // MCAN_INTR_SRC_ERR_LOG_OVRFLW
    uint8_t ErrorPassive; // MCAN_INTR_SRC_ERR_PASSIVE
    uint8_t WarningStatus; // MCAN_INTR_SRC_WARNING_STATUS
    uint8_t BusOffStatus; // MCAN_INTR_SRC_BUS_OFF_STATUS
    uint8_t Watchdog; // MCAN_INTR_SRC_WATCHDOG
    uint8_t ErrorArbitrationPhase; // MCAN_INTR_SRC_PROTOCOL_ERR_ARB
    uint8_t ProtocolErrorInDataPhase; // MCAN_INTR_SRC_PROTOCOL_ERR_DATA
    uint8_t AccesstoReserverAddress; // MCAN_INTR_SRC_RES_ADDR_ACCESS
    uint8_t Unknown;

} MCANInterruptCounters;

void MCANInit(void);
static void MCANConfig(void);
static void MCANIntrConfig(void);
//void MCANsendThing(InventevSendMsgType *thing );
void MCANstartupMsg();
void MCANpackFloat (float * myFloat, MCAN_TxBufElement * myMsg);

void MCANsendZeroMonitoring (void);
static void fixCANID( MCAN_RxBufElement *elem );

__interrupt void MCANIntr1ISR(void);

#endif
