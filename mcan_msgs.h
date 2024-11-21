#ifndef MCAN_MSGS_H_
#define MCAN_MSGS_H_

#ifdef __cplusplus
extern "C"
{
#endif


#define MCAN_MSGS_TX_MSG_DATA_LENGTH         8U

#include <stdint.h>
#include <string.h>
#include <cla_inv_cont_shared.h>
#include <mcan.h>

#include "mcanbus.h"

typedef struct MCANMsgReceivedDecodeObj
{
    // authState authStateRcv;


} MCANMsgReceivedDecodeObj;

typedef struct MCANMsgSendMsgType
{
    const uint8_t canId;          // CANID to send with this data
    float * pvalue;               // read this pointer for the value ..
    volatile float value;         // read value from this float directly
    uint16_t * puint16_value;     // pointer to a uint16_t type value
    volatile uint16_t timestamp;  // what time was the data gathered
    uint32_t count;
} MCANMsgSendMsgType;

// Sending CANID reference IDs / X and TX are represented here.
// Receive objects from other components on the MCAN BUS
//  Could be display (prefix: 0x0 00 ) , or other boards (prefix: 0x[d] 00 )
typedef enum MCANMsgRXIdSuffix
{
    E_MCANDATA_MSGID_SUFFIX_NULL             = 0x00,   // maps also to E_MCANDATA_NULL
    E_MCANDATA_MSGID_SUFFIX_Van              = 0x01,
    E_MCANDATA_MSGID_SUFFIX_currentA         = 0x02,
    E_MCANDATA_MSGID_SUFFIX_powerA           = 0x03,
    E_MCANDATA_MSGID_SUFFIX_Vbn              = 0x04,
    E_MCANDATA_MSGID_SUFFIX_currentB         = 0x05,
    E_MCANDATA_MSGID_SUFFIX_powerB           = 0x06,
    E_MCANDATA_MSGID_SUFFIX_voltBatt         = 0x07,
    E_MCANDATA_MSGID_SUFFIX_status           = 0x08,
    E_MCANDATA_MSGID_SUFFIX_MCUtimer         = 0x09,
    E_MCANDATA_MSGID_SUFFIX_IoTechaOnline    = 0x0A,
    // Note the gap for 0x0B
    E_MCANDATA_MSGID_SUFFIX_COMMAND_StartAC         = 0x0C,
    E_MCANDATA_MSGID_SUFFIX_COMMAND_StopAC          = 0x0D,
    E_MCANDATA_MSGID_SUFFIX_COMMAND_EmergencyStop   = 0x0E,
    E_MCANDATA_MSGID_SUFFIX_VERSION                 = 0x0F,
    E_MCANDATA_MSGID_SUFFIX_LAST                    = 0x10

} MCANMsgRXIdSuffix;

typedef struct MCANMsgReceiveMsgType
{
    MCANMsgRXIdSuffix suffix;          // CANID suffix on the wire
    uint16_t prefix;  // sender
    volatile uint16_t value;         // read value from this float directly
    volatile uint16_t timestamp;  // what time was the data gathered
    uint32_t count;               // how many of these did we receive
} MCANMsgReceiveMsgType;


typedef struct MCANMsgReceivedObj
{
    MCANMsgReceiveMsgType StartACRcv;
    MCANMsgReceiveMsgType StopACRcv;
    MCANMsgReceiveMsgType EmergencyStopRcv;
    MCANMsgReceivedDecodeObj decode;
} MCANMsgReceivedObj;

typedef struct MCANMsgSendObj
{
    uint16_t lastSendTimestamp;  // track last message handling time

    MCANMsgSendMsgType Van;
    MCANMsgSendMsgType currentA;
    MCANMsgSendMsgType powerA;
    MCANMsgSendMsgType Vbn;
    MCANMsgSendMsgType currentB;
    MCANMsgSendMsgType powerB;
    MCANMsgSendMsgType voltBatt;
    MCANMsgSendMsgType statusMsg;
    MCANMsgSendMsgType MCUtimer;
    MCANMsgSendMsgType IoTechaOnlineMsg;
} MCANMsgSendObj;

extern volatile uint16_t cpuTimer0IntCount;
extern MCANMsgReceivedObj SessionRcvMsgs;
extern uint16_t BOARD_ID;    // see MCANMsg_inverter_cont


void initMCANMsgsMessages();
void processMCAMsg( MCAN_RxBufElement * RxMsg );
void MCANMsgStartupMsg();

void MCANMsgSendThing();




#endif  // MCAN_MSGS_H_

