/******************************************************************************
SUMMARY

This communications abstraction focuses on the CAN message setup, and formats
required for the solution.

Care is taken to avoid collisions with other pieces of code in the project.
Since the project is using globals, we are polluting the global namespace in
a similar manner.  To reduce the occurance of collisions most data is held in
structs and "objects", to prevent accidental overwriting from other pieces of
code.

NOTES
Take care to watch for global variable naming collisions.

Conversion from float -> CAN -- CAN -> float will lose precisions and is done in
a "works for now" kind of way.  Consider improving the type conversion in the future.

******************************************************************************/

#ifndef __CANBUS_H__
#define __CANBUS_H__

#ifdef __cplusplus
extern "C"
{
#endif

// For CAN 'data thing' message packing
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h> /* strtoimax */
#include <errno.h>
#include <stdint.h>
#include "driverlib.h"
#include "device.h"  // for DEVICE_SYSCLK_FREQ
#include "cana_msgs.h"

// Preprocessor directives

__interrupt void canaISR(void);


typedef struct msgType
{
    const uint16_t canId;      // CANID to send with this data
    const uint8_t canTxObjId;  // registered object id
    volatile float value;         // read value
    const char * name[10];     // what is this thing we are tracking?
    volatile uint16_t timestamp;  // what time was the data gathered
    uint32_t count;
} msgType;

// Create our data structures
typedef enum txObjId
{
    E_NULL  = 0,
    E_DATA_VAL1_TX_MSGID = 0x14,
    E_DATA_VAL2_TX_MSGID = 0x15,
    E_DATA_VAL3_TX_MSGID = 0x16,
    E_DATA_VAL4_TX_MSGID = 0x17,
    E_DATA_VAL5_TX_MSGID = 0x18,
    E_DATA_VAL6_TX_MSGID = 0x19,
} txObjId;

typedef struct objData {
    msgType val1;
    msgType val2;
    msgType val3;
    msgType val4;
    msgType val5;
    msgType val6;
} objData;

// Globals

//-----------------------------------
// CAN Message ID Counters and interrupts tracked with main logic
//-----------------------------------

extern volatile uint16_t CANAmsgCount;

extern SystemReceivedObj DispenserMsgs;

// Interrupt tracking
//extern uint16_t it;  // for CAN interrupt status interrupt type
extern uint32_t StatusArray[20];
extern uint32_t GlobalStatusISR;
//-----------------------------------

//-----------------------------------
// CANA Data Buffers
//-----------------------------------
#define TX_MSG_DATA_LENGTH  8

extern uint16_t txMsgData[TX_MSG_DATA_LENGTH];

extern volatile uint32_t errorFlag;

__interrupt void canaISR(void);

//  Initialize the CAN interface for sending messages
void CANAInit();
void initCANAMsgs(void);


// CAN Message Float sender
void CANAsendThing( msgType *thing );

#ifdef __cplusplus
}
#endif // extern "C"

#endif
