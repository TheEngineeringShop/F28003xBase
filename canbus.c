
#include "canbus.h"

// Globals defined in the inventev_inverter_cont.c
extern uint16_t cpuTimer0IntCount; // to track the time
extern uint32_t GlobalStatusISR;


__interrupt void
canaISR(void)
{
    uint32_t status;

    DINT;
    // Read the CAN interrupt status to find the cause of the interrupt
    status = CAN_getInterruptCause(CANA_BASE);

    GlobalStatusISR=status;

    // If the cause is a controller status interrupt, then get the status
    if(status == CAN_INT_INT0ID_STATUS)
    {
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CAN_getStatus(CANA_BASE);
        //GlobalStatusISR=status;

        // Check to see if an error occurred.
        if(((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 7) &&
           ((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 0))
        {
            // Set a flag to indicate some errors may have occurred.
            //errorFlag = 1;
            errorFlag=0;  // just kidding!
        }
    }

    // Check if the cause is the transmit message object 1
    else if(status == COUNTER_TYPE)
    {
            // Getting to this point means that the TX interrupt occurred on
            // message object 1, and the message TX is complete.  Clear the
            // message object interrupt.
            CAN_clearInterruptStatus(CANA_BASE, COUNTER_TYPE);

            // Increment a counter to track how many messages have been sent.
            //txMsgCount1++;

            //GlobalStatusISR=status;

            // Since the message was sent, clear any error flags.
            errorFlag = 0;
    }

    else if(status == VDC_OUT_TYPE)
    {
            // Getting to this point means that the TX interrupt occurred on
            // message object 3, and the message TX is complete.  Clear the
            // message object interrupt.
            CAN_clearInterruptStatus(CANA_BASE, VDC_OUT_TYPE);

            // Increment a counter to keep track of how many messages have been
            // sent.
            //txMsgCount2++;

            //GlobalStatusISR=status;

            // Since the message was sent, clear any error flags.
            errorFlag = 0;
    }

    else if(status == IDC_OUT_TYPE)
    {
            //
            // Getting to this point means that the TX interrupt occurred on
            // message object 5, and the message TX is complete.  Clear the
            // message object interrupt.

            CAN_clearInterruptStatus(CANA_BASE, IDC_OUT_TYPE);

            //
            // Increment a counter to keep track of how many messages have been
            // sent.  In a real application this could be used to set flags to
            // indicate when a message is sent.
            //
            //txMsgCount3++;

            //GlobalStatusISR=status;

            //
            // Since the message was sent, clear any error flags.
            errorFlag = 0;
    }

    else if(status == DC_PWR_ACTIVE_TYPE)
    {
            //
            // Getting to this point means that the TX interrupt occurred on
            // message object 7, and the message TX is complete.  Clear the
            // message object interrupt.
            CAN_clearInterruptStatus(CANA_BASE, DC_PWR_ACTIVE_TYPE);

            //
            // Increment a counter to keep track of how many messages have been
            // sent.  In a real application this could be used to set flags to
            // indicate when a message is sent.
            //
            //txMsgCount4++;

            //GlobalStatusISR=status;

            //
            // Since the message was sent, clear any error flags.
            //
            errorFlag = 0;
    }

    else if(status == FAULT_STATUS_45_TYPE)
    {
            // Getting to this point means that the TX interrupt occurred on
            // message object 9, and the message TX is complete.  Clear the
            // message object interrupt.

            CAN_clearInterruptStatus(CANA_BASE, FAULT_STATUS_45_TYPE);

            // Increment a counter to keep track of how many messages have been
            // sent.  In a real application this could be used to set flags to
            // indicate when a message is sent.
            // txMsgCount5++;

            //GlobalStatusISR=status;

            //
            // Since the message was sent, clear any error flags.
            //
            errorFlag = 0;
    }

    else if(status == CURRENT_AND_POWER_LIMIT_TYPE)
    {
            // Getting to this point means that the TX interrupt occurred on
            // message object 11, and the message TX is complete.  Clear the
            // message object interrupt.
            CAN_clearInterruptStatus(CANA_BASE, CURRENT_AND_POWER_LIMIT_TYPE);

            // Increment a counter to keep track of how many messages have been
            // sent.  In a real application this could be used to set flags to
            // indicate when a message is sent.
            //txMsgCount6++;

            //GlobalStatusISR=status;

            // Since the message was sent, clear any error flags.
            errorFlag = 0;
    }

    else if(status == FAULT_STATUS_123_TYPE)
    {
            //
            // Getting to this point means that the TX interrupt occurred on
            // message object 13, and the message TX is complete.  Clear the
            // message object interrupt.
            CAN_clearInterruptStatus(CANA_BASE, FAULT_STATUS_123_TYPE);

            // Increment a counter to keep track of how many messages have been
            // sent.  In a real application this could be used to set flags to
            // indicate when a message is sent.
            //txMsgCount7++;

            //GlobalStatusISR=status;

            // Since the message was sent, clear any error flags.
            errorFlag = 0;
    }


    // Check if the cause is the receive message object 2
    else if(status == CCS_STATUS_TYPE)
    {
        // Get the received message
        CAN_readMessage(CANA_BASE, CCS_STATUS_TYPE, DispenserMsgs.CCSStatus);

        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        CAN_clearInterruptStatus(CANA_BASE, CCS_STATUS_TYPE);

        processCCSStatusMsg();
        //GlobalStatusISR=status;

        // Since the message was received, clear any error flags.
        errorFlag = 0;
    }

    else if(status == PRECHARGE_ENABLE_TYPE)
    {
            // Get the received message
        CAN_readMessage(CANA_BASE, PRECHARGE_ENABLE_TYPE, DispenserMsgs.PreChargeEnable);

        // Getting to this point means that the RX interrupt occurred on
        // message object 4, and the message RX is complete.  Clear the
        // message object interrupt.
        CAN_clearInterruptStatus(CANA_BASE, PRECHARGE_ENABLE_TYPE);
        DispenserMsgs.PreChargeEnableCounter++;

        GlobalStatusISR=status;

        // Since the message was received, clear any error flags
        errorFlag = 0;
    }
    else if(status == IOTECHA_LIMIT_TYPE)
    {
        // Get the received message
        CAN_readMessage(CANA_BASE, IOTECHA_LIMIT_TYPE, DispenserMsgs.Limits);

        processCurrentPowerVoltageLimitsMsg();
        // Getting to this point means that the RX interrupt occurred on
        // message object 6, and the message RX is complete.  Clear the
        // message object interrupt.
        CAN_clearInterruptStatus(CANA_BASE, IOTECHA_LIMIT_TYPE);

        GlobalStatusISR=status;

        // Since the message was received, clear any error flags.
        errorFlag = 0;
    }
    else if(status == VDC_OUT_SETPOINT_TYPE)
    {
        // Get the received message
        CAN_readMessage(CANA_BASE, VDC_OUT_SETPOINT_TYPE, DispenserMsgs.VDCOutSetpoint);

//        if(DispenserMsgs.VDCOutSetpoint[0]) iotechaTest0 = DispenserMsgs.VDCOutSetpoint[0];
//        if(DispenserMsgs.VDCOutSetpoint[1]) iotechaTest1 = DispenserMsgs.VDCOutSetpoint[1];
        processVDCOut();
        // Getting to this point means that the RX interrupt occurred on
        // message object 8 ( VDC_OUT_SETPOINT_TYPE enum ), and the message RX is complete.  Clear the
        // message object interrupt.
        CAN_clearInterruptStatus(CANA_BASE, VDC_OUT_SETPOINT_TYPE);

        GlobalStatusISR=status;

        // Since the message was received, clear any error flags.
        errorFlag = 0;
    }
    // If something unexpected caused the interrupt, this would handle it.
    else
    {
        // Spurious interrupt handling can go here.

    }

    // Clear the global interrupt flag for the CAN interrupt line
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

    // Acknowledge this interrupt located in group 9
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

void initCANAMsgs(void){

    /***********************************************************
     * Initialize the transmit message object used for sending CAN messages.
     * Message Object Parameters:
     * Message Object ID Number: 1
     * Message Identifier: 0x220 , 0544
     * Message Frame: Standard
     * Message Type: Transmit
     * Message ID Mask: 0x0
     * Message Object Flags: Transmit Interrupt
     * Message Data Length: 2 Bytes
     *
     * Description : Inventev board is expected to receive a counter message, incremented 1 for every new message sent to it.
     *
     *     Appears to be a counter ( 0xFF | 0xFF ) that simply rolls over  0-255 for each
     *     Left hands side is 34 (decimal) less than the right hand side
     *
     *  Note: Rhombus does not implement this counter, so it is probably not truly required.
     *
     ***********************************************************/
    CAN_setupMessageObject(CANA_BASE, COUNTER_TYPE, 0x220, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           COUNTER_TYPE_DATA_LENGTH);

    /***********************************************************
     * Initialize the transmit message object used for sending CAN messages.
     * Message Object Parameters:
     * Message Object ID Number: 3
     * Message Identifier: 0x224 , 0548
     * Message Frame: Standard
     * Message Type: Transmit
     * Message ID Mask: 0x0
     * Message Object Flags: Transmit Interrupt
     * Message Data Length: 2 Bytes
     *
     * Description : Inventev board provides Vdc_out , DC Voltage outbound, ( 0.1V )
     *
     ***********************************************************/
    CAN_setupMessageObject(CANA_BASE, VDC_OUT_TYPE, 0x224, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           VDC_OUT_TYPE_DATA_LENGTH);

    /***********************************************************
     * Initialize the transmit message object used for sending CAN messages.
     * Message Object Parameters:
     * Message Object ID Number: 5
     * Message Identifier: 0x225 , 549
     * Message Frame: Standard
     * Message Type: Transmit
     * Message ID Mask: 0x0
     * Message Object Flags: Transmit Interrupt
     * Message Data Length: 2 Bytes
     *
     * Description : Inventev board provides Idc_out , DC current outbound
     *
     ***********************************************************/
    CAN_setupMessageObject(CANA_BASE, IDC_OUT_TYPE, 0x225, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           IDC_OUT_TYPE_DATA_LENGTH);

    /***********************************************************
     * Initialize the transmit message object used for sending CAN messages.
     * Message Object Parameters:
     * Message Object ID Number: 7
     * Message Identifier: 0x228 , 552
     * Message Frame: Standard
     * Message Type: Transmit
     * Message ID Mask: 0x0
     * Message Object Flags: Transmit Interrupt
     * Message Data Length: 2 Bytes
     *
     * Description : Inventev board provides Pdc ( 10W ), DC Power, in 10W units every 50ms
     *
     ***********************************************************/
    CAN_setupMessageObject(CANA_BASE, DC_PWR_ACTIVE_TYPE, 0x228, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           DC_PWR_ACTIVE_TYPE_DATA_LENGTH);

    /***********************************************************
     * Initialize the transmit message object used for sending CAN messages.
     * Message Object Parameters:
     * Message Object ID Number: 9
     * Message Identifier: 0x227 , 0551
     * Message Frame: Standard
     * Message Type: Transmit
     * Message ID Mask: 0x0
     * Message Object Flags: Transmit Interrupt
     * Message Data Length: 4 Bytes
     *
     * Description : Inventev board provides FAULT_STATUS 45
     *
     ***********************************************************/
    CAN_setupMessageObject(CANA_BASE, FAULT_STATUS_45_TYPE, 0x227, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           FAULT_STATUS_45_TYPE_DATA_LENGTH);

    /***********************************************************
    * Initialize the transmit message object used for sending CAN messages.
    * Message Object Parameters:
    * Message Object ID Number: 11
    * Message Identifier: 0x229 , 0553
    * Message Frame: Standard
    * Message Type: Transmit
    * Message ID Mask: 0x0
    * Message Object Flags: Transmit Interrupt
    * Message Data Length: 4 Bytes
    *
    * Description : Inventev board provides internal current limit AND power limit every 500ms
    *
    ***********************************************************/

    CAN_setupMessageObject(CANA_BASE, CURRENT_AND_POWER_LIMIT_TYPE, 0x229, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           CURRENT_AND_POWER_LIMIT_TYPE_DATA_LENGTH);
    /***********************************************************
     * Initialize the transmit message object used for sending CAN messages.
     * Message Object Parameters:
     * Message Object ID Number: 13
     * Message Identifier: 0x222 , 0546
     * Message Frame: Standard
     * Message Type: Transmit
     * Message ID Mask: 0x0
     * Message Object Flags: Transmit Interrupt
     * Message Data Length: 8 Bytes
     *
     * Description : Inventev board provides FAULT_STATUS 123
     *
     ***********************************************************/

    CAN_setupMessageObject(CANA_BASE, FAULT_STATUS_123_TYPE, 0x222, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           FAULT_STATUS_123_TYPE_DATA_LENGTH);

    /********************************************************
     * Initialize the receive message object used for receiving CAN messages.
     * Message Object Parameters:
     * Message Object ID Number: 2
     * Message Identifier: 0x362 , 0866
     * Message Frame: Standard
     * Message Type: ReceiveGlobalStatusISR=status;
     * Message ID Mask: 0x0
     * Message Object Flags: Receive Interrupt
     * Message Data Length: 4 Bytes (Note that DLC field is a "don't care"
     *     for a Receive mailbox )
     *
     * Description : IoTecha board supplies CCS_STATUS
     *
     ***********************************************************/
    CAN_setupMessageObject(CANA_BASE, CCS_STATUS_TYPE, 0x362, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                           CCS_STATUS_TYPE_DATA_LENGTH);

    /********************************************************
     * Initialize the receive message object used for receiving CAN messages.
     * Message Object Parameters:
     * Message Object ID Number: 4
     * Message Identifier: 0x360 , 0864
     * Message Frame: Standard
     * Message Type: Receive
     * Message ID Mask: 0x0
     * Message Object Flags: Receive Interrupt
     * Message Data Length: 1 Bytes (Note that DLC field is a "don't care"
     *   for a Receive mailbox
     *
     *  Description : IoTecha board supplies PRECHARGE_ENABLE flags
     *     5 = Reset Faults
     *     4 = Start charging
     *     0 = stop or disable
     *
     ***********************************************************/
    CAN_setupMessageObject(CANA_BASE, PRECHARGE_ENABLE_TYPE, 0x360, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                           PRECHARGE_ENABLE_TYPE_DATA_LENGTH);

     /********************************************************
      * Initialize the receive message object used for receiving CAN messages.
      * Message Object Parameters:
      * Message Object ID Number: 6
      * Message Identifier: 0x33F , 0831
      * Message Frame: Standard
      * Message Type: ReceiveGlobalStatusISR=status;
      * Message ID Mask: 0x0
      * Message Object Flags: Receive Interrupt
      * Message Data Length: 6 Bytes (Note that DLC field is a "don't care"
      *     for a Receive mailbox
      *
      *  Description : IoTecha board supplies binary packed ( FF FF | FF FF | FF FF )
      *  1. Vdc_max_limit - 0x00 - 0xFFFF  ( 65535 )
      *  2. Pdc_limit  - 0x00 - 0xFFFF ( 65535 )
      *  3. Iout_limit - 0x00 - 0xFFFF ( 65535 )
      *
      *********************************************************/
    CAN_setupMessageObject(CANA_BASE, IOTECHA_LIMIT_TYPE, 0x33F, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                           IOTECHA_LIMIT_TYPE_DATA_LENGTH);
    /********************************************************
     * Initialize the receive message object used for receiving CAN messages.
     *  Message Object Parameters:
     *  Message Object ID Number: 8
     *  Message Identifier: 0x32F  , 0815
     *  Message Frame: Standard
     *  Message Type: Receive
     *  Message ID Mask: 0x0
     *  Message Object Flags: Receive Interrupt
     *  Message Data Length: 2 Bytes (Note that DLC field is a "don't care"
     *    for a Receive mailbox
     *
     *  Description : Vdc_out_setpoint  ( 0.1V ), this is the DC voltage the IoTecha board
     *    is asking us to match for precharge
    ********************************************************/
    CAN_setupMessageObject(CANA_BASE, VDC_OUT_SETPOINT_TYPE, 0x32F, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                           VDC_OUT_SETPOINT_TYPE_DATA_LENGTH);

    // TX Objects tied to data structures you define
    /*
    CAN_setupMessageObject(CANA_BASE, funData.val1.canTxObjId, funData.val1.canId,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_TX_INT_ENABLE, TX_MSG_DATA_LENGTH);

    CAN_setupMessageObject(CANA_BASE, funData.val2.canTxObjId, funData.val2.canId,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_TX_INT_ENABLE, TX_MSG_DATA_LENGTH);

    CAN_setupMessageObject(CANA_BASE, funData.val3.canTxObjId, funData.val3.canId,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_TX_INT_ENABLE, TX_MSG_DATA_LENGTH);

    CAN_setupMessageObject(CANA_BASE, funData.val4.canTxObjId, funData.val4.canId,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_TX_INT_ENABLE, TX_MSG_DATA_LENGTH);

    CAN_setupMessageObject(CANA_BASE, funData.val5.canTxObjId, funData.val5.canId,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_TX_INT_ENABLE, TX_MSG_DATA_LENGTH);

    CAN_setupMessageObject(CANA_BASE, funData.val6.canTxObjId, funData.val6.canId,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_TX_INT_ENABLE, TX_MSG_DATA_LENGTH);
    */
}

// Initialize the CANA peripheral
void CANAInit()
{
    CANAmsgCount = 0;  // global counter

    // Initialize the CAN controllers
    CAN_initModule(CANA_BASE);

    // Set up the CAN bus bit rate to 125kHz for each module
    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 125000, 20);

    // Enable interrupts on the CAN peripheral.
    CAN_enableInterrupt(CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);

    // Enable interrupts on the CAN A peripheral.
    CAN_enableInterrupt(CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR |
                        CAN_INT_STATUS);

    // register all the objects.
    initCANAMsgs();

    // Setup Interrupts
    Interrupt_register(INT_CANA0, &canaISR);
    CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    // Start CAN module operations
    CAN_startModule(CANA_BASE);

    // Enable the CANA interrupt signal
    Interrupt_enable(INT_CANA0);

}

/*************
 *  Works fine, just saving space.
 *
 * CANA Message Float sender.
 ********************/
/*
void CANAsendThing( msgType *thing )
{

    float fVal1 = thing->value;
    // stupidly convert to string and just send this to the other side.
    // optimization to properly pack messages and serialize / de-serialize is left for a future optimization
    char str[6];

    //sprintf ( str, "%f", fVal1);  // 6 places, a string -- float conversion fails on the 0039C, broken TI code. runs forever!!
    // have to roll our own here ...
    char *tmpSign = (fVal1 < 0) ? "-" : "";
    float tmpVal = abs(fVal1);

    int tmpInt1 = fVal1;                  // Get integer
    float tmpFrac = fVal1 - tmpInt1;      // Get fraction
    int tmpInt2 = trunc(tmpFrac * 10000);  // convert fraction to integer.
    // Print as parts, note that you need 0-padding for fractional bit.

    sprintf (str, "%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);

    uint16_t i;
    for ( i =0 ; i < 6 ; i++ ) // magic number is the precision for the float in sprintf
    {
        txMsgData[i] = (uint16_t)str[i];
    }

    // Transmit the message.
    CAN_sendMessage(CANA_BASE, thing->canTxObjId, TX_MSG_DATA_LENGTH,
                    txMsgData);
    thing->timestamp = cpuTimer0IntCount;
}
*/
