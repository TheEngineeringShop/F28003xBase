#include "mcanbus.h"

// Global Variables.
volatile uint32_t isrIntr0Flag;     // used for
volatile uint32_t isrIntr1Flag;     // user for into/out-of the interrupt handler
volatile uint16_t monEn;   // MCAN Bus monitoring disabled = 0

// Counters
MCANInterruptCounters AllIntr0MCANInterruptCounters;
MCANInterruptCounters AllIntr1MCANInterruptCounters;

// MCAN_RxBufElement rxMsg1;  // received message, for passing elsewhere.   //MCAN_ELEM_SIZE_8BYTES


// Standard ID is read incorrectly for CAN using the TI toolkit, we have to fix it.
static void fixCANID( MCAN_RxBufElement *elem )
{
//    uint32_t startId = elem->id;
//    uint32_t maskedId = (uint32_t) ((elem->id & MCAN_CUSTOM_CANID_BITMASK));
//    uint32_t shiftedId = maskedId >> MCAN_CUSTOM_CANID_SHIFT;

    elem->id = (uint32_t) ((elem->id & MCAN_CUSTOM_CANID_BITMASK)  >> MCAN_CUSTOM_CANID_SHIFT);    // note : max is 0x7FF

}


// This is Interrupt Service Routine for MCAN interrupt 1.
// This is tested.
__interrupt void MCANIntr1ISR(void)
{
    volatile uint32_t intrStatus;   // if not set as volatile, it can be set to 0 accidentally by HW_WR_FIELD32() below.
//    MCAN_RxNewDataStatus newData;

    intrStatus = MCAN_getIntrStatus(MCANA_DRIVER_BASE);
//    uint32_t interruptType = ( intrStatus - MCANA_DRIVER_BASE );
    HW_WR_FIELD32(MCANA_DRIVER_BASE + MCAN_MCANSS_EOI, MCAN_MCANSS_EOI, 0x2U);
    // Clear the interrupt Status.
    MCAN_clearIntrStatus(MCANA_DRIVER_BASE, intrStatus);

    // Update the interrupt flag value, used in the sending caller to let it know we were run.
    isrIntr1Flag = 0U;

    // Check if there is a message in the dedicated RX buffer.
    if((MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG & intrStatus) == MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG)
    {
         // Placeholder for generic receive message handling

    }
    // check if there this interrupt was triggered by a message in RX FIFO 1
    if((    MCAN_INTR_SRC_RX_FIFO1_NEW_MSG  & intrStatus) == MCAN_INTR_SRC_RX_FIFO1_NEW_MSG)
    {
        MCAN_RxBufElement rxMsg1;  // handling receive message
        MCAN_RxFIFOStatus RxFS;

        RxFS.num = MCAN_RX_FIFO_NUM_1; // We can see the FIFO status!
        MCAN_getRxFIFOStatus(MCANA_DRIVER_BASE, &RxFS);  // check for fillLvl

        rxMsg1.id = 0; // 0 it before handling.
        MCAN_readMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_FIFO, 0U, MCAN_RX_FIFO_NUM_1, &rxMsg1);  // note that "0U" here is ignored.
        fixCANID( &rxMsg1 );

//      Use only if message is received in buffer element 0, and NOT the FIFO.
//        if((newData.statusLow & (1UL << 0U)) != 0)  {
//            MCAN_readMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0U, 0, &rxMsg1);
//            MCAN_clearNewDataStatus(MCANA_DRIVER_BASE, &newData);  //  Clearing the NewData registers
        processMCANMsg(&rxMsg1);

        MCAN_writeRxFIFOAck(MCANA_DRIVER_BASE, MCAN_RX_FIFO_NUM_1, RxFS.getIdx);  // clear FIFO1 up to the fill level
    }
    if (( MCAN_INTR_SRC_TRANS_COMPLETE & intrStatus ) == MCAN_INTR_SRC_TRANS_COMPLETE )
    {
         // Transmit message handling here.
    }

    //  Count the interrupt types
    switch ( intrStatus )
    {
        // Rx FIFO 0 New Message interrupt
        case MCAN_INTR_SRC_RX_FIFO0_NEW_MSG:
            AllIntr1MCANInterruptCounters.RXFIFO0NewMsg++;
            break;
       // Rx FIFO 0 Watermark Reached interrupt
        case MCAN_INTR_SRC_RX_FIFO0_WATERMARK:
            AllIntr1MCANInterruptCounters.RXFIFO0Watermark++;
            break;

        // Rx FIFO 0 Full interrupt
        case MCAN_INTR_SRC_RX_FIFO0_FULL:
            AllIntr1MCANInterruptCounters.RXFIFO0Full++;
            break;
        // Rx FIFO 0 Message Lost interrupt
        case MCAN_INTR_SRC_RX_FIFO0_MSG_LOST:
            AllIntr1MCANInterruptCounters.RXFIFO0MsgLost++;
            break;

        // Rx FIFO 1 New Message interrupt         
        case MCAN_INTR_SRC_RX_FIFO1_NEW_MSG:
            AllIntr1MCANInterruptCounters.RXFIFO1NewMsg++;
            break;

        // Rx FIFO 1 Watermark Reached interrupt
        case MCAN_INTR_SRC_RX_FIFO1_WATERMARK:
            AllIntr1MCANInterruptCounters.RXFIFO1Watermark++;
            break;
        
        // Rx FIFO 1 Full interrupt
        case MCAN_INTR_SRC_RX_FIFO1_FULL:
            AllIntr1MCANInterruptCounters.RXFIFO1Full++;
            break;

        // Rx FIFO 1 Message Lost interrupt
        case MCAN_INTR_SRC_RX_FIFO1_MSG_LOST:
            AllIntr1MCANInterruptCounters.RXFIFO1MsgLost++;
            break;
            
        // High Priority Message interrupt
        case MCAN_INTR_SRC_HIGH_PRIO_MSG:
            AllIntr1MCANInterruptCounters.HighPriority++;
            break;

        // Transmission Completed interrupt
        case MCAN_INTR_SRC_TRANS_COMPLETE:
            AllIntr1MCANInterruptCounters.TransmissionComplete++;
            break;

        // Transmission Cancellation Finished interrupt
        case MCAN_INTR_SRC_TRANS_CANCEL_FINISH:
            AllIntr1MCANInterruptCounters.TransmissionCancellationFinished++;
            break;

        // Tx FIFO Empty interrupt
        case MCAN_INTR_SRC_TX_FIFO_EMPTY:
            AllIntr1MCANInterruptCounters.TXFIFOEmpty++;
            break;

        // Tx Event FIFO New Entry interrupt
        case MCAN_INTR_SRC_TX_EVT_FIFO_NEW_ENTRY:
            AllIntr1MCANInterruptCounters.TXFIFONewEntry++;
            break;

        // Tx Event FIFO Watermark Reached interrupt
        case MCAN_INTR_SRC_TX_EVT_FIFO_WATERMARK:
            AllIntr1MCANInterruptCounters.TXFIFOWatermark++;
            break;

        // Tx Event FIFO Full interrupt
        case MCAN_INTR_SRC_TX_EVT_FIFO_FULL:
            AllIntr1MCANInterruptCounters.TXFIFOFull++;
            break;

        // Tx Event FIFO Element Lost interrupt
        case MCAN_INTR_SRC_TX_EVT_FIFO_ELEM_LOST:
            AllIntr1MCANInterruptCounters.TXFIFOElementLost++;
            break;

        // Timestamp Wraparound interrupt
        case MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND:
            AllIntr1MCANInterruptCounters.TimestampWraparound++;
            break;

        // Message RAM Access Failure interrupt
        case MCAN_INTR_SRC_MSG_RAM_ACCESS_FAILURE :
            AllIntr1MCANInterruptCounters.MsgRamAccessFailure++;
            break;

        // Timeout Occurred interrupt
        case MCAN_INTR_SRC_TIMEOUT:
            AllIntr1MCANInterruptCounters.Timeout++;
            break;

        // Message stored to Dedicated Rx Buffer interrupt
        case MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG:
            AllIntr1MCANInterruptCounters.DedicatedRXBufferMsg++;
            break;

        // Bit Error Corrected interrupt
        case MCAN_INTR_SRC_BIT_ERR_CORRECTED:
            AllIntr1MCANInterruptCounters.BitErrorCorrected++;
            break;

        // Bit Error Uncorrected interrupt
        case MCAN_INTR_SRC_BIT_ERR_UNCORRECTED:
            AllIntr1MCANInterruptCounters.BitErrorUncorrected++;
            break;

        // Error Logging Overflow interrupt
        case MCAN_INTR_SRC_ERR_LOG_OVRFLW :
            AllIntr1MCANInterruptCounters.ErrorLogOverflow++;
            break;

        // Error Passive interrupt
        case MCAN_INTR_SRC_ERR_PASSIVE:
            AllIntr1MCANInterruptCounters.ErrorPassive++;
            break;

        // Warning Status interrupt
        case MCAN_INTR_SRC_WARNING_STATUS:
            AllIntr1MCANInterruptCounters.WarningStatus++;
            break;

        // Bus_Off Status interrupt
        case MCAN_INTR_SRC_BUS_OFF_STATUS:
            AllIntr1MCANInterruptCounters.BusOffStatus++;
            break;

        // Watchdog Interrupt interrupt
        case MCAN_INTR_SRC_WATCHDOG:
            AllIntr1MCANInterruptCounters.Watchdog++;
            break;

        // Protocol Error in Arbitration Phase interrupt
        case MCAN_INTR_SRC_PROTOCOL_ERR_ARB:
            AllIntr1MCANInterruptCounters.ErrorArbitrationPhase++;
            break;

        // Protocol Error in Data Phase interrupt
        case MCAN_INTR_SRC_PROTOCOL_ERR_DATA:
            AllIntr1MCANInterruptCounters.ProtocolErrorInDataPhase++;
            break;

        // Access to Reserved Address interrupt */        
        case MCAN_INTR_SRC_RES_ADDR_ACCESS:
            AllIntr1MCANInterruptCounters.AccesstoReserverAddress++;
            break;

        default:
            AllIntr1MCANInterruptCounters.Unknown++;
    }

    // Acknowledge this interrupt located in group 9
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


// This is Interrupt Service Routine for MCAN interrupt 0.
// Receiving interrupt, rips the flag RF1N
// RX messages currently set to be received to Intr1, this codepath should be dead.
// This is untested.
__interrupt void MCANIntr0ISR(void)
{
    uint32_t intrStatus;
    //MCAN_RxNewDataStatus newData;
    MCAN_RxBufElement rxMsg1;

    intrStatus = MCAN_getIntrStatus(MCANA_DRIVER_BASE);

    //  Clearing the interrupt lineNum
    HW_WR_FIELD32(MCANA_DRIVER_BASE + MCAN_MCANSS_EOI, MCAN_MCANSS_EOI, 0x2U);

    // Clear the interrupt Status.
    MCAN_clearIntrStatus(MCANA_DRIVER_BASE, intrStatus);

    // Update the flag value.
    isrIntr0Flag = 0U;

    //  Check to see if the interrupt is caused by a message being
    //  received in dedicated RX Buffers
    if((MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG & intrStatus) == MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG)
    {
        MCAN_readMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0U,
                          1, &rxMsg1);

    }
    else
    {
       //  Interrupt handling for other interrupt sources goes here
    }

    //  Count the interrupt types
    switch ( intrStatus )
    {
        // Rx FIFO 0 New Message interrupt
        case MCAN_INTR_SRC_RX_FIFO0_NEW_MSG:
            AllIntr0MCANInterruptCounters.RXFIFO0NewMsg++;
            break;
       // Rx FIFO 0 Watermark Reached interrupt
        case MCAN_INTR_SRC_RX_FIFO0_WATERMARK:
            AllIntr0MCANInterruptCounters.RXFIFO0Watermark++;
            break;

        // Rx FIFO 0 Full interrupt
        case MCAN_INTR_SRC_RX_FIFO0_FULL:
            AllIntr0MCANInterruptCounters.RXFIFO0Full++;
            break;
        // Rx FIFO 0 Message Lost interrupt
        case MCAN_INTR_SRC_RX_FIFO0_MSG_LOST:
            AllIntr0MCANInterruptCounters.RXFIFO0MsgLost++;
            break;

        // Rx FIFO 1 New Message interrupt         
        case MCAN_INTR_SRC_RX_FIFO1_NEW_MSG:
            AllIntr0MCANInterruptCounters.RXFIFO1NewMsg++;
            break;

        // Rx FIFO 1 Watermark Reached interrupt
        case MCAN_INTR_SRC_RX_FIFO1_WATERMARK:
            AllIntr0MCANInterruptCounters.RXFIFO1Watermark++;
            break;
        
        // Rx FIFO 1 Full interrupt
        case MCAN_INTR_SRC_RX_FIFO1_FULL:
            AllIntr0MCANInterruptCounters.RXFIFO1Full++;
            break;

        // Rx FIFO 1 Message Lost interrupt
        case MCAN_INTR_SRC_RX_FIFO1_MSG_LOST:
            AllIntr0MCANInterruptCounters.RXFIFO1MsgLost++;
            break;
            
        // High Priority Message interrupt
        case MCAN_INTR_SRC_HIGH_PRIO_MSG:
            AllIntr0MCANInterruptCounters.HighPriority++;
            break;

        // Transmission Completed interrupt
        case MCAN_INTR_SRC_TRANS_COMPLETE:
            AllIntr0MCANInterruptCounters.TransmissionComplete++;
            break;

        // Transmission Cancellation Finished interrupt
        case MCAN_INTR_SRC_TRANS_CANCEL_FINISH:
            AllIntr0MCANInterruptCounters.TransmissionCancellationFinished++;
            break;

        // Tx FIFO Empty interrupt
        case MCAN_INTR_SRC_TX_FIFO_EMPTY:
            AllIntr0MCANInterruptCounters.TXFIFOEmpty++;
            break;

        // Tx Event FIFO New Entry interrupt
        case MCAN_INTR_SRC_TX_EVT_FIFO_NEW_ENTRY:
            AllIntr0MCANInterruptCounters.TXFIFONewEntry++;
            break;

        // Tx Event FIFO Watermark Reached interrupt
        case MCAN_INTR_SRC_TX_EVT_FIFO_WATERMARK:
            AllIntr0MCANInterruptCounters.TXFIFOWatermark++;
            break;

        // Tx Event FIFO Full interrupt
        case MCAN_INTR_SRC_TX_EVT_FIFO_FULL:
            AllIntr0MCANInterruptCounters.TXFIFOFull++;
            break;

        // Tx Event FIFO Element Lost interrupt
        case MCAN_INTR_SRC_TX_EVT_FIFO_ELEM_LOST:
            AllIntr0MCANInterruptCounters.TXFIFOElementLost++;
            break;

        // Timestamp Wraparound interrupt
        case MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND:
            AllIntr0MCANInterruptCounters.TimestampWraparound++;
            break;

        // Message RAM Access Failure interrupt
        case MCAN_INTR_SRC_MSG_RAM_ACCESS_FAILURE :
            AllIntr0MCANInterruptCounters.MsgRamAccessFailure++;
            break;

        // Timeout Occurred interrupt
        case MCAN_INTR_SRC_TIMEOUT:
            AllIntr0MCANInterruptCounters.Timeout++;
            break;

        // Message stored to Dedicated Rx Buffer interrupt
        case MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG:
            AllIntr0MCANInterruptCounters.DedicatedRXBufferMsg++;
            break;

        // Bit Error Corrected interrupt
        case MCAN_INTR_SRC_BIT_ERR_CORRECTED:
            AllIntr0MCANInterruptCounters.BitErrorCorrected++;
            break;

        // Bit Error Uncorrected interrupt
        case MCAN_INTR_SRC_BIT_ERR_UNCORRECTED:
            AllIntr0MCANInterruptCounters.BitErrorUncorrected++;
            break;

        // Error Logging Overflow interrupt
        case MCAN_INTR_SRC_ERR_LOG_OVRFLW :
            AllIntr0MCANInterruptCounters.ErrorLogOverflow++;
            break;

        // Error Passive interrupt
        case MCAN_INTR_SRC_ERR_PASSIVE:
            AllIntr0MCANInterruptCounters.ErrorPassive++;
            break;

        // Warning Status interrupt
        case MCAN_INTR_SRC_WARNING_STATUS:
            AllIntr0MCANInterruptCounters.WarningStatus++;
            break;

        // Bus_Off Status interrupt
        case MCAN_INTR_SRC_BUS_OFF_STATUS:
            AllIntr0MCANInterruptCounters.BusOffStatus++;
            break;

        // Watchdog Interrupt interrupt
        case MCAN_INTR_SRC_WATCHDOG:
            AllIntr0MCANInterruptCounters.Watchdog++;
            break;

        // Protocol Error in Arbitration Phase interrupt
        case MCAN_INTR_SRC_PROTOCOL_ERR_ARB:
            AllIntr0MCANInterruptCounters.ErrorArbitrationPhase++;
            break;

        // Protocol Error in Data Phase interrupt
        case MCAN_INTR_SRC_PROTOCOL_ERR_DATA:
            AllIntr0MCANInterruptCounters.ProtocolErrorInDataPhase++;
            break;

        // Access to Reserved Address interrupt
        case MCAN_INTR_SRC_RES_ADDR_ACCESS:
            AllIntr0MCANInterruptCounters.AccesstoReserverAddress++;
            break;

        default:
            AllIntr0MCANInterruptCounters.Unknown++;
    }

    // Acknowledge this interrupt located in group 9
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


/***********************************************************************
 *  Initialize the MCAN peripheral
 * refer to document in repository: ti_c2000_DCAN_to_MCAN_migration_guide_sprad59.pdf
 * 3.2 MCAN Initialization
 *   1. Configure Message RAM (See Section 5).
 *   2. Configure CAN mode (Classic CAN or CAN FD).
 *   3. Configure bit-timing (See Section 4).
 *   4. Configure bit-rate switching (enable or disable).
 *   5. Configure Filter Elements (optional → can also be done after initialization and while module is operational).
 *
 *  Note that the status registers related to Tx/Rx are reset on switching to init mode in MCAN.
 *
 * 3.3 Initialization sequence
 *   The individual steps for initialization of DCAN and MCAN modules, along with the key differences have been
 *   listed in Table 3-1:

 * Table 3-1. DCAN/MCAN Initialization Sequence
 * Operation MCAN
 * 1. Enter Initialization Mode
 *     Set MCAN_CCCR.INIT bit and check that the bit has been set ( MCAN_setOpMode )
 * 2. Unlock Protected Registers
 *     Set MCAN_CCCR.CCE bit
 * 3. Configure CAN Mode and Bit Rate Switching
 *     Set MCAN_CCCR.FDOE bit for CAN FD function
 *     Set MCAN_CCCR.BRSE bit to enable Bit Rate Switching (BRS)
 *   (Both bits need to be 0 for Classic CAN Communication)
 * 4. Configure bit-timing
 *    Configure MCAN_NBTP register
 * 5. Configure data bit-timing
 *    Configure MCAN_DBTP register (Not needed for Classic CAN as BRS is disabled)
 * 6. Message RAM Configuration
 *    See Message RAM configuration
 * 7. Global Filter Configuration, if required (determines how the module handles nonmatching frames).
 *    Set MCAN_GFC Register
 * 8. Receive and Transmit Configuration (can be done at run time as well)
 *    Filter Configuration
 * 9. Lock Protected Registers
 *    Clear MCAN_CCCR.CCE bit
 * 10. Return module to normal operation
 *   Clear MCAN_CCCR.INIT bit  ( MCAN_setOpMode )
 *
* In addition to the steps shown above, for MCAN, the MCAN Clock Divider may need to be set up as part of the
* initialization process. This configuration is typically done via the AUXCLKDIVSEL register (refer to the
* device specific TRM to determine the register for clock division). For 120 MHz and 200 MHz devices, C2000ware
* examples configure the MCAN bit-clock to 40 MHz. If an application desires a smaller time quanta (TQ), other
* configurations for the bit-clock are possible.
 ***********************************************************************/
void MCANInit()
{
    volatile uint32_t mode = 0U;

    // Set the globals for use
    isrIntr0Flag = 1U;
    isrIntr1Flag = 1U;
    monEn = 0x0;

    // Set all interrupt counters to 0;
 //   memset(&AllIntr0MCANInterruptCounters, 0, sizeof(AllIntr0MCANInterruptCounters));
    memset(&AllIntr1MCANInterruptCounters, 0, sizeof(AllIntr1MCANInterruptCounters));

    // Configure GPIO pins for MCANTX/MCANRX operation
    //GPIO_togglePin(GPIO_56_GPIO56);  // for testing
    GPIO_setPinConfig(GPIO_56_MCAN_TX);
    GPIO_setPinConfig(GPIO_57_MCAN_RX);

    #ifdef F2838x
    // Allocate shared peripheral to C28X ( applicable only for 320F2838X )
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_MCAN_A, 0x0U );

    // Configure the divisor for MCAN bit-clock
    SysCtl_setMCANClk(SYSCTL_MCANCLK_DIV_5);
    #else
    // Configure the divisor for the MCAN bit-clock
    SysCtl_setMCANClk(SYSCTL_MCANCLK_DIV_3);
    #endif

    // CrossBar and ISR Configuration.
    MCANIntrConfig();

    // Configure the MCAN Module.
    MCANConfig();

    // Enable Interrupts.
    MCAN_enableIntr(MCANA_DRIVER_BASE, MCAN_INTR_MASK_ALL, 1U);

    // Strangely cannot enable line 0
    // Select Interrupt Line, and enable it
    //MCAN_selectIntrLine(MCANA_DRIVER_BASE, MCAN_INTR_MASK_ALL, MCAN_INTR_LINE_NUM_0);
    //MCAN_enableIntrLine(MCANA_DRIVER_BASE, MCAN_INTR_LINE_NUM_0, 1U);

    // Select Interrupt Line, and enable it
    MCAN_selectIntrLine(MCANA_DRIVER_BASE, MCAN_INTR_MASK_ALL, MCAN_INTR_LINE_NUM_1);
    MCAN_enableIntrLine(MCANA_DRIVER_BASE, MCAN_INTR_LINE_NUM_1, 1U);

    // Enable Transmission interrupt.
    // Needs to be  done individually for each Tx Buffer Element
    // Additionally, transmission related interrupt sources will need to be enabled first
    MCAN_txBufTransIntrEnable(MCANA_DRIVER_BASE, 1U,1U);

}


/**********************************************************
 * MCAN Interrupt Handling
 * This function will configure X-BAR for MCAN interrupts.
 * Device-level Interrupt Configurations:
 *     1. Initialize PIE and PIE Vector Table. Enable Global and Real-time Interrupts.
 *     2. Configure the interrupt handler in the PIE Vector Table. Enable interrupt in the interrupt controller.
 *
 * Module-level Interrupt Configurations
 *     1. Enable interrupt sources using the register (MCAN_IR), where each bit corresponds to a single interrupt
 *         source. Enable interrupt lines as required using the register (MCAN_ILE).
 *     2. Select interrupt lines where interrupt source is to be routed using the register (MCAN_ILS), where each bit
 *         corresponds to a single interrupt source.
 *     3. Interrupt Service Routine (ISR) : Read Interrupt Register (MCAN_IR) to determine the source of the interrupt
 *         (any of the 30 individual interrupt sources). Clear the interrupt by writing to the same register. Clear the
 *         interrupt line by writing to the register (MCANSS_EOI).
 *     4. Acknowledge the interrupt via PIEACK.
 *
 **********************************************************/
static void MCANIntrConfig(void)
{
    // Can't seem to enable interrupt line 0
    //    Interrupt_register(INT_MCANA_0,&MCANIntr0ISR);
    //    Interrupt_enable(INT_MCANA_0);

    // Handles all MCAN messages TX and RX
    Interrupt_register(INT_MCANA_1,&MCANIntr1ISR);
    Interrupt_enable(INT_MCANA_1);

    // turn on interrupts from main init(), with Interrupt_enableGlobal();
}


static void MCANConfig(void)
{
    MCAN_RevisionId revId;
    MCAN_InitParams initParams;
    MCAN_ConfigParams configParams;
    MCAN_MsgRAMConfigParams    msgRAMConfigParams;
    MCAN_StdMsgIDFilterElement stdFiltelem;  // required when e need to filter elements
    MCAN_BitTimingParams       bitTimes;

    //  Initializing all structs to zero to prevent stray values
    memset(&initParams, 0, sizeof(initParams));
    memset(&configParams, 0, sizeof(configParams));
    memset(&msgRAMConfigParams, 0, sizeof(msgRAMConfigParams));
    memset(&stdFiltelem, 0, sizeof(stdFiltelem));
    memset(&bitTimes, 0, sizeof(bitTimes));

    // Initialize MCAN Init parameters.
    initParams.fdMode            = 0x0U;  // FD operation enabled(1), disabled (0)
    initParams.brsEnable         = 0x0U;  // Bit rate switching for
                                          // transmissions enabled (1) , disabled (0)
    initParams.txpEnable         = 0x0U;  // Transmit pause disabled.
    initParams.efbi              = 0x0U;  // Edge filtering disabled.
    initParams.pxhddisable       = 0x0U;  // Protocol exception handling enabled
    initParams.darEnable         = 0x0U;  // Disable Automatic retransmission of
                                          // messages not transmitted successfully
    initParams.wkupReqEnable     = 0x1U;  // Wakeup request is enabled.
    initParams.autoWkupEnable    = 0x1U;  // Auto-Wakeup is enabled.
    initParams.emulationEnable   = 0x1U;  // Emulation/Debug Suspend is enabled.
    initParams.tdcEnable         = 0x1U;  // Transmitter Delay Compensation is enabled.
    initParams.wdcPreload        = 0xFFU; // Start value of the Message RAM Watchdog Counter preload.

    // Transmitter Delay Compensation parameters.
    initParams.tdcConfig.tdcf    = 0xAU;
    initParams.tdcConfig.tdco    = 0x6U;

    // Initialize MCAN Config parameters.
    configParams.monEnable         = monEn; // Bus Monitoring Mode is disabled
    configParams.asmEnable         = 0x0U;  // Normal CAN operation.
    configParams.tsPrescalar       = 0xFU;  // Prescaler Value.
    configParams.tsSelect          = 0x0U;  // Timestamp counter value.
    configParams.timeoutSelect     = MCAN_TIMEOUT_SELECT_CONT;     // Timeout counter source select.
    configParams.timeoutPreload    = 0xFFFFU; // Start value of the Timeout Counter.
    configParams.timeoutCntEnable  = 0x0U; // Timeout Counter is disabled.
    // Configure a filter
    configParams.filterConfig.rrfs = 0x1U; // Reject all remote frames with 29-bit extended IDs.
    configParams.filterConfig.rrfe = 0x1U; // Reject all remote frames with 11-bit standard IDs.
    configParams.filterConfig.anfe = 0x1U; // Accept in Rx FIFO 1.
    configParams.filterConfig.anfs = 0x1U; // Accept in Rx FIFO 1.

    /*****************************************
    * Message RAM Configuration Section Here
    *
    * In MCAN, Driverlib APIs can be used to carry out the read or write operations with the Message RAM directly.
    * The Message RAM is structured differently in MCAN as compared to DCAN. In DCAN, the number of message
    * objects in the Message RAM is fixed at 32 and each message object can be configured for either transmit or
    * receive operation.
    *
    * However, in MCAN, Message RAM can be configured to have the following sections:
    * 1. Standard Filter Element
    * 2. Extended Filter Element
    * 3. Rx Buffer
    * 4. Rx FIFO
    * 5. Tx Buffer
    * 6. Tx FIFO or Tx Queue
    * 7. Tx Event FIFO
    *
    * The design of MCAN Message RAM offers tremendous flexibility, enabling allocation of the available memory
    * to each of the sections mentioned above based on the application needs. The sections can be ordered in any
    * manner and the unused sections can be allocated zero memory. Note that Message RAM size can vary from
    * one device to another. Refer the device-specific data sheet for more information.
    **********************************************************************************************/

    // Initialize Message RAM Sections Configuration Parameters
    // Standard ID Filter List Start Address
    msgRAMConfigParams.flssa                = MCAN_STD_ID_FILT_START_ADDR;
    // List Size: Standard ID
    msgRAMConfigParams.lss                  = MCAN_STD_ID_FILTER_NUM;

    // Extended ID Filter List Start Address
    msgRAMConfigParams.flesa                = MCAN_EXT_ID_FILT_START_ADDR;

    // List Size: Extended ID
    msgRAMConfigParams.lse                  = MCAN_EXT_ID_FILTER_NUM;

    // Tx Buffers Start Address
    msgRAMConfigParams.txStartAddr          = MCAN_TX_BUFF_START_ADDR;

    // Number of Dedicated Transmit Buffers
    msgRAMConfigParams.txBufNum             = MCAN_TX_BUFF_SIZE;

    msgRAMConfigParams.txFIFOSize           = MCAN_TX_FQ_SIZE; // no message buffer for Tx FIFO/Queue.
    msgRAMConfigParams.txBufMode            = 0U; //Tx FIFO operation
    msgRAMConfigParams.txBufElemSize        = MCAN_TX_BUFF_ELEM_SIZE;  // Tx Buffer Element Size.
    msgRAMConfigParams.txEventFIFOStartAddr = MCAN_TX_EVENT_START_ADDR; // Tx Event FIFO Start Address.
    msgRAMConfigParams.txEventFIFOSize      = MCAN_TX_BUFF_SIZE; // Event FIFO Size.
    msgRAMConfigParams.txEventFIFOWaterMark = 3U;     // Level for Tx Event FIFO watermark interrupt.

    //-----------------RX-----------------------------
    // FIFO 0
    msgRAMConfigParams.rxFIFO0startAddr     = MCAN_FIFO_0_START_ADDR;  // Rx FIFO0 Start Address.
    msgRAMConfigParams.rxFIFO0size          = MCAN_FIFO_0_NUM;         // Number of Rx FIFO elements.
    msgRAMConfigParams.rxFIFO0waterMark     = 3U;   // Rx FIFO0 Watermark.
    msgRAMConfigParams.rxFIFO0OpMode        = 1U;   // FIFO overwrite mode.
    msgRAMConfigParams.rxFIFO0ElemSize      = MCAN_FIFO_0_ELEM_SIZE;    // Rx FIFO0 Element Size.

    // FIFO 1
    msgRAMConfigParams.rxFIFO1startAddr     = MCAN_FIFO_1_START_ADDR;  // Rx FIFO1 Start Address.
    msgRAMConfigParams.rxFIFO1size          = MCAN_FIFO_1_NUM;        // Number of Rx FIFO elements.
    msgRAMConfigParams.rxFIFO1waterMark     = 3U; // Level for Rx FIFO 1  watermark interrupt.
    msgRAMConfigParams.rxFIFO1OpMode        = 1U; // FIFO overwrite mode.
    msgRAMConfigParams.rxFIFO1ElemSize      = MCAN_FIFO_1_ELEM_SIZE;    // Rx FIFO1 Element Size.

    // General Receive Configuration
    msgRAMConfigParams.rxBufStartAddr       = MCAN_RX_BUFF_START_ADDR;  // Rx Buffer Start Address.
    msgRAMConfigParams.rxBufElemSize        = MCAN_RX_BUFF_ELEM_SIZE;   // Rx Buffer Element Size.

    // Initialize Rx Buffer Configuration parameters.
    stdFiltelem.sfid2              = 0x0U; // Standard Filter ID 2.
    stdFiltelem.sfid1              = 0x5U; // Standard Filter ID 1, 0 means "don't care", seems it doesn't matter now.
//    stdFiltelem.sfec               = 0x7U; // Store into Rx Buffer
    stdFiltelem.sfec               = MCAN_STDFILTEC_FIFO1;  // store in FIFO1
    stdFiltelem.sft                = 0x0U;    //  MCAN_STDFILT_DISABLED;   //  0x0U = Range filter from SFID1 to SFID2

    /************************
     *  Bit Timing Configuration Section Here
     *  NOTE: Bit Timings not required for legacy CAN, but required for startup of MCAN
     ************************/

    // Initialize bit timings
    bitTimes.nomRatePrescalar   = 0x3U; // Nominal Baud Rate Pre-scaler.
    bitTimes.nomTimeSeg1        = 0x9U; // Nominal Time segment before SP
    bitTimes.nomTimeSeg2        = 0x8U; // Nominal Time segment after SP
    bitTimes.nomSynchJumpWidth  = 0x8U; // Nominal SJW
    bitTimes.dataRatePrescalar  = 0x1U; // Data Baud Rate Pre-scaler.
    bitTimes.dataTimeSeg1       = 0x9U; // Data Time segment before SP
    bitTimes.dataTimeSeg2       = 0x8U; // Data Time segment after SP
    bitTimes.dataSynchJumpWidth = 0x8U; // Data SJW

    // Get MCANSS Revision ID.
    MCAN_getRevisionId(MCANA_DRIVER_BASE, &revId);

    // Wait for Memory initialization to be completed.
    while(FALSE == MCAN_isMemInitDone(MCANA_DRIVER_BASE));

    // Put MCAN in SW initialization mode.
    MCAN_setOpMode(MCANA_DRIVER_BASE, MCAN_OPERATION_MODE_SW_INIT);

    // Wait till MCAN in Initialization mode ( MCAN_CCCR.INIT == 1 )
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(MCANA_DRIVER_BASE));

    // Initialize MCAN module.
    MCAN_init(MCANA_DRIVER_BASE, &initParams);

    // Configure MCAN module.
    MCAN_config(MCANA_DRIVER_BASE, &configParams);

    // Configure Bit timings.
    MCAN_setBitTime(MCANA_DRIVER_BASE, &bitTimes);

    // Configure Message RAM Sections
    MCAN_msgRAMConfig(MCANA_DRIVER_BASE, &msgRAMConfigParams);

    // Configure Standard ID filter element
    MCAN_addStdMsgIDFilter(MCANA_DRIVER_BASE, 0U, &stdFiltelem);

    // Disable internal loopback mode
    MCAN_lpbkModeEnable(MCANA_DRIVER_BASE, MCAN_LPBK_MODE_EXTERNAL, FALSE);

    // Take MCAN out of the SW initialization mode
    MCAN_setOpMode(MCANA_DRIVER_BASE, MCAN_OPERATION_MODE_NORMAL);


    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(MCANA_DRIVER_BASE))
    {

    } // wait for initialization

}
