
#include <cana_msgs.h>

extern SystemReceivedObj DispenserMsgs;
extern SystemSendObj DispenserSendMsgs;
extern float VehicleVDCcommand;

void initCANAMessages()
{
    // Clear CANA Dispenser received messages
    memset(DispenserMsgs.PreChargeEnable, 0, PRECHARGE_ENABLE_TYPE_DATA_LENGTH);
    memset(DispenserMsgs.Limits, 0, IOTECHA_LIMIT_TYPE_DATA_LENGTH);
    memset(DispenserMsgs.VDCOutSetpoint, 0, VDC_OUT_SETPOINT_TYPE_DATA_LENGTH);
    DispenserMsgs.decode.VDCMaxLimit = 0;
    DispenserMsgs.decode.PDCLimit = 0;
    DispenserMsgs.decode.IOutLimit = 0;

    // Clear CANA Dispenser sender messages
    memset(DispenserSendMsgs.Counter, 0, COUNTER_TYPE_DATA_LENGTH);
    memset(DispenserSendMsgs.VDCOut, 0, VDC_OUT_TYPE_DATA_LENGTH);
    memset(DispenserSendMsgs.IDCOut, 0, IDC_OUT_TYPE_DATA_LENGTH);
    memset(DispenserSendMsgs.DCPActive, 0, DC_PWR_ACTIVE_TYPE_DATA_LENGTH);

    memset(DispenserSendMsgs.FaultStatus45, 0,  FAULT_STATUS_45_TYPE_DATA_LENGTH);
    memset(DispenserSendMsgs.CurrentAndPowerLimit, 0 ,CURRENT_AND_POWER_LIMIT_TYPE_DATA_LENGTH);
    memset(DispenserSendMsgs.FaultStatus123, 0 , FAULT_STATUS_123_TYPE_DATA_LENGTH);

    // Initialize the transmit message object data buffer to be sent
    DispenserSendMsgs.Counter[0] = 0x12;
    DispenserSendMsgs.Counter[1] = 0x34;

    // Initialize the transmit message object data buffer to be sent
    DispenserSendMsgs.VDCOut[0] = 0x00;
    DispenserSendMsgs.VDCOut[1] = 0x00;
    DispenserSendMsgs.FaultStatus45[5] = 0x01;

    uint16_t i;
    for ( i = 0; i < CURRENT_AND_POWER_LIMIT_TYPE_DATA_LENGTH ; i++ )
    {
        DispenserSendMsgs.CurrentAndPowerLimit[i] = 0;
    }

}

void processCurrentPowerVoltageLimitsMsg()
{

    // Split into 3 parts
    DispenserMsgs.decode.IOutLimit = (float)(( DispenserMsgs.Limits[1] << 8) | DispenserMsgs.Limits[0])*CURRENT_LIMIT_SCALE;
    DispenserMsgs.decode.PDCLimit = (float)(( DispenserMsgs.Limits[3] << 8) | DispenserMsgs.Limits[2])*POWER_LIMIT_SCALE;
    DispenserMsgs.decode.VDCMaxLimit = (float)(( DispenserMsgs.Limits[5] << 8) | DispenserMsgs.Limits[4])*DCV_LIMIT_SCALE;

    DispenserMsgs.LimitsCounter++;
}

void processCCSStatusMsg()
{
    // Magic bitmask
     //     8 [0]    16 [1] |    24 [2]    32 [3] |    40 [4]     48[5] |    56 [6]    64 [7]
     // 0000 0000 0000 0000 | 0000 0000 0000 0000 | 0000 0000 0000 0000 | 0000 0000 0000 0000

    // CCS_Status1_write_frame from the manufacturer lists these bytes as 1-4.  Here, they're listed as 0-3.

    // Decoding word 0 of the 362h message CCS_Status1_write_frame.
    // The lower 3 bits of this are Auth_State and the upper 5 are Cable Check.
    DispenserMsgs.decode.authStateRcv = (DispenserMsgs.CCSStatus[0]) & (0b0000000000000111);
    DispenserMsgs.decode.cableCheckStateRcv = ((DispenserMsgs.CCSStatus[0]) & (0b0000000011111000))>>3;

    // Decoding word 1 of the 362h message CCS_Status1_write_frame.
    // This whole word is a numeric indication of why the last charging session stopped.
    DispenserMsgs.decode.lastSessionStopReasonRcv = DispenserMsgs.CCSStatus[1];

    // Decoding word 2 of the 362h message CCS_Status1_write_frame.
    //   The lowest 4 bits of this is the CP state, and 3 of the 4 next bits are other status bits.  Bit
    //   7 is not used.
    DispenserMsgs.decode.controlPilotStateRcv = (DispenserMsgs.CCSStatus[2]) & (0b0000000000001111);
    DispenserMsgs.decode.vehicleChargingEnabledRcv = ((DispenserMsgs.CCSStatus[2]) & (0b0000000000010000))>>4;
    DispenserMsgs.decode.chargingInProgressRcv = ((DispenserMsgs.CCSStatus[2]) & (0b0000000000100000))>>5;
    DispenserMsgs.decode.vehicleContactorOpenRcv = ((DispenserMsgs.CCSStatus[2]) & (0b0000000001000000))>>6;

    // Decoding word 3 of the 362h message CCS_Status1_write_frame.  This is an integer value for the
    // vehicle state of charge
    DispenserMsgs.decode.vehicleBatteryStateOfChargeRcv = DispenserMsgs.CCSStatus[3];

    // Debug
    // Upon Error, the SOC disappears, so this stores what it was when it was sent.
    //    if(DispenserMsgs.decode.vehicleBatteryStateOfChargeRcv) iotechaTest = DispenserMsgs.decode.vehicleBatteryStateOfChargeRcv;

}

void processVDCOut()
{
    DispenserMsgs.decode.VDCOutSetPointRcv = ( DispenserMsgs.VDCOutSetpoint[1] << 8);
    DispenserMsgs.decode.VDCOutSetPointRcv += DispenserMsgs.VDCOutSetpoint[0];

    DispenserMsgs.VDCOutSetpointCounter++;

}
