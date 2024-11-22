 /*
 * canbus_msgs.h
 *
 *      Known issues:
 */

#ifndef CANA_MSGS_H_
#define CANA_MSGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define COUNTER_TYPE_DATA_LENGTH     2   // TX
#define VDC_OUT_TYPE_DATA_LENGTH     8   // TX
#define IDC_OUT_TYPE_DATA_LENGTH     8   // TX
#define DC_PWR_ACTIVE_TYPE_DATA_LENGTH     8  // TX

#define FAULT_STATUS_45_TYPE_DATA_LENGTH             8   // TX , extended
#define CURRENT_AND_POWER_LIMIT_TYPE_DATA_LENGTH     8   // TX
#define FAULT_STATUS_123_TYPE_DATA_LENGTH            8   // TX

#define CCS_STATUS_TYPE_DATA_LENGTH             4   // RX , extended
#define PRECHARGE_ENABLE_TYPE_DATA_LENGTH       1   // RX , shortened
#define IOTECHA_LIMIT_TYPE_DATA_LENGTH          6   // RX
#define VDC_OUT_SETPOINT_TYPE_DATA_LENGTH       2   // RX

 // Refer to initCANAMsgs() for descriptions of these types
#define COUNTER_TYPE            1   // TX
#define CCS_STATUS_TYPE         2   // RX

#define VDC_OUT_TYPE            3   // TX
#define PRECHARGE_ENABLE_TYPE   4   // RX

#define IDC_OUT_TYPE            5   // TX
#define IOTECHA_LIMIT_TYPE      6   // RX

#define DC_PWR_ACTIVE_TYPE      7   // TX
#define VDC_OUT_SETPOINT_TYPE   8   // RX

#define FAULT_STATUS_45_TYPE            9     // TX
#define CURRENT_AND_POWER_LIMIT_TYPE    11    // TX
#define FAULT_STATUS_123_TYPE           13    // TX

// scaling factors for limits coming from Iotecha
#define CURRENT_LIMIT_SCALE     .01
#define POWER_LIMIT_SCALE       10
#define DCV_LIMIT_SCALE         .1

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "cla_jit_shared.h"

typedef enum authState
{
    E_AUTH_STATE_NA = 0,
    E_AUTH_STATE_AUTH = 1,
    E_AUTH_STATE_DEAUTH = 2,
    E_AUTH_STATE_TIMEOUT = 3,
    E_AUTH_STATE_LAST = 4
} authState;

typedef enum cableCheckState
{
    E_CABLECHECK_STATE_NA = 0,
    E_CABLECHECK_STATE_OK = 1,
    E_CABLECHECK_STATE_IN_PROGRESS = 2,
    E_CABLECHECK_STATE_PRECONDITION_CP_FAIL = 3,
    E_CABLECHECK_STATE_PRECONDITION_PP_FAIL = 4,
    E_CABLECHECK_STATE_IMD_FAIL = 5,
    E_CABLECHECK_STATE_VOLTAGE_FAIL = 6,
    E_CABLECHECK_STATE_RECTIFIER_FAIL = 7,
    E_CABLECHECK_STATE_ENABLED = 8,
    E_CABLECHECK_STATE_SHORT_CIRCUIT = 9,
    E_CABLECHECK_STATE_IMD_ISOLATION_FAIL = 10,
    E_CABLECHECK_STATE_IMD_SELFTEST_DETECT_FAIL = 11,
    E_CABLECHECK_STATE_IMD_SELFTEST_CLEAR_FAIL = 12,
    E_CABLECHECK_STATE_RECTIFIER_ERROR = 13,
    E_CABLECHECK_STATE_IMD_ERROR = 14,
    E_CABLECHECK_STATE_CONTACTORS_HEALTH_CHECK_FAIL = 15,
    E_CABLECHECK_STATE_FAULT = 16,
    E_CABLECHECK_STATE_INVALID_CP_STATE = 17,
    E_CABLECHECK_STATE_INVALID_PP_STATE = 18,
    E_CABLECHECK_STATE_UNEXPECTED_CP_PWM_COMMAND = 19,
    E_CABLECHECK_STATE_LAST = 20
} cableCheckState;

typedef enum lastSessionStopReason
{
    E_LAST_STOP_REASON_V2G_UNKNOWN = 0,
    E_LAST_STOP_REASON_V2G_SESSION_STOP = 1,
    E_LAST_STOP_REASON_V2G_SEQUENCE_ERROR = 2,
    E_LAST_STOP_REASON_VOLTAGE_LIMIT_EXCEEDED = 3,
    E_LAST_STOP_REASON_CURRENT_LIMIT_EXCEEDED = 4,
    E_LAST_STOP_REASON_PRECHARGE_RES_TIMEOUT = 5,
    E_LAST_STOP_REASON_CURRENT_DEMAND_RES_TIMEOUT = 6,
    E_LAST_STOP_REASON_WELDING_DETECTION_RES_TIMEOUT = 7,
    E_LAST_STOP_REASON_POWER_DELIVERY_RES_TIMEOUT = 8,
    E_LAST_STOP_REASON_NO_ACTIVITY_TIMEOUT = 9,
    E_LAST_STOP_REASON_CP_STATE_DISCONNECT_A = 10,
    E_LAST_STOP_REASON_CP_STATE_NOT_READY_TO_CHARGE_B = 11,
    E_LAST_STOP_REASON_PRE_COM_VOLTAGE_CHECK = 12,
    E_LAST_STOP_REASON_V2G_TCP_CONNECTION_CLOSED = 13,
    E_LAST_STOP_REASON_EV_IS_NOT_READY_FOR_POWER_DELIVERY = 14,
    E_LAST_STOP_REASON_EIM_AUTH_FORBIDDEN = 15,
    E_LAST_STOP_REASON_EXTERNAL_PAUSE_STOP = 16,
    E_LAST_STOP_REASON_CP_STATE_SWITCH_TO_B_DURING_CHARGING = 17,
    E_LAST_STOP_REASON_PLC_LINK_LOST = 18,
    E_LAST_STOP_REASON_CP_STATE_EVSE_NOT_AVAILABLE = 19,
    E_LAST_STOP_REASON_WRONG_CHARGE_PARAMETERS = 20,
    E_LAST_STOP_REASON_EXTERNAL_STOP_SESSION = 21,
    E_LAST_STOP_REASON_PRECHARGE_RAISE_VOLTAGE_TIMEOUT = 22,
    E_LAST_STOP_REASON_BASIC_CHARGING_SWITCH_TO_B = 23,
    E_LAST_STOP_REASON_OUTER_PWM_STOP = 24,
    E_LAST_STOP_REASON_V2G_SUPPORTED_SCHEMA_NOT_FOUND = 25,
    E_LAST_STOP_REASON_LAST = 26
} lastSessionStopReason;

/*******************************************************************************
 *  |-------------|---------------------|-----------------------|----------------|---------------|
 *  | Base status |   Charging status   | Resistance, CP-PE     | Resistance, R2 | Voltage, CP-PE|
 *  |-------------|---------------------|-----------------------|----------------|---------------|
 *  | Status A    | Standby             |Open or (infinity)Ohms |                |        +12 V  |
 *  | Status B    | Vehicle detected    |    2740 Ohms          |                |  +9(+/- 1) V  |
 *  | Status C    | Ready (charging)    |     882 Ohms          |   1300  Ohms   |  +6(+/- 1) V  |
 *  | Status D    | With ventilation    |     246 Ohms          |    270  Ohms   |  +3(+/- 1) V  |
 *  | Status E    | No power (shut off) |                       |                |          0 V  |
 *  | Status F    | Error               |                       |                |        -12 V  |
 *  |-------------|---------------------|-----------------------|----------------|---------------|
 *
 *  Control Pilot (Current limit): The charging station can use the wave signal
 * to describe the maximum current that is available via the charging station with
 * the help of pulse-width modulation: a 16% PWM is a 10 A maximum, a 25% PWM is a
 * 16 A maximum, a 50% PWM is a 32 A maximum and a 90% PWM flags a fast charge
 * option.

 * The PWM duty cycle of the 1 kHz CP signal indicates the maximum allowed mains
 * current. According to the SAE it includes socket outlet, cable and vehicle
 * inlet. In the US, the definition of the ampacity (ampere capacity, or current
 * capacity) is split for continuous and short term operation. The SAE defines
 * the ampacity value to be derived by a formula based on the 1 ms full cycle (of
 * the 1 kHz signal) with the maximum continuous ampere rating being 0.6 A per 10
 * us up to 850 us (with the lowest (100 us/10 us) x 0.6 A = 6 A). Above 850 us,
 * the formula requires subtraction of 640 us and multiplying the difference
 * by 2.5. For example ((960 us - 640 us)/10 us) x 2.5 A = 80 A.
 *
 * PWM duty cycle indicating ampere capacity[30]
 * PWM SAE continuous  SAE short term
 * 50% 30 A    36 A peak
 * 40% 24 A    30 A peak
 * 30% 18 A    22 A peak
 * 25% 15 A    20 A peak
 * 16% 9.6 A
 * 10% 6 A
 *
*******************************************************************************/

typedef enum controlPilotState  // CP State
{
    E_CONTROL_PILOT_STATE_NA = 0,
    E_CONTROL_PILOT_STATE_A = 1,
    E_CONTROL_PILOT_STATE_B = 2,
    E_CONTROL_PILOT_STATE_C = 3,
    E_CONTROL_PILOT_STATE_D = 4,
    E_CONTROL_PILOT_STATE_E = 5,
    E_CONTROL_PILOT_STATE_F = 6,
    E_CONTROL_PILOT_STATE_LAST = 7
} controlPilotState;

typedef struct SystemReceivedDecodeObj
{

    // These are set by the inbound (RX) message in Limits[]
    float IOutLimit;     // first 2 bytes, note signed
    float PDCLimit;      // 2nd 2 bytes, note signed
    float VDCMaxLimit;   // 3rd 2 Bytes, note signed

    // CCS Status Parsing
    authState authStateRcv;
    cableCheckState cableCheckStateRcv;                 // see cableCheckState
    lastSessionStopReason lastSessionStopReasonRcv;
    controlPilotState controlPilotStateRcv;             // see controlPilotState
    bool vehicleChargingEnabledRcv;
    bool chargingInProgressRcv;
    bool vehicleContactorOpenRcv;
    uint16_t vehicleBatteryStateOfChargeRcv;  // SOC
    uint16_t undefinedRcv;  // unknown 2 bits go here.

    // read masks stored in the process* function calls.

    // VDC Setpoint
    uint16_t VDCOutSetPointRcv;
    float VehicleVDCcommand;    // for consumption in the power sections of code.
} SystemReceivedDecodeObj;

typedef struct SystemReceivedObj {

    uint16_t CCSStatus[CCS_STATUS_TYPE_DATA_LENGTH];
    uint16_t CCSStatusCounter;

    uint16_t PreChargeEnable[PRECHARGE_ENABLE_TYPE_DATA_LENGTH];
    uint16_t PreChargeEnableCounter;

    uint16_t Limits[IOTECHA_LIMIT_TYPE_DATA_LENGTH];
    uint16_t LimitsCounter;

    uint16_t VDCOutSetpoint[VDC_OUT_SETPOINT_TYPE_DATA_LENGTH];
    uint16_t VDCOutSetpointCounter;

    SystemReceivedDecodeObj decode;

} SystemReceivedObj;

typedef struct SystemSendObj {
    uint16_t lastSendTimestamp;  // track last message handling
    uint16_t Counter[COUNTER_TYPE_DATA_LENGTH];
    uint16_t VDCOut[VDC_OUT_TYPE_DATA_LENGTH];
    uint16_t IDCOut[IDC_OUT_TYPE_DATA_LENGTH];
    uint16_t DCPActive[DC_PWR_ACTIVE_TYPE_DATA_LENGTH];
    uint16_t FaultStatus45[FAULT_STATUS_45_TYPE_DATA_LENGTH];
    uint16_t CurrentAndPowerLimit[CURRENT_AND_POWER_LIMIT_TYPE_DATA_LENGTH];
    uint16_t FaultStatus123[FAULT_STATUS_123_TYPE_DATA_LENGTH];
} SystemSendObj;

extern SystemSendObj DispenserSendMsgs;
extern SRMS forScreen;

void initCANAMessages();
void processCurrentPowerVoltageLimitsMsg();
void processCCSStatusMsg();
void processVDCOut();

#ifdef __cplusplus
}
#endif // extern "C"

#endif /* CANA_MSGS_H_ */
