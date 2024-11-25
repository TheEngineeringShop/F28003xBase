#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C"
{
#endif


#include <CLAmath.h>
#include <math.h>

#include "cla_jit_shared.h"

#include "driverlib.h"
#include "device.h"
#include "board.h"

#include "version.h"

#include "canbus.h"
#include "mcanbus.h"

#include "cana_msgs.h"  // rides on CANA
#include "mcan_msgs.h" // rides on MCAN

// Globals - Timer element
volatile uint16_t cpuTimer0IntCount;

// Status states
typedef enum sysStatus
{
    E_STATUS_NULL = 0,
    E_STATUS_INITIALIZING = 1,
    E_STATUS_DISCONNECTED = 2,
    E_STATUS_CONNECTED_AND_STARTING_OUTPUT = 3,
    E_STATUS_CONNECTED_AND_NOT_STARTING_OUTPUT = 4,
    E_STATUS_GENERATING_AC = 5,
    E_STATUS_ERROR = 6,
} sysStatus;


typedef enum interruptLength  // only defines the ones used in code
{
    E_INT_50MS = 1U,
    E_INT_100MS = 2U,
    E_INT_250MS = 5U,
    E_INT_500MS = 10U,
    E_INT_1S = 20U,
    E_INT_5S = 100U,
} interruptLength;


MCANMsgSendObj InventevMonitoringData = {
   // Voltage A
  .Van.canId = E_MCANDATA_MSGID_SUFFIX_Van,
  .Van.pvalue = &forScreen.VanRMS, // pointer to the value instead of the value itself
  .Van.timestamp = 0,
  .Van.count = 0,
  // Current A
  .currentA.canId = E_MCANDATA_MSGID_SUFFIX_currentA,
  .currentA.pvalue = &forScreen.IaAvgRMS, // pointer to the value instead of the value itself
  .currentA.timestamp = 0,
  .currentA.count = 0,
  // Power A
  .powerA.canId = E_MCANDATA_MSGID_SUFFIX_powerA,
  .powerA.value = 0,   // calculated in powerUpdate()
  .powerA.timestamp = 0,
  .powerA.count = 0,
  // Voltage B
  .Vbn.canId = E_MCANDATA_MSGID_SUFFIX_Vbn,
  .Vbn.pvalue = &forScreen.VbnRMS, // pointer to the value instead of the value itself
  .Vbn.timestamp = 0,
  .Vbn.count = 0,
  // Current B
  .currentB.canId = E_MCANDATA_MSGID_SUFFIX_currentB,
  .currentB.pvalue = &forScreen.IbAvgRMS, // pointer to the value instead of the value itself
  .currentB.timestamp = 0,
  .currentB.count = 0,
  // Power B
  .powerB.canId = E_MCANDATA_MSGID_SUFFIX_powerB,
  .powerB.value = 0,    // calculated in powerUpdate()
  .powerB.timestamp = 0,
  .powerB.count = 0,
  // Battery Voltage
  .voltBatt.canId = E_MCANDATA_MSGID_SUFFIX_voltBatt,
  .voltBatt.pvalue = &forScreen.VDC,
  .voltBatt.timestamp = 0,
  .voltBatt.count = 0,
   // Status Message - System State
  .statusMsg.canId = E_MCANDATA_MSGID_SUFFIX_status,
  .statusMsg.value = E_STATUS_INITIALIZING,
  .statusMsg.timestamp = 0,
  .statusMsg.count = 0,
  // CPU0 Level Timer 0, current set to 50ms counting.
  .MCUtimer.canId = E_MCANDATA_MSGID_SUFFIX_MCUtimer,
  .MCUtimer.value = 0,  // incremented each run of the timer
  .MCUtimer.timestamp = 0,
  .MCUtimer.count = 0,
};

extern Scal jitCal;

SystemReceivedObj DispenserMsgs;
SystemSendObj DispenserSendMsgs;


// Function Prototypes

void globalsInit();

void initEPWM1(void);
void initEPWM2(void);
void initEPWM3(void);
void initEPWM4(void);
void initEPWM5(void);
void initEPWM6(void);
void initEPWM8(void);

void initCLA(void);

void initADCA(void);
void initADCASOC(void);
void initADCB(void);
void initADCBSOC(void);
void initADCC(void);
void initADCCSOC(void);

void setupProfileGpio(void);
void setupAnalogPins(void);
void zeroCalibration(void);

void sendMonitoringData();  // higher speed monitoring data , to screen
void sendStatusMessage();  // lower speed data , to screen
void initMonitoring();
void powerUpdate();
void init();

void initCPUTimer0(void);
void initializeCalibration(void);
void turnOffInverter(void);

void updateDispenser();
void runState();


// Interrupt Function Prototypes
__interrupt void cpuTimer0ISR(void);
__interrupt void adcA1ISR(void);

__attribute__((interrupt))  void cla1Isr1(void);

#ifdef __cplusplus
}
#endif // extern "C"

#endif
