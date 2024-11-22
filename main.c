#include "main.h"

// Define constants
#define RESULTS_BUFFER_SIZE     667

// Global variables

//#pragma DATA_SECTION(amPrecharging,"Cla1ToCpuMsgRAM");
int amPrecharging = 0;
//#pragma DATA_SECTION(amRunning,"Cla1ToCpuMsgRAM");
int amRunning = 0;

int startTheInverter = 0;
int stopTheInverter = 0;
int gdResetTime = 200;
#pragma DATA_SECTION(inverterRunning,"Cla1ToCpuMsgRAM");
int inverterRunning = 0;

// Autocalibration Variables
int doCalibration = 0;
uint16_t autoCalTimer = 0;
uint16_t autoCalReps = 0;
int autoCaled = 0;

float *p;

float anglestep = ANGLESTEP;
float angle = 0;
#pragma DATA_SECTION(sinewave,"CpuToCla1MsgRAM");
float sinewave = 0;
#pragma DATA_SECTION(SjitCal,"CpuToCla1MsgRAM");
Scal SjitCal;

#pragma DATA_SECTION(inverseVDC,"CpuToCla1MsgRAM");
float inverseVDC = 0;
#pragma DATA_SECTION(VehicleVDCcommand,"CpuToCla1MsgRAM");
float VehicleVDCcommand = 0;

#pragma DATA_SECTION(claTest,"Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(SInverterValues,"Cla1ToCpuMsgRAM");
float claTest;
Sresults SInverterValues;

// Error shutdown timers and variables
int DCOV_Timer = 0;
float DCOV_Value = 375;
int DCOV_Timeout = 200;
int DCOV = 0;

float myADC0Results[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t index;                              // Index into result buffer
volatile uint16_t bufferFull;                // Flag to indicate buffer is full

float adcARes0;

float dutyAtest = .9;
float dutyBtest = .9;
float dutyNtest = .5;
float sinDutyAtest = 0;
float sinDutyBtest = 0;

float rampStep = 1/(float)EPWM1_FREQ;
float rampVal = 0;
int InverterRamped = 0;
int startRamp = 0;

// For the PLL and also to generate RMS values for readback to the touch screen
#pragma DATA_SECTION(outputBuf,"bufloc");
float outputBuf[RMS_BUF_LENGTH];
uint16_t bufCounter;
float sinFreq;
uint16_t i8 = 0;
uint16_t lockCounterMax = 2000;                 // counter to determine PLL lock
uint16_t lockCounter = 0;
int locklost = 0;
uint16_t lock = 0;                        // Is the PLL locked?
uint16_t precharged = 0;

// next set for LPF's for PLL
float z11 = 0;
float z12 = 0;
float z21 = 0;
float z22 = 0;
float LPF1out = 0;
float LPF2out = 0;
float pll_a1 = -1.98997210635412;
float pll_a2 = 0.989997246016863;
float pll_b0 = 6.28491568581783e-06;
float pll_b1 = 1.25698313716357e-05;
float pll_b2 = 6.28491568581783e-06;
// next set for PLL PI
float PLL_contin = 0;
float last_PLL_contin = 0;
float PLL_Ki = 25;//2000/360;
float PLL_Kp = 2;//5000/360;
float PLL_integrand = 0;
float PLL_contout = 0;
float presentFrequency = 55;
// variables for the rest of the PLL
float initialFreq = 55;
float lastPresentFrequency = 55;
float anglePU = 0;
#pragma DATA_SECTION(sinPLL,"CpuToCla1MsgRAM");
float cosPLL = 0;
float sinPLL = 0;
float divForRMS = 0;


int RMScandidate = 0;               //0 is Van, 1 is Vbn, 2 is IaAvg, 3 is IbAvg, 4 is VanRemote for PLL
float Van_RMS_sum = 0;
float Vbn_RMS_sum = 0;
float IaAvg_RMS_sum = 0;
float IbAvg_RMS_sum = 0;
float VanRem_RMS_sum = 0;
float VanRemRMS = 0;
#pragma DATA_SECTION(forScreen,"CpuToCla1MsgRAM")
SRMS forScreen;



// State Machine Stuff.
// State 0 - Off
// State 1 - Precharge
// State 2 - Running
// State 3 - Shutdown
#pragma DATA_SECTION(stateMachineState,"CpuToCla1MsgRAM");
int stateMachineState = 0;
int stopCommand = 0;
//#pragma DATA_SECTION(PrechargeComplete,"Cla1ToCpuMsgRAM");
int PrechargeComplete = 0;
#pragma DATA_SECTION(DCVerror,"Cla1ToCpuMsgRAM");
float DCVerror = 0;
int prechargeSettlingTimer = 0;
float prechargeSettlingWindow = 3;





void init (void)
{

    // Initialize device clock and peripherals
    Device_init();

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();

    // Set up GPIOs
    setupProfileGpio();

    // Set up analog input pins
    setupAnalogPins();

    // Globals initialization
    globalsInit();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    // resets interrupt handling for any previously registered handlers
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).  breaks registration of interrupt handles registered before this point.
    Interrupt_initVectorTable();

    // Disable sync(Freeze clock to PWM as well)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Initialize EPWM modules, 1-6 for driving FETs, 8 for driving the fan
    initEPWM1();
    initEPWM2();
    initEPWM3();
    initEPWM4();

    initEPWM5();
    initEPWM6();
    // left out 7
    initEPWM8();

    // Initialize ADCA, ADCB, and ADCC
    initADCA();
    initADCASOC();

    initADCB();
    initADCBSOC();

    initADCC();
    initADCCSOC();

    // Configures the ADC module's offset trim
    ASysCtl_setAnalogReference1P65( ASYSCTL_VREFHI );
    ADC_setOffsetTrimAll(ADC_REFERENCE_INTERNAL,ADC_REFERENCE_3_3V);

    // Initialize resources
    Board_init();
    initCLA();
    initializeCalibration();
    initCPUTimer0();
    initMonitoring();
    // CANA, MCAN setup
    CANAInit();
    MCANInit();
//    initInventevMessages();

    //  ---  Interrupts start running below this point ----
    // Interrupt Settings for INT_myADC0_1
    Interrupt_register(INT_ADCA1, &adcA1ISR);
    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_EPWM1_TZ);

    // Enable global interrupts.
    EINT;

    // Enable sync and clock to PWM
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // MCAN Messaging up *after* the interrupts are running. otherwise you will hang.
    MCANMsgStartupMsg();   // send a message indicating the MCAN comms lines are up.

}


void main(void)
{
    init();

    for(;;)  // run forever, all done with interrupts
    {
        runState();
    }
}


void runState()
{

    if ( ( cpuTimer0IntCount % E_INT_100MS == 0 )        // 50ms timer * 20 = 1 second interval
            &&
            (  InventevMonitoringData.MCUtimer.timestamp != cpuTimer0IntCount) )
    {
        //lastSendMonitoringTime -- implement me to fix the multiple messages every so often on the MCAN send
        InventevMonitoringData.MCUtimer.value = (float)(cpuTimer0IntCount / 20);  // 20 to get to a 1 second counter
        InventevMonitoringData.MCUtimer.timestamp = cpuTimer0IntCount;
        sendMonitoringData();
    }

    if ( cpuTimer0IntCount % E_INT_50MS == 0
            &&
            DispenserSendMsgs.lastSendTimestamp != cpuTimer0IntCount)   // 50ms timer * 1 = 50 ms intervals
    {

        updateDispenser(); // timer checks and should run only 1x per 50ms check
        DispenserSendMsgs.lastSendTimestamp = cpuTimer0IntCount;
    }

}



// ADC Initialization
// Function to configure and power up ADC A
void initADCA(void)
{
    // Setup VREF as internal
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    // Set ADCCLK divider to /2
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);

    // Set pulse positions to late
    //ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_ACQ_WIN);
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    // Set interrupt offset delay as 10 cycles based on the calculation
    // shown in example header
    // Calculate optimal delay
    ADC_setInterruptCycleOffset(ADCA_BASE, 10);

    // Power up the ADCs and then delay for 1 ms
    ADC_enableConverter(ADCA_BASE);

    DEVICE_DELAY_US(1000);

    // Disables SOC burst mode.
    ADC_disableBurstMode(ADCA_BASE);

    // Sets the priority mode of the SOCs.
    ADC_setSOCPriority(ADCA_BASE, ADC_PRI_ALL_HIPRI);

}

// ADC SOC Initialization
//
// Description: This function will configure the ADC, channel A0 to start
// its conversion on a trigger from EPWM1 (EPMW1SOCA). The ADC will sample this
// channel continuously. After each conversion it will assert ADCINT1, which
// is then used to trigger task 1 of the CLA
void initADCASOC(void)
{
    // Configure SOC0 of ADCA
    // - SOC0 will be triggered by EPWM1SOCA
    // - SOC0 will convert pin A0 with a sample window of 10 SYSCLK cycles.
    // - EOC0 will be generated at the end of conversion
    // - SOC0 will sample on each trigger regardless of the interrupt flag

    // Start of Conversion 0 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 0
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN1
    //      Channel Function: ISENS_OUT1 (A1)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 10U);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);

    // Start of Conversion 1 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 1
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN7
    //      Channel Function: ISENS_OUT6 (B2)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN7, 10U);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);

    // Start of Conversion 2 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 2
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN0
    //      Channel Function: VDCP
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 10U);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);

    // Start of Conversion 3 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 3
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN2
    //      Channel Function: VAC1 (Local A)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 10U);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 4 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 4
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN3
    //      Channel Function: VAC4 (Remote A)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 10U);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 5 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 5
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN8
    //      Channel Function: Heatsink Temperature 1
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //

    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN8, 10U);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);

    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER5);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}
// ADC Initialization
//
// Function to configure and power up ADC B
void initADCB(void)
{
    // Setup VREF as internal
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    // Set ADCCLK divider to /2
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_2_0);

    // Set pulse positions to late
    //ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_ACQ_WIN);
    //ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);

    // Set interrupt offset delay as 40 cycles based on the calculation
    // shown in example header
    //ADC_setInterruptCycleOffset(ADCB_BASE, 40);

    // Power up the ADCs and then delay for 1 ms
    ADC_enableConverter(ADCB_BASE);

    DEVICE_DELAY_US(1000);
    // Disables SOC burst mode.
    ADC_disableBurstMode(ADCB_BASE);
    // Sets the priority mode of the SOCs.
    ADC_setSOCPriority(ADCB_BASE, ADC_PRI_ALL_HIPRI);
}

// ADC SOC Initialization
//
// Description: This function will configure the ADC, channel A0 to start
// its conversion on a trigger from EPWM1 (EPMW1SOCA). The ADC will sample this
// channel continuously. After each conversion it will assert ADCINT1, which
// is then used to trigger task 1 of the CLA
void initADCBSOC(void)
{
    // Configure SOC0 of ADCB
    // - SOC0 will be triggered by EPWM1SOCA
    // - SOC0 will convert pin A0 with a sample window of 10 SYSCLK cycles.
    // - EOC0 will be generated at the end of conversion
    // - SOC0 will sample on each trigger regardless of the interrupt flag


    // Start of Conversion 0 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 0
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN8
    //      Channel Function: ISENS_OUT4 (A2)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN8, 10U);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
    //

    //
    // Start of Conversion 1 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 1
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN0
    //      Channel Function: VDCN
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 10U);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 2 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 2
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN3
    //      Channel Function: ISENS_OUT5 (N2)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 10U);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 3 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 3
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN2
    //      Channel Function: VAC2 (Local B)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 10U);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 4 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 4
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN4
    //      Channel Function: VAC5 (Remote B)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN4, 10U);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 5 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 5
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN5
    //      Channel Function: Heatsink Temp 2
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //

    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN5, 10U);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);

}


// ADC Initialization
//
// Function to configure and power up ADC C
//
void initADCC(void)
{
    //
    // Setup VREF as internal
    //
    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    //
    // Set ADCCLK divider to /2
    //
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_2_0);

    //
    // Set pulse positions to late
    //
    //ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_ACQ_WIN);
    //ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Set interrupt offset delay as 40 cycles based on the calculation
    // shown in example header
    //
    //ADC_setInterruptCycleOffset(ADCC_BASE, 40);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(ADCC_BASE);

    DEVICE_DELAY_US(1000);
    //
    // Disables SOC burst mode.
    //
    ADC_disableBurstMode(ADCC_BASE);
    //
    // Sets the priority mode of the SOCs.
    //
    ADC_setSOCPriority(ADCC_BASE, ADC_PRI_ALL_HIPRI);
}

//
// ADC SOC Initialization
//
// Description: This function will configure the ADC, channel A0 to start
// its conversion on a trigger from EPWM1 (EPMW1SOCA). The ADC will sample this
// channel continuously. After each conversion it will assert ADCINT1, which
// is then used to trigger task 1 of the CLA
//
void initADCCSOC(void)
{
    //
    // Configure SOC0 of ADCC
    // - SOC0 will be triggered by EPWM1SOCA
    // - SOC0 will convert pin A0 with a sample window of 10 SYSCLK cycles.
    // - EOC0 will be generated at the end of conversion
    // - SOC0 will sample on each trigger regardless of the interrupt flag
    //

    //
    // Start of Conversion 0 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 0
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN0
    //      Channel Function: ISENS_OUT7 (DC current)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 10U);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
    //

    //
    // Start of Conversion 0 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 1
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN1
    //      Channel Function: ISENS_OUT3 (B1)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 10U);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 2 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 2
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN10
    //      Channel Function: ISENS_OUT2 (N1)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN10, 10U);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 3 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 3
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN2
    //      Channel Function: VAC3 (Local B to A)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 10U);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 4 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 4
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN4
    //      Channel Function: VAC6 (Remote B to A)
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN4, 10U);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);

    //
    // Start of Conversion 5 Configuration
    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    //      SOC number      : 5
    //      Trigger         : ADC_TRIGGER_EPWM1_SOCA
    //      Channel         : ADC_CH_ADCIN5
    //      Channel Function: Ambient Temperature
    //      Sample Window   : 10 SYSCLK cycles
    //      Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
    //

    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN5, 10U);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);

}

//
// EPWM Initialization
//
// Description: EPWM1A will run at EPWM1_FREQ (40 kHz) and is used to generate
// the PWM output. It also serves as the sampling clock for ADC channel A0
// The default time base for the EPWM module is half the system clock
//
void initEPWM1(void)
{
    EALLOW;
    // Set up EPWM1 to
    // - run on a base clock of SYSCLK
    // - have a period of EPWM_PERIOD
    // - run in count up down mode
    // - initialize the COMPA register to realize 0.1 duty

    // Disable SOCA and SOCB
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_B);

    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM1_BASE, EPWM1_PERIOD);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);

    // Set initial values for COMPA and COMPB registers
    // Enable shadow loading mode
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(0.1f * EPWM1_PERIOD));
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, (uint16_t)(0.1f * EPWM1_PERIOD));
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);


    // Set counter mode
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    // Set phase shifting and synchronization parameters
    EPWM_setCountModeAfterSync(EPWM1_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);
    EPWM_setPhaseShift(EPWM1_BASE, 0);
    EPWM_setSyncInPulseSource(EPWM1_BASE, EPWM_SYNC_IN_PULSE_SRC_DISABLE);
    EPWM_enableSyncOutPulseSource(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    // Configuring action-qualifiers for EPWM1
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
//
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


    // Configuring dead-band zone in EPWM register
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);
    EPWM_setRisingEdgeDelayCountShadowLoadMode(EPWM1_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(EPWM1_BASE);
    EPWM_setRisingEdgeDelayCount(EPWM1_BASE, EPWM_DT_CONST);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(EPWM1_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(EPWM1_BASE);
    EPWM_setFallingEdgeDelayCount(EPWM1_BASE, EPWM_DT_CONST);

    // Enable SOC-A and set it to assert when the counter hits
    // zero. It asserts on every event
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1U);

    // Set up Trip Zone
    EPWM_setTripZoneAction(EPWM1_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_enableTripZoneSignals(EPWM1_BASE, EPWM_TZ_SIGNAL_OSHT1);
    // start with PWMs off
    EPWM_forceTripZoneEvent(EPWM1_BASE,EPWM_TZ_FORCE_EVENT_OST);

    // EPWM 1 should run freely in emulation mode
    //EPWM_setEmulationMode(EPWM1_BASE, EPWM_EMULATION_FREE_RUN);

    EDIS;

}

void initEPWM2(void)
{
    EALLOW;
    // Set up EPWM2 to
    // - run on a base clock of SYSCLK
    // - have a period of EPWM_PERIOD
    // - run in count up down mode
    // - initialize the COMPA register to realize 0.1 duty

    // Disable SOCA and SOCB
    //EPWM_disableADCTrigger(EPWM2_BASE, EPWM_SOC_A);
    //EPWM_disableADCTrigger(EPWM2_BASE, EPWM_SOC_B);

    // Set the local ePWM module clock divider to /1
    // Set EPMWM period and base counter value
    //
    EPWM_setClockPrescaler(EPWM2_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM2_BASE, EPWM1_PERIOD);
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0);

    // Set initial values for COMPA and COMPB registers
    // Enable shadow loading mode
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, 750U);
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, 750U);
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // Set counter mode
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);


    //EPWM_forceSyncPulse(EPWM2_BASE);
    EPWM_setSyncInPulseSource(EPWM2_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1);
    EPWM_setCountModeAfterSync(EPWM2_BASE, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setPhaseShift(EPWM2_BASE, 0);
    EPWM_forceSyncPulse(EPWM2_BASE);


    // Configuring action-qualifiers for EPWM2
//    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
//    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    // Configuring dead-band zone in EPWM register
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);
    EPWM_setRisingEdgeDelayCountShadowLoadMode(EPWM2_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(EPWM2_BASE);
    EPWM_setRisingEdgeDelayCount(EPWM2_BASE, EPWM_DT_CONST);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(EPWM2_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(EPWM2_BASE);
    EPWM_setFallingEdgeDelayCount(EPWM2_BASE, EPWM_DT_CONST);

    // Set up Trip Zone
    EPWM_setTripZoneAction(EPWM2_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_enableTripZoneSignals(EPWM2_BASE, EPWM_TZ_SIGNAL_OSHT1);
    // start with PWMs off
    EPWM_forceTripZoneEvent(EPWM2_BASE,EPWM_TZ_FORCE_EVENT_OST);

    EDIS;
}

void initEPWM3(void)
{
    EALLOW;
    //
    // Set up EPWM3 to
    // - run on a base clock of SYSCLK
    // - have a period of EPWM_PERIOD
    // - run in count up down mode
    // - initialize the COMPA register to realize 0.1 duty

    //
    // Disable SOCA and SOCB
    //
    //EPWM_disableADCTrigger(EPWM3_BASE, EPWM_SOC_A);
    //EPWM_disableADCTrigger(EPWM3_BASE, EPWM_SOC_B);

    //
    // Set the local ePWM module clock divider to /1
    // Set EPMWM period and base counter value
    //
    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM3_BASE, EPWM1_PERIOD);
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0);

    //
    // Set initial values for COMPA and COMPB registers
    // Enable shadow loading mode
    //
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, 750U);
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_B, 750U);
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);


    //EPWM_forceSyncPulse(EPWM3_BASE);
    EPWM_setSyncInPulseSource(EPWM3_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM2);
    EPWM_setCountModeAfterSync(EPWM3_BASE, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
    EPWM_enablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setPhaseShift(EPWM3_BASE, 0);
    EPWM_forceSyncPulse(EPWM3_BASE);


    //
    // Configuring action-qualifiers for EPWM2
    //
//    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
//    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    // Configuring dead-band zone in EPWM register
    //
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
    EPWM_setRisingEdgeDelayCountShadowLoadMode(EPWM3_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(EPWM3_BASE);
    EPWM_setRisingEdgeDelayCount(EPWM3_BASE, EPWM_DT_CONST);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(EPWM3_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(EPWM3_BASE);
    EPWM_setFallingEdgeDelayCount(EPWM3_BASE, EPWM_DT_CONST);


    // Set up Trip Zone
    EPWM_setTripZoneAction(EPWM3_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_enableTripZoneSignals(EPWM3_BASE, EPWM_TZ_SIGNAL_OSHT1);
    // start with PWMs off
    EPWM_forceTripZoneEvent(EPWM3_BASE,EPWM_TZ_FORCE_EVENT_OST);


    EDIS;

}

void initEPWM4(void)
{
    EALLOW;
    //
    // Set up EPWM4 to
    // - run on a base clock of SYSCLK
    // - have a period of EPWM_PERIOD
    // - run in count up down mode
    // - initialize the COMPA register to realize 0.1 duty

    //
    // Disable SOCA and SOCB
    //
    //EPWM_disableADCTrigger(EPWM4_BASE, EPWM_SOC_A);
    //EPWM_disableADCTrigger(EPWM4_BASE, EPWM_SOC_B);

    //
    // Set the local ePWM module clock divider to /1
    // Set EPMWM period and base counter value
    //
    EPWM_setClockPrescaler(EPWM4_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM4_BASE, EPWM1_PERIOD);
    EPWM_setTimeBaseCounter(EPWM4_BASE, 0);

    //
    // Set initial values for COMPA and COMPB registers
    // Enable shadow loading mode
    //
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(0.1f * EPWM1_PERIOD));
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_B, (uint16_t)(0.1f * EPWM1_PERIOD));
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP_DOWN);


    //EPWM_forceSyncPulse(EPWM4_BASE);
    EPWM_setCountModeAfterSync(EPWM4_BASE, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
    EPWM_enablePhaseShiftLoad(EPWM4_BASE);
    EPWM_setPhaseShift(EPWM4_BASE, PHASE180);
    EPWM_setSyncInPulseSource(EPWM4_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1);
    EPWM_enableSyncOutPulseSource(EPWM4_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
//    EPWM_forceSyncPulse(EPWM4_BASE);


    //
    // Configuring action-qualifiers for EPWM
    //
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    //
    // Configuring dead-band zone in EPWM register
    //
    EPWM_setDeadBandDelayPolarity(EPWM4_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_RED, true);
    EPWM_setRisingEdgeDelayCountShadowLoadMode(EPWM4_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(EPWM4_BASE);
    EPWM_setRisingEdgeDelayCount(EPWM4_BASE, EPWM_DT_CONST);
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(EPWM4_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(EPWM4_BASE);
    EPWM_setFallingEdgeDelayCount(EPWM4_BASE, EPWM_DT_CONST);

    //
    // Set up Trip Zone
    //
    EPWM_setTripZoneAction(EPWM4_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_enableTripZoneSignals(EPWM4_BASE, EPWM_TZ_SIGNAL_OSHT1);
    // start with PWMs off
    EPWM_forceTripZoneEvent(EPWM4_BASE,EPWM_TZ_FORCE_EVENT_OST);



    EDIS;
}

void initEPWM5(void)
{
    EALLOW;
    //
    // Set up EPWM5 to
    // - run on a base clock of SYSCLK
    // - have a period of EPWM_PERIOD
    // - run in count up down mode
    // - initialize the COMPA register to realize 0.1 duty

    //
    // Disable SOCA and SOCB
    //
    //EPWM_disableADCTrigger(EPWM5_BASE, EPWM_SOC_A);
    //EPWM_disableADCTrigger(EPWM5_BASE, EPWM_SOC_B);

    //
    // Set the local ePWM module clock divider to /1
    // Set EPMWM period and base counter value
    //
    EPWM_setClockPrescaler(EPWM5_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM5_BASE, EPWM1_PERIOD);
    EPWM_setTimeBaseCounter(EPWM5_BASE, 0);

    //
    // Set initial values for COMPA and COMPB registers
    // Enable shadow loading mode
    //
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, 750U);
    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_B, 750U);
    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM5_BASE, EPWM_COUNTER_MODE_UP_DOWN);


    //EPWM_forceSyncPulse(EPWM5_BASE);
    EPWM_setCountModeAfterSync(EPWM5_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);
    EPWM_enablePhaseShiftLoad(EPWM5_BASE);
    EPWM_setPhaseShift(EPWM5_BASE, PHASE180);
    EPWM_forceSyncPulse(EPWM5_BASE);


    //
    // Configuring action-qualifiers for EPWM2
    //
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    //
    // Configuring dead-band zone in EPWM register
    //
    EPWM_setDeadBandDelayPolarity(EPWM5_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_RED, true);
    EPWM_setRisingEdgeDelayCountShadowLoadMode(EPWM5_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(EPWM5_BASE);
    EPWM_setRisingEdgeDelayCount(EPWM5_BASE, EPWM_DT_CONST);
    EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(EPWM5_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(EPWM5_BASE);
    EPWM_setFallingEdgeDelayCount(EPWM5_BASE, EPWM_DT_CONST);


    //
    // Set up Trip Zone
    //
    EPWM_setTripZoneAction(EPWM5_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_enableTripZoneSignals(EPWM5_BASE, EPWM_TZ_SIGNAL_OSHT1);
    // start with PWMs off
    EPWM_forceTripZoneEvent(EPWM5_BASE,EPWM_TZ_FORCE_EVENT_OST);


    EDIS;
}

void initEPWM6(void)
{
    EALLOW;

    // Set up EPWM6 to
    // - run on a base clock of SYSCLK
    // - have a period of EPWM_PERIOD
    // - run in count up down mode
    // - initialize the COMPA register to realize 0.1 duty

    //
    // Disable SOCA and SOCB
    //
    //EPWM_disableADCTrigger(EPWM6_BASE, EPWM_SOC_A);
    //EPWM_disableADCTrigger(EPWM6_BASE, EPWM_SOC_B);

    //
    // Set the local ePWM module clock divider to /1
    // Set EPMWM period and base counter value
    //
    EPWM_setClockPrescaler(EPWM6_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM6_BASE, EPWM1_PERIOD);
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0);

    //
    // Set initial values for COMPA and COMPB registers
    // Enable shadow loading mode
    //
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, 750U);
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_B, 750U);
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // Set counter mode
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP_DOWN);


    //EPWM_forceSyncPulse(EPWM6_BASE);
    EPWM_setCountModeAfterSync(EPWM6_BASE, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
    EPWM_enablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setPhaseShift(EPWM6_BASE, PHASE180);
    EPWM_forceSyncPulse(EPWM6_BASE);

    // Configuring action-qualifiers for EPWM
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    // Configuring dead-band zone in EPWM register
    EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_RED, true);
    EPWM_setRisingEdgeDelayCountShadowLoadMode(EPWM6_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(EPWM6_BASE);
    EPWM_setRisingEdgeDelayCount(EPWM6_BASE, EPWM_DT_CONST);
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(EPWM6_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(EPWM6_BASE);
    EPWM_setFallingEdgeDelayCount(EPWM6_BASE, EPWM_DT_CONST);

    // Set up Trip Zone
    EPWM_setTripZoneAction(EPWM6_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_enableTripZoneSignals(EPWM6_BASE, EPWM_TZ_SIGNAL_OSHT1);
    // start with PWMs off
    EPWM_forceTripZoneEvent(EPWM6_BASE,EPWM_TZ_FORCE_EVENT_OST);


    EDIS;
}

void initEPWM8(void)
{
    EALLOW;
    //
    // Set up EPWM8 to
    // - run on a base clock of SYSCLK
    // - have a period of EPWM_PERIOD*10
    // - run in count up down mode
    // - initialize the COMPA register to realize 0.0 duty
    // - This is for the fan, so only need pwmA
    //
    // Disable SOCA and SOCB
    //
    //EPWM_disableADCTrigger(EPWM8_BASE, EPWM_SOC_A);
    //EPWM_disableADCTrigger(EPWM8_BASE, EPWM_SOC_B);

    //
    // Set the local ePWM module clock divider to /1
    // Set EPMWM period and base counter value
    //
    EPWM_setClockPrescaler(EPWM8_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM8_BASE, EPWM1_PERIOD*10);
    EPWM_setTimeBaseCounter(EPWM8_BASE, 0);

    //
    // Set initial values for COMPA and COMPB registers
    // Enable shadow loading mode
    //
    EPWM_setCounterCompareValue(EPWM8_BASE, EPWM_COUNTER_COMPARE_A, 0U);
    EPWM_setCounterCompareShadowLoadMode(EPWM8_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
//    EPWM_setCounterCompareValue(EPWM8_BASE, EPWM_COUNTER_COMPARE_B, 0U);
//    EPWM_setCounterCompareShadowLoadMode(EPWM8_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM8_BASE, EPWM_COUNTER_MODE_UP_DOWN);


    //EPWM_forceSyncPulse(EPWM8_BASE);
//    EPWM_setCountModeAfterSync(EPWM8_BASE, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
//    EPWM_enablePhaseShiftLoad(EPWM8_BASE);
////    EPWM_setPhaseShift(EPWM8_BASE, PHASE180);
//    EPWM_forceSyncPulse(EPWM8_BASE);


    //
    // Configuring action-qualifiers for EPWM
    //
    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

//    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
//    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
//    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
//    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
//    EPWM_setActionQualifierAction(EPWM8_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    //
    // Configuring dead-band zone in EPWM register
    //
//    EPWM_setDeadBandDelayPolarity(EPWM8_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
//    EPWM_setDeadBandDelayMode(EPWM8_BASE, EPWM_DB_RED, true);
//    EPWM_setRisingEdgeDelayCountShadowLoadMode(EPWM8_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);
//    EPWM_disableRisingEdgeDelayCountShadowLoadMode(EPWM8_BASE);
//    EPWM_setRisingEdgeDelayCount(EPWM8_BASE, EPWM_DT_CONST);
//    EPWM_setDeadBandDelayMode(EPWM8_BASE, EPWM_DB_FED, true);
//    EPWM_setFallingEdgeDelayCountShadowLoadMode(EPWM8_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);
//    EPWM_disableFallingEdgeDelayCountShadowLoadMode(EPWM8_BASE);
//    EPWM_setFallingEdgeDelayCount(EPWM8_BASE, EPWM_DT_CONST);

    EDIS;
}


// CLA Initialization
void initCLA(void)
{
    // Force task 8, the one time initialization task to initialize
    // the CLA global variables
    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_8);
}

// Setting up GPIO2 for profiling and set CLA as its controller
void setupProfileGpio(void)
{

        // GPIO0 is set to EPWM1A
        GPIO_setControllerCore(0U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(0U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_0_EPWM1_A);

        // GPIO1 is set to EPWM1B
        GPIO_setControllerCore(1U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(1U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_1_EPWM1_B);

        // GPIO2 is set to EPWM2A
        GPIO_setControllerCore(2U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(2U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_2_EPWM2_A);

        // GPIO3 is set to EPWM2B
        GPIO_setControllerCore(3U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(3U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_3_EPWM2_B);

        // GPIO6 is set to Slave Out (SO) for SPI Memory- was set to EPWM4A, needs reassignment
        //GPIO_setDirectionMode(6U, GPIO_DIR_MODE_OUT);
//        GPIO_setControllerCore(6U, GPIO_CORE_CPU1);
//        GPIO_setPadConfig(6U,GPIO_PIN_TYPE_STD);
//        GPIO_setPinConfig(GPIO_6_EPWM4_A);

        // GPIO7 is set to Slave in (SI) for SPI Memory- was set to EPWM4B, needs reassignment
        //GPIO_setDirectionMode(7U, GPIO_DIR_MODE_OUT);
//        GPIO_setControllerCore(7U, GPIO_CORE_CPU1);
//        GPIO_setPadConfig(7U,GPIO_PIN_TYPE_STD);
//        GPIO_setPinConfig(GPIO_7_EPWM4_B);

       //  GPIO8 is set to Relay Precharge Drive
        GPIO_setDirectionMode(8U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(8U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(8U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_8_GPIO8);

        // GPIO9 is set as input pin for DIP Switch
        GPIO_setDirectionMode(9U, GPIO_DIR_MODE_IN);
        GPIO_setControllerCore(9U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(9U,GPIO_PIN_TYPE_STD);
 //       GPIO_setPinConfig(GPIO_9_GPIO9);
//        GPIO_writePin(9U,0U);

        // GPIO10 is set to EPWM6A
        //GPIO_setDirectionMode(10U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(10U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(10U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_10_EPWM6_A);

        // GPIO11 is set to EPWM6B
        //GPIO_setDirectionMode(11U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(11U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(11U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_11_EPWM6_B);

        // GPIO14 is set to EPWM3A
        //GPIO_setDirectionMode(14U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(14U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(14U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_14_EPWM3_A);

        // GPIO15 is set to EPWM3B
        //GPIO_setDirectionMode(15U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(15U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(15U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_15_EPWM3_B);

        // GPIO16 is set to EPWM5A
        //GPIO_setDirectionMode(16U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(16U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(16U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_16_EPWM5_A);

        // GPIO17 is set to EPWM5B
        //GPIO_setDirectionMode(16U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(17U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(17U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_17_EPWM5_B);

        // GPIO22 is set to EPWM4A
        //GPIO_setDirectionMode(22U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(22U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(22U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_22_EPWM4_A);

        // GPIO23 is set to EPWM4B
        //GPIO_setDirectionMode(23U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(23U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(23U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_23_EPWM4_B);

        // GPIO24 is set to EPWM8A for Fan
        //GPIO_setDirectionMode(24U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(24U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(24U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_24_EPWM8_A);

        // GPIO33 is set to Relay Run Drive
        GPIO_setDirectionMode(33U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(33U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(33U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_33_GPIO33);

        // GPIO25 is configured as output and CLA is assigned its controller- needs to be configured for Fan tach
//        GPIO_setDirectionMode(25U,GPIO_DIR_MODE_OUT);
//        GPIO_setQualificationMode(25U,GPIO_QUAL_SYNC);
//        GPIO_setPinConfig(GPIO_25_GPIO25);
//        GPIO_writePin(25U,0);
//        GPIO_setControllerCore(25U, GPIO_CORE_CPU1_CLA1);

        // GPIO34 is set to NOT Gate Drive Reset
        GPIO_setDirectionMode(34U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(34U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(34U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_34_GPIO34);
        GPIO_writePin(34U,1);

        // GPIO35 is set to TDI for JTAG- was EPWM5B
        //GPIO_setDirectionMode(35U, GPIO_DIR_MODE_OUT);
//        GPIO_setControllerCore(35U, GPIO_CORE_CPU1);
//        GPIO_setPadConfig(35U,GPIO_PIN_TYPE_STD);
//        GPIO_setPinConfig(GPIO_35_EPWM5_B);

        // GPIO47 is set to Run Gate Drive
        GPIO_setDirectionMode(47U, GPIO_DIR_MODE_OUT);
        GPIO_setControllerCore(47U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(47U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_47_GPIO47);

        // GPIO55 is Trip Zone
        GPIO_setDirectionMode(55U, GPIO_DIR_MODE_IN);
        GPIO_setQualificationMode(55U, GPIO_QUAL_6SAMPLE);
        GPIO_setControllerCore(55U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(55U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_55_GPIO55);
        XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT1, 55);

// --------- CANA
        // GPIO31 is set to CANATX (CANA)
        GPIO_setControllerCore(31U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(31U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_31_CANA_TX);

        // GPIO30 is set to CANARX (CANA)
        GPIO_setControllerCore(30U, GPIO_CORE_CPU1);
        GPIO_setPadConfig(30U,GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_30_CANA_RX);
// --------- CANA

}

void setupAnalogPins(void){

        // PinMux for modules assigned to CPU1

        // ANALOG -> myANALOGPinMux0 Pinmux

        //VDCP
        // Analog PinMux for A0/B15/C15/DACA_OUT
        GPIO_setPinConfig(GPIO_231_GPIO231);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(231, GPIO_ANALOG_ENABLED);

        //ISENS_OUT1 - IA1
        // Analog PinMux for A1/B7/DACB_OUT
        GPIO_setPinConfig(GPIO_232_GPIO232);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(232, GPIO_ANALOG_ENABLED);

        // ISENS_OUT2 - IB1
        // Analog PinMux for A10/B1/C10
        GPIO_setPinConfig(GPIO_230_GPIO230);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(230, GPIO_ANALOG_ENABLED);

        // ISENS_OUT7 - IDC
        // Analog PinMux for A11/B10/C0
        GPIO_setPinConfig(GPIO_237_GPIO237);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(237, GPIO_ANALOG_ENABLED);

        // Amb_Temp
        // Analog PinMux for A12, C5
        GPIO_setPinConfig(GPIO_238_GPIO238);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(238, GPIO_ANALOG_ENABLED);

        // VAC6 -  Phase C to Neutral, not used in Split Phase
        // Analog PinMux for A14/B14/C4
        GPIO_setPinConfig(GPIO_239_GPIO239);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(239, GPIO_ANALOG_ENABLED);

        // VAC1- Phase A to Neutral local in Split Phase, Phase A to C in 3 phase
        // Analog PinMux for A2/B6/C9
        GPIO_setPinConfig(GPIO_224_GPIO224);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(224, GPIO_ANALOG_ENABLED);

        // VAC4- Phase A to Neutral remote in Split and 3 Phase
        // Analog PinMux for A3, C7/B9
        GPIO_setPinConfig(GPIO_229_GPIO229);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(229, GPIO_ANALOG_ENABLED);

        // ISENS_OUT4 - IA2
        // Analog PinMux for A4/B8
        GPIO_setPinConfig(GPIO_225_GPIO225);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(225, GPIO_ANALOG_ENABLED);

        // Reserved, commented out
        // Analog PinMux for A5
//        GPIO_setPinConfig(GPIO_249_GPIO249);
//        // AIO -> Analog mode selected
//        GPIO_setAnalogMode(249, GPIO_ANALOG_ENABLED);
//        // Analog PinMux for A6
//        GPIO_setPinConfig(GPIO_228_GPIO228);
//        // AIO -> Analog mode selected
//        GPIO_setAnalogMode(228, GPIO_ANALOG_ENABLED);


        // HS_Temp1
        // Analog PinMux for A8
        GPIO_setPinConfig(GPIO_240_GPIO240);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(240, GPIO_ANALOG_ENABLED);

        // Not used, commented out
        // Analog PinMux for A9
//        GPIO_setPinConfig(GPIO_227_GPIO227);
//        // AIO -> Analog mode selected
//        GPIO_setAnalogMode(227, GPIO_ANALOG_ENABLED);

        // VDCN
        // Analog PinMux for B0/C11
        GPIO_setPinConfig(GPIO_253_GPIO253);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(253, GPIO_ANALOG_ENABLED);

        // Not used, commented out
        // Analog PinMux for B11
//        GPIO_setPinConfig(GPIO_251_GPIO251);
//        // AIO -> Analog mode selected
//        GPIO_setAnalogMode(251, GPIO_ANALOG_ENABLED);
//        // Analog PinMux for B11, GPIO21
//        GPIO_setPinConfig(GPIO_21_GPIO21);
        // AGPIO -> Analog mode selected
        // On 100PZ package for F28003x, if both "B5, GPIO20" and "B5" are selected as analog,
        // "B5, GPIO20" will drive the ADC instead of "B5"
        // On 100PZ package for F28003x, if both "B11, GPIO21" and "B11" are selected as analog,
        // "B11, GPIO21" will drive the ADC instead of "B11"
//        GPIO_setAnalogMode(21, GPIO_ANALOG_ENABLED);

        // VAC2- Local B to N in Split Phase, Local B to C in 3 Phase
        // Analog PinMux for B2/C6
        GPIO_setPinConfig(GPIO_226_GPIO226);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(226, GPIO_ANALOG_ENABLED);

        // ISENS_OUT5 - IB2
        // Analog PinMux for B3/VDAC
        GPIO_setPinConfig(GPIO_242_GPIO242);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(242, GPIO_ANALOG_ENABLED);

        // VAC5- Remote B to N in split phase and 3 phase
        // Analog PinMux for B4/C8
        GPIO_setPinConfig(GPIO_236_GPIO236);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(236, GPIO_ANALOG_ENABLED);

        // HS_Temp2
        // Analog PinMux for B5
        GPIO_setPinConfig(GPIO_252_GPIO252);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(252, GPIO_ANALOG_ENABLED);
        // Analog PinMux for B5, GPIO20


        GPIO_setPinConfig(GPIO_20_GPIO20);
        // AGPIO -> Analog mode selected
        // On 100PZ package for F28003x, if both "B5, GPIO20" and "B5" are selected as analog,
        // "B5, GPIO20" will drive the ADC instead of "B5"
        // On 100PZ package for F28003x, if both "B11, GPIO21" and "B11" are selected as analog,
        // "B11, GPIO21" will drive the ADC instead of "B11"
        //GPIO_setAnalogMode(20, GPIO_ANALOG_ENABLED);
        // Analog PinMux for C1
//        GPIO_setPinConfig(GPIO_248_GPIO248);
//        // AIO -> Analog mode selected
//        GPIO_setAnalogMode(248, GPIO_ANALOG_ENABLED);

        // Not used, commented out
        // Analog PinMux for C14
//        GPIO_setPinConfig(GPIO_247_GPIO247);
//        // AIO -> Analog mode selected
//        GPIO_setAnalogMode(247, GPIO_ANALOG_ENABLED);

        // VAC3- Local Phase B to A in Split Phase and 3 Phase
        // Analog PinMux for C2/B12
        GPIO_setPinConfig(GPIO_244_GPIO244);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(244, GPIO_ANALOG_ENABLED);

        // ISENS_OUT6 - IC2
        // Analog PinMux for C3/A7
        GPIO_setPinConfig(GPIO_245_GPIO245);
        // AIO -> Analog mode selected
        GPIO_setAnalogMode(245, GPIO_ANALOG_ENABLED);

}

