#include "main.h"


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
//    CANAInit();
//    MCANInit();
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
    startupMsg();   // send a message indicating the MCAN comms lines are up.

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

    if ( cpuTimer0IntCount % E_INT_5S == 0         // 50ms timer * 100 = 5 second interval
            &&
            DispenserMsgs.IoTechaOnlineTimestamp != cpuTimer0IntCount )
    {
            if ( (DispenserMsgs.IoTechaOnlineTimestamp - cpuTimer0IntCount) >= E_INT_5S )  // if we have not heard from it in more than 5 seconds, mark it dead)
            {
                DispenserMsgs.IoTechaOnlineTimestamp = cpuTimer0IntCount;
                DispenserMsgs.IoTechaOnline = false;
            }
    }

}
