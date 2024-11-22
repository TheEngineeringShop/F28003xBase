//#############################################################################
//  Originally -
// FILE:   cla_ex5_adc_just_in_time_shared.h
// TITLE:  Just-in-time ADC sampling with CLA
// This header file contains defines, variables and prototypes that are shared
// among the C28x and the CLA
//
//#############################################################################
// $Copyright:
// Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef CLA_JIT_SHARED_H_
#define CLA_JIT_SHARED_H_

#ifdef __cplusplus
extern "C" {
#endif

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

// Defines
#define EPWM1_FREQ          40000UL   // 40 kHz
#define PWM_SEC_PERIOD      .000025    // hard coded for loop efficiency, make sure to change this when you change EPWM1 Freq
#define EPWM1_PERIOD        (uint16_t)(DEVICE_SYSCLK_FREQ / (EPWM1_FREQ))/2 // /2 because of updown
#define SINEFREQ            60
#define ANGLESTEP           ((float)SINEFREQ/((float)EPWM1_FREQ))
//#define EPWM1_PERIOD        1500    // used for up down mode
#define PHASE180            (uint16_t) EPWM1_PERIOD
//#define EPWM_PERIOD       3000UL    // used for up or down mode
#define EPWM_DT_CONST       50U       // used to set deadtime within EPWM registers provides 400ns
#define RMS_BUF_LENGTH      667       // 1 Line cycle in switching periods, 40000/60 = 667

// Globals
typedef struct{
   int16_t VDCP_Offset;
   int16_t Ia1_Offset;
   int16_t Van_Offset;
   int16_t VanRemote_Offset;
   int16_t Ia2_Offset;
   int16_t HStemperature1_Offset;
   int16_t VDCN_Offset;
   int16_t Ib1_Offset;
   int16_t Vbn_Offset;
   int16_t Ib2_Offset;
   int16_t VbnRemote_Offset;
   int16_t HStemperature2_Offset;
   int16_t IDC_Offset;
   int16_t In1_Offset;
   int16_t Vba_Offset;
   int16_t In2_Offset;
   int16_t VbaRemote_Offset;
   int16_t AmbientTemperature_Offset;

   float VDCP_Gain;
   float Ia1_Gain;
   float Van_Gain;
   float VanRemote_Gain;
   float Ia2_Gain;
   float VDCN_Gain;
   float Ib1_Gain;
   float Vbn_Gain;
   float Ib2_Gain;
   float VbnRemote_Gain;
   float IDC_Gain;
   float In1_Gain;
   float Vba_Gain;
   float VbaRemote_Gain;
   float In2_Gain;

} Scal;

typedef struct{
    float VDCP;
    float Ia1;
    float Van;
    float VanRemote;
    float Ia2;
    float HStemperature1;

    float VDCN;
    float Ib1;
    float Vbn;
    float Ib2;
    float VbnRemote;
    float HStemperature2;

    float IDC;
    float In1;
    float Vba;
    float In2;
    float VbaRemote;
    float AmbientTemperature;

    float IaAvg;
    float IbAvg;
    float InAvg;

    float IstarA;
    float IstarB;
    float DstarA;
    float DstarB;

    float VdcTotal;
    float IDCFilt;
} Sresults;

typedef struct{
    float VDC;
    float VanRMS;
    float VbnRMS;
    float IaAvgRMS;
    float IbAvgRMS;
    float IDC;
    float DCpower;
} SRMS;


// Variables passed between the 2
extern float sinewave;
extern Scal jitCal;


extern float claTest;
extern Sresults InverterValues;
extern float inverseVDC;
extern int stateMachineState;
extern int amPrecharging;
extern int amRunning;
extern int inverterRunning;
extern SRMS forScreen;
extern float sinPLL;
extern float VehicleVDCcommand;
extern int PrechargeComplete;
extern float DCVerror;


//Task 1 (C) Variables

//Task 2 (C) Variables

//Task 3 (C) Variables

//Task 4 (C) Variables

//Task 5 (C) Variables

//Task 6 (C) Variables

//Task 7 (C) Variables

//Task 8 (C) Variables

//Common (C) Variables


// Function Prototypes
__attribute__((interrupt))  void Cla1Task1();
__attribute__((interrupt))  void Cla1Task2();
__attribute__((interrupt))  void Cla1Task3();
__attribute__((interrupt))  void Cla1Task4();
__attribute__((interrupt))  void Cla1Task5();
__attribute__((interrupt))  void Cla1Task6();
__attribute__((interrupt))  void Cla1Task7();
__attribute__((interrupt))  void Cla1Task8();

#ifdef __cplusplus
}
#endif // extern "C"
#endif //CLA_JIT_SHARED_H_
