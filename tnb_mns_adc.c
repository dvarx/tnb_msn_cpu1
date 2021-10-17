/*
 * tnb_mns_adc.c
 *
 *  Created on: Oct 17, 2021
 *      Author: dvarx
 */

#include "driverlib.h"
#include "device.h"
#include "stdbool.h"
#include "tnb_mns_cpu1.h"

//
// Defines
//
#define EX_ADC_RESOLUTION       12
// 12 for 12-bit conversion resolution, which supports single-ended signaling
// Or 16 for 16-bit conversion resolution, which supports single-ended or
// differential signaling
#define EX_ADC_SIGNALMODE       "SINGLE-ENDED"
//"SINGLE-ENDED" for ADC_MODE_SINGLE_ENDED:
// Sample on single pin (VREFLO is the low reference)
// Or "Differential" for ADC_MODE_DIFFERENTIAL:
// Sample on pair of pins (difference between pins is converted, subject to
// common mode voltage requirements; see the device data manual)


//
// Function to configure and power up ADCs A,B,C,D
//
void initADCs(void)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCD_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCD_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(EX_ADC_RESOLUTION == 16)
    #if(EX_ADC_SIGNALMODE == "SINGLE-ENDED")
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);
    #elif(EX_ADC_SIGNALMODE == "DIFFERENTIAL")
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    #endif
#endif

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCD_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);
    ADC_enableConverter(ADCD_BASE);

    DEVICE_DELAY_US(1000);
}

//
// Function to configure SOCs 0 and 1 of ADCs A and C.
//
void initADCSOCs(void)
{
    //----------------------------------------------------------------
    // ADCA Configuration
    //  ADCA measures: [-]
    //----------------------------------------------------------------
/* ADCA not currently used
#if(EX_ADC_RESOLUTION == 12)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN0, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN1, 15);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN0, 64);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN1, 64);
#endif

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
*/
    //----------------------------------------------------------------
    // ADCB Configuration
    //  ADCB measures: [iA(B0),vA(B2),iB(B4),vB(B5)]
    //----------------------------------------------------------------

#if(EX_ADC_RESOLUTION == 12)
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN0, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN4, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN5, 15);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN0, 64);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN1, 64);
#endif

    //
    // Set SOC3 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER3);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //----------------------------------------------------------------
    // ADCC Configuration
    //  ADCC measures: [-]
    //----------------------------------------------------------------
/*  ADCC not currently used
#if(EX_ADC_RESOLUTION == 12)
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN3, 15);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 64);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN3, 64);
#endif

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
*/
    //----------------------------------------------------------------
    // ADCD Configuration
    //  ADCD measures: [iC(D2),vC(D3)]
    //----------------------------------------------------------------

#if(EX_ADC_RESOLUTION == 12)
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN3, 15);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 64);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN3, 64);
#endif

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
}

// This function reads the analog inputs and stores them in the system_dyn_state structure
void readAnalogInputs(void){
    // ADC B Measurements -----------------------------------------------

    // Initiate SOCs of ADCB
    ADC_forceMultipleSOC(ADCB_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3));
    // Wait for ADCA to complete, then acknowledge flag
    while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.ia = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
    system_dyn_state.va = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
    system_dyn_state.ib = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);
    system_dyn_state.vb = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // ADC D Measurements -----------------------------------------------
    // Initiate SOCs of ADCD
    ADC_forceMultipleSOC(ADCD_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1));
    // Wait for ADCD to complete, then acknowledge flag
    while(ADC_getInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1) == false){}
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
}
