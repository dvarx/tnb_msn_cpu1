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

//current sensor calibration values
float isensoroffsets[6]={2048.47,2049.42,2052.62,2048,2048,2048};
float isensorgains[6]={-0.0076219958202716825,
                       -0.0076219958202716825*3.9/4.0,
                       -0.0076219958202716825*4.01/4.0,
                       -0.0076219958202716825,
                       -0.0076219958202716825,
                       -0.0076219958202716825};

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
    //  ADCA measures: [iD(A0),iG(A1),iE(A2)]
    //----------------------------------------------------------------
    #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0, 15);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN1, 15);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2, 15);
    #elif(EX_ADC_RESOLUTION == 16)
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0, 64);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN1, 64);
    #endif

    //
    // Set SOC2 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //----------------------------------------------------------------
    // ADCB Configuration
    //  ADCB measures: [iA(B0),iB(B4)]
    //----------------------------------------------------------------

    #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0, 15);
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN4, 15);
    #elif(EX_ADC_RESOLUTION == 16)
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0, 64);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN1, 64);
    #endif

    //
    // Set SOC4 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);


    //----------------------------------------------------------------
    // ADCC Configuration
    //  ADCC measures: [iH(C2),iF(C3)]
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



    //----------------------------------------------------------------
    // ADCD Configuration
    //  ADCD measures: [iC(D2),Ii(D3)]
    //----------------------------------------------------------------

    #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2, 15);
        ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
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
    ADC_setInterruptSource(ADCD_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCD_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
}

/*
 * calibration measurement is of the form m=alpha'*i+beta-0.280000001' => i=alpha*m+beta with alpha=1/alpha' and beta=-beta'/alpha
 *
 * we have a 12bit signed measurement, therefore the current is given by
 *
 * (ADC_OUT-2^11)*(1.5V/SENSOR_MAX_VOLT_OUTPUT)/2^11
 *
 * with SENSOR_MAX_VOLT_OUTPUT either 100mV/A or 50mV/A
 * and the 1.5V correspond to the reference voltage 3.0V/2
 */
float calib_factor_current_alpha=-1.5/0.05/2048;
float calib_factor_current_betas[]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

inline float conv_adc_meas_to_current_a(const uint16_t adc_output,unsigned int channelno){
    return calib_factor_current_alpha*(float)((int16_t)adc_output-(int16_t)2048)-calib_factor_current_betas[channelno];
}


#define BUFFER_NO 512
uint16_t buffer_i0s[BUFFER_NO];
uint16_t buffer_i1s[BUFFER_NO];
uint16_t buffer_i2s[BUFFER_NO];
float buffer_i0s_fl[BUFFER_NO];
float buffer_i1s_fl[BUFFER_NO];
float buffer_i2s_fl[BUFFER_NO];
uint16_t buffer_cnt=0;
// This function reads the analog inputs and stores them in the system_dyn_state structure
void readAnalogInputs(void){
    // ADC A Measurements -----------------------------------------------
    ADC_forceMultipleSOC(ADCA_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2));
    // Wait for ADCA to complete, then acknowledge flag
    // ADCA measures: [iD(A0),iG(A1),iE(A2)]
    while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.is[3] = conv_adc_meas_to_current_a(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0),3);
    system_dyn_state.is[6] = conv_adc_meas_to_current_a(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1),6);
    system_dyn_state.is[4] = conv_adc_meas_to_current_a(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2),4);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // ADC B Measurements -----------------------------------------------
    ADC_forceMultipleSOC(ADCB_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1));
    // Wait for ADCB to complete, then acknowledge flag
    // ADCB measures: [iA(B0),iB(B4)]
    while(ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.is[0] = conv_adc_meas_to_current_a(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0),0);
    system_dyn_state.is[1] = conv_adc_meas_to_current_a(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1),1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    // ADC C Measurements -----------------------------------------------
    ADC_forceMultipleSOC(ADCC_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2));
    // Wait for ADCC to complete, then acknowledge flag
    // ADCC measures: [iI(C2),iF(C3)]
    while(ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.is[8] = conv_adc_meas_to_current_a(ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0),7);
    system_dyn_state.is[5] = conv_adc_meas_to_current_a(ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1),5);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    // ADC D Measurements -----------------------------------------------
    ADC_forceMultipleSOC(ADCD_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1));
    // Wait for ADCD to complete, then acknowledge flag
    // ADCD measures: [iC(D2),iH(D3)]
    while(ADC_getInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.is[2] = conv_adc_meas_to_current_a(ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0),2);
    system_dyn_state.is[7] = conv_adc_meas_to_current_a(ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1),8);
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
}
