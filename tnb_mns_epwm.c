/*
 * tnb_mns_epwm.c
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#include "tnb_mns_epwm.h"
#include "driverlib.h"
#include "device.h"


//sets up the pinmux and config for a buck stage
void setup_pin_config_buck(struct buck_configuration config){
    //----- Buck
    //Buck Enable
    GPIO_setDirectionMode(config.enable_gpio, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(config.enable_gpio,GPIO_PIN_TYPE_STD);        //push pull output
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Buck State
    GPIO_setDirectionMode(config.state_gpio,GPIO_DIR_MODE_IN);     //input
    GPIO_setPadConfig(config.state_gpio,GPIO_PIN_TYPE_STD);        //floating input
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Bridge H
    GPIO_setDirectionMode(config.bridge_h_pin,GPIO_DIR_MODE_OUT);     //output
    GPIO_setPinConfig(config.bridge_h_pinconfig);
    //Bridge L
    GPIO_setDirectionMode(config.bridge_l_pin,GPIO_DIR_MODE_OUT);     //output
    GPIO_setPinConfig(config.bridge_l_pinconfig);
    //PWM Setup
    setup_epwm_buck(config.epwmbase);
}

//sets up the pinmux and config for a bridge stage
void setup_pin_config_bridge(struct bridge_configuration config){
    //----- Bridge U
    //Bridge Enable
    GPIO_setDirectionMode(config.enable_gpio, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(config.enable_gpio,GPIO_PIN_TYPE_STD);        //push pull output
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Bridge State U
    GPIO_setDirectionMode(config.state_u_gpio,GPIO_DIR_MODE_IN);     //input
    GPIO_setPadConfig(config.state_u_gpio,GPIO_PIN_TYPE_STD);        //floating input
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Bridge State V
    GPIO_setDirectionMode(config.state_v_gpio,GPIO_DIR_MODE_IN);     //input
    GPIO_setPadConfig(config.state_v_gpio,GPIO_PIN_TYPE_STD);        //floating input
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Bridge H
    GPIO_setDirectionMode(config.bridge_h_pin,GPIO_DIR_MODE_OUT);     //output
    GPIO_setPinConfig(config.bridge_h_pinconfig);
    //Bridge L
    GPIO_setDirectionMode(config.bridge_l_pin,GPIO_DIR_MODE_OUT);     //output
    GPIO_setPinConfig(config.bridge_l_pinconfig);
    //PWM Setup
    setup_epwm_bridge(config.epwmbase);
}

void setup_epwm_buck(uint32_t base){
    initEPWMWithoutDB(base);
    setupEPWMActiveHighComplementary(base);
    //clock prescaling results in a PWM clock of around 50kHz
    EPWM_setClockPrescaler(base,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    set_duty_buck(&cha_buck,0.5);
}

void setup_epwm_bridge(uint32_t base){
    initEPWMWithoutDB(base);
    setupEPWMActiveHighComplementary(base);
    //clock prescaling results in a PWM clock of around 50kHz
    EPWM_setClockPrescaler(base,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
}

void set_duty_buck(const struct buck_configuration* config, float duty){
    uint16_t duty_int=duty*EPWM_TIMER_TBPRD_BUCK;
    EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_A, duty_int);
    EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_B, EPWM_TIMER_TBPRD_BUCK-duty_int);
}

void initEPWMWithoutDB(uint32_t base)
{
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, EPWM_TIMER_TBPRD_BUCK);
    EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0U);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(base,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(base,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, EPWM_TIMER_TBPRD_BUCK/4);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, 3*EPWM_TIMER_TBPRD_BUCK/4);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);


    EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

}

void setupEPWMActiveHighComplementary(uint32_t base)
{
    //
    // Use EPWMA as the input for both RED and FED
    //
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);

    //
    // Set the RED and FED values
    //
    EPWM_setFallingEdgeDelayCount(base, 25);
    EPWM_setRisingEdgeDelayCount(base, 25);

    //
    // Invert only the Falling Edge delayed output (AHC)
    //
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    //
    // Use the delayed signals instead of the original signals
    //
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);

    //
    // DO NOT Switch Output A with Output B
    //
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);

}
