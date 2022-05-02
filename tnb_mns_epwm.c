/*
 * tnb_mns_epwm.c
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#include "tnb_mns_epwm.h"
#include "driverlib.h"
#include "device.h"
#include "stdint.h"
#include "tnb_mns_cpu1.h"
#include <stdbool.h>

//sets up the pinmux and config for a buck stage
void setup_pin_config_buck(const struct buck_configuration* config){
    //----- Buck
    //Buck Enable
    GPIO_setDirectionMode(config->enable_gpio, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(config->enable_gpio,GPIO_PIN_TYPE_STD);        //push pull output
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Buck State
    GPIO_setDirectionMode(config->state_gpio,GPIO_DIR_MODE_IN);     //input
    GPIO_setPadConfig(config->state_gpio,GPIO_PIN_TYPE_STD);        //floating input
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Bridge H
    GPIO_setDirectionMode(config->bridge_h_pin,GPIO_DIR_MODE_OUT);     //output
    GPIO_setPinConfig(config->bridge_h_pinconfig);
    //Bridge L
    GPIO_setDirectionMode(config->bridge_l_pin,GPIO_DIR_MODE_OUT);     //output
    GPIO_setPinConfig(config->bridge_l_pinconfig);
    //PWM Setup
    initEPWMWithoutDB(config->epwmbase,true);
    setupEPWMActiveHighComplementary(config->epwmbase);
    //clock prescaling results in a PWM clock of around 50kHz
    EPWM_setClockPrescaler(config->epwmbase,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //PWM Setup to ~ 50kHz 50% duty
    set_duty_buck(config,0.5);
}

//sets up the pinmux and epwm module config for a bridge stage. duty=50% , freq~48kHz
void setup_pinmux_config_bridge(const struct bridge_configuration* config){
    //----- Bridge U
    //Bridge Enable
    GPIO_setDirectionMode(config->enable_gpio, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(config->enable_gpio,GPIO_PIN_TYPE_STD);        //push pull output
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Bridge State U
    GPIO_setDirectionMode(config->state_u_gpio,GPIO_DIR_MODE_IN);     //input
    GPIO_setPadConfig(config->state_u_gpio,GPIO_PIN_TYPE_STD);        //floating input
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Bridge State V
    GPIO_setDirectionMode(config->state_v_gpio,GPIO_DIR_MODE_IN);     //input
    GPIO_setPadConfig(config->state_v_gpio,GPIO_PIN_TYPE_STD);        //floating input
    //a call to GPIO_setPinConfig(..) here is not necessary since default pin config is as GPIO
    //Bridge H
    GPIO_setDirectionMode(config->bridge_h_pin,GPIO_DIR_MODE_OUT);     //output
    GPIO_setPinConfig(config->bridge_h_pinconfig);
    //Bridge L
    GPIO_setDirectionMode(config->bridge_l_pin,GPIO_DIR_MODE_OUT);     //output
    GPIO_setPinConfig(config->bridge_l_pinconfig);
    //PWM Setup to ~ 50kHz 50% duty
    initEPWMWithoutDB(config->epwmbase,false);
    setupEPWMActiveHighComplementary(config->epwmbase);
    //clock prescaling results in a PWM clock of around 50kHz
    EPWM_setClockPrescaler(config->epwmbase,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
}

void set_duty_buck(const struct buck_configuration* config, double duty){
    uint16_t duty_int=duty*EPWM_TIMER_TBPRD_BUCK;
    //set the duty based on whether the channel has an inverted duty logic or not
    if(config->is_inverted){
        EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_A, EPWM_TIMER_TBPRD_BUCK-duty_int);
        EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_B, duty_int);
    }
    else{
        EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_A, duty_int);
        EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_B, EPWM_TIMER_TBPRD_BUCK-duty_int);
    }
}

void set_duty_bridge(const struct bridge_configuration* config, double duty){
    uint16_t duty_int=duty*EPWM_TIMER_TBPRD_BRIDGE;
    //set the duty based on whether the channel has an inverted duty logic or not
    if(config->is_inverted){
        EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_A, EPWM_TIMER_TBPRD_BRIDGE-duty_int);
        EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_B, duty_int);
    }
    else{
        EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_A, duty_int);
        EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_B, EPWM_TIMER_TBPRD_BRIDGE-duty_int);
    }
}

void setup_phase_control(struct driver_channel** channels,float phase1_in, float phase2_in){
    // TODO : Implement phase shift load and synchronization
    uint16_t phase1=2*EPWM_TIMER_TBPRD_BRIDGE*phase1_in;
    uint16_t phase2=2*EPWM_TIMER_TBPRD_BRIDGE*phase2_in;

    // -- Channel 0 Setup
    //enable sync output of EPWM of channel 0, sync output will be generated when timer reaches zero
    EPWM_enableSyncOutPulseSource(channels[0]->bridge_config->epwmbase,EPWM_SYNCOUTEN_ZEROEN);
    //channel 0 does not do a phase shift load
    EPWM_disablePhaseShiftLoad(channels[0]->bridge_config->epwmbase);
    //set phase shift register to zero
    EPWM_setPhaseShift(channels[0]->bridge_config->epwmbase, 0);
    // -- Channel 1 Setup
    //set the sync input source to EPWM7 (the EPWM generator of channel bridge 0)
    EPWM_setSyncInPulseSource(channels[1]->bridge_config->epwmbase,EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM7);
    //enable phase shift load for channel 1
    EPWM_enablePhaseShiftLoad(channels[1]->bridge_config->epwmbase);
    //set phase shift register
    EPWM_setPhaseShift(channels[1]->bridge_config->epwmbase, phase1);
    // -- Channel 2 Setup
    //set the sync input source to EPWM7 (the EPWM generator of channel bridge 0)
    EPWM_setSyncInPulseSource(channels[2]->bridge_config->epwmbase,EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM7);
    //enable phase shift load for channel 1
    EPWM_enablePhaseShiftLoad(channels[2]->bridge_config->epwmbase);
    //set phase shift register
    EPWM_setPhaseShift(channels[2]->bridge_config->epwmbase, phase2);
}

void initEPWMWithoutDB(uint32_t base,bool is_buck)
{
    //
    // Set-up TBCLK
    //
    if(is_buck)
        EPWM_setTimeBasePeriod(base, EPWM_TIMER_TBPRD_BUCK);
    else
        EPWM_setTimeBasePeriod(base, EPWM_TIMER_TBPRD_BRIDGE);
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
    if(is_buck){
        EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, EPWM_TIMER_TBPRD_BUCK/2);
        EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, EPWM_TIMER_TBPRD_BUCK/2);
    }
    else{
        EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, EPWM_TIMER_TBPRD_BRIDGE/2);
        EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, EPWM_TIMER_TBPRD_BRIDGE/2);
    }

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
    // TODO : The constant value of 40 here sets the interlock of the buck stage to around 400ns, might need to adjust / optimize this at some point
    EPWM_setFallingEdgeDelayCount(base, 40);
    EPWM_setRisingEdgeDelayCount(base, 40);

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

void set_enabled(void* config,bool is_buck,bool enable){
    if(is_buck){
        struct buck_configuration* config_=config;
        if(enable)
            GPIO_writePin(config_->enable_gpio,DRIVER_ENABLE_POLARITY);
        else
            GPIO_writePin(config_->enable_gpio,DRIVER_DISABLE_POLARITY);
    }
    else{
        struct bridge_configuration* config_=config;
        if(enable)
            GPIO_writePin(config_->enable_gpio,DRIVER_ENABLE_POLARITY);
        else
            GPIO_writePin(config_->enable_gpio,DRIVER_DISABLE_POLARITY);

    }
}

//set pwm frequency of bridge
void set_freq_bridge(const struct bridge_configuration* config,const uint32_t freq_mhz){
    /* General Formulas & Relationships (see also OneNote notes)
     *
     * Tpwm=1/fpwm
     * fEPWM=100MH(frequency going into the EPWM modules, can be divided down by clock dividers of EPWM module)
     *
     * For symmetrical EPWM (e.g. count up and down)
     * -----------------
     * countermax=TimeBasePeriod=Tpwm*fEPWM/(2*CLOCK_DIVIDER_1*CLOCK_DIVIDER_2)
     *
     * The maximum period for a set of clock dividers is given as
     * maxperiod=2*CLOCK_DIVIDER_1*CLOCK_DIVIDER_2*2^16/fepwm
     *
     * For non-symmetrical EPWM (e.g. dount up or count down)
     * -----------------
     * countermax=TimeBasePeriod=Tpwm*fEPWM/(CLOCK_DIVIDER_1*CLOCK_DIVIDER_2)
     */

    //the ePWM clock coming in is 100MHz
    /*
     * choosing the prescalers like this results in:
     *
     * fpwm_max=250kHz
     * fpwm_min=7.62Hz
     * TimeBasePeriod(3kHz)=1666
     * TimeBasePeriod(1kHz)=5000
     * TimeBasePeriod(100Hz)=50000
     */
    EPWM_setClockPrescaler(config->epwmbase, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    unsigned int counterlimit=10000000/(2*2*1000*freq_mhz);

    EPWM_setFallingEdgeDelayCount(config->epwmbase, 1);
    EPWM_setRisingEdgeDelayCount(config->epwmbase, 1);
    EPWM_setTimeBasePeriod(config->epwmbase, counterlimit);
    EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_A, counterlimit/2);
    EPWM_setCounterCompareValue(config->epwmbase, EPWM_COUNTER_COMPARE_B, counterlimit/2);
}
