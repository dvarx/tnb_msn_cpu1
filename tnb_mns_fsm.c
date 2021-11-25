/*
 * tnb_mns_fsm.c
 *
 *  Created on: Oct 19, 2021
 *      Author: dvarx
 */

#include "tnb_mns_fsm.h"
#include "tnb_mns_cpu1.h"
#include <stdint.h>
#include "tnb_mns_epwm.h"

struct channel_fsm coil_fsm_states[NO_CHANNELS];

// ---------------------------------
// FSM flags used to trigger FSM transitions
// ---------------------------------
fsm_flag fsm_req_flags_en_buck[NO_CHANNELS]={0};
fsm_flag fsm_req_flags_run_regular[NO_CHANNELS]={0};
fsm_flag fsm_req_flags_run_resonant[NO_CHANNELS]={0};
fsm_flag fsm_req_flags_stop[NO_CHANNELS];

// ---------------------------------
// Variables Used By Channel FSM
// ---------------------------------
unsigned int fsm_aux_counter=0;
//Lookup Array For Exit Functions
fsm_function fsm_exit_functions[]={READY_exit,BUCK_ENABLED_exit,INIT_REGULAR_RUN_exit,RUNNING_REGULAR_exit,INIT_RESONANT_RUN_exit,RUNNING_RESONANT_exit,TERMINATE_RESONANT_exit};

void run_channel_fsm(struct driver_channel* channel){
    uint8_t n=channel->channel_no;
    if(fsm_req_flags_stop[n]){
        //if channel in resonant mode we need to go to TERMINATE_RESONANT
        if(channel->channel_state==RUN_RESONANT){
            RUNNING_RESONANT_exit(n);
            TERMINATE_RESONANT_enter(n);
            channel->channel_state=TERMINATE_RESONANT;
        }
        //otherwise call the exit function of the currently active state and go to READY
        else{
            fsm_exit_functions[channel->channel_state](n);
            READY_enter(n);
            channel->channel_state=READY;
        }
    }
    switch(channel->channel_state){
    case READY:
            if(fsm_req_flags_en_buck[channel->channel_no]){
                READY_exit(n);
                BUCK_ENABLED_enter(n);
                channel->channel_state=BUCK_ENABLED;
            }
            else{
                READY_during(n);
            }
            break;
    case BUCK_ENABLED:
            if(fsm_req_flags_run_regular[channel->channel_no]){
                BUCK_ENABLED_exit(n);
                INIT_REGULAR_RUN_enter(n);
                channel->channel_state=INIT_REGULAR;
            }
            else if(fsm_req_flags_run_resonant[channel->channel_no]){
                BUCK_ENABLED_exit(n);
                INIT_RESONANT_RUN_enter(n);
                channel->channel_state=INIT_RESONANT;
            }
            else{
                BUCK_ENABLED_during(n);
            }
            break;
    case INIT_REGULAR:
            if(fsm_aux_counter<10)
                INIT_REGULAR_RUN_during(n);
            else{
                INIT_REGULAR_RUN_exit(n);
                RUNNING_REGULAR_enter(n);
                channel->channel_state=RUN_REGULAR;
            }
            break;
    case RUN_REGULAR:
            RUNNING_REGULAR_during(n);
            break;
    case INIT_RESONANT:
            if(fsm_aux_counter<100)
                INIT_RESONANT_RUN_during(n);
            else{
                INIT_RESONANT_RUN_exit(n);
                RUNNING_RESONANT_enter(n);
                channel->channel_state=RUN_RESONANT;
            }
            break;
    case RUN_RESONANT:
            RUNNING_RESONANT_during(n);
            break;
    case FAULT: break;
    case TERMINATE_RESONANT:
            if(fsm_aux_counter<1000)
                TERMINATE_RESONANT_during(n);
            else{
                TERMINATE_RESONANT_exit(n);
                READY_enter(n);
                channel->channel_state=READY;
            }
            break;
    }

    //reset the fsm request flags
    fsm_req_flags_en_buck[n]=false;
    fsm_req_flags_run_regular[n]=false;
    fsm_req_flags_run_resonant[n]=false;
    fsm_req_flags_stop[n]=false;
}

// ---------------------------------
// Channel FSM structs defining FSM functions ENTER,DURING,EXIT
// ---------------------------------
void READY_enter(uint8_t channelno){
    //disable buck
    GPIO_writePin(driver_channels[channelno]->buck_config->enable_gpio,0);
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,0);
}
void READY_during(uint8_t channelno){return;}
void READY_exit(uint8_t channelno){return;}
//BUCK_ENABLED state
void BUCK_ENABLED_enter(uint8_t channelno){
    //enable buck
    GPIO_writePin(driver_channels[channelno]->buck_config->enable_gpio,1);
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,0);
    //reset the desired buck duty cycle to zero
    reset_first_order(des_duty_buck_filt+channelno);
}
void BUCK_ENABLED_during(uint8_t channelno){
    return;
}
void BUCK_ENABLED_exit(uint8_t channelno){return;}
//INIT_REGULAR_RUN state
void INIT_REGULAR_RUN_enter(uint8_t channelno){
    //reset the init regular counter
    fsm_aux_counter=0;
    //enable buck
    GPIO_writePin(driver_channels[channelno]->buck_config->enable_gpio,1);
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,0);
}
void INIT_REGULAR_RUN_during(uint8_t channelno){
    fsm_aux_counter++;
}
void INIT_REGULAR_RUN_exit(uint8_t channelno){return;}
//RUNNING_REGULAR state
void RUNNING_REGULAR_enter(uint8_t channelno){
    //enable buck
    GPIO_writePin(driver_channels[channelno]->buck_config->enable_gpio,1);
    //configure & enable bridge
    setup_pinmux_config_bridge(driver_channels[channelno]->bridge_config);
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,1);
}
void RUNNING_REGULAR_during(uint8_t channelno){
    return;
}
void RUNNING_REGULAR_exit(uint8_t channelno){
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,0);
}
//INIT_RESONANT RUN state
void INIT_RESONANT_RUN_enter(uint8_t channelno){
    fsm_aux_counter=0;
    GPIO_writePin(driver_channels[channelno]->enable_resonant_gpio,1);
}
void INIT_RESONANT_RUN_during(uint8_t channelno){
    fsm_aux_counter++;
}
void INIT_RESONANT_RUN_exit(uint8_t channelno){return;}
//INIT_REGULAR_RUN state
void RUNNING_RESONANT_enter(uint8_t channelno){
    //configure & enable bridge
    setup_pinmux_config_bridge(driver_channels[channelno]->bridge_config);
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,1);
}
void RUNNING_RESONANT_during(uint8_t channelno){
}
void RUNNING_RESONANT_exit(uint8_t channelno){
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,0);
}
//TERMINATE_RESONANT state
void TERMINATE_RESONANT_enter(uint8_t channelno){
    //reset counter
    fsm_aux_counter=0;
    //disable bridge so that energy can flow from resonant cap/inductor to DC link
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,0);
}
void TERMINATE_RESONANT_during(uint8_t channelno){
    fsm_aux_counter++;
}
void TERMINATE_RESONANT_exit(uint8_t channelno){
    //pull the enable resonator line down, this will close the relay and bypass the capacitorbank
    GPIO_writePin(driver_channels[channelno]->enable_resonant_gpio,0);
    //reset the desired resonant frequency
    des_freq_resonant_mhz[channelno]=DEFAULT_RES_FREQ_MILLIHZ;
}
