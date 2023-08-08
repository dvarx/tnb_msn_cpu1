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
#include <math.h>

struct channel_fsm coil_fsm_states[NO_CHANNELS];

//number of control cycles system remains in TERMINATE_REGULAR state
const unsigned int TERMINATE_REGULAR_TIMEVAL=(unsigned int)(300);

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
            if(fsm_req_flags_stop[n]){
                //STOP BUCK_ENABLED
                BUCK_ENABLED_exit(n);
                READY_enter(n);
                channel->channel_state=READY;
                break;
            }
            else{
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
            }
    case INIT_REGULAR:
            /*
             * right now we do no react to a stop flag here, instead we enter the RUN_REGULAR state and then stop.
             */
            if(fsm_aux_counter<10)
                INIT_REGULAR_RUN_during(n);
            else{
                INIT_REGULAR_RUN_exit(n);
                RUNNING_REGULAR_enter(n);
                channel->channel_state=RUN_REGULAR;
            }
            break;
    case RUN_REGULAR:
            if(fsm_req_flags_stop[n]){
                //STOP RUN_REGULAR
                RUNNING_REGULAR_exit(n);
                TERMINATE_REGULAR_enter(n);
                channel->channel_state=TERMINATE_REGULAR;
                break;
            }
            else{
                RUNNING_REGULAR_during(n);
                break;
            }
    case INIT_RESONANT:
        /*
         * right now we do no react to a stop flag here, instead we enter the RUN_RESONANT state and then stop.
         */
            if(fsm_aux_counter<100)
                INIT_RESONANT_RUN_during(n);
            else{
                INIT_RESONANT_RUN_exit(n);
                RUNNING_RESONANT_enter(n);
                channel->channel_state=RUN_RESONANT;
            }
            break;
    case RUN_RESONANT:
        if(fsm_req_flags_stop[n]){
            //STOP RUN_RESONANT
            RUNNING_RESONANT_exit(n);
            TERMINATE_RESONANT_enter(n);
            channel->channel_state=TERMINATE_RESONANT;
            break;
        }
        else{
            RUNNING_RESONANT_during(n);
            break;
        }
    case FAULT: break;
    case TERMINATE_RESONANT:
            //we do not need to react to a stop flag here since it is already in the process of stopping
            if(fsm_aux_counter<1000)
                TERMINATE_RESONANT_during(n);
            else{
                TERMINATE_RESONANT_exit(n);
                READY_enter(n);
                channel->channel_state=READY;
            }
            break;
    case TERMINATE_REGULAR:
            //we do not need to react to a stop flag here since it is already in the process of stopping
            if(fsm_aux_counter<TERMINATE_REGULAR_TIMEVAL)
                TERMINATE_REGULAR_during(n);
            else{
                TERMINATE_REGULAR_exit(n);
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
    GPIO_writePin(driver_channels[channelno]->buck_config->enable_gpio,DRIVER_DISABLE_POLARITY);
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,DRIVER_DISABLE_POLARITY);
    //reset desBucks
    unsigned int i=0;
    for(i=0; i<NO_CHANNELS; i++){
        des_duty_buck[i]=0;
        des_currents[i]=0;
    }
}
void READY_during(uint8_t channelno){return;}
void READY_exit(uint8_t channelno){return;}
//BUCK_ENABLED state
void BUCK_ENABLED_enter(uint8_t channelno){
    //enable buck
    GPIO_writePin(driver_channels[channelno]->buck_config->enable_gpio,DRIVER_ENABLE_POLARITY);
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,DRIVER_DISABLE_POLARITY);
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
    GPIO_writePin(driver_channels[channelno]->buck_config->enable_gpio,DRIVER_ENABLE_POLARITY);
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,DRIVER_DISABLE_POLARITY);
}
void INIT_REGULAR_RUN_during(uint8_t channelno){
    fsm_aux_counter++;
}
void INIT_REGULAR_RUN_exit(uint8_t channelno){return;}
//RUNNING_REGULAR state
void RUNNING_REGULAR_enter(uint8_t channelno){
    //
    // Precompute sine/cosine/phase buffers
    //
    unsigned int i=0;
    //compute period_no
    period_no=round(1/(fres*deltaT));
    for(i=0; i<period_no; i++){
        float iflt=i;
        cosinebuf[i]=cos(deltaT*iflt*fres*2*M_PI);
        nsinebuf[i]=-sin(deltaT*iflt*fres*2*M_PI);
        phasebuf[i]=deltaT*iflt*fres*2*M_PI;
    }
    //reset pids
    for(i=0; i<NO_CHANNELS; i++){
        reset_pid(ctrl_i_ds+i);
        reset_pid(ctrl_i_qs+i);
    }

    //enable buck
    GPIO_writePin(driver_channels[channelno]->buck_config->enable_gpio,DRIVER_ENABLE_POLARITY);
    //configure & enable bridge
    setup_pinmux_config_bridge(driver_channels[channelno]->bridge_config);
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,DRIVER_ENABLE_POLARITY);
    //reset PID controller of the channel for regular mode here
    reset_pid(current_pi+channelno);
    return;
}
void RUNNING_REGULAR_during(uint8_t channelno){
    return;
}
void RUNNING_REGULAR_exit(uint8_t channelno){
    //set the duty to 50% in order to generate 0V across the coil so the current can die down
    //we then enter the TERMINATE_REGULAR mode wait a bit and finally disable the driver stage
    set_duty_bridge(driver_channels[channelno]->bridge_config,0.5);
}
//TERMINATE_REGULAR state
void TERMINATE_REGULAR_enter(uint8_t channelno){
    //reset counter
    fsm_aux_counter=0;
}
void TERMINATE_REGULAR_during(uint8_t channelno){
    fsm_aux_counter++;
}
void TERMINATE_REGULAR_exit(uint8_t channelno){
    //disable bridge before exiting the TERMINATE_REGULAR state
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,DRIVER_DISABLE_POLARITY);
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
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,DRIVER_ENABLE_POLARITY);
    // TODO-PID : reset PID controller for resonant current control here
}
void RUNNING_RESONANT_during(uint8_t channelno){
}
void RUNNING_RESONANT_exit(uint8_t channelno){
    //disable bridge
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,DRIVER_DISABLE_POLARITY);
}
//TERMINATE_RESONANT state
void TERMINATE_RESONANT_enter(uint8_t channelno){
    //reset counter
    fsm_aux_counter=0;
    //disable bridge so that energy can flow from resonant cap/inductor to DC link
    GPIO_writePin(driver_channels[channelno]->bridge_config->enable_gpio,DRIVER_DISABLE_POLARITY);
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
