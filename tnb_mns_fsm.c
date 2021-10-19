/*
 * tnb_mns_fsm.c
 *
 *  Created on: Oct 19, 2021
 *      Author: dvarx
 */

#include "tnb_mns_fsm.h"
#include "tnb_mns_cpu1.h"
#include <stdint.h>

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
unsigned int init_regular_counter=0;
//Lookup Array For Exit Functions
fsm_function fsm_exit_functions[]={READY_exit,BUCK_ENABLED_exit,INIT_REGULAR_RUN_exit,RUNNING_REGULAR_exit};

void run_channel_fsm(struct driver_channel* channel){
    uint8_t no=channel->channel_no;
    if(fsm_req_flags_stop[no]){
        //call the exit function of the currently active state
        fsm_exit_functions[channel->channel_state](no);
        channel->channel_state=READY;
    }
    switch(channel->channel_state){
        READY:
            if(fsm_req_flags_en_buck[channel->channel_no]){
                READY_exit(no);
                BUCK_ENABLED_enter(no);
                channel->channel_state=BUCK_ENABLED;
            }
            else{
                READY_during(no);
            }
        BUCK_ENABLED:
            if(fsm_req_flags_run_regular[channel->channel_no]){
                BUCK_ENABLED_exit(no);
                INIT_REGULAR_RUN_enter(no);
                channel->channel_state=INIT_REGULAR;
            }
            else if(fsm_req_flags_run_resonant[channel->channel_no]){
                BUCK_ENABLED_exit(no);
                //
            }
            else{
                BUCK_ENABLED_during(no);
            }
        INIT_REGULAR:
            if(init_regular_counter<10)
                INIT_REGULAR_RUN_during(no);
            else{
                INIT_REGULAR_RUN_exit(no);
                RUNNING_REGULAR_enter(no);
                channel->channel_state=RUN_REGULAR;
            }
        RUN_REGULAR:
        INIT_RESONANT:
        RESONANT:
        FAULT:
    }

    //reset the fsm request flags
    fsm_req_flags_en_buck[no]=false;
    fsm_req_flags_run_regular[no]=false;
    fsm_req_flags_run_resonant[no]=false;
    fsm_req_flags_stop[no]=false;
}

// ---------------------------------
// Channel FSM structs defining FSM functions ENTER,DURING,EXIT
// ---------------------------------
void READY_enter(uint8_t channelno){return;}
void READY_during(uint8_t channelno){return;}
void READY_exit(uint8_t channelno){return;}
//BUCK_ENABLED state
void BUCK_ENABLED_enter(uint8_t channelno){return;}
void BUCK_ENABLED_during(uint8_t channelno){return;}
void BUCK_ENABLED_exit(uint8_t channelno){return;}
//INIT_REGULAR_RUN state
void INIT_REGULAR_RUN_enter(uint8_t channelno){init_regular_counter=0;}
void INIT_REGULAR_RUN_during(uint8_t channelno){return;}
void INIT_REGULAR_RUN_exit(uint8_t channelno){return;}
//RUNNING_REGULAR state
void RUNNING_REGULAR_enter(uint8_t channelno){return;}
void RUNNING_REGULAR_during(uint8_t channelno){return;}
void RUNNING_REGULAR_exit(uint8_t channelno){return;}
