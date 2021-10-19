/*
 * tnb_mns_fsm.c
 *
 *  Created on: Oct 19, 2021
 *      Author: dvarx
 */

#include "tnb_mns_fsm.h"
#include "tnb_mns_cpu1.h"

struct channel_fsm coil_fsm_states[NO_CHANNELS];

// ---------------------------------
// FSM flags used to trigger FSM transitions
// ---------------------------------
fsm_flag fsm_req_flags_en_buck[NO_CHANNELS]={0};
fsm_flag fsm_req_flags_run_regular[NO_CHANNELS]={0};
fsm_flag fsm_req_flags_run_resonant[NO_CHANNELS]={0};
fsm_flag fsm_req_disable[NO_CHANNELS];

void run_channel_fsm(struct driver_channel* channel){
    uint8_t no=channel->channel_no;
    switch(channel->channel_state){
    default:
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
        INIT_REGULAR:
        RUN_REGULAR:
        INIT_RESONANT:
        RESONANT:
        FAULT:
        return;
    }
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
