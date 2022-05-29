/*
 * tnb_mns_fsm.h
 *
 *  Created on: Oct 19, 2021
 *      Author: dvarx
 */

#ifndef TNB_MNS_FSM_H_
#define TNB_MNS_FSM_H_

#include <stdint.h>
#include "tnb_mns_cpu1.h"
#include <stdbool.h>
#include "fbctrl.h"

extern const unsigned int TERMINATE_REGULAR_TIMEVAL;

typedef void (*fsm_function)(uint8_t);
typedef bool fsm_flag;

void run_channel_fsm(struct driver_channel*);

struct channel_fsm{
    enum driver_channel_state state;
    fsm_function enter_function;
    fsm_function exit_function;
    fsm_function during_function;
};

// ---------------------------------
// FSM functions ENTER,DURING,EXIT
// ---------------------------------
//dummy function
inline void do_nothing(void){return;}
extern struct channel_fsm coil_fsm_states[NO_CHANNELS];
//READY state
void READY_enter(uint8_t);
void READY_during(uint8_t);
void READY_exit(uint8_t);
//BUCK_ENABLED state
void BUCK_ENABLED_enter(uint8_t);
void BUCK_ENABLED_during(uint8_t);
void BUCK_ENABLED_exit(uint8_t);
//INIT_REGULAR_RUN state
void INIT_REGULAR_RUN_enter(uint8_t);
void INIT_REGULAR_RUN_during(uint8_t);
void INIT_REGULAR_RUN_exit(uint8_t);
//RUNNING_REGULAR state
void RUNNING_REGULAR_enter(uint8_t);
void RUNNING_REGULAR_during(uint8_t);
void RUNNING_REGULAR_exit(uint8_t);
//INIT_RESONANT RUN state
void INIT_RESONANT_RUN_enter(uint8_t);
void INIT_RESONANT_RUN_during(uint8_t);
void INIT_RESONANT_RUN_exit(uint8_t);
//INIT_REGULAR_RUN state
void RUNNING_RESONANT_enter(uint8_t);
void RUNNING_RESONANT_during(uint8_t);
void RUNNING_RESONANT_exit(uint8_t);
//TERMINATE_RESONANT state
void TERMINATE_RESONANT_enter(uint8_t);
void TERMINATE_RESONANT_during(uint8_t);
void TERMINATE_RESONANT_exit(uint8_t);
//TERMINATE_REGULAR state
void TERMINATE_REGULAR_enter(uint8_t);
void TERMINATE_REGULAR_during(uint8_t);
void TERMINATE_REGULAR_exit(uint8_t);

// ---------------------------------
// FSM flags used to trigger FSM transitions
// ---------------------------------
extern fsm_flag fsm_req_flags_en_buck[NO_CHANNELS];
extern fsm_flag fsm_req_flags_run_regular[NO_CHANNELS];
extern fsm_flag fsm_req_flags_run_resonant[NO_CHANNELS];
extern fsm_flag fsm_req_flags_stop[NO_CHANNELS];

// ---------------------------------
// User Inputs set via Comm Interface
// ---------------------------------


#endif /* TNB_MNS_FSM_H_ */
