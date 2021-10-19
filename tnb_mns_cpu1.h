/*
 * tnb_mns_cpu1.h
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#ifndef TNB_MNS_CPU1_H_
#define TNB_MNS_CPU1_H_

#include <stdint.h>
#include "driverlib.h"
#include "device.h"
#include "tnb_mns_cpu1.h"

struct buck_configuration{
    uint32_t enable_gpio;
    uint32_t state_gpio;
    uint32_t bridge_h_pin;
    uint32_t bridge_h_pinconfig;
    uint32_t bridge_l_pin;
    uint32_t bridge_l_pinconfig;
    uint32_t epwmbase;
};

struct bridge_configuration{
    uint32_t enable_gpio;
    uint32_t state_u_gpio;
    uint32_t state_v_gpio;
    uint32_t bridge_h_pin;
    uint32_t bridge_h_pinconfig;
    uint32_t bridge_l_pin;
    uint32_t bridge_l_pinconfig;
    uint32_t epwmbase;
};

struct system_dynamic_state{
    float ia;
    float ib;
    float ic;
    float va;
    float vb;
    float vc;
};

extern struct buck_configuration cha_buck;
extern struct buck_configuration chb_buck;
extern struct buck_configuration chc_buck;

extern struct bridge_configuration cha_bridge;
extern struct bridge_configuration chb_bridge;
extern struct bridge_configuration chc_bridge;

// ---------------------
// Main Program related globals
// ---------------------
extern bool run_main_task;      //variable is set by CPU1 ISR
extern struct system_dynamic_state system_dyn_state;
extern uint32_t enable_res_cap_a;     //variable control the resonant relay of channel a, if it is set to 1, res cap switched in
extern uint32_t enable_res_cap_b;     //variable control the resonant relay of channel b, if it is set to 1, res cap switched in
extern uint32_t enable_res_cap_c;     //variable control the resonant relay of channel c, if it is set to 1, res cap switched in
extern uint32_t enable_buck_a;        //variable to enable buck stage of channel a
extern uint32_t enable_buck_b;        //variable to enable buck stage of channel b
extern uint32_t enable_buck_c;        //variable to enable buck stage of channel c
extern uint32_t enable_bridge_a;      //variable to enable bridge state of channel a
extern uint32_t enable_bridge_b;      //variable to enable bridge state of channel a
extern uint32_t enable_bridge_c;      //variable to enable bridge state of channel a
#define ENABLE_RES_CAP_A_GPIO 78
#define ENABLE_RES_CAP_B_GPIO 80
#define ENABLE_RES_CAP_C_GPIO 82
#define ENABLE_BUCK_A_GPIO 40
#define ENABLE_BUCK_B_GPIO 35
#define ENABLE_BUCK_C_GPIO 95
#define ENABLE_BRIDGE_A_GPIO 30
#define ENABLE_BRIDGE_B_GPIO 63
#define ENABLE_BRIDGE_C_GPIO 107

// ---------------------
// Main CPU Timer Related Functions
// ---------------------

//
// Globals
//
extern uint16_t cpuTimer0IntCount;
//extern uint16_t cpuTimer1IntCount;
//extern uint16_t cpuTimer2IntCount;

__interrupt void cpuTimer0ISR(void);
void initCPUTimers(void);
void configCPUTimer(uint32_t, uint32_t);


#endif /* TNB_MNS_CPU1_H_ */
