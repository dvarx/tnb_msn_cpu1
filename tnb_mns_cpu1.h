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
#include "stdint.h"

#define NO_CHANNELS 3

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
    bool resonant_active;
};

struct system_dynamic_state{
    float ia;
    float ib;
    float ic;
    float va;
    float vb;
    float vc;
};

enum driver_channel_state {READY=0,BUCK_ENABLED=1,INIT_REGULAR=2,RUN_REGULAR=3,INIT_RESONANT=4,RUN_RESONANT=5,FAULT=6};

struct driver_channel{
    uint8_t channel_no;
    struct buck_configuration* buck_config;
    struct bridge_configuration* bridge_config;
    enum driver_channel_state channel_state;
};

extern struct buck_configuration cha_buck;
extern struct buck_configuration chb_buck;
extern struct buck_configuration chc_buck;
extern struct driver_channel channela;
extern struct driver_channel channelb;
extern struct driver_channel channelc;
extern struct driver_channel* driver_channels[NO_CHANNELS];

extern struct bridge_configuration cha_bridge;
extern struct bridge_configuration chb_bridge;
extern struct bridge_configuration chc_bridge;

// ---------------------
// Main Program related globals
// ---------------------

extern bool run_main_task;      //variable is set by CPU1 ISR
extern struct system_dynamic_state system_dyn_state;
extern double duty_bridge_a;
extern double duty_bridge_b;
extern double duty_bridge_c;
extern double duty_buck_a;
extern double duty_buck_b;
extern double duty_buck_c;
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
extern uint32_t freq_cha_resonant_mhz;
extern uint32_t freq_chb_resonant_mhz;
extern uint32_t freq_chc_resonant_mhz;

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
