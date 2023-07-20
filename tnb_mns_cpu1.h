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
#include "fbctrl.h"
#include "comm_interface.h"
#include "tnb_mns_adc.h"

#define NO_CHANNELS 3
#define HEARTBEAT_GPIO 35 // pin 121 on breakoutboard
#define SAMPLING_GPIO 99 // pin 96 on breakoutboard
#define MAIN_RELAY_GPIO 92
#define SLAVE_RELAY_GPIO 62
#define DEFAULT_RES_FREQ_MILLIHZ    10000000
#define MINIMUM_RES_FREQ_MILLIHZ    80000
#define LED_1_GPIO 31
#define LED_2_GPIO 34
#define COMMUNICATION_TIMEOUT_MS    500
//define the pin polarity of the voltage needed to enable the ate driver
#define DRIVER_ENABLE_POLARITY 0
#define DRIVER_DISABLE_POLARITY 1

struct buck_configuration{
    uint32_t enable_gpio;
    uint32_t state_gpio;
    uint32_t bridge_h_pin;
    uint32_t bridge_h_pinconfig;
    uint32_t bridge_l_pin;
    uint32_t bridge_l_pinconfig;
    uint32_t epwmbase;
    bool is_inverted;
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
    bool is_inverted;
};

struct system_dynamic_state{
    float is[NO_CHANNELS];
    float vs[NO_CHANNELS];
    float is_res[NO_CHANNELS];
};

enum driver_channel_state {READY=0,BUCK_ENABLED=1,INIT_REGULAR=2,RUN_REGULAR=3,INIT_RESONANT=4,RUN_RESONANT=5,FAULT=6,TERMINATE_RESONANT=7,TERMINATE_REGULAR=8};

struct driver_channel{
    uint8_t channel_no;
    struct buck_configuration* buck_config;
    struct bridge_configuration* bridge_config;
    enum driver_channel_state channel_state;
    uint32_t enable_resonant_gpio;
};

extern struct buck_configuration cha_buck;
extern struct buck_configuration chb_buck;
extern struct buck_configuration chc_buck;
extern struct buck_configuration chd_buck;
extern struct buck_configuration che_buck;
extern struct buck_configuration chf_buck;
extern struct driver_channel channela;
extern struct driver_channel channelb;
extern struct driver_channel channelc;
extern struct driver_channel channeld;
extern struct driver_channel channele;
extern struct driver_channel channelf;
extern struct driver_channel* driver_channels[NO_CHANNELS];

extern struct bridge_configuration cha_bridge;
extern struct bridge_configuration chb_bridge;
extern struct bridge_configuration chc_bridge;
extern struct bridge_configuration chd_bridge;
extern struct bridge_configuration che_bridge;
extern struct bridge_configuration chf_bridge;

// ---------------------
// Main Program related globals
// ---------------------

extern bool run_main_task;                              //variable is set by CPU1 ISR
extern struct system_dynamic_state system_dyn_state;
extern float des_duty_bridge[NO_CHANNELS];             //desired duties for bridges, set by COMM interface
extern float des_duty_buck[NO_CHANNELS];               //desired duties for bucks, set by COMM interface
extern float des_currents[NO_CHANNELS];
extern uint32_t des_freq_resonant_mhz[NO_CHANNELS];     //desired frequencies for resonant bridges, set by COMM interface
extern struct first_order des_duty_buck_filt[NO_CHANNELS];
extern struct pi_controller current_pi[NO_CHANNELS];
extern struct tnb_mns_msg ipc_tnb_mns_msg;
extern bool communication_active;                       //variable indicates whether there is a TCP connection active (true if a package was received in the last 200ms)

//variables related to resonant control
extern float fres;
extern float actvolts[3];
extern float actthetas[3];
//debugging purposes -----------------------
extern float actvolts_[3];
extern float actthetas_[3];
//------------------------------------------
extern float periodstart;               //start of current point of oscillation
//definition of the system impedance matrix
extern float zmatr[NO_CHANNELS][NO_CHANNELS];
extern float zmati[NO_CHANNELS][NO_CHANNELS];
//input to impedance matrix, corresponds
extern struct pi_controller ctrl_i_ds[3];
extern struct pi_controller ctrl_i_qs[3];
extern float xvecd[NO_CHANNELS];
extern float xvecq[NO_CHANNELS];
extern float rvecd[NO_CHANNELS];
extern float rvecq[NO_CHANNELS];
extern float vvecd[NO_CHANNELS];
extern float vvecq[NO_CHANNELS];
inline void matmul2(const float mat[2][2],const float vec[2], float res[2]){
    res[0]=mat[0][0]*vec[0]+mat[0][1]*vec[1];
    res[1]=mat[1][0]*vec[0]+mat[1][1]*vec[1];
}
inline void matmul3(const float mat[3][3],const float vec[3], float res[3]){
    res[0]=mat[0][0]*vec[0]+mat[0][1]*vec[1]+mat[0][2]*vec[2];
    res[1]=mat[1][0]*vec[0]+mat[1][1]*vec[1]+mat[1][2]*vec[2];
    res[2]=mat[2][0]*vec[0]+mat[2][1]*vec[1]+mat[2][2]*vec[2];
}
// ---------------------
// Main CPU Timer Related Functions
// ---------------------

//
// Globals
//
extern uint32_t mastercounter;
extern float mastertime;
//extern uint16_t cpuTimer1IntCount;
//extern uint16_t cpuTimer2IntCount;

//resonant control related
#define ADC_BUF_SIZE 1024
//extern float adc_buffer[7][ADC_BUF_SIZE];
extern float* adc_buffer[7];
extern float* adc_buffer_idq[6];
extern uint16_t adc_buffer_cnt;
extern uint16_t buffer_idq_cnt;
extern uint16_t act_volt_buffer_cnt;
extern uint16_t buffer_prdstrt_pointer_0;      //pointer that points to the sample at the beginning of the latest oscillation period in the ADC buffer
extern uint16_t buffer_prdstrt_pointer_1;      //pointer that points to the sample at the beginning of the last oscillation period in the ADC buffer
extern bool adc_record;

__interrupt void cpuTimer0ISR(void);
__interrupt void IPC_ISR0(void);
__interrupt void cpuTimer1ISR(void);
void initCPUTimers(void);
void configCPUTimer(uint32_t, uint32_t);


#endif /* TNB_MNS_CPU1_H_ */
