/*
 * tnb_mns_epwm.h
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#ifndef TNB_MNS_EPWM_H_
#define TNB_MNS_EPWM_H_

#include "tnb_mns_cpu1.h"
#include <stdint.h>

#define EPWM_TIMER_TBPRD_BUCK       512UL
#define EPWM_TIMER_TBPRD_BRIDGE     1024UL

//initializes pins & pads for buck half bridge
void setup_pin_config_buck(const struct buck_configuration*);
//intiializes pins & pads for bridge half bridges
void setup_pinmux_config_bridge(const struct bridge_configuration* config);
//helper function
void init_epwm(uint32_t,bool);
//helper function
void setup_epwm_deadband(uint32_t);
//set the duty cycle of a buck half bridge
void set_duty_buck(const struct buck_configuration*,double);
//set pwm frequency of bridge
void set_freq_bridge(const unsigned int i,const uint32_t);
//set the duty cycle of bridge half bridegs
void set_duty_bridge(const struct bridge_configuration*,double);
//enables or disables the associated half bridge
void set_enabled(void*,bool,bool);
//function used to set the phases of each channel
void setup_phase_control(struct driver_channel** channels,float phase1_in, float phase2_in);
//synchronizes the epwm counter of channel <channel_to_sync> to the epwm counter of channel 0
void synchronize_pwm_tochannel0(struct driver_channel** channels, const unsigned int channel_to_sync);
//unsynchronize the epwm counter of channel <channel_to_sync> to epwm counter of channel 0
void unsynchronize_pwm_tochannel0(struct driver_channel** channels, const unsigned int channel_to_sync);

#endif /* TNB_MNS_EPWM_H_ */
