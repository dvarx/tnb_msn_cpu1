/*
 * mdriver_epwm.h
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#ifndef MDRIVER_EPWM_H_
#define MDRIVER_EPWM_H_

#include <mdriver_cpu1.h>
#include <stdint.h>

#define EPWM_TIMER_TBPRD_BUCK       512UL
#define EPWM_TIMER_TBPRD_BRIDGE     128UL

//initializes pins & pads for buck half bridge
void setup_pin_config_buck(const struct buck_configuration*);
//intiializes pins & pads for bridge half bridges
void setup_pinmux_config_bridge(const struct bridge_configuration* config,uint8_t channelno);
//helper function
void initEPWMWithoutDB(uint32_t,bool);
//helper function
void setupEPWMActiveHighComplementary(uint32_t);
//set the duty cycle of a buck half bridge
void set_duty_buck(const struct buck_configuration*,double);
//set pwm frequency of bridge
void set_freq_bridge(const struct bridge_configuration*,const uint32_t);
//set the duty cycle of bridge half bridegs
void set_duty_bridge(const struct bridge_configuration*,double,uint8_t channelno);
//enables or disables the associated half bridge
void set_enabled(void*,bool,bool);

void synchronize_pwm_to_epwm12(struct driver_channel**, const unsigned int);
void init_trigger_epwm(uint32_t base);

#endif /* MDRIVER_EPWM_H_ */
