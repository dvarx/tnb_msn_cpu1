/*
 * tnb_mns_epwm.h
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#ifndef TNB_MNS_EPWM_H_
#define TNB_MNS_EPWM_H_

#include "tnb_mns_cpu1.h"

#define EPWM_TIMER_TBPRD_BUCK       1024UL
#define EPWM_TIMER_TBPRD_BRIDGE     1024UL

//initializes pins & pads for buck half bridge
void setup_pin_config_buck(const struct buck_configuration*);
//intiializes pins & pads for bridge half bridges
void setup_pin_config_bridge(const struct bridge_configuration* config);
//helper function
void initEPWMWithoutDB(uint32_t);
//helper function
void setupEPWMActiveHighComplementary(uint32_t);
//set the duty cycle of a buck half bridge
void set_duty_buck(const struct buck_configuration*,float);
//set the duty cycle of bridge half bridegs
void set_duty_bridge(const struct bridge_configuration*,float);
//enables or disables the associated half bridge
void set_enabled(void*,bool,bool);

#endif /* TNB_MNS_EPWM_H_ */
