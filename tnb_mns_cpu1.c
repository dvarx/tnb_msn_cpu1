/*
 * tnb_mns_cpu1.c
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#include "tnb_mns_cpu1.h"
#include "driverlib.h"
#include "device.h"

struct buck_configuration cha_buck={40,41,8,GPIO_8_EPWM5A,9,GPIO_9_EPWM5B,EPWM5_BASE};
struct bridge_configuration cha_bridge={30,22,23,12,GPIO_12_EPWM7A,13,GPIO_13_EPWM7B,EPWM7_BASE};

struct buck_configuration chb_buck={35,61,14,GPIO_14_EPWM8A,15,GPIO_15_EPWM8B,EPWM8_BASE};
struct bridge_configuration chb_bridge={63,61,65,6,GPIO_6_EPWM4A,7,GPIO_7_EPWM4B,EPWM4_BASE};

struct buck_configuration chc_buck={48,89,4,GPIO_4_EPWM3A,5,GPIO_5_EPWM3B,EPWM3_BASE};
struct bridge_configuration chc_bridge={164,133,93,0,GPIO_0_EPWM1A,1,GPIO_1_EPWM1B,EPWM1_BASE};
