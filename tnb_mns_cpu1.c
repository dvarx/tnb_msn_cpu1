/*
 * tnb_mns_cpu1.c
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#include "tnb_mns_cpu1.h"
#include "driverlib.h"
#include "device.h"

// ------------------------------------------------------------------------------------
// Pin & Pad Configuration Structures
// ------------------------------------------------------------------------------------

struct buck_configuration cha_buck={40,41,8,GPIO_8_EPWM5A,9,GPIO_9_EPWM5B,EPWM5_BASE};
struct bridge_configuration cha_bridge={30,22,23,12,GPIO_12_EPWM7A,13,GPIO_13_EPWM7B,EPWM7_BASE,false};
struct driver_channel channela={0,&cha_buck,&cha_bridge,READY};

struct buck_configuration chb_buck={35,60,14,GPIO_14_EPWM8A,15,GPIO_15_EPWM8B,EPWM8_BASE};
struct bridge_configuration chb_bridge={63,61,65,6,GPIO_6_EPWM4A,7,GPIO_7_EPWM4B,EPWM4_BASE,false};
struct driver_channel channelb={0,&chb_buck,&chb_bridge,READY};

struct buck_configuration chc_buck={95,89,5,GPIO_5_EPWM3B,4,GPIO_4_EPWM3A,EPWM3_BASE};              // ! Buck H & L Pins seemingly exchanged on board
struct bridge_configuration chc_bridge={107,133,93,0,GPIO_0_EPWM1A,1,GPIO_1_EPWM1B,EPWM1_BASE,false};
struct driver_channel channelc={0,&chc_buck,&chc_bridge,READY};

struct driver_channel* driver_channels[NO_CHANNELS]={&channela,&channelb,&channelc};

// ---------------------
// Main Program related globals
// ---------------------

bool run_main_task=false;
struct system_dynamic_state system_dyn_state;
uint32_t enable_res_cap_a=0;     //variable control the resonant relay of channel a, if it is set to 1, res cap switched in
uint32_t enable_res_cap_b=0;     //variable control the resonant relay of channel b, if it is set to 1, res cap switched in
uint32_t enable_res_cap_c=0;     //variable control the resonant relay of channel c, if it is set to 1, res cap switched in
uint32_t enable_buck_a=0;        //variable to enable buck stage of channel a
uint32_t enable_buck_b=0;        //variable to enable buck stage of channel b
uint32_t enable_buck_c=0;        //variable to enable buck stage of channel c
uint32_t enable_bridge_a=0;      //variable to enable bridge state of channel a
uint32_t enable_bridge_b=0;      //variable to enable bridge state of channel a
uint32_t enable_bridge_c=0;      //variable to enable bridge state of channel a
double duty_bridge_a=0.5;
double duty_bridge_b=0.5;
double duty_bridge_c=0.5;
double duty_buck_a=0.5;
double duty_buck_b=0.5;
double duty_buck_c=0.5;
uint32_t freq_cha_resonant_mhz=5000000;
uint32_t freq_chb_resonant_mhz=5000000;
uint32_t freq_chc_resonant_mhz=5000000;

// ------------------------------------------------------------------------------------
// Main CPU Timer Related Functions
// ------------------------------------------------------------------------------------

uint16_t cpuTimer0IntCount;
uint16_t cpuTimer1IntCount;
uint16_t cpuTimer2IntCount;

//
// initCPUTimers - This function initializes all three CPU timers
// to a known state.
//
void
initCPUTimers(void)
{
    //
    // Initialize timer period to maximum
    //
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);
    //CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);
    //CPUTimer_setPeriod(CPUTIMER2_BASE, 0xFFFFFFFF);

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    //CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    //CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);

    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    //CPUTimer_stopTimer(CPUTIMER1_BASE);
    //CPUTimer_stopTimer(CPUTIMER2_BASE);

    //
    // Reload all counter register with period value
    //
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    //CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    //CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);

    //
    // Reset interrupt counter
    //
    cpuTimer0IntCount = 0;
    //cpuTimer1IntCount = 0;
    //cpuTimer2IntCount = 0;
}

//
// configCPUTimer - This function initializes the selected timer to the
// period specified by the "freq" and "period" variables. The "freq" is
// CPU frequency in Hz and the period in uSeconds. The timer is held in
// the stopped state after configuration.
//
void
configCPUTimer(uint32_t cpuTimer, uint32_t period)
{
    uint32_t temp, freq = DEVICE_SYSCLK_FREQ;

    //
    // Initialize timer period:
    //
    temp = ((freq / 1000000) * period);
    CPUTimer_setPeriod(cpuTimer, temp);

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CPUTimer_setPreScaler(cpuTimer, 0);

    //
    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    //
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(cpuTimer);

    //
    // Resets interrupt counters for the three cpuTimers
    //
    if (cpuTimer == CPUTIMER0_BASE)
    {
        cpuTimer0IntCount = 0;
    }
    else if(cpuTimer == CPUTIMER1_BASE)
    {
        cpuTimer1IntCount = 0;
    }
    else if(cpuTimer == CPUTIMER2_BASE)
    {
        cpuTimer2IntCount = 0;
    }
}

//
// cpuTimer0ISR - Counter for CpuTimer0
//
__interrupt void
cpuTimer0ISR(void)
{
    cpuTimer0IntCount++;

    run_main_task=true;

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
