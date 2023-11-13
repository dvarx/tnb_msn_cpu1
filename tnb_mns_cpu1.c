/*
 * tnb_mns_cpu1.c
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#include "tnb_mns_cpu1.h"
#include "driverlib.h"
#include "device.h"
#include "fbctrl.h"
#include "tnb_mns_defs.h"
#include "tnb_mns_fsm.h"
#include "tnb_mns_epwm.h"
#include <math.h>

// ------------------------------------------------------------------------------------
// Pin & Pad Configuration Structures
// ------------------------------------------------------------------------------------

// channel a
struct buck_configuration cha_buck={40,41,8,GPIO_8_EPWM5A,9,GPIO_9_EPWM5B,EPWM5_BASE,false};
struct bridge_configuration cha_bridge={30,22,23,12,GPIO_12_EPWM7A,13,GPIO_13_EPWM7B,EPWM7_BASE,false,false};
struct driver_channel channela={0,&cha_buck,&cha_bridge,READY,83};

// channel b
struct buck_configuration chb_buck={35,60,15,GPIO_15_EPWM8B,14,GPIO_14_EPWM8A,EPWM8_BASE,false};
struct bridge_configuration chb_bridge={63,61,65,6,GPIO_6_EPWM4A,7,GPIO_7_EPWM4B,EPWM4_BASE,false,false};
struct driver_channel channelb={1,&chb_buck,&chb_bridge,READY,85};

// channel c
struct buck_configuration chc_buck={95,89,4,GPIO_4_EPWM3A,5,GPIO_5_EPWM3B,EPWM3_BASE,true};
struct bridge_configuration chc_bridge={107,133,93,0,GPIO_0_EPWM1A,1,GPIO_1_EPWM1B,EPWM1_BASE,false,false};
struct driver_channel channelc={2,&chc_buck,&chc_bridge,READY,87};

// channel d
struct buck_configuration chd_buck={32,33,2,GPIO_2_EPWM2A,3,GPIO_3_EPWM2B,EPWM2_BASE,true};
//struct bridge_configuration chd_bridge={48,49,54,10,GPIO_10_EPWM6A,11,GPIO_11_EPWM6B,EPWM6_BASE,false};
struct bridge_configuration chd_bridge={48,49,54,11,GPIO_11_EPWM6B,10,GPIO_10_EPWM6A,EPWM6_BASE,false,true};

struct driver_channel channeld={3,&chd_buck,&chd_bridge,READY,80}; // TODO : resonant enable pin

// channel e
struct buck_configuration che_buck={125,45,16,GPIO_16_EPWM9A,17,GPIO_17_EPWM9B,EPWM9_BASE,true};
struct bridge_configuration che_bridge={50,51,55,18,GPIO_18_EPWM10A,19,GPIO_19_EPWM10B,EPWM10_BASE,false,true};
struct driver_channel channele={4,&che_buck,&che_bridge,READY,80}; // TODO : resonant enable pin

// channel f
struct buck_configuration chf_buck={96,98,24,GPIO_24_EPWM13A,25,GPIO_25_EPWM13B,EPWM13_BASE,true};
struct bridge_configuration chf_bridge={52,53,56,26,GPIO_26_EPWM14A,27,GPIO_27_EPWM14B,EPWM14_BASE,false,true};
struct driver_channel channelf={5,&chf_buck,&chf_bridge,READY,80}; // TODO : resonant enable pin

struct driver_channel* driver_channels[NO_CHANNELS]={&channela,&channelb,&channelc,&channeld,&channele,&channelf};

// ---------------------
// Main Program related globals
// ---------------------

bool run_main_task=false;
struct system_dynamic_state system_dyn_state;
struct system_dynamic_state system_dyn_state_filtered;
uint32_t enable_res_cap_a=0;     //variable control the resonant relay of channel a, if it is set to 1, res cap switched in
uint32_t enable_res_cap_b=0;     //variable control the resonant relay of channel b, if it is set to 1, res cap switched in
uint32_t enable_res_cap_c=0;     //variable control the resonant relay of channel c, if it is set to 1, res cap switched in
float des_duty_bridge[NO_CHANNELS]={0.5,0.5,0.5,0.5,0.5,0.5};
float des_currents[NO_CHANNELS]={0.0,0.0,0.0,0.0,0.0,0.0};
float des_duty_buck[NO_CHANNELS]={0,0,0,0,0,0};
bool communication_active=true;
float mastertime=0.0;

//resonant control related
//float adc_buffer[7][ADC_BUF_SIZE];

//buffers used for estimating idqs
#pragma DATA_SECTION(obs_buffer_current_0,"OBS_BUFFER_I0")
#pragma DATA_SECTION(obs_buffer_current_1,"OBS_BUFFER_I1")
#pragma DATA_SECTION(obs_buffer_current_2,"OBS_BUFFER_I2")
float obs_buffer_current_0[OBS_BUF_SIZE];
float obs_buffer_current_1[OBS_BUF_SIZE];
float obs_buffer_current_2[OBS_BUF_SIZE];
#pragma DATA_SECTION(obs_buffer_current_0_aux,"OBS_BUFFER_I0AUX")
#pragma DATA_SECTION(obs_buffer_current_1_aux,"OBS_BUFFER_I1AUX")
#pragma DATA_SECTION(obs_buffer_current_2_aux,"OBS_BUFFER_I2AUX")
float obs_buffer_current_0_aux[OBS_BUF_SIZE];
float obs_buffer_current_1_aux[OBS_BUF_SIZE];
float obs_buffer_current_2_aux[OBS_BUF_SIZE];
float cosinebuf[OBS_BUF_SIZE];
float nsinebuf[OBS_BUF_SIZE];
float phasebuf[OBS_BUF_SIZE];

float* obs_buffer[3]={obs_buffer_current_0,obs_buffer_current_1,obs_buffer_current_2
};
float* obs_buffer_aux[3]={obs_buffer_current_0_aux,obs_buffer_current_1_aux,obs_buffer_current_2_aux};
bool use_aux_current_buffer=false;

//buffers for system analysis / debugging
#pragma DATA_SECTION(adc_buffer_current_0,"ADC_BUFFER_I0")
#pragma DATA_SECTION(adc_buffer_current_1,"ADC_BUFFER_I1")
#pragma DATA_SECTION(adc_buffer_current_2,"ADC_BUFFER_I2")
#pragma DATA_SECTION(adc_buffer_voltage_0,"ADC_BUFFER_V0")
#pragma DATA_SECTION(adc_buffer_voltage_1,"ADC_BUFFER_V1")
#pragma DATA_SECTION(adc_buffer_voltage_2,"ADC_BUFFER_V2")
#pragma DATA_SECTION(adc_buffer_refsig,"BUFFER_REFSIG")
float adc_buffer_current_0[ADC_BUF_SIZE];
float adc_buffer_current_1[ADC_BUF_SIZE];
float adc_buffer_current_2[ADC_BUF_SIZE];
float adc_buffer_voltage_0[ADC_BUF_SIZE];
float adc_buffer_voltage_1[ADC_BUF_SIZE];
float adc_buffer_voltage_2[ADC_BUF_SIZE];
float adc_buffer_refsig[ADC_BUF_SIZE];
float* adc_buffer[]={adc_buffer_current_0,adc_buffer_current_1,adc_buffer_current_2,
                   adc_buffer_voltage_0,adc_buffer_voltage_1,adc_buffer_voltage_2,
                   adc_buffer_refsig
};

uint16_t buffer_idq_cnt=0;
#pragma DATA_SECTION(buffer_id_0,"BUFFER_ID0")
#pragma DATA_SECTION(buffer_id_1,"BUFFER_ID1")
#pragma DATA_SECTION(buffer_id_2,"BUFFER_ID2")
#pragma DATA_SECTION(buffer_iq_0,"BUFFER_IQ0")
#pragma DATA_SECTION(buffer_iq_1,"BUFFER_IQ1")
#pragma DATA_SECTION(buffer_iq_2,"BUFFER_IQ2")
float buffer_iq_0[ADC_BUF_SIZE];
float buffer_iq_1[ADC_BUF_SIZE];
float buffer_iq_2[ADC_BUF_SIZE];
float buffer_id_0[ADC_BUF_SIZE];
float buffer_id_1[ADC_BUF_SIZE];
float buffer_id_2[ADC_BUF_SIZE];
uint16_t modidqsample=50;
unsigned int period_no=217;
const float dc_link_voltage=96.0;

float* buffer_idq[6]={buffer_id_0,buffer_id_1,buffer_id_2,
                      buffer_iq_0,buffer_iq_1,buffer_iq_2
};

uint16_t obs_buffer_cnt=0;
uint16_t adc_buffer_cnt=0;
uint16_t act_volt_buffer_cnt=0;
uint16_t buffer_prdstrt_pointer_0=0;      //pointer that points to the sample at the beginning of the latest oscillation period in the ADC buffer
uint16_t buffer_prdstrt_pointer_1=0;      //pointer that points to the sample at the beginning of the last oscillation period in the ADC buffer
bool adc_record=0;

//variables related to resonant control
float fres=214;
float actvolts[3]={0.0,0.0,0.0};
float actthetas[3]={0.0,0.0,0.0};
#define CTRLKP 0.0
#define CTRLKI 60
struct pi_controller_dq ctrl_i_dqs[3]={
                                   {CTRLKP,CTRLKI,0,0,0,0,0,0},
                                   {CTRLKP,CTRLKI,0,0,0,0,0,0},
                                   {CTRLKP,CTRLKI,0,0,0,0,0,0}
};
//------------------------------------------
float periodstart=0;               //start of current point of oscillation

//definition of the system impedance matrix (coupled system 94.5Hz)
/*
float zmatr[NO_CHANNELS][NO_CHANNELS]={
                   {6.1,0,0},
                   {0,6.1,0},
                   {0.0,0.0,6.1}
};
float zmati[NO_CHANNELS][NO_CHANNELS]={
                   {-0.17,-2.9,-2.84},
                   {-2.85,0.5,-2.9},
                   {-2.85,-2.98,0.6}
};
*/

//definition of the system impedance matrix (decoupled system 89Hz)

float zmatr[NO_CHANNELS][NO_CHANNELS]={
                   {5.1,0,0},
                   {0,5.1,0},
                   {0.0,0.0,5.1}
};
float zmati[NO_CHANNELS][NO_CHANNELS]={
                   {0,0,0},
                   {0,0,0},
                   {0,0,0}
};


//definition of the system impedance matrix (decoupled system 108.5HzHz)
/*
float zmatr[NO_CHANNELS][NO_CHANNELS]={
                   {5.1,0,0},
                   {0,5.1,0},
                   {0.0,0.0,5.1}
};
float zmati[NO_CHANNELS][NO_CHANNELS]={
                   {0,0,0},
                   {0,0,0},
                   {0,0,0}
};
*/


//definition of the system impedance matrix (decoupled system 424hz)
/*
float zmatr[NO_CHANNELS][NO_CHANNELS]={
                   {7.45,0.0,0.0},
                   {0.0,7.65,0.0},
                   {0.0,0.0,7.7}
};
float zmati[NO_CHANNELS][NO_CHANNELS]={
                   {0,0,0},
                   {0,0,0},
                   {0,0,0}
};
*/



//definition of the system impedance matrix (coupled system 452hz)
/*
float zmatr[NO_CHANNELS][NO_CHANNELS]={
                   {10,0.0,0.0},
                   {0.0,10,0.0},
                   {0.0,0.0,10}
};
float zmati[NO_CHANNELS][NO_CHANNELS]={
                   {-0.8,-16,-16},
                   {-16,-2.17,-16},
                   {-16,-16,-1.95}
};
*/



//definition of the system impedance matrix (coupled system 232Hz)
/*
float zmatr[NO_CHANNELS][NO_CHANNELS]={
                   {8,0.5,0.5},
                   {0.5,8,0.5},
                   {0.5,0.5,8}
};
float zmati[NO_CHANNELS][NO_CHANNELS]={
                   {1.7,-8.2,-8.2},
                   {-8.2,0.4,-8.2},
                   {-8.2,-8.2,0.56}
};
*/

//definition of the system impedance matrix (decoupled system 216Hz)
/*
float zmatr[NO_CHANNELS][NO_CHANNELS]={
                   {6,0.0,0.0},
                   {0.0,6,0.0},
                   {0.0,0.0,6}
};
float zmati[NO_CHANNELS][NO_CHANNELS]={
                   {0.0,0.0,0.0},
                   {0.0,0.0,0.0},
                   {0.0,0.0,0.0}
};
*/







//input to impedance matrix, corresponds
float xvecd[NO_CHANNELS]={0};
float xvecq[NO_CHANNELS]={0};
float vvecd[NO_CHANNELS]={0};
float vvecq[NO_CHANNELS]={0};
float rvecd[NO_CHANNELS]={0};
float rvecq[NO_CHANNELS]={0};
struct pi_controller current_pi[NO_CHANNELS]={
                                 {CTRL_KP,CTRL_KI,0.0,0.0,0.0,0.0},
                                 {CTRL_KP,CTRL_KI,0.0,0.0,0.0,0.0},
                                 {CTRL_KP,CTRL_KI,0.0,0.0,0.0,0.0},
                                 {CTRL_KP,CTRL_KI,0.0,0.0,0.0,0.0},
                                 {CTRL_KP,CTRL_KI,0.0,0.0,0.0,0.0},
                                 {CTRL_KP,CTRL_KI,0.0,0.0,0.0,0.0}
};
struct first_order des_duty_buck_filt[NO_CHANNELS]={
                                 {1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),-(1.0-2.0*TAU_BUCK_DUTY/deltaT)/(1.0+2.0*TAU_BUCK_DUTY/deltaT),0,0,0},
                                 {1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),-(1.0-2.0*TAU_BUCK_DUTY/deltaT)/(1.0+2.0*TAU_BUCK_DUTY/deltaT),0,0,0},
                                 {1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),-(1.0-2.0*TAU_BUCK_DUTY/deltaT)/(1.0+2.0*TAU_BUCK_DUTY/deltaT),0,0,0},
                                 {1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),-(1.0-2.0*TAU_BUCK_DUTY/deltaT)/(1.0+2.0*TAU_BUCK_DUTY/deltaT),0,0,0},
                                 {1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),-(1.0-2.0*TAU_BUCK_DUTY/deltaT)/(1.0+2.0*TAU_BUCK_DUTY/deltaT),0,0,0},
                                 {1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),1.0/(1.0+2.0*TAU_BUCK_DUTY/deltaT),-(1.0-2.0*TAU_BUCK_DUTY/deltaT)/(1.0+2.0*TAU_BUCK_DUTY/deltaT),0,0,0}
};
uint32_t des_freq_resonant_mhz[NO_CHANNELS]={DEFAULT_RES_FREQ_MILLIHZ,DEFAULT_RES_FREQ_MILLIHZ,DEFAULT_RES_FREQ_MILLIHZ};
struct tnb_mns_msg_c2000 ipc_tnb_mns_msg_c2000;
// ------------------------------------------------------------------------------------
// Main CPU Timer Related Functions
// ------------------------------------------------------------------------------------

uint32_t mastercounter=0;
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
    mastercounter = 0;
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
        mastercounter = 0;
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
// IPC ISR for Flag 0.
// This flag is set by the CM to send commands & data to CPU1
//
// IPC Defines TODO : Remove these variables & the single IPC communication at startup
#define IPC_CMD_READ_MEM   0x1001
#define IPC_CMD_RESP       0x2001

#define TEST_PASS          0x5555
#define TEST_FAIL          0xAAAA

__interrupt void IPC_ISR0()
{
    return;
    int i;
    uint32_t command, addr, data;
    bool status = false;

    //
    // Read the command
    //
    IPC_readCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);

    if(command == IPC_MSG_NEW_MSG){
        //copy tnb mns message
        memcpy(&ipc_tnb_mns_msg_c2000,(struct tnb_mns_msg*)addr,sizeof(ipc_tnb_mns_msg_c2000));

        //check flags and copy their values in the flag arrays used by the FSM
        unsigned short i=0;

        for(i=0; i<NO_CHANNELS; i++){
            //check the BUCK_EN byte
            if(ipc_tnb_mns_msg_c2000.buck_flg_byte&(1<<i))
                fsm_req_flags_en_buck[i]=true;
            //check the STOP byte
            if(ipc_tnb_mns_msg_c2000.stp_flg_byte&(1<<i))
                fsm_req_flags_stop[i]=true;
            //check the RUN_REGULAR byte
            if(ipc_tnb_mns_msg_c2000.regen_flg_byte&(1<<i))
                fsm_req_flags_run_res[i]=true;
            //check the RUN_RESONANT byte
            if(ipc_tnb_mns_msg_c2000.resen_flg_byte&(1<<i))
                fsm_req_flags_run_reg[i]=true;
        }
        //check the currents and duties and translate them
        //do not allow modifying these values when the system is not running, e.g.
        //do not modify values when system in READY state
        for(i=0; i<NO_CHANNELS; i++){
            if(driver_channels[i]->channel_state==RUN_RES||driver_channels[i]->channel_state==BUCK_ENABLED||driver_channels[i]->channel_state==RUN_REG){
                //currents are sent in units of [mA]
                des_currents[i]=(float)(ipc_tnb_mns_msg_c2000.desCurrents[i])*1e-3;
                des_duty_buck[i]=(float)(ipc_tnb_mns_msg_c2000.desDuties[i])*(1.0/(float)(UINT16_MAX));
            }
            else{
                des_currents[i]=0.0;
                des_duty_buck[i]=0.0;
            }
        }
        //check frequencies and set them
        //changing the resonant frequency is only allowed in the RUN_RESONANT state
        for(i=0; i<NO_CHANNELS; i++){
            if(driver_channels[i]->channel_state!=RUN_REG)
                des_freq_resonant_mhz[i]=DEFAULT_RES_FREQ_MILLIHZ;
            else{
                if(ipc_tnb_mns_msg_c2000.desFreqs[i]<MINIMUM_RES_FREQ_MILLIHZ)
                    des_freq_resonant_mhz[i]=MINIMUM_RES_FREQ_MILLIHZ;
                else
                    des_freq_resonant_mhz[i]=(uint32_t)(ipc_tnb_mns_msg_c2000.desFreqs[i]);
            }
        }
        //set the communication_active variable
        communication_active=true;
        //reset the timer
        CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    }

    if(command == IPC_CMD_READ_MEM)
    {
        status = true;

        //
        // Read and compare data
        //
        for(i=0; i<data; i++)
        {
            if(*((uint32_t *)addr + i) != i)
                status = false;
        }

        //
        // Send response to C28x core
        //
        if(status)
        {
            IPC_sendResponse(IPC_CPU1_L_CM_R, TEST_PASS);
        }
        else
        {
            IPC_sendResponse(IPC_CPU1_L_CM_R, TEST_FAIL);
        }
    }

    //
    // Acknowledge the flag
    //
    IPC_ackFlagRtoL(IPC_CPU1_L_CM_R, IPC_FLAG0);

    //
    // Acknowledge Interrupt Group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}

//
// cpuTimer0ISR - Counter for CpuTimer0
//
__interrupt void
cpuTimer0ISR(void)
{
    GPIO_writePin(HEARTBEAT_GPIO,1);
    mastercounter++;

    //process analog signals from last iteration
    float currentcos=cosinebuf[mastercounter%period_no];
    if(adc_record)
        adc_buffer[6][adc_buffer_cnt]=currentcos;

    //read adc results
    float ia=conv_adc_meas_to_current_a(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0),0);
    float ib=conv_adc_meas_to_current_a(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1),1);
    float ic=conv_adc_meas_to_current_a(ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0),2);

    //record data in ADC buffer and OBS buffer if required
    if(adc_record){
        adc_buffer[0][adc_buffer_cnt]=ia;
        adc_buffer[1][adc_buffer_cnt]=ib;
        adc_buffer[2][adc_buffer_cnt]=ic;
    }
    if(!use_aux_current_buffer){
        obs_buffer[0][obs_buffer_cnt]=ia;
        obs_buffer[1][obs_buffer_cnt]=ib;
        obs_buffer[2][obs_buffer_cnt]=ic;
    }
    else{
        obs_buffer_aux[0][obs_buffer_cnt]=ia;
        obs_buffer_aux[1][obs_buffer_cnt]=ib;
        obs_buffer_aux[2][obs_buffer_cnt]=ic;
    }

    //start new sampling
    ADC_forceMultipleSOC(ADCB_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1));
    ADC_forceMultipleSOC(ADCD_BASE, (ADC_FORCE_SOC0 ));

    //update pwm if necessary
    unsigned int i=0;
    //set output duties for bridge [regular mode]
    for(i=0; i<NO_CHANNELS; i++){
        if(driver_channels[i]->channel_state==RUN_RES){
            //compute feed forward actuation term (limits [-1,1] for this duty) - feed-forward term currently not used
            float act_voltage_ff=actvolts[i]*cos(phasebuf[mastercounter%period_no]+actthetas[i]);
            //store actuation voltages in ADC buffer
            if(adc_record)
                adc_buffer[3+i][adc_buffer_cnt]=act_voltage_ff;
            float duty_ff=act_voltage_ff*VOLTAGE_DCLINK_INV;
            //float duty_fb=act_voltage_fb*(1/voltage_dclink);

            //convert normalized duty cycle, limit it and apply
            float duty_bridge=0.5*(1+(duty_ff));
            if(duty_bridge>0.95)
                duty_bridge=0.95;
            if(duty_bridge<0.05)
                duty_bridge=0.05;
            set_duty_bridge(driver_channels[i]->bridge_config,duty_bridge);
        }
    }

    //update buffer counters
    obs_buffer_cnt+=1;
    adc_buffer_cnt=(1+adc_buffer_cnt)%ADC_BUF_SIZE;

    if(mastercounter%period_no==0){
        //write into the other currents buffer for the next frame period
        use_aux_current_buffer=!use_aux_current_buffer;
        //launch the main control task
        run_main_task=true;
        obs_buffer_cnt=0;
    }

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    GPIO_writePin(HEARTBEAT_GPIO,0);
}

//
// cpuTimer1ISR - Counter for CpuTimer1
//
__interrupt void cpuTimer1ISR(void){
    communication_active=false;

    //reset CPU Timer 1 (it counts downwards)
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
}
