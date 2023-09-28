//#############################################################################
//
// FILE:   cm_common_config_c28x.c
//
// TITLE:  C28x Common Configurations to be used for the CM Side.
//
//! \addtogroup driver_example_list
//! <h1>C28x Common Configurations</h1>
//!
//! This example configures the GPIOs and Allocates the shared peripherals
//! according to the defines selected by the users.
//!
//
//#############################################################################
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "tnb_mns_cpu1.h"
#include "tnb_mns_epwm.h"
#include "tnb_mns_adc.h"
#include <stdbool.h>
#include "tnb_mns_fsm.h"
#include "fbctrl.h"
#include "tnb_mns_cpu1.h"
#include "tnb_mns_defs.h"
#include <math.h>
#include "ipc.h"
#include "tnb_mns_cpu1.h"

bool run_main_control_task=false;
bool enable_waveform_debugging=false;
bool use_pi=true;
float curcos=0.0;
float cursin=0.0;
bool readcpu2=true;

//communication struct from CPU2 to CPU1
struct comm_cpu2cpu1{
    float ids[3];
    float iqs[3];
    float freq;
    bool en_flags[3];
    bool runreg_flags[3];
    bool stop_flags[3];
};
struct comm_cpu2cpu1* msg_cpu2tocpu1=(struct comm_cpu2cpu1*)0x03B000;
struct comm_cpu1cpu2{
    float actvolts[3];
    float ids[3];
    float iqs[3];
};
struct comm_cpu1cpu2* msg_cpu1tocpu2=(struct comm_cpu1cpu2*)0x03A000;

bool actsaturated[3]={false,false,false};

//#define SYSID

void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    /*-----------------------------------------------------
     * CPU2 STARTUP CODE
     -----------------------------------------------------*/

    //set controlling CPU for SCI module 1 to CPU2
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL5_SCI, 1, SYSCTL_CPUSEL_CPU2);
    //
    // GPIO28 is the SCI Rx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU2);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    //
    // GPIO29 is the SCI Tx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU2);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    //
    // Boot CPU2 core
    //
    #ifdef _FLASH
        Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
    #else
        Device_bootCPU2(BOOTMODE_BOOT_TO_M0RAM);
    #endif


/*
    //
    // Boot CM core
    //
#ifdef _FLASH
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
*/

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Setup heartbeat GPIO & input relay gpios
    //heartbeat
    GPIO_setDirectionMode(HEARTBEAT_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(HEARTBEAT_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //sampling signal
    GPIO_setDirectionMode(SAMPLING_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(SAMPLING_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //main task signal
    GPIO_setDirectionMode(MAIN_TASK_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(MAIN_TASK_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //main input relay
    GPIO_setDirectionMode(MAIN_RELAY_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(MAIN_RELAY_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //input relay to slave
    GPIO_setDirectionMode(SLAVE_RELAY_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(SLAVE_RELAY_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //LED 1 for debugging
    GPIO_setDirectionMode(LED_1_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(LED_1_GPIO,GPIO_PIN_TYPE_STD);
    GPIO_writePin(LED_1_GPIO,1);
    //LED 2 for debugging
    GPIO_setDirectionMode(LED_2_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(LED_2_GPIO,GPIO_PIN_TYPE_STD);
    GPIO_writePin(LED_2_GPIO,1);

    //
    // Initialize ADCs
    //
    //setup ADC clock, single-ended mode, enable ADCs
    initADCs();
    //setup ADC SOC configurations
    initADCSOCs();

    //
    // Initialize Half Bridges
    //
    //channel A
    setup_pin_config_buck(&cha_buck);
    setup_pinmux_config_bridge(&cha_bridge);
    //channel B
    setup_pin_config_buck(&chb_buck);
    setup_pinmux_config_bridge(&chb_bridge);
    //channel C
    setup_pin_config_buck(&chc_buck);
    setup_pinmux_config_bridge(&chc_bridge);
    //channel D
    setup_pin_config_buck(&chd_buck);
    setup_pinmux_config_bridge(&chd_bridge);
    //channel E
    setup_pin_config_buck(&che_buck);
    setup_pinmux_config_bridge(&che_bridge);
    //channel F
    setup_pin_config_buck(&chf_buck);
    setup_pinmux_config_bridge(&chf_bridge);

    //
    // Enable Half Bridges
    //
    set_enabled(&cha_buck,true,true);
    set_enabled(&cha_bridge,false,true);
    set_enabled(&chb_buck,true,true);
    set_enabled(&chb_bridge,false,true);
    set_enabled(&chc_buck,true,true);
    set_enabled(&chc_bridge,false,true);
    set_enabled(&chd_buck,true,true);
    set_enabled(&chd_bridge,false,true);
    set_enabled(&che_buck,true,true);
    set_enabled(&che_bridge,false,true);
    set_enabled(&chf_buck,true,true);
    set_enabled(&chf_bridge,false,true);

    //
    // Initialize Reed Switch Interface
    //
    unsigned int n=0;
    for(n=0; n<NO_CHANNELS; n++){
        GPIO_setDirectionMode(driver_channels[n]->enable_resonant_gpio, GPIO_DIR_MODE_OUT);   //output
        GPIO_setPadConfig(driver_channels[n]->enable_resonant_gpio,GPIO_PIN_TYPE_STD);        //push pull output
    }

    //
    // Setup main control task interrupt
    //
    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    Interrupt_initModule();
    // Initializes the PIE vector table with pointers to the shell Interrupt Service Routines (ISRs)
    Interrupt_initVectorTable();
    //--------------- IPC interrupt ---------------
    //clear any IPC flags
    //IPC_clearFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG_ALL);
    //register IPC interrupt from CM to CPU1 using IPC_INT0
    //IPC_registerInterrupt(IPC_CPU1_L_CM_R, IPC_INT0, IPC_ISR0);
    //synchronize CM and CPU1 using IPC_FLAG31
    //IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);
    //--------------- CPU1 Timer0 interrupt (main task) ---------------
    // Register ISR for cupTimer0
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    // Initialize CPUTimer0
    configCPUTimer(CPUTIMER0_BASE, deltaT*1e6);
    // Enable CPUTimer0 Interrupt within CPUTimer0 Module
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    // Enable TIMER0 Interrupt on CPU coming from TIMER0
    Interrupt_enable(INT_TIMER0);
    // Start CPUTimer0
    CPUTimer_startTimer(CPUTIMER0_BASE);
    //--------------- CPU1 Timer1 interrupt (communication active) ---------------
    // Register ISR for cupTimer1
    Interrupt_register(INT_TIMER1, &cpuTimer1ISR);
    // Initialize CPUTimer1
    configCPUTimer(CPUTIMER1_BASE, 500000);
    // Enable CPUTimer0 Interrupt within CPUTimer1 Module
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    // Enable TIMER1 Interrupt on CPU coming from TIMER1
    //Interrupt_enable(INT_TIMER1);
    // Start CPUTimer0
    //CPUTimer_startTimer(CPUTIMER1_BASE);

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

    /*
#ifdef ETHERNET
    //
    // Set up EnetCLK to use SYSPLL as the clock source and set the
    // clock divider to 2.
    //
    // This way we ensure that the PTP clock is 100 MHz. Note that this value
    // is not automatically/dynamically known to the CM core and hence it needs
    // to be made available to the CM side code beforehand.
    SysCtl_setEnetClk(SYSCTL_ENETCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL);

    //
    // Configure the GPIOs for ETHERNET.
    //

    //
    // MDIO Signals
    //
    GPIO_setPinConfig(GPIO_105_ENET_MDIO_CLK);
    GPIO_setPinConfig(GPIO_106_ENET_MDIO_DATA);

    //
    // Use this only for RMII Mode
    //GPIO_setPinConfig(GPIO_73_ENET_RMII_CLK);
    //

    //
    //MII Signals
    //
    GPIO_setPinConfig(GPIO_109_ENET_MII_CRS);
    GPIO_setPinConfig(GPIO_110_ENET_MII_COL);

    GPIO_setPinConfig(GPIO_75_ENET_MII_TX_DATA0);
    GPIO_setPinConfig(GPIO_122_ENET_MII_TX_DATA1);
    GPIO_setPinConfig(GPIO_123_ENET_MII_TX_DATA2);
    GPIO_setPinConfig(GPIO_124_ENET_MII_TX_DATA3);

    //
    //Use this only if the TX Error pin has to be connected
    //GPIO_setPinConfig(GPIO_46_ENET_MII_TX_ERR);
    //

    GPIO_setPinConfig(GPIO_118_ENET_MII_TX_EN);

    GPIO_setPinConfig(GPIO_114_ENET_MII_RX_DATA0);
    GPIO_setPinConfig(GPIO_115_ENET_MII_RX_DATA1);
    GPIO_setPinConfig(GPIO_116_ENET_MII_RX_DATA2);
    GPIO_setPinConfig(GPIO_117_ENET_MII_RX_DATA3);
    GPIO_setPinConfig(GPIO_113_ENET_MII_RX_ERR);
    GPIO_setPinConfig(GPIO_112_ENET_MII_RX_DV);

    GPIO_setPinConfig(GPIO_44_ENET_MII_TX_CLK);
    GPIO_setPinConfig(GPIO_111_ENET_MII_RX_CLK);

    //
    //Power down pin to bring the external PHY out of Power down
    //
    GPIO_setDirectionMode(108, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(108, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(108,1);

    //
    //PHY Reset Pin to be driven High to bring external PHY out of Reset
    //

    GPIO_setDirectionMode(119, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(119, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(119,1);
#endif
*/

#ifdef MCAN
    //
    // Setting the MCAN Clock.
    //
    SysCtl_setMCANClk(SYSCTL_MCANCLK_DIV_4);

    //
    // Configuring the GPIOs for MCAN.
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_MCANRXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_MCANTXA);
#endif

#ifdef CANA
    //
    // Configuring the GPIOs for CAN A.
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);

    //
    // Allocate Shared Peripheral CAN A to the CM Side.
    //
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_CAN_A,0x1U);
#endif

#ifdef CANB
    //
    // Configuring the GPIOs for CAN B.
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);

    //
    // Allocate Shared Peripheral CAN B to the CM Side.
    //
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_CAN_B,0x1U);
#endif

#ifdef UART
    //
    // Configure GPIO85 as the UART Rx pin.
    //
    GPIO_setPinConfig(GPIO_85_UARTA_RX);
    GPIO_setDirectionMode(85, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(85, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(85, GPIO_QUAL_ASYNC);

    //
    // Configure GPIO84 as the UART Tx pin.
    //
    GPIO_setPinConfig(GPIO_84_UARTA_TX);
    GPIO_setDirectionMode(84, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(84, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(84, GPIO_QUAL_ASYNC);
#endif

#ifdef USB
#ifdef USE_20MHZ_XTAL
    //
    // Set up the auxiliary PLL so a 60 MHz output clock is provided to the USB module.
    // This fixed frequency is required for all USB operations.
    //
    SysCtl_setAuxClock(SYSCTL_AUXPLL_OSCSRC_XTAL |
                       SYSCTL_AUXPLL_IMULT(48) |
                       SYSCTL_REFDIV(2U) | SYSCTL_ODIV(4U) |
                       SYSCTL_AUXPLL_DIV_2 |
                       SYSCTL_AUXPLL_ENABLE |
                       SYSCTL_DCC_BASE_0);
#else
    //
    // Set up the auxiliary PLL so a 60 MHz output clock is provided to the USB module.
    // This fixed frequency is required for all USB operations.
    //
    SysCtl_setAuxClock(SYSCTL_AUXPLL_OSCSRC_XTAL |
                       SYSCTL_AUXPLL_IMULT(48) |
                       SYSCTL_REFDIV(2U) | SYSCTL_ODIV(5U) |
                       SYSCTL_AUXPLL_DIV_2 |
                       SYSCTL_AUXPLL_ENABLE |
                       SYSCTL_DCC_BASE_0);
#endif

    //
    // Allocate Shared Peripheral USB to the CM Side.
    //
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);

    GPIO_setPinConfig(GPIO_0_GPIO0);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(0, GPIO_CORE_CM);

    //
    // Set the master core of GPIOs to CM.
    //
    GPIO_setMasterCore(42, GPIO_CORE_CM);
    GPIO_setMasterCore(43, GPIO_CORE_CM);
    GPIO_setMasterCore(46, GPIO_CORE_CM);
    GPIO_setMasterCore(47, GPIO_CORE_CM);
    GPIO_setMasterCore(120, GPIO_CORE_CM);
    GPIO_setMasterCore(121, GPIO_CORE_CM);

    //
    // Set the USB DM and DP GPIOs.
    //
    GPIO_setAnalogMode(42, GPIO_ANALOG_ENABLED);
    GPIO_setAnalogMode(43, GPIO_ANALOG_ENABLED);

    //
    // Set the direction for VBUS and ID.
    //
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setDirectionMode(47, GPIO_DIR_MODE_IN);

    //
    // Configure the Power Fault.
    //
    GPIO_setMasterCore(120, GPIO_CORE_CM);
    GPIO_setDirectionMode(120, GPIO_DIR_MODE_IN);

    //
    // Configure the External Power Signal Enable.
    //
    GPIO_setMasterCore(121, GPIO_CORE_CM);
	GPIO_setDirectionMode(121, GPIO_DIR_MODE_OUT);
	GPIO_writePin(121, 1);

    //
    // Set the CM Clock to run at 120MHz.
    // The CM Clock is a fractional multiple of the AUXPLL Clock (120 Mhz) from
    // which the USB Clock (60 MHz) is derived.
    //
    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_1, SYSCTL_SOURCE_AUXPLL);
#endif

    //
    // Initialize Outputs
    //
    for(n=0; n<NO_CHANNELS; n++){
        READY_enter(n);
        driver_channels[n]->channel_state=READY;
    }

    float ivecqtmp[3];
    float ivecdtmp[3];
    // Main Loop
    while(1){
        if(run_main_task){
            //toggle heartbeat gpio
            GPIO_writePin(MAIN_TASK_GPIO,1);

            /* -------------------------------------
             * update sinusoidal PWMs (do this at 1/10th of the main rate, e.g. 10kHz)
             * -------------------------------------
             */
            //loop variable
            unsigned int i=0;
            unsigned int channel_counter=0;

            #ifndef SYSID
            if(readcpu2){
                //read flags
                for(i=0; i<NO_CHANNELS; i++){
                    fsm_req_flags_en_buck[i]=msg_cpu2tocpu1->en_flags[i];
                    fsm_req_flags_stop[i]=msg_cpu2tocpu1->stop_flags[i];
                    fsm_req_flags_run_regular[i]=msg_cpu2tocpu1->runreg_flags[i];
                }
                //translate desired frequency from mHz to Hz
                fres=((float)msg_cpu2tocpu1->freq);
                for(i=0; i<NO_CHANNELS; i++){
                    rvecd[i]=msg_cpu2tocpu1->ids[i];
                    rvecq[i]=msg_cpu2tocpu1->iqs[i];
                }
            }
            #endif


            //---------------------
            // run main state machine and control loops (do this at 1/100th of the main rate, e.g. 1kHz)
            //---------------------
            //Main Relay Opening Logic
            bool main_relay_active=false;
            for(channel_counter=0; channel_counter<NO_CHANNELS; channel_counter++){
                run_channel_fsm(driver_channels[channel_counter]);
                //we enable the main relay when one channel is not in state READY anymore (e.g. when one channel requires power)
                if(driver_channels[channel_counter]->channel_state!=READY)
                    main_relay_active=true;
            }
            GPIO_writePin(MAIN_RELAY_GPIO,main_relay_active);
            GPIO_writePin(SLAVE_RELAY_GPIO,main_relay_active);
            GPIO_writePin(LED_1_GPIO,!main_relay_active);
            //Communication Active Logic (If no communication, issue a STOP command
            if(!communication_active){
                for(channel_counter=0; channel_counter<NO_CHANNELS; channel_counter++){
                    fsm_req_flags_stop[channel_counter]=1;
                }
            }

            //estimate ids and iqs
            //if we are currently recording into aux buffer, use values from the normal buffer to estimate ids and iqs
            ivecdtmp[0]=0;
            ivecdtmp[1]=0;
            ivecdtmp[2]=0;
            ivecqtmp[0]=0;
            ivecqtmp[1]=0;
            ivecqtmp[2]=0;
            if(use_aux_current_buffer){
                for(i=0; i<period_no; i++){
                    ivecdtmp[0]+=obs_buffer[0][i]*cosinebuf[i];
                    ivecdtmp[1]+=obs_buffer[1][i]*cosinebuf[i];
                    ivecdtmp[2]+=obs_buffer[2][i]*cosinebuf[i];
                    ivecqtmp[0]+=obs_buffer[0][i]*nsinebuf[i];
                    ivecqtmp[1]+=obs_buffer[1][i]*nsinebuf[i];
                    ivecqtmp[2]+=obs_buffer[2][i]*nsinebuf[i];
                }
            }
            else{
                for(i=0; i<period_no; i++){
                    ivecdtmp[0]+=obs_buffer_aux[0][i]*cosinebuf[i];
                    ivecdtmp[1]+=obs_buffer_aux[1][i]*cosinebuf[i];
                    ivecdtmp[2]+=obs_buffer_aux[2][i]*cosinebuf[i];
                    ivecqtmp[0]+=obs_buffer_aux[0][i]*nsinebuf[i];
                    ivecqtmp[1]+=obs_buffer_aux[1][i]*nsinebuf[i];
                    ivecqtmp[2]+=obs_buffer_aux[2][i]*nsinebuf[i];
                }
            }
            ivecd[0]=ivecdtmp[0]*2/period_no;
            ivecd[1]=ivecdtmp[1]*2/period_no;
            ivecd[2]=ivecdtmp[2]*2/period_no;
            ivecq[0]=ivecqtmp[0]*2/period_no;
            ivecq[1]=ivecqtmp[1]*2/period_no;
            ivecq[2]=ivecqtmp[2]*2/period_no;
            //store computed ids and iqs in buffer
            if(adc_record){
                buffer_idq[0][buffer_idq_cnt]=ivecd[0];
                buffer_idq[1][buffer_idq_cnt]=ivecd[1];
                buffer_idq[2][buffer_idq_cnt]=ivecd[2];
                buffer_idq[3][buffer_idq_cnt]=ivecq[0];
                buffer_idq[4][buffer_idq_cnt]=ivecq[1];
                buffer_idq[5][buffer_idq_cnt]=ivecq[2];
                buffer_idq_cnt=(buffer_idq_cnt+1)%ADC_BUF_SIZE;
            }

            //message ids and iqs and actvolts to CPU2
            for(i=0; i<3; i++){
                msg_cpu1tocpu2->ids[i]=ivecd[i];
                msg_cpu1tocpu2->iqs[i]=ivecq[i];
                msg_cpu1tocpu2->actvolts[i]=actvolts[i];
            }


            //PID control laws, compute voltage phasor necessary for actuation
            if(driver_channels[0]->channel_state==RUN_REGULAR&&driver_channels[1]->channel_state==RUN_REGULAR&&driver_channels[2]->channel_state==RUN_REGULAR){
                //temporary storage
                float vec1[NO_CHANNELS];
                float vec2[NO_CHANNELS];

                //run pi controllers
                xvecd[0]=0.0;//0.5*rvecd[0];
                xvecd[1]=0.0;//0.5*rvecd[1];
                xvecd[2]=0.0;//0.5*rvecd[2];
                xvecq[0]=0.0;//0.5*rvecq[0];
                xvecq[1]=0.0;//0.5*rvecq[1];
                xvecq[2]=0.0;//0.5*rvecq[2];
                if(use_pi){
                    for(i=0; i<3; i++){
                        update_pid_dq(&ctrl_i_dqs[i],rvecd[i],rvecq[i],ivecd[i],ivecq[i]);
                        xvecd[i]+=ctrl_i_dqs[i].ud;
                        xvecq[i]+=ctrl_i_dqs[i].uq;
                    }
                }
                //compute real component of actuation voltage
                matmul3(zmatr,xvecd,vec1);
                matmul3(zmati,xvecq,vec2);
                vvecd[0]=vec1[0]-vec2[0];
                vvecd[1]=vec1[1]-vec2[1];
                vvecd[2]=vec1[2]-vec2[2];
                //compute imaginary component of actuation voltage
                matmul3(zmatr,xvecq,vec1);
                matmul3(zmati,xvecd,vec2);
                vvecq[0]=vec1[0]+vec2[0];
                vvecq[1]=vec1[1]+vec2[1];
                vvecq[2]=vec1[2]+vec2[2];
                #ifndef SYSID
                //compute voltage magnitudes
                actvolts[0]=sqrt(vvecd[0]*vvecd[0]+vvecq[0]*vvecq[0]);
                actvolts[1]=sqrt(vvecd[1]*vvecd[1]+vvecq[1]*vvecq[1]);
                actvolts[2]=sqrt(vvecd[2]*vvecd[2]+vvecq[2]*vvecq[2]);
                //compute voltage angles
                actthetas[0]=atan2(vvecq[0],vvecd[0]);
                actthetas[1]=atan2(vvecq[1],vvecd[1]);
                actthetas[2]=atan2(vvecq[2],vvecd[2]);
                #endif
            }

            GPIO_writePin(MAIN_TASK_GPIO,0);
            run_main_task=false;
        }
    }
}
