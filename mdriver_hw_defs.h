/*
 * mdriver_defs.h
 *
 *  Created on: Dec 9, 2021
 *      Author: dvarx
 */

#ifndef MDRIVER_HW_DEFS_H_
#define MDRIVER_HW_DEFS_H_

#define NO_CHANNELS 6

// ---------------------------------------------
//  CURRENT SENSOR DEFINITIONS
// ---------------------------------------------
/*
 * Hardware configuration specific settings
 * TMCS1100A1 50mV/A    [-30A,30A]
 * TMCS1100A2 100mV/A   [-15A,15A]
 * TMCS1100A3 200mV/A   [-7.5A,7.5A]
 * TMCS1100A4 400mV/A   [-3.75A,3.75A]
 */

//sensitivity of the current sensor in [V/A] (see table above)
#define I_SENSITIVITY 0.050
/*
 * define the USE_CALIBRATED_CURRENT_SENSORS below to allow for calibrated current measurements
 * the calibration parameters of the current sensor calibration can be added in `hardware_defs.c`
 */

//enable this definition to use calibrated current sensors
extern const float calib_factor_current_alphas[];
extern const float calib_factor_current_betas[];

// ---------------------------------------------
//  CONTROL MODE DEFINITIONS
// ---------------------------------------------
/*
 * change these definitions to toggle between CLOSED_LOOP, TUNE_CLOSED_LOOP and FEED_FORWARD_ONLY
 */


/*
 * if this define is made, the system will use a square wave as a
 * reference in RUN_REGULAR mode which can be used to adjust the PI parameters
 * */
//#define TUNE_CLOSED_LOOP

/*
 * if this define is made, the system will only use a feedforward term in RUN_REGULAR mode.
 * this can be useful for initial testing of the actuation / current sensor since it avoids
 * unstable closed loop operation in case of an erronous feedback loop
 */
//#define FEED_FORWARD_ONLY

/*
 * Use this configuration if the closed-loop tuning has been done
 */
#define CLOSED_LOOP


// ---------------------------------------------
//  FEEDBACK & HARDWARE DEFINITIONS
// ---------------------------------------------

//constants related to output inductor current PI controller
#define RDC 7.2                 //used for feedforward control (if enabled)
#define VIN 96
#define CTRL_KP 100.0           //used for current feedback controller
#define CTRL_KI 20000.0         //used for current feedback controller

//buck duty first order time constants (ms)
#define TAU_BUCK_DUTY 300e-3


#endif /* MDRIVER_HW_DEFS_H_ */
