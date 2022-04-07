/*
 * tnb_mns_defs.h
 *
 *  Created on: Dec 9, 2021
 *      Author: dvarx
 */

#ifndef TNB_MNS_DEFS_H_
#define TNB_MNS_DEFS_H_

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


//constants related to output inductor current PI controller
#define RDC 3.3
#define VIN 96
#define CTRL_KP 40.0
#define CTRL_KI 2000.0

//buck duty first order time constants (ms)
#define TAU_BUCK_DUTY 300e-3


#endif /* TNB_MNS_DEFS_H_ */
