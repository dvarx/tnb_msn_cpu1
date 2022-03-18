/*
 * tnb_mns_defs.h
 *
 *  Created on: Dec 9, 2021
 *      Author: dvarx
 */

#ifndef TNB_MNS_DEFS_H_
#define TNB_MNS_DEFS_H_

//#define TUNE_CLOSED_LOOP

//constants related to output inductor current PI controller
#define RDC 3.3
#define VIN 96
#define CTRL_KP 30.0
#define CTRL_KI 2000.0

//buck duty first order time constants (ms)
#define TAU_BUCK_DUTY 300e-3


#endif /* TNB_MNS_DEFS_H_ */
