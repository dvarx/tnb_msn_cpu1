/*
 * hardware_defs.c
 *
 *  Created on: Apr 29, 2025
 *      Author: dvarx
 */


#include "mdriver_hw_defs.h"

//if calibrated current sensors are used, add their sensitivities and offsets in this file
/*
 * calibrated sensitivities and offsets
 * to determine the calibration factor:
 * 1 - compile the firmware without the USE_CALIBRATED_CURRENT_SENSORS flag
 * 2 - apply a test current I_REF while measuring the actual current I_ACTUAL
 * 3 - replace the correction factor 1.0 below with I_REF/I_ACTUAL
 * (e.g. if 2A are applied and 3A are measured, set the correction factor to 2/3)
 */
const float calib_factor_current_alphas[]={
                                           1.0  *   (-1.5/I_SENSITIVITY/2048),
                                           1.0  *   (-1.5/I_SENSITIVITY/2048),
                                           1.0  *   (-1.5/I_SENSITIVITY/2048),
                                           1.0  *   (-1.5/I_SENSITIVITY/2048),
                                           1.0  *   (-1.5/I_SENSITIVITY/2048),
                                           1.0  *   (-1.5/I_SENSITIVITY/2048)};
const float calib_factor_current_betas[NO_CHANNELS]={
                                                     0.0,
                                                     0.0,
                                                     0.0,
                                                     0.0,
                                                     0.0,
                                                     0.0};
