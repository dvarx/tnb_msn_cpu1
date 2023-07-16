/*
 * tnb_mns_adc.h
 *
 *  Created on: Oct 17, 2021
 *      Author: dvarx
 */

#ifndef TNB_MNS_ADC_H_
#define TNB_MNS_ADC_H_

extern float ivecq[3];
extern float ivecd[3];

extern const float calib_factor_current_alpha;
extern const float calib_factor_current_beta;

inline float conv_adc_meas_to_current_a(const uint16_t adc_output){
    return calib_factor_current_alpha*(float)((int16_t)adc_output-(int16_t)2048)+calib_factor_current_beta;
}

void initADCs(void);

void initADCSOCs(void);

void readAnalogInputs(void);



#endif /* TNB_MNS_ADC_H_ */
