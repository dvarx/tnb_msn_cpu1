/*
 * fbctrl.h
 *
 *  Created on: Nov 23, 2021
 *      Author: dvarx
 */

#ifndef FBCTRL_H_
#define FBCTRL_H_

#include <stdbool.h>
#include "fir_coeffs.h"

//controller time interval (main task interval / adc sampling frequency)
#define deltaT 1e-3
//this variable defines the ratio between control frequency fc and sampling frequency / main task frequency fs
#define F_CONTROL_MOD 10
//time interval (in seconds) at which the discrete controller is run
#define DELTAT_C deltaT*F_CONTROL_MOD
//discrete controller time interval
#define deltaT_c deltaT*f_control_mod

struct pi_controller{
    float kp;
    float ki;
    float r;
    float errint;
    float errnm1;
    float u;
};

struct fir_filter{
    unsigned int order;
    unsigned int bufptr;
    const float* coeffs;
    float xsbuf[N_CURRENTLOWPASS];
    float out;
};

/*
 * First order IIR filter
 */
struct first_order{
    float a0;
    float a1;
    float b1;
    float x_nm1;
    float y_nm1;
    float y;
};

inline float update_fir(struct fir_filter* filt, float x){
    unsigned int i;
    //update the element in the buffer
    filt->xsbuf[filt->bufptr]=x;
    float out=0;
    for(i=0; i<(filt->order); i++)
        out+=filt->coeffs[i]*filt->xsbuf[(filt->order+filt->bufptr-i)%filt->order];
    filt->out=out;
    filt->bufptr=(filt->bufptr+1)%filt->order;
    return out;
}

/*
 * ctrl: pointer to Falsethe pi_controller structure
 * r: r[n] the current reference input
 * y: y[n] the current output measurement
 */
inline float update_pid(struct pi_controller* ctrlr,float r,float y,bool output_saturated){
    //compute the current error
    float e=r-y;
    //update the error integral using trapezoidal rule, but only if the output is not saturated (anti-windup measure)
    if(!output_saturated)
        ctrlr->errint+=(deltaT*0.5)*(e+ctrlr->errnm1);
    //store the current error for later use
    ctrlr->errnm1=e;
    //update the output
    ctrlr->u=ctrlr->kp*(e)+ctrlr->ki*ctrlr->errint;
    return ctrlr->u;
}

inline float update_first_order(struct first_order* fir,float x){
    //compute new output
    float y=fir->a0*x+fir->a1*fir->x_nm1+fir->b1*fir->y_nm1;
    //store the old output and input
    fir->y_nm1=y;
    fir->x_nm1=x;
    fir->y=y;
    return y;
}

/*
 * resets first order filter to zero
 */
inline void reset_first_order(struct first_order* fir){
    fir->x_nm1=0;
    fir->y_nm1=0;
    fir->y=0;
}

inline void reset_pid(struct pi_controller* ctrlr){
    ctrlr->errint=0;
    ctrlr->errnm1=0;
}

#endif /* FBCTRL_H_ */
