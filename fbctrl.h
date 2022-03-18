/*
 * fbctrl.h
 *
 *  Created on: Nov 23, 2021
 *      Author: dvarx
 */

#ifndef FBCTRL_H_
#define FBCTRL_H_

#include <stdbool.h>

//controller time interval
#define deltaT 100e-6

struct pi_controller{
    float kp;
    float ki;
    float r;
    float errint;
    float errnm1;
    float u;
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

/*
 * ctrl: pointer to the pi_controller structure
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
