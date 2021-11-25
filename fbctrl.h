/*
 * fbctrl.h
 *
 *  Created on: Nov 23, 2021
 *      Author: dvarx
 */

#ifndef FBCTRL_H_
#define FBCTRL_H_

//controller time interval
#define deltaT 100e-6

struct pi_controller{
    float kp;
    float ki;
    float r;
    float errint;
    float errnm1;
    float y;
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
 * ref: reference value that the pi controller should track
 */
inline void set_reference(struct pi_controller* ctrlr,float refr){
    //ctrlr->r=refr;
    return;
}

/*
 * ctrl: pointer to the pi_controller structure
 * x: next input to pi controller
 */
inline float update_pid(struct pi_controller* ctrlr,float x){
    float e=ctrlr->r-x;
    //update the error integral using trapezoidal rule
    ctrlr->errint+=(deltaT*0.5)*(e+ctrlr->errnm1);
    //store the current error
    ctrlr->errnm1=e;
    //update the output
    ctrlr->y=ctrlr->kp*(e)+ctrlr->ki*ctrlr->errint;
    return ctrlr->y;
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


#endif /* FBCTRL_H_ */
