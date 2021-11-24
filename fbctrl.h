/*
 * fbctrl.h
 *
 *  Created on: Nov 23, 2021
 *      Author: dvarx
 */

#ifndef FBCTRL_H_
#define FBCTRL_H_

struct pi_controller{
    float kp;
    float ki;
    float r;
    float errint;
    float y;
};

/*
 * First order IIR filter
 */
struct first_order{
    float b0;
    float b1;
    float a1;
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
    //update the error integral
    ctrlr->errint+=e;
    ctrlr->y=ctrlr->kp*(e)+ctrlr->ki*ctrlr->errint;
    return ctrlr->y;
}


inline void update_first_order(struct first_order* fir,float x){
    float y=fir->a1*fir->y_nm1+fir->b0*x+fir->b1*fir->x_nm1;
    fir->y_nm1=fir->y;
    fir->y_nm1=y;
}


#endif /* FBCTRL_H_ */
