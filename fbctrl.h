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

struct first_order{

};

/*
 * ctrl: pointer to the pi_controller structure
 * ref: reference value that the pi controller should track
 */
inline void set_reference(const pi_controller& ctrl,ref){
    ctrl->r=ref;
}

/*
 * ctrl: pointer to the pi_controller structure
 * x: next input to pi controller
 */
inline float update_pid(const pi_controller& ctrl,float x){
    float e=ctrl->r-x;
    //update the error integral
    ctrl->errint+=e;
    ctrl->y=ctrl->kp*(e)+ki*ctrl->errint;
    return ctrl->y;
}



#endif /* FBCTRL_H_ */
