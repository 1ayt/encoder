/*
 * PID_High.h
 *
 *  Created on: 2022Äê1ÔÂ16ÈÕ
 *      Author: xqx
 */

#ifndef CODE_PID_HIGH_H_
#define CODE_PID_HIGH_H_

float PID_Calculate(float current_value,float desired_value,float output_value_limit,float Kp,float Ki,float Kd,float dt);

#endif /* CODE_PID_HIGH_H_ */
