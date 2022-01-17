/*
 * PID_High.c
 *
 *  Created on: 2022年1月16日
 *      Author: xqx
 */

#include "PID_High.h"

float PID_Calculate(float current_value,float desired_value,float output_value_limit,float Kp,float Ki,float Kd,float dt)
{
    float output_value;
    float cur_error;
    float pre_error;
    float integ;
    float deriv;
    float outP;
    float outI;
    float outD;

    cur_error = desired_value - current_value;
    integ = integ + cur_error * dt;
    deriv = (cur_error - pre_error) / dt;

    outP = Kp * cur_error;
    outI = Ki * integ;
    outD = Kd * deriv;

    output_value = outP + outI + outD;

    //PID输出限幅
    if(output_value >= output_value_limit)
        output_value = output_value_limit;
    else
        if(output_value <= -output_value_limit)
            output_value = -output_value_limit;


    pre_error = cur_error;

    return output_value;

}


