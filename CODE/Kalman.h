/*
 * Kalman.h
 *
 *  Created on: 2022Äê1ÔÂ13ÈÕ
 *      Author: xqx
 */

#ifndef USER_KALMAN_H_
#define USER_KALMAN_H_

float Kalman_filter_Pitch(float Gyro ,float Angle_See);
float Kalman_filter_Roll(float Gyro_1 ,float Angle_See_1);
float Kalman_filter_Yaw(float Gyro_2 ,float Angle_See_2);


#endif /* USER_KALMAN_H_ */
