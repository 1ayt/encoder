/*
 * Angle_Calculate.h
 *
 *  Created on: 2022Äê1ÔÂ14ÈÕ
 *      Author: xqx
 */

#ifndef USER_ANGLE_CALCULATE_H_
#define USER_ANGLE_CALCULATE_H_

float Accel_x(float mpu_acc_x);
float Accel_y(float mpu_acc_y);
float Accel_z(float mpu_acc_z);
float Gyro_x(float mpu_gyro_x);
float Gyro_y(float mpu_gyro_y);
float Gyro_z(float mpu_gyro_z);

float Angle_Yaw(float mpu_gyro_z);
float Angle_Pitch(float mpu_acc_z,float mpu_acc_x);
float Angle_Roll(float mpu_acc_z,float mpu_acc_y);

#endif /* USER_ANGLE_CALCULATE_H_ */
