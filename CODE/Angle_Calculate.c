/*
 * Angle_Calculate.c
 *
 *  Created on: 2022年1月14日
 *      Author: xqx
 */

#include "Angle_Calculate.h"
#include "SEEKFREE_MPU6050.h"

//Yaw,Pitch,Roll 角度值
float Yaw,Pitch,Roll;

//声明mpu6050.h文件中的变量
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;

/* brief  : 返回x轴角加速度
 * Param1 : mpu_acc_x : mpu6050返回的x轴角加速度值
 */
float Accel_x(float mpu_acc_x)
{
    float Accel_x;

    Accel_x = 8*9.8*(mpu_acc_x)/32768;

    return Accel_x;
}

/* brief  : 返回y轴角加速度
 * Param1 : mpu_acc_y : mpu6050返回的y轴角加速度值
 */
float Accel_y(float mpu_acc_y)
{
    float Accel_y;

    Accel_y = 8*9.8*(mpu_acc_y)/32768;

    return Accel_y;
}

/* brief  : 返回z轴角加速度
 * Param1 : mpu_acc_z : mpu6050返回的z轴角加速度值
 */
float Accel_z(float mpu_acc_z)
{
    float Accel_z;

    Accel_z = 8*9.8*(mpu_acc_z)/32768;

    return Accel_z;
}

/* brief  : 返回x轴角速度
 * Param1 : mpu_gyro_x : mpu6050返回的x轴角速度值
 */
float Gyro_x(float mpu_gyro_x)
{
    float Gyro_x;

    Gyro_x = 2000*mpu_gyro_x/32768;

    return Gyro_x;
}

/* brief  : 返回y轴角速度
 * Param1 : mpu_gyro_y : mpu6050返回的y轴角速度值
 */
float Gyro_y(float mpu_gyro_y)
{
    float Gyro_y;

    Gyro_y = 2000*mpu_gyro_y/32768;

    return Gyro_y;
}

/* brief  : 返回z轴角速度
 * Param1 : mpu_gyro_z : mpu6050返回的z轴角速度值
 */
float Gyro_z(float mpu_gyro_z)
{
    float Gyro_z;

    Gyro_z = 2000*mpu_gyro_z/32768;

    return Gyro_z;
}
/* brief  : 返回Yaw角度
 * Param1 : mpu_gyro_z : mpu6050返回的z轴角速度值
 */
float Angle_Yaw(float mpu_gyro_z)
{
    Yaw = Yaw + (2000*mpu_gyro_z/32768) * 0.008;
    return Yaw;
}


/* brief  : 返回Pitch角度
 * Param1 : mpu_acc_z : mpu6050返回的z轴角加速度值
 * Param2 : mpu_acc_x : mpu6050返回的x轴角加速度值
 */
float Angle_Pitch(float mpu_acc_z,float mpu_acc_x)
{
    Pitch = atan2(8*9.8*(mpu_acc_z)/32768,8*9.8*(mpu_acc_x)/32768)*180.0/3.14;
    return Pitch;
}


/* brief  : 返回Roll角度
 * Param1 : mpu_acc_z : mpu6050返回的z轴角加速度值
 * param2 : mpu_acc_y : mpu6050返回的y轴角加速度值
 */
float Angle_Roll(float mpu_acc_z,float mpu_acc_y)
{
    Roll = atan2(8*9.8*(mpu_acc_y)/32768,8*9.8*(mpu_acc_z)/32768)*180.0/3.14;
    return Roll;
}

