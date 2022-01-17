/*
 * Angle_Calculate.c
 *
 *  Created on: 2022��1��14��
 *      Author: xqx
 */

#include "Angle_Calculate.h"
#include "SEEKFREE_MPU6050.h"

//Yaw,Pitch,Roll �Ƕ�ֵ
float Yaw,Pitch,Roll;

//����mpu6050.h�ļ��еı���
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;

/* brief  : ����x��Ǽ��ٶ�
 * Param1 : mpu_acc_x : mpu6050���ص�x��Ǽ��ٶ�ֵ
 */
float Accel_x(float mpu_acc_x)
{
    float Accel_x;

    Accel_x = 8*9.8*(mpu_acc_x)/32768;

    return Accel_x;
}

/* brief  : ����y��Ǽ��ٶ�
 * Param1 : mpu_acc_y : mpu6050���ص�y��Ǽ��ٶ�ֵ
 */
float Accel_y(float mpu_acc_y)
{
    float Accel_y;

    Accel_y = 8*9.8*(mpu_acc_y)/32768;

    return Accel_y;
}

/* brief  : ����z��Ǽ��ٶ�
 * Param1 : mpu_acc_z : mpu6050���ص�z��Ǽ��ٶ�ֵ
 */
float Accel_z(float mpu_acc_z)
{
    float Accel_z;

    Accel_z = 8*9.8*(mpu_acc_z)/32768;

    return Accel_z;
}

/* brief  : ����x����ٶ�
 * Param1 : mpu_gyro_x : mpu6050���ص�x����ٶ�ֵ
 */
float Gyro_x(float mpu_gyro_x)
{
    float Gyro_x;

    Gyro_x = 2000*mpu_gyro_x/32768;

    return Gyro_x;
}

/* brief  : ����y����ٶ�
 * Param1 : mpu_gyro_y : mpu6050���ص�y����ٶ�ֵ
 */
float Gyro_y(float mpu_gyro_y)
{
    float Gyro_y;

    Gyro_y = 2000*mpu_gyro_y/32768;

    return Gyro_y;
}

/* brief  : ����z����ٶ�
 * Param1 : mpu_gyro_z : mpu6050���ص�z����ٶ�ֵ
 */
float Gyro_z(float mpu_gyro_z)
{
    float Gyro_z;

    Gyro_z = 2000*mpu_gyro_z/32768;

    return Gyro_z;
}
/* brief  : ����Yaw�Ƕ�
 * Param1 : mpu_gyro_z : mpu6050���ص�z����ٶ�ֵ
 */
float Angle_Yaw(float mpu_gyro_z)
{
    Yaw = Yaw + (2000*mpu_gyro_z/32768) * 0.008;
    return Yaw;
}


/* brief  : ����Pitch�Ƕ�
 * Param1 : mpu_acc_z : mpu6050���ص�z��Ǽ��ٶ�ֵ
 * Param2 : mpu_acc_x : mpu6050���ص�x��Ǽ��ٶ�ֵ
 */
float Angle_Pitch(float mpu_acc_z,float mpu_acc_x)
{
    Pitch = atan2(8*9.8*(mpu_acc_z)/32768,8*9.8*(mpu_acc_x)/32768)*180.0/3.14;
    return Pitch;
}


/* brief  : ����Roll�Ƕ�
 * Param1 : mpu_acc_z : mpu6050���ص�z��Ǽ��ٶ�ֵ
 * param2 : mpu_acc_y : mpu6050���ص�y��Ǽ��ٶ�ֵ
 */
float Angle_Roll(float mpu_acc_z,float mpu_acc_y)
{
    Roll = atan2(8*9.8*(mpu_acc_y)/32768,8*9.8*(mpu_acc_z)/32768)*180.0/3.14;
    return Roll;
}

