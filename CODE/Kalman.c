/*
 * Kalman.c
 *
 *  Created on: 2022��1��13��
 *      Author: xqx
 */

#include "Kalman.h"
/*Pitch�˲�����
 *
 */
float dt = 0.008;     //����ʱ��

float Angle;              //�Ƕȵ��������ֵ
float Gyro_bias = 0.1;          //���ٶ�ƫ��ֵ

float Cov_angle = 0.01;      //�����ǲ����Ľ��ٶȵ�Э����
float Cov_bias = 0.01;       //������Ư��ֵ��Э����
float Cov_accel = 0.01;       //���ٶȼƲ����ļ��ٶ�Э����

//P����
float P00 = 1.0;
float P01 = 0.0;
float P10 = 0.0;
float P11 = 1.0;

//�������������
float K0;
float K1;

float Angle_err;


//���������ǽ��ٶȺͼ���Ƕ�
float Kalman_filter_Pitch(float Gyro ,float Angle_See)
{
    /*״̬����
     * | Angle     |   | 1   -dt| | Angle     |   | dt|
     * |           | = |        | |           | + |   | Gyro;
     * | Gyro_bias |   | 0     1| | Gyro_bias |   | 0 |
     */
    Angle = Angle - Gyro_bias * dt + Gyro * dt;
    Gyro_bias = Gyro_bias;

    //P�������
    P00 = P00 - P10 * dt + (P11 * dt - P01) * dt + Cov_angle;
    P01 = P01 - P11 * dt;
    P10 = P10 - P11 * dt;
    P11 = P11 + Cov_bias;

    //��⿨�����������
    K0 = P00 / (P00 + Cov_accel);
    K1 = P10 / (P00 + Cov_accel);

    Angle_err = Angle_See - Angle;

    //��������
    Angle = Angle + K0 * Angle_err;
    Gyro_bias = Gyro_bias + K1 * Angle_err;

    //����P����
    P00 = P00 - K0 * P00;
    P01 = P01 - K0 * P01;
    P10 = P10 - K1 * P00;
    P11 = P11 - K1 * P01;

    return Angle;
}

/*Roll�˲�����
 *
 *
 */
float dt_1 = 0.008;     //����ʱ��

float Angle_1;              //�Ƕȵ��������ֵ
float Gyro_bias_1 = 0.1;          //���ٶ�ƫ��ֵ

float Cov_angle_1 = 0.01;      //�����ǲ����Ľ��ٶȵ�Э����
float Cov_bias_1 = 0.01;       //������Ư��ֵ��Э����
float Cov_accel_1 =0.01;       //���ٶȼƲ����ļ��ٶ�Э����

//P����
float P00_1 = 1.0;
float P01_1 = 0.0;
float P10_1 = 0.0;
float P11_1 = 1.0;

//�������������
float K0_1;
float K1_1;

float Angle_err_1;

float Kalman_filter_Roll(float Gyro_1 ,float Angle_See_1)
{
    Angle_1 = Angle_1 - Gyro_bias_1 * dt_1 + Gyro_1 * dt;
    Gyro_bias_1 = Gyro_bias_1;

    //P�������
    P00_1 = P00_1 - P10_1 * dt_1 + (P11_1 * dt_1 - P01_1) * dt_1 + Cov_angle_1;
    P01_1 = P01_1 - P11_1 * dt_1;
    P10_1 = P10_1 - P11_1 * dt_1;
    P11_1 = P11_1 + Cov_bias_1;

    //��⿨�����������
    K0_1 = P00_1 / (P00_1 + Cov_accel_1);
    K1_1 = P10_1 / (P00_1 + Cov_accel_1);

    Angle_err_1 = Angle_See_1 - Angle_1;

    //��������
    Angle_1 = Angle_1 + K0_1 * Angle_err_1;
    Gyro_bias_1 = Gyro_bias + K1_1 * Angle_err_1;

    //����P����
    P00_1 = P00_1 - K0_1 * P00_1;
    P01_1 = P01_1 - K0_1 * P01_1;
    P10_1 = P10_1 - K1_1 * P00_1;
    P11_1 = P11_1 - K1_1 * P01_1;

    return Angle_1;
}

//Yaw�˲�����
float dt_2 = 0.008;     //����ʱ��

float Angle_2;              //�Ƕȵ��������ֵ
float Gyro_bias_2 = 0.1;          //���ٶ�ƫ��ֵ

float Cov_angle_2 = 0.01;      //�����ǲ����Ľ��ٶȵ�Э����
float Cov_bias_2 = 0.01;       //������Ư��ֵ��Э����
float Cov_accel_2 = 0.01;       //���ٶȼƲ����ļ��ٶ�Э����

//P����
float P00_2 = 1.0;
float P01_2 = 0.0;
float P10_2 = 0.0;
float P11_2 = 1.0;

//�������������
float K0_2;
float K1_2;

float Angle_err_2;
float Gyro_calculate; //�˲���Ľ��ٶ�

float Kalman_filter_Yaw(float Gyro_2 ,float Angle_See_2) //���������ǽ��ٶȺͼ���Ƕ�
{
    /*״̬����
     * | Angle     |   | 1   -dt| | Angle     |   | dt|
     * |           | = |        | |           | + |   | Gyro;
     * | Gyro_bias |   | 0     1| | Gyro_bias |   | 0 |
     */
    Angle_2 = Angle_2 - Gyro_bias_2 * dt_2 + Gyro_2 * dt_2;
    Gyro_bias_2 = Gyro_bias_2;

    //P�������
    P00_2 = P00_2 - P10_2 * dt_2 + (P11_2 * dt_2 - P01_2) * dt_2 + Cov_angle_2;
    P01_2 = P01_2 - P11_2 * dt_2;
    P10_2 = P10_2 - P11_2 * dt_2;
    P11_2 = P11_2 + Cov_bias_2;

    //��⿨�����������
    K0_2 = P00_2 / (P00_2 + Cov_accel_2);
    K1_2 = P10_2 / (P00_2 + Cov_accel_2);

    Angle_err_2 = Angle_See_2 - Angle_2;

    //��������
    Angle_2 = Angle_2 + K0_2 * Angle_err_2;
    Gyro_bias_2 = Gyro_bias_2 + K1_2 * Angle_err_2;
    Gyro_calculate = Gyro_2 - Gyro_bias_2;

    //����P����
    P00_2 = P00_2 - K0_2 * P00_2;
    P01_2 = P01_2 - K0_2 * P01_2;
    P10_2 = P10_2 - K1_2 * P00_2;
    P11_2 = P11_2 - K1_2 * P01_2;

    return Angle_2;
}
