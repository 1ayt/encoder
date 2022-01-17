/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/


#include "headfile.h"
#include "encoder.h"
//#include "PID_attitude.h"
#include "STM0_interrupt.h"
#include "Kalman.h"
#include "PID_High.h"
#include "Angle_Calculate.h"
#include "SEEKFREE_MPU6050.h"
#pragma section all "cpu0_dsram"


int count = 0;
extern int count_right;
extern int count_left;
extern int count_r;
extern int count_l;

extern uint8 flag_handle;
extern int16 mpu_acc_z;
extern int16 mpu_acc_y;

float i;
float w;
float m = 45.0;
float z=45.0;

int core0_main(void)
{
	disableInterrupts();
	get_clk();//获取时钟频率  务必保留

    //用户在此处调用各种初始化函数等

	pit_init(CCU6_0,PIT_CH0,100000);
	eru_init(ERU_CH0_REQ0_P15_4,RISING);

	mpu6050_init();
	gpio_init(P00_9, GPI, 0, NO_PULL);

    enableInterrupts();
    initPeripherals();

    pit_enable_interrupt(CCU6_0,PIT_CH0);
    eru_enable_interrupt(ERU_CH0_REQ0_P15_4);

    pit_start(CCU6_0,PIT_CH0);

    while (TRUE)
    {

        printf("\r\n右转速度：%d   ",count_r);
        printf("左转速度：%d   ",count_l);
        //pid_OutputValue
        printf("Roll : %f   ",Kalman_filter_Roll(Gyro_x((float)mpu_gyro_x),Angle_Roll((float)mpu_acc_z,(float)mpu_acc_y)));


        //i = PID_Calculate(z,90.0,1000,1,0.1,0.1,0.05);
        //w = PID_Calculate(15.0,i,1000,1,0.1,0.1,0.05);

        //w = 0.1 * PID_Calculate(m,i,1000,1,0.1,0.1,0.05);

        //m = m + w;

        //z = z + m;

        //printf("\r\ni : %f   ",i);
        //printf("w : %f   ",w);
        //printf("m : %f   ",m);
        //printf("z : %f   ",z);
    	systick_delay_ms(STM0, 50);

    	flag_handle = 1;

   }
}

#pragma section all restore
