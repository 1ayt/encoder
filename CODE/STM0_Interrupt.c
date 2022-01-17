/*
 * STM0_Interrupt.c
 *
 *  Created on: 2022Äê1ÔÂ10ÈÕ
 *      Author: xqx
 */

#include "STM0_Interrupt.h"
#include "SEEKFREE_MPU6050.h"
#include "Kalman.h"
#include "Angle_Calculate.h"

uint8 flag_handle;
float Yaw_Angle;

IFX_INTERRUPT(isrSTM, 0, ISR_PRIORITY_STM);

IfxStm_CompareConfig g_STMConf;                                 /* STM configuration structure                      */
Ifx_TickTime g_ticksFor500ms;                                   /* Variable to store the number of ticks to wait    */

void isrSTM(void)
{
    /* Update the compare register value that will trigger the next interrupt and toggle the LED */
    IfxStm_increaseCompare(STM, g_STMConf.comparator, g_ticksFor500ms);
    if(flag_handle == 1)
    {

        get_accdata();
        get_gyro();
        flag_handle = 0;
        //Yaw_Angle =  Angle_Yaw(mpu_gyro_z);

    }

}

/* Function to initialize the LED */
void initLED(void)
{
    IfxPort_setPinMode(LED, IfxPort_Mode_outputPushPullGeneral);    /* Initialize LED port pin                      */
    IfxPort_setPinState(LED, IfxPort_State_high);                   /* Turn off LED (LED is low-level active)       */
}

/* Function to initialize the STM */
void initSTM(void)
{
    IfxStm_initCompareConfig(&g_STMConf);           /* Initialize the configuration structure with default values   */

    g_STMConf.triggerPriority = ISR_PRIORITY_STM;   /* Set the priority of the interrupt                            */
    g_STMConf.typeOfService = IfxSrc_Tos_cpu0;      /* Set the service provider for the interrupts                  */
    g_STMConf.ticks = g_ticksFor500ms;              /* Set the number of ticks after which the timer triggers an
                                                     * interrupt for the first time                                 */
    IfxStm_initCompare(STM, &g_STMConf);            /* Initialize the STM with the user configuration               */
}

/* Function to initialize all the peripherals and variables used */
void initPeripherals(void)
{
    /* Initialize time constant */
    g_ticksFor500ms = IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, TIMER_INT_TIME);

    initLED();                                      /* Initialize the port pin to which the LED is connected        */
    initSTM();                                      /* Configure the STM module                                     */
}

