
/**********************************************************************************************************************
 * \file    STM0_Interrupt.h
 * \brief
 * \version V1.0.0
 * \date    2022Äê1ÔÂ10ÈÕ
 * \author  xqx
 *********************************************************************************************************************/


#ifndef LIBRARIES_STM0_INTERRUPT_H_
#define LIBRARIES_STM0_INTERRUPT_H_

#include <Stm/Std/IfxStm.h>
#include "Bsp.h"
#include "IfxPort.h"
#include "IfxStm.h"

#define ISR_PRIORITY_STM        40                              /* Priority for interrupt ISR                       */
#define TIMER_INT_TIME          8                             /* Time between interrupts in ms                    */

#define LED                     &MODULE_P33,8                   /* LED toggled in Interrupt Service Routine (ISR)   */
#define STM                     &MODULE_STM0                    /* STM0 is used in this example                     */


void initPeripherals(void);

#endif
