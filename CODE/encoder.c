/*
 * encoder.c
 *
 *  Created on: 2022Äê1ÔÂ15ÈÕ
 *      Author: xqx
 */

#include "encoder.h"

void encoder_read_right(int* a,int count_right)
{
    *a = count_right;
    count_right = 0;
}


void encoder_read_left(int* a,int count_left)
{
    *a = count_left;
    count_left = 0;
}

void encoder_count_reset(int* count)
{
    *count = 0;
}
