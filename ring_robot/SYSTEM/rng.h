/*************************************************************
*     Open source but not allowed to modify
*File name      : rng.h
*Auther         : 张沁
*Version        : v1.0
*Date           : 2016-10-11
*Description    : 该文件包含了rng.c中的所有函数原型
*Function List  : 
*************************************************************/
#ifndef __RNG_H
#define __RNG_H

#include "stm32f4xx.h"
#include "delay.h"

u8 RNG_Init(void);  //初始化RNG
u32 RNG_Get_RandomNum(void);  //得到随机数
int RNG_Get_RandomRange(int min, int max);  //得到指定范围的随机数
	
#endif

