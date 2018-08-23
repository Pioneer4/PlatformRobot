/*************************************************************
*     Open source but not allowed to modify
*File name      : rng.h
*Auther         : ����
*Version        : v1.0
*Date           : 2016-10-11
*Description    : ���ļ�������rng.c�е����к���ԭ��
*Function List  : 
*************************************************************/
#ifndef __RNG_H
#define __RNG_H

#include "stm32f4xx.h"
#include "delay.h"

u8 RNG_Init(void);  //��ʼ��RNG
u32 RNG_Get_RandomNum(void);  //�õ������
int RNG_Get_RandomRange(int min, int max);  //�õ�ָ����Χ�������
	
#endif

