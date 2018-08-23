/*************************************************************
*     Open source but not allowed to modify
*File name      : rng.c
*Auther         : ����
*Version        : v1.0
*Date           : 2016-10-11
*Description    : ���ļ�������RNG����������������ĳ�ʼ��
*Function List  : 
*************************************************************/

#include "myinclude.h"

/*************************************************************
*Function Name  : RNG_Init
*Auther         : ����
*Vertion        : v1.0
*Date           : 2016-10-11
*Description    : ��ʼ��RNG
*Input          ��
*Output         ��
*Return         ��0����������   1������������
*************************************************************/
u8 RNG_Init(void)
{
	u16 retry = 0;
	
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);  //��ʼ��ʱ��
	RNG_Cmd(ENABLE);  //ʹ��RNG
	
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET && retry < 10000)  //�ȴ����������
	{
		retry++;
		delay_us(100);
	}
	
	if(retry >= 10000)
		return 1;  //���������������������
	return 0;
}


/*************************************************************
*Function Name  : RNG_Get_RandomNum
*Auther         : ����
*Vertion        : v1.0
*Date           : 2016-10-11
*Description    : ���������
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
u32 RNG_Get_RandomNum(void)
{
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET);  //�ȴ����������
	
	return RNG_GetRandomNumber();  //���ز����������
}


/*************************************************************
*Function Name  : RNG_Get_RandomRange
*Auther         : ����
*Vertion        : v1.0
*Date           : 2016-10-11
*Description    : �õ�ָ����Χ�������
*Input          ��1. main: ָ����Χ����Сֵ
				  2. max:  ָ����Χ�����ֵ
*Output         ��
*Return         ��
*************************************************************/
int RNG_Get_RandomRange(int min, int max)
{
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET);  //�ȴ����������
	
	return(RNG_GetRandomNumber()%(max - min + 1) + min);
}
