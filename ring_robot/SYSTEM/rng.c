/*************************************************************
*     Open source but not allowed to modify
*File name      : rng.c
*Auther         : 张沁
*Version        : v1.0
*Date           : 2016-10-11
*Description    : 该文件包含了RNG（随机数发生器）的初始化
*Function List  : 
*************************************************************/

#include "myinclude.h"

/*************************************************************
*Function Name  : RNG_Init
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2016-10-11
*Description    : 初始化RNG
*Input          ：
*Output         ：
*Return         ：0：工作正常   1：工作不正常
*************************************************************/
u8 RNG_Init(void)
{
	u16 retry = 0;
	
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);  //初始化时钟
	RNG_Cmd(ENABLE);  //使能RNG
	
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET && retry < 10000)  //等待随机数就绪
	{
		retry++;
		delay_us(100);
	}
	
	if(retry >= 10000)
		return 1;  //随机数产生器工作不正常
	return 0;
}


/*************************************************************
*Function Name  : RNG_Get_RandomNum
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2016-10-11
*Description    : 产生随机数
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
u32 RNG_Get_RandomNum(void)
{
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET);  //等待随机数就绪
	
	return RNG_GetRandomNumber();  //返回产生的随机数
}


/*************************************************************
*Function Name  : RNG_Get_RandomRange
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2016-10-11
*Description    : 得到指定范围的随机数
*Input          ：1. main: 指定范围的最小值
				  2. max:  指定范围的最大值
*Output         ：
*Return         ：
*************************************************************/
int RNG_Get_RandomRange(int min, int max)
{
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET);  //等待随机数就绪
	
	return(RNG_GetRandomNumber()%(max - min + 1) + min);
}
