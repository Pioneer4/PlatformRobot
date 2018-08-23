/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : savedata.c
*Auther         : 张沁
*Version        : v1.0
*Date           : 2017-10-31
*Description    : 该文件包含了数据存储、读取等操作
*Function List  : 
*************************************************************/

#include "myinclude.h"
#include "includes.h"


/*************************************************************
*Function Name  : SpeedSaveData
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-10-09
*Description    : 保存设定的速度参数
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
//保存FLASH里面的地址区间基址,占用24个字节(RANGE:SPEED_SAVE_ADDR_BASE~SPEED_SAVE_ADDR_BASE+23)
#define  SPEED_SAVE_ADDR_BASE  20
void SpeedSaveData(void)
{
	INT8U result_data[12];  //存储缓冲区
	INT16U temp = 0;
	
	/******将p_speed1Select->(patrol、attack、rotation)换为存储数据******/
	/*p_speed1Select->patrol*/
	temp = p_speed1Select->patrol;
	result_data[0] =  (u8)(temp >> 8);
	result_data[1] =  (u8)temp;	
	/*p_speed1Select->attack*/
	temp = p_speed1Select->attack;
	result_data[2] =  (u8)(temp >> 8);
	result_data[3] =  (u8)temp;
	/*p_speed1Select->rotation*/
	temp = p_speed1Select->rotation;
	result_data[4] =  (u8)(temp >> 8);
	result_data[5] =  (u8)temp;
	
	/*p_speed2Select->patrol*/
	temp = p_speed2Select->patrol;
	result_data[6] =  (u8)(temp >> 8);
	result_data[7] =  (u8)temp;	
	/*p_speed2Select->attack*/
	temp = p_speed2Select->attack;
	result_data[8] =  (u8)(temp >> 8);
	result_data[9] =  (u8)temp;
	/*p_speed2Select->rotation*/
	temp = p_speed2Select->rotation;
	result_data[10] =  (u8)(temp >> 8);
	result_data[11] =  (u8)temp;
	
	W25QXX_Write(result_data, SPEED_SAVE_ADDR_BASE, 12);  //校正结果写入FLASH
}


/*************************************************************
*Function Name  : SpeedGetData
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-10-31
*Description    : 得到保存在FLASH里面的速度参数
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void SpeedGetData(void)
{
	INT8U result_data[12];  //存储缓冲区
	INT16U temp = 0;
	
	W25QXX_Read(result_data, SPEED_SAVE_ADDR_BASE, 12);  //读取FLASH中的数据
	
	/*得到p_speed1Select->patrol*/
	temp = result_data[0];
	temp = (temp << 8) + result_data[1];
	p_speed1Select->patrol = temp;
	/*得到p_speed1Select->attack*/
	temp = result_data[2];
	temp = (temp << 8) + result_data[3];
	p_speed1Select->attack = temp;
	/*得到p_speed1Select->rotation*/
	temp = result_data[4];
	temp = (temp << 8) + result_data[5];
	p_speed1Select->rotation = temp;
	
	/*得到p_speed2Select->patrol*/
	temp = result_data[6];
	temp = (temp << 8) + result_data[7];
	p_speed2Select->patrol = temp;
	/*得到p_speed2Select->attack*/
	temp = result_data[8];
	temp = (temp << 8) + result_data[9];
	p_speed2Select->attack = temp;
	/*得到p_speed2Select->rotation*/
	temp = result_data[10];
	temp = (temp << 8) + result_data[11];
	p_speed2Select->rotation = temp;
}
