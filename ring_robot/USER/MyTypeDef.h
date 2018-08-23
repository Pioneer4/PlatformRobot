/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : MyTypeDef.H
*Auther         : 张沁
*Version        : v1.0
*Date           : 2017-10-31
*Description    : 通用数据类型定义区(如全局变量等)
*Function List  : 
*************************************************************/
#ifndef _MYTYPEDEF_H_
#define _MYTYPEDEF_H_

#include "includes.h"

/***全局变量定义区***/
#ifdef MY_CPP_GLOBALS
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN INT8U sensorState;     /* 传感器检测后返回的机器人边缘警告、物体出现位置					  */
EXTERN INT8U robotState;      /* 擂台机器人运行状态                                				  */
EXTERN INT8U robotRotationDir;/* 检测到擂台边缘后，机器人首先后退，然后自转，旋转方向由该标志决定 */
EXTERN INT8U climbSensorVal;  /* 作为登台机构电机的位置反馈变量(光码盘为20分) 					  */

/*
speedSwitchFlag: 速度显示、修改标志位
Bit7: 1 允许进行修改操作  0 禁止进行修改
Bit5: 1 将显示和修改位置定位到"rotation"(自转速度) (中间两个电机)
Bit4: 1 将显示和修改位置定位到"attack"  (攻击速度) (中间两个电机)
Bit3: 1 将显示和修改位置定位到"patrol"  (巡航速度) (中间两个电机)
Bit2: 1 将显示和修改位置定位到"rotation"(自转速度) (边缘四个电机)
Bit1: 1 将显示和修改位置定位到"attack"  (攻击速度) (边缘四个电机)
Bit0: 1 将显示和修改位置定位到"patrol"  (巡航速度) (边缘四个电机)
*/ 
EXTERN INT8U speedSwitchFlag;

/*
speedSwitchFlag: 速度显示、修改标志位
Bit7: 1 将显示和修改位置定位到"TurnR"  (调试模式下：向右转)
Bit6: 1 将显示和修改位置定位到"TurnL"  (调试模式下：向左转)
Bit5: 1 将显示和修改位置定位到"Back"   (调试模式下：向后走)
Bit4: 1 将显示和修改位置定位到"Ahead"  (调试模式下：向前走)
Bit3: 1 将显示和修改位置定位到"Fight"  (对战模式          )
Bit2: 1 将显示和修改位置定位到"Push"   (推木块模式        )
Bit1: 1 将显示和修改位置定位到"Climb"  (登台模式          )
Bit0: 1 将显示和修改位置定位到"Debug"  (调试模式          )
*/ 
EXTERN INT8U modeSwitchFlag;


/**************擂台机器人运行速度 相关参数定义区*****************/
typedef struct SPEED_SELECT
{
	INT16U patrol;             //巡逻速度
	INT16U attack;             //攻击速度
	INT16U rotation;     	   //自转速度
} _SPEED_SELECT;
EXTERN _SPEED_SELECT speed1Select, *p_speed1Select; 
EXTERN _SPEED_SELECT speed2Select, *p_speed2Select;
/**************************************************************/


/******************接收数据定义区(From K60)*********************/
typedef struct RECEIVE_DATA
{
	INT32U photoelectricSensor;//光电对管数据
	INT16S anglex;             //x轴角度
	INT16S angley;             //y轴角度
	INT16S anglez, setAnglez;  //z轴角度、下一时刻机器人应该运行到的z轴角度
} _RECEIVE_DATA;
EXTERN _RECEIVE_DATA receiveData, *p_receiveData;
/**************************************************************/


#endif//_MYTYPEDEF_H_
