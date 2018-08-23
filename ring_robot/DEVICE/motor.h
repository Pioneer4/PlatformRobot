/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : motor.h
*Auther         : 张沁
*Version        : v1.0
*Date           : 2017-11-04
*Description    : 该文件包含了motor.c中所有的函数原型
*Function List  : 
*************************************************************/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "myinclude.h"

#define MOTOR1_PIN1 PAout(4)
#define MOTOR1_PIN2 PAout(5)

#define MOTOR_FORWARD  2
#define MOTOR_BACKWARD 1
#define MOTOR_STOP     0
 
void AllMotorInit(void);         		 /* 初始化机器人的所有电机   */
void Edge4MotorInit(void);       		 /* 初始化边缘4个电机        */
void Mid2MotorInit(void);        		 /* 初始化中间2个电机        */
void ClimbMotorInit(void);       		 /* 初始化负责登台机构的电机 */
void ServoInit(void);             		 /* 初始化舵机SD5            */

void ClimbSensorInit(void);				 /* 作为登台机构电机的位置反馈*/

void RobotMoveContrl(INT8U type);        /* 机器人运行状态控制       */
void Servo1Control(INT16U servolAngleVal);/* 舵机角度控制         */
void ClimbMotorControl(INT16U climbVal); /* 登台电机速度控制         */
void SixMotorControl(INT16U climbVal);   /* 六路电机速度控制         */

#if 0 
void DcMotorInit(void);      //初始化直流电机
void Motor1SpeedOut(u16 motor1SpeedVal);
void Motor2SpeedOut(u16 motor2SpeedVal);
void Motor3SpeedOut(u16 motor3SpeedVal);
void Motor4SpeedOut(u16 motor4SpeedVal);
#endif

#endif
