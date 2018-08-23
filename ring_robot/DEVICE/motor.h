/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : motor.h
*Auther         : ����
*Version        : v1.0
*Date           : 2017-11-04
*Description    : ���ļ�������motor.c�����еĺ���ԭ��
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
 
void AllMotorInit(void);         		 /* ��ʼ�������˵����е��   */
void Edge4MotorInit(void);       		 /* ��ʼ����Ե4�����        */
void Mid2MotorInit(void);        		 /* ��ʼ���м�2�����        */
void ClimbMotorInit(void);       		 /* ��ʼ�������̨�����ĵ�� */
void ServoInit(void);             		 /* ��ʼ�����SD5            */

void ClimbSensorInit(void);				 /* ��Ϊ��̨���������λ�÷���*/

void RobotMoveContrl(INT8U type);        /* ����������״̬����       */
void Servo1Control(INT16U servolAngleVal);/* ����Ƕȿ���         */
void ClimbMotorControl(INT16U climbVal); /* ��̨����ٶȿ���         */
void SixMotorControl(INT16U climbVal);   /* ��·����ٶȿ���         */

#if 0 
void DcMotorInit(void);      //��ʼ��ֱ�����
void Motor1SpeedOut(u16 motor1SpeedVal);
void Motor2SpeedOut(u16 motor2SpeedVal);
void Motor3SpeedOut(u16 motor3SpeedVal);
void Motor4SpeedOut(u16 motor4SpeedVal);
#endif

#endif
