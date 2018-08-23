/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : MyTypeDef.H
*Auther         : ����
*Version        : v1.0
*Date           : 2017-10-31
*Description    : ͨ���������Ͷ�����(��ȫ�ֱ�����)
*Function List  : 
*************************************************************/
#ifndef _MYTYPEDEF_H_
#define _MYTYPEDEF_H_

#include "includes.h"

/***ȫ�ֱ���������***/
#ifdef MY_CPP_GLOBALS
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN INT8U sensorState;     /* ���������󷵻صĻ����˱�Ե���桢�������λ��					  */
EXTERN INT8U robotState;      /* ��̨����������״̬                                				  */
EXTERN INT8U robotRotationDir;/* ��⵽��̨��Ե�󣬻��������Ⱥ��ˣ�Ȼ����ת����ת�����ɸñ�־���� */
EXTERN INT8U climbSensorVal;  /* ��Ϊ��̨���������λ�÷�������(������Ϊ20��) 					  */

/*
speedSwitchFlag: �ٶ���ʾ���޸ı�־λ
Bit7: 1 ��������޸Ĳ���  0 ��ֹ�����޸�
Bit5: 1 ����ʾ���޸�λ�ö�λ��"rotation"(��ת�ٶ�) (�м��������)
Bit4: 1 ����ʾ���޸�λ�ö�λ��"attack"  (�����ٶ�) (�м��������)
Bit3: 1 ����ʾ���޸�λ�ö�λ��"patrol"  (Ѳ���ٶ�) (�м��������)
Bit2: 1 ����ʾ���޸�λ�ö�λ��"rotation"(��ת�ٶ�) (��Ե�ĸ����)
Bit1: 1 ����ʾ���޸�λ�ö�λ��"attack"  (�����ٶ�) (��Ե�ĸ����)
Bit0: 1 ����ʾ���޸�λ�ö�λ��"patrol"  (Ѳ���ٶ�) (��Ե�ĸ����)
*/ 
EXTERN INT8U speedSwitchFlag;

/*
speedSwitchFlag: �ٶ���ʾ���޸ı�־λ
Bit7: 1 ����ʾ���޸�λ�ö�λ��"TurnR"  (����ģʽ�£�����ת)
Bit6: 1 ����ʾ���޸�λ�ö�λ��"TurnL"  (����ģʽ�£�����ת)
Bit5: 1 ����ʾ���޸�λ�ö�λ��"Back"   (����ģʽ�£������)
Bit4: 1 ����ʾ���޸�λ�ö�λ��"Ahead"  (����ģʽ�£���ǰ��)
Bit3: 1 ����ʾ���޸�λ�ö�λ��"Fight"  (��սģʽ          )
Bit2: 1 ����ʾ���޸�λ�ö�λ��"Push"   (��ľ��ģʽ        )
Bit1: 1 ����ʾ���޸�λ�ö�λ��"Climb"  (��̨ģʽ          )
Bit0: 1 ����ʾ���޸�λ�ö�λ��"Debug"  (����ģʽ          )
*/ 
EXTERN INT8U modeSwitchFlag;


/**************��̨�����������ٶ� ��ز���������*****************/
typedef struct SPEED_SELECT
{
	INT16U patrol;             //Ѳ���ٶ�
	INT16U attack;             //�����ٶ�
	INT16U rotation;     	   //��ת�ٶ�
} _SPEED_SELECT;
EXTERN _SPEED_SELECT speed1Select, *p_speed1Select; 
EXTERN _SPEED_SELECT speed2Select, *p_speed2Select;
/**************************************************************/


/******************�������ݶ�����(From K60)*********************/
typedef struct RECEIVE_DATA
{
	INT32U photoelectricSensor;//���Թ�����
	INT16S anglex;             //x��Ƕ�
	INT16S angley;             //y��Ƕ�
	INT16S anglez, setAnglez;  //z��Ƕȡ���һʱ�̻�����Ӧ�����е���z��Ƕ�
} _RECEIVE_DATA;
EXTERN _RECEIVE_DATA receiveData, *p_receiveData;
/**************************************************************/


#endif//_MYTYPEDEF_H_
