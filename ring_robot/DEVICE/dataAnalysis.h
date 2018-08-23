/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : dataAnalysis.h
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-02
*Description    : ���ļ�����dataAnalysis.c�����еĺ���ԭ��
*Function List  : 
*************************************************************/
#ifndef __DATAANALYSIS_H
#define __DATAANALYSIS_H

#include "myinclude.h"

#define TURN_LEFT    1   /* ����ת */
#define TURN_RIGHT   2   /* ����ת */

INT8U ElecSensorRead(u8 sensorNum);
void SensorShowCircle(u8 sensorNum);
void SensorAllClrCircle(void);
void SetRotationAngle(u8 type, INT16U angle);
INT8U RotationJudgeOk(u8 type);


#endif
