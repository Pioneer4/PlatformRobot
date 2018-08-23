/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : dataAnalysis.c
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-02
*Description    : ���ļ������˴���������(From K60)�Ĵ�������
*Function List  : 
*************************************************************/

#include "myinclude.h"

/*************************************************************
*Function Name  : ElecSensorRead
*Auther         : ����
*Version        : V1.0
*Date           : 2017-11-02
*Description    : ��ȡ��̨�����˹��Թܵļ��ֵ
*Input          ��sensorNum  Ҫ��ȡ�Ĺ��Թܱ��(1--16)
*Output         ��
*Return         ��0����⵽����  1��û��⵽����
*************************************************************/
INT8U ElecSensorRead(u8 sensorNum)
{
	sensorNum--;  //���ں������λ����
	
	if ((p_receiveData->photoelectricSensor & (0x01 << sensorNum)) == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


/*************************************************************
*Function Name  : SensorShowCircle
*Auther         : ����
*Version        : V1.0
*Date           : 2017-11-02
*Description    : ��sensorNum�Ź��Թܼ�⵽���壬LCD���ڶ�Ӧ
*				  �ı������ʾʵ��Բ
*Input          ��sensorNum  Ҫ��Բ�Ĺ��Թܱ��(1--16)
*Output         ��
*Return         ��
*************************************************************/
void SensorShowCircle(u8 sensorNum)
{
	switch (sensorNum)
	{
		/*�ĸ������Ե���Ĺ�翪��*/
		case  1: Show_Circle(87,  194, 3, BLUE);  break;
		case  2: Show_Circle(153, 194, 3, BLUE);  break;
		case  3: Show_Circle(87,  266, 3, BLUE);  break;
		case  4: Show_Circle(153, 266, 3, BLUE);  break;
		/*�ĸ�����ǰ���Ĺ�翪��*/
		case  5: Show_Circle(97,  175, 3, BLUE);  break;
		case  6: Show_Circle(113, 175, 3, BLUE);  break;
		case  7: Show_Circle(127, 175, 3, BLUE);  break;
		case  8: Show_Circle(142, 175, 3, BLUE);  break;
		/*�ĸ�����󷽵Ĺ�翪��*/
		case  9: Show_Circle(97,  285, 3, BLUE);  break;
		case 10: Show_Circle(113, 285, 3, BLUE);  break;
		case 11: Show_Circle(127, 285, 3, BLUE);  break;
		case 12: Show_Circle(142, 285, 3, BLUE);  break;
		/*���������󷽵Ĺ�翪��*/
		case 13: Show_Circle(75,  222, 3, BLUE);  break;
		//case 14: Show_Circle(75,  230, 3, BLUE);  break;
		case 14: Show_Circle(75,  238, 3, BLUE);  break;
		/*���������ҷ��Ĺ�翪��*/
		case 15: Show_Circle(165, 222, 3, BLUE);  break;
		//case 17: Show_Circle(165, 230, 3, BLUE);  break;
		case 16: Show_Circle(165, 238, 3, BLUE);  break;
		default: break;
	}
}

/*************************************************************
*Function Name  : SensorAllClrCircle
*Auther         : ����
*Version        : V1.0
*Date           : 2017-11-02
*Description    : ���LCD�����й��Թܶ�Ӧ��ʵ��Բ
*Input          ��
*Return         ��
*************************************************************/
void SensorAllClrCircle(void)
{
		/*�ĸ������Ե���Ĺ�翪��*/
		Show_Circle(87,  194, 3, WHITE); 
		Show_Circle(153, 194, 3, WHITE); 
		Show_Circle(87,  266, 3, WHITE); 
		Show_Circle(153, 266, 3, WHITE);  
		/*�ĸ�����ǰ���Ĺ�翪��*/
		Show_Circle(97,  175, 3, WHITE);
		Show_Circle(113, 175, 3, WHITE); 
		Show_Circle(127, 175, 3, WHITE);
		Show_Circle(142, 175, 3, WHITE);
		/*�ĸ�����󷽵Ĺ�翪��*/
		Show_Circle(97,  285, 3, WHITE);
		Show_Circle(113, 285, 3, WHITE);
		Show_Circle(127, 285, 3, WHITE);
		Show_Circle(142, 285, 3, WHITE);
		/*���������󷽵Ĺ�翪��*/
		Show_Circle(75,  222, 3, WHITE);
		//Show_Circle(75,  230, 3, WHITE);
		Show_Circle(75,  238, 3, WHITE);
		/*���������ҷ��Ĺ�翪��*/
		Show_Circle(165, 222, 3, WHITE);
		//Show_Circle(165, 230, 3, WHITE);
		Show_Circle(165, 238, 3, WHITE);
}


/*************************************************************
*Function Name  : SetRotationAngle
*Auther         : ����
*Version        : V1.0
*Date           : 2017-11-03
*Description    : ���û�������ת������ת�Ƕ�
*Input          ��1.type ��ת����  @arg TURN_LEFT:��ת 
*								   @arg TURN_RIGHT:��ת 
*				  2.angle ��ת�Ƕ� @arg 0--360
*Return         ��
*************************************************************/
void SetRotationAngle(u8 type, INT16U angle)
{
	INT16S angleCalculate = 0;
	
	/*����ת*/
	if ( type == TURN_LEFT )
	{
		angleCalculate = p_receiveData->anglez + angle;
		if ( angleCalculate < 360 )     p_receiveData->setAnglez = angleCalculate;
		else if (angleCalculate == 360) p_receiveData->setAnglez = 0;
		else                            p_receiveData->setAnglez = angleCalculate - 360;
	}
	/*����ת*/
	else
	{
		angleCalculate = p_receiveData->anglez - angle;
		if ( angleCalculate > 0)        p_receiveData->setAnglez = angleCalculate;
		else if (angleCalculate == 0)   p_receiveData->setAnglez = 360;
		else                            p_receiveData->setAnglez = 360 + angleCalculate;
	}
}

/*************************************************************
*Function Name  : RotationJudgeOk
*Auther         : ����
*Version        : V1.0
*Date           : 2017-11-03
*Description    : �жϻ�������ת�Ƿ����
*Input          ��1.type ��ת����  @arg TURN_LEFT:��ת 
*								   @arg TURN_RIGHT:��ת 
*Return         ��0: ��ת���   1:��תδ���
*************************************************************/
INT8U RotationJudgeOk(u8 type)
{
	INT16S angleBoundVal1 = 0;  /* �߽�ֵ1 */
	INT16S angleBoundVal2 = 0;  /* �߽�ֵ2 */
	
	/*����ת*/
	if ( type == TURN_LEFT )
	{
		angleBoundVal1 = p_receiveData->setAnglez;
		angleBoundVal2 = p_receiveData->setAnglez + ROTATION_ERR_RANGE;
		if (angleBoundVal2 <= 360)
		{
			if (angleBoundVal1<=p_receiveData->anglez && p_receiveData->anglez<=angleBoundVal2)
			{
				return 0;
			}
			else 
			{
				return 1;
			}
		}
		else
		{
			angleBoundVal2 -= 360;
			if (angleBoundVal1<=p_receiveData->anglez || p_receiveData->anglez<=angleBoundVal2)
			{
				return 0;
			}
			else 
			{
				return 1;
			}
			
		}
	}
	/*����ת*/
	else
	{
		angleBoundVal1 = p_receiveData->setAnglez;
		angleBoundVal2 = p_receiveData->setAnglez - ROTATION_ERR_RANGE;
		if (angleBoundVal2 >= 0)
		{
			if (angleBoundVal2<=p_receiveData->anglez && p_receiveData->anglez<=angleBoundVal1)
			{
				return 0;
			}
			else 
			{
				return 1;
			}
		}
		else
		{
			angleBoundVal2 += 360;
			if (angleBoundVal2<=p_receiveData->anglez || p_receiveData->anglez<=angleBoundVal1)
			{
				return 0;
			}
			else 
			{
				return 1;
			}
		}
	}
}
