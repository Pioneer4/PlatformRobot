/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : dataAnalysis.c
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-02
*Description    : 该文件包含了传感器数据(From K60)的处理、解析
*Function List  : 
*************************************************************/

#include "myinclude.h"

/*************************************************************
*Function Name  : ElecSensorRead
*Auther         : 张沁
*Version        : V1.0
*Date           : 2017-11-02
*Description    : 读取擂台机器人光电对管的检测值
*Input          ：sensorNum  要读取的光电对管编号(1--16)
*Output         ：
*Return         ：0：检测到物体  1：没检测到物体
*************************************************************/
INT8U ElecSensorRead(u8 sensorNum)
{
	sensorNum--;  //用于后面的移位处理
	
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
*Auther         : 张沁
*Version        : V1.0
*Date           : 2017-11-02
*Description    : 第sensorNum号光电对管检测到物体，LCD则在对应
*				  的编号上显示实心圆
*Input          ：sensorNum  要画圆的光电对管编号(1--16)
*Output         ：
*Return         ：
*************************************************************/
void SensorShowCircle(u8 sensorNum)
{
	switch (sensorNum)
	{
		/*四个负责边缘检测的光电开关*/
		case  1: Show_Circle(87,  194, 3, BLUE);  break;
		case  2: Show_Circle(153, 194, 3, BLUE);  break;
		case  3: Show_Circle(87,  266, 3, BLUE);  break;
		case  4: Show_Circle(153, 266, 3, BLUE);  break;
		/*四个负责前方的光电开关*/
		case  5: Show_Circle(97,  175, 3, BLUE);  break;
		case  6: Show_Circle(113, 175, 3, BLUE);  break;
		case  7: Show_Circle(127, 175, 3, BLUE);  break;
		case  8: Show_Circle(142, 175, 3, BLUE);  break;
		/*四个负责后方的光电开关*/
		case  9: Show_Circle(97,  285, 3, BLUE);  break;
		case 10: Show_Circle(113, 285, 3, BLUE);  break;
		case 11: Show_Circle(127, 285, 3, BLUE);  break;
		case 12: Show_Circle(142, 285, 3, BLUE);  break;
		/*三个负责左方的光电开关*/
		case 13: Show_Circle(75,  222, 3, BLUE);  break;
		//case 14: Show_Circle(75,  230, 3, BLUE);  break;
		case 14: Show_Circle(75,  238, 3, BLUE);  break;
		/*三个负责右方的光电开关*/
		case 15: Show_Circle(165, 222, 3, BLUE);  break;
		//case 17: Show_Circle(165, 230, 3, BLUE);  break;
		case 16: Show_Circle(165, 238, 3, BLUE);  break;
		default: break;
	}
}

/*************************************************************
*Function Name  : SensorAllClrCircle
*Auther         : 张沁
*Version        : V1.0
*Date           : 2017-11-02
*Description    : 清除LCD上所有光电对管对应的实心圆
*Input          ：
*Return         ：
*************************************************************/
void SensorAllClrCircle(void)
{
		/*四个负责边缘检测的光电开关*/
		Show_Circle(87,  194, 3, WHITE); 
		Show_Circle(153, 194, 3, WHITE); 
		Show_Circle(87,  266, 3, WHITE); 
		Show_Circle(153, 266, 3, WHITE);  
		/*四个负责前方的光电开关*/
		Show_Circle(97,  175, 3, WHITE);
		Show_Circle(113, 175, 3, WHITE); 
		Show_Circle(127, 175, 3, WHITE);
		Show_Circle(142, 175, 3, WHITE);
		/*四个负责后方的光电开关*/
		Show_Circle(97,  285, 3, WHITE);
		Show_Circle(113, 285, 3, WHITE);
		Show_Circle(127, 285, 3, WHITE);
		Show_Circle(142, 285, 3, WHITE);
		/*两个负责左方的光电开关*/
		Show_Circle(75,  222, 3, WHITE);
		//Show_Circle(75,  230, 3, WHITE);
		Show_Circle(75,  238, 3, WHITE);
		/*两个负责右方的光电开关*/
		Show_Circle(165, 222, 3, WHITE);
		//Show_Circle(165, 230, 3, WHITE);
		Show_Circle(165, 238, 3, WHITE);
}


/*************************************************************
*Function Name  : SetRotationAngle
*Auther         : 张沁
*Version        : V1.0
*Date           : 2017-11-03
*Description    : 设置机器人旋转方向、旋转角度
*Input          ：1.type 旋转类型  @arg TURN_LEFT:左转 
*								   @arg TURN_RIGHT:右转 
*				  2.angle 旋转角度 @arg 0--360
*Return         ：
*************************************************************/
void SetRotationAngle(u8 type, INT16U angle)
{
	INT16S angleCalculate = 0;
	
	/*向左转*/
	if ( type == TURN_LEFT )
	{
		angleCalculate = p_receiveData->anglez + angle;
		if ( angleCalculate < 360 )     p_receiveData->setAnglez = angleCalculate;
		else if (angleCalculate == 360) p_receiveData->setAnglez = 0;
		else                            p_receiveData->setAnglez = angleCalculate - 360;
	}
	/*向右转*/
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
*Auther         : 张沁
*Version        : V1.0
*Date           : 2017-11-03
*Description    : 判断机器人旋转是否完成
*Input          ：1.type 旋转类型  @arg TURN_LEFT:左转 
*								   @arg TURN_RIGHT:右转 
*Return         ：0: 旋转完成   1:旋转未完成
*************************************************************/
INT8U RotationJudgeOk(u8 type)
{
	INT16S angleBoundVal1 = 0;  /* 边界值1 */
	INT16S angleBoundVal2 = 0;  /* 边界值2 */
	
	/*向左转*/
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
	/*向右转*/
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
