/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : motor.c
*Auther         : ����
*Version        : v1.0
*Date           : 2017-11-04
*Description    : ���ļ���������̨���������е���ĳ�ʼ�����Լ�
*                 �����˵��˶����ơ�
*Function List  : 
*************************************************************/

#include "myinclude.h"

/*************************************************************
*Function Name  : AllMotorInit
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : ��ʼ�������˵����е�� 
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void AllMotorInit(void)    
{
	Edge4MotorInit();   /* ��ʼ����Ե4�����        */
	Mid2MotorInit();    /* ��ʼ���м�2�����        */
	ClimbMotorInit();   /* ��ʼ�������̨�����ĵ�� */
	ServoInit();		/* ��ʼ�����               */
	ClimbSensorInit();  /* ��ʼ��������������PE0  */
}


/*************************************************************
*Function Name  : Edge4MotorInit
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : ��ʼ����Ե4��ֱ�����  
*				  TIM3��������PWM��10KHz��
*                 PC6~PC9 �ֱ��Ӧͨ�� CH1~CH4
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void Edge4MotorInit(void)   
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	  //TIM3ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	  //ʹ��PORTCʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	      //�ٶ�100MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);                    //��ʼ��PC6,PC7,PC8,PC9
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);   //GPIOC6����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);   //GPIOC7����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);   //GPIOC8����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);   //GPIOC9����Ϊ��ʱ��3
	
	TIM_TimeBaseStructure.TIM_Prescaler = 21 - 1;             //��ʱ����Ƶ 84M/21 = 4M
	TIM_TimeBaseStructure.TIM_Period = 400 - 1;               //�Զ���װ��ֵ 4M/400 = 10KHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse = 0;                        //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR4�ϵ�Ԥװ�ؼĴ���
 
    TIM_ARRPreloadConfig(TIM3, ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM3, ENABLE);             //ʹ��TIM3
}


/*************************************************************
*Function Name  : Mid2MotorInit
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : ��ʼ���м�2��ֱ�����
*				  TIM4��������PWM��10KHz��
*                 PB6~PB9 �ֱ��Ӧͨ�� CH1~CH4
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void Mid2MotorInit(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	  //TIM4ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	  //ʹ��PORTBʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	      //�ٶ�100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);                    //��ʼ��PA0,PA1,PA2,PA3
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);   //GPIOB6����Ϊ��ʱ��4
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);   //GPIOB7����Ϊ��ʱ��4
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);   //GPIOB8����Ϊ��ʱ��4
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);   //GPIOB9����Ϊ��ʱ��4
	
	TIM_TimeBaseStructure.TIM_Prescaler = 21 - 1;             //��ʱ����Ƶ 84M/21 = 4M
	TIM_TimeBaseStructure.TIM_Period = 400 - 1;               //�Զ���װ��ֵ 4M/400 = 10KHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse = 0;                        //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR4�ϵ�Ԥװ�ؼĴ���
 
    TIM_ARRPreloadConfig(TIM4, ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM4, ENABLE);             //ʹ��TIM4
}

/*************************************************************
*Function Name  : ClimbMotorInit
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : ��ʼ�������̨�����ĵ��
*				  TIM9��������PWM��10KHz��
*                 PE5~PE6 �ֱ��Ӧͨ�� CH1~CH2
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void ClimbMotorInit(void)   
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	  //TIM9ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	  //ʹ��PORTEʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	      //�ٶ�100MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);                    //��ʼ��PE5��PE6
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);   //GPIOE5����Ϊ��ʱ��9
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);   //GPIOE6����Ϊ��ʱ��9
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;             //��ʱ����Ƶ 168M/42 =  4M
	TIM_TimeBaseStructure.TIM_Period = 400 - 1;               //�Զ���װ��ֵ 4M/400 = 10KHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse = 0;                        //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM9��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM9��CCR2�ϵ�Ԥװ�ؼĴ���
 
    TIM_ARRPreloadConfig(TIM9, ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM9, ENABLE);             //ʹ��TIM9
}

 

/*************************************************************
*Function Name  : ServoInit
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : ��ʼ�����SD5
*				  TIM2��������PWM��50Hz��
*                 PA0~PA3 �ֱ��Ӧͨ�� CH1~CH4
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void ServoInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	  //TIM2ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	  //ʹ��PORTAʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	      //�ٶ�100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);                    //��ʼ��PA0,PA1,PA2,PA3
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);   //GPIOA0����Ϊ��ʱ��5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);   //GPIOA1����Ϊ��ʱ��5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);   //GPIOA2����Ϊ��ʱ��5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);   //GPIOA3����Ϊ��ʱ��5
	
	TIM_TimeBaseStructure.TIM_Prescaler = 2800 - 1;           //��ʱ����Ƶ 84M/2800 = 30KHz
	TIM_TimeBaseStructure.TIM_Period = 100 - 1;               //�Զ���װ��ֵ 30K/100 = 300Hz
//	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;           //��ʱ����Ƶ 84M/8400 = 10KHz
//	TIM_TimeBaseStructure.TIM_Period = 200 - 1;               //�Զ���װ��ֵ 10K/200 = 50Hz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse = 0;                        //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR4�ϵ�Ԥװ�ؼĴ���
 
    TIM_ARRPreloadConfig(TIM2, ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM2, ENABLE);             //ʹ��TIM2	
}

/*************************************************************
*Function Name  : RobotMoveContrl
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : ���ƻ���������״̬����ͬ״̬�˶��ķ����ٶȲ�ͬ
*Input          ��type �˶�״̬
* 				  @arg ROBOT_PATROL     (Ѳ��ģʽ        )   
*				  @arg ROBOT_ATTACK_F   (����״̬����ǰ��)  
*				  @arg ROBOT_ATTACK_B   (����״̬������)   
*				  @arg ROBOT_ROTATION_L (��ת״̬������ת)  
*				  @arg ROBOT_ROTATION_R (��ת״̬������ת)
*				  @arg ROBOT_PUSH_F     (��ľ��  ����ǰ��)
*				  @arg ROBOT_PUSH_B     (��ľ��  ������)
*				  @arg ROBOT_Ergent_B	(��⵽��Ե������)
*				  @arg ROBOT_Ergent_A 	(��⵽��Ե��ǰ��)
*Output         ��
*Return         ��
*************************************************************/
void RobotMoveContrl(INT8U type) 
{
	switch (type)
	{
		case ROBOT_PATROL:       /* Ѳ��ģʽ */
		{
			TIM_SetCompare1(TIM3, p_speed1Select->patrol);
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare3(TIM3, p_speed1Select->patrol);
			TIM_SetCompare4(TIM3, 0);
			
			TIM_SetCompare1(TIM4, p_speed2Select->patrol);
			TIM_SetCompare2(TIM4, 0);
			TIM_SetCompare3(TIM4, p_speed2Select->patrol);
			TIM_SetCompare4(TIM4, 0);
			break;
		}
		case ROBOT_ATTACK_F:     /* ����״̬����ǰ�� */
		{
			TIM_SetCompare1(TIM3, p_speed1Select->attack);
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare3(TIM3, p_speed1Select->attack);
			TIM_SetCompare4(TIM3, 0);
			
			TIM_SetCompare1(TIM4, p_speed2Select->attack);
			TIM_SetCompare2(TIM4, 0);
			TIM_SetCompare3(TIM4, p_speed2Select->attack);
			TIM_SetCompare4(TIM4, 0);
			break;
		}
		case ROBOT_ATTACK_B:     /* ����״̬������ */
		{
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, p_speed1Select->attack);
			TIM_SetCompare3(TIM3, 0);
			TIM_SetCompare4(TIM3, p_speed1Select->attack);
			
			TIM_SetCompare1(TIM4, 0);
			TIM_SetCompare2(TIM4, p_speed2Select->attack);
			TIM_SetCompare3(TIM4, 0);
			TIM_SetCompare4(TIM4, p_speed2Select->attack);
			break;
		}
		case ROBOT_ROTATION_L:     /* ��ת״̬������ת */
		{
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, p_speed1Select->rotation);
			TIM_SetCompare3(TIM3, p_speed1Select->rotation);
			TIM_SetCompare4(TIM3, 0);
			
			TIM_SetCompare1(TIM4, 0);
			TIM_SetCompare2(TIM4, p_speed2Select->rotation);
			TIM_SetCompare3(TIM4, p_speed2Select->rotation);
			TIM_SetCompare4(TIM4, 0);
			break;
		}
		case ROBOT_ROTATION_R:     /* ��ת״̬������ת */
		{
			TIM_SetCompare1(TIM3, p_speed1Select->rotation);
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare3(TIM3, 0);
			TIM_SetCompare4(TIM3, p_speed1Select->rotation);
			
			TIM_SetCompare1(TIM4, p_speed2Select->rotation);
			TIM_SetCompare2(TIM4, 0);
			TIM_SetCompare3(TIM4, 0);
			TIM_SetCompare4(TIM4, p_speed2Select->rotation);
			break;
		}
		case ROBOT_PUSH_F:        /* ��ľ�飺��ǰ�� */
		{
			TIM_SetCompare1(TIM3, p_speed1Select->patrol);
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare3(TIM3, p_speed1Select->patrol);
			TIM_SetCompare4(TIM3, 0);
			
			TIM_SetCompare1(TIM4, p_speed2Select->patrol);
			TIM_SetCompare2(TIM4, 0);
			TIM_SetCompare3(TIM4, p_speed2Select->patrol);
			TIM_SetCompare4(TIM4, 0);
			break;
		}
		case ROBOT_PUSH_B:        /* ��ľ�飺���� */
		{
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, p_speed1Select->patrol);
			TIM_SetCompare3(TIM3, 0);
			TIM_SetCompare4(TIM3, p_speed1Select->patrol);
			
			TIM_SetCompare1(TIM4, 0);
			TIM_SetCompare2(TIM4, p_speed2Select->patrol);
			TIM_SetCompare3(TIM4, 0);
			TIM_SetCompare4(TIM4, p_speed2Select->patrol);
			break;
		}
		case ROBOT_Ergent_B:	  /* ��⵽��Ե������ */
		{
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, BOUND_WARN_SPEED_4);
			TIM_SetCompare3(TIM3, 0);
			TIM_SetCompare4(TIM3, BOUND_WARN_SPEED_4);
			
			TIM_SetCompare1(TIM4, 0);
			TIM_SetCompare2(TIM4, BOUND_WARN_SPEED_2);
			TIM_SetCompare3(TIM4, 0);
			TIM_SetCompare4(TIM4, BOUND_WARN_SPEED_2);
			break;
		}
		case ROBOT_Ergent_A:	  /* ��⵽��Ե��ǰ�� */
		{
			TIM_SetCompare1(TIM3, BOUND_WARN_SPEED_4);
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare3(TIM3, BOUND_WARN_SPEED_4);
			TIM_SetCompare4(TIM3, 0);
			
			TIM_SetCompare1(TIM4, BOUND_WARN_SPEED_2);
			TIM_SetCompare2(TIM4, 0);
			TIM_SetCompare3(TIM4, BOUND_WARN_SPEED_2);
			TIM_SetCompare4(TIM4, 0);
			break;
		}
		default: break;
	}
}

/*************************************************************
*Function Name  : Servo1Control
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : 1�Ŷ���Ƕȿ���
*Input          ��servolAngleVal  @arg 0~30  
*                 ռ�ձ� = ��servolAngleVal+1��/ 100
*Output         ��
*Return         ��
*************************************************************/
void Servo1Control(INT16U servolAngleVal)
{
	TIM_SetCompare1(TIM2, servolAngleVal);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}


/*************************************************************
*Function Name  : ClimbMotorControl
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : ��̨����ٶȿ���
*Input          ��climbVal  @arg 0~399  
*                 ռ�ձ� = ��servolAngleVal+1��/ 399
*Output         ��
*Return         ��
*************************************************************/
void ClimbMotorControl(INT16U climbVal)
{
	TIM_SetCompare1(TIM9, climbVal);
	TIM_SetCompare2(TIM9, 0);
}

/*************************************************************
*Function Name  : SixMotorControl
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : ��·����ٶȿ���(����̨ʹ��)
*Input          ��climbVal  @arg 0~399  
*                 ռ�ձ� = ��servolAngleVal+1��/ 399
*Output         ��
*Return         ��
*************************************************************/
void SixMotorControl(INT16U climbVal)
{
	TIM_SetCompare1(TIM3, climbVal);
	TIM_SetCompare2(TIM3, 0);
	TIM_SetCompare3(TIM3, climbVal);
	TIM_SetCompare4(TIM3, 0);

	TIM_SetCompare1(TIM4, climbVal);
	TIM_SetCompare2(TIM4, 0);
	TIM_SetCompare3(TIM4, climbVal);
	TIM_SetCompare4(TIM4, 0);
}

/*************************************************************
*Function Name  : ClimbSensorInit
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : ��ʼ��������������PE0
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void ClimbSensorInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);   //ʹ��AHB1ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //ʹ��APB2ʼ��
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/******����EXTI0�ж��߿�����***********/
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;            //�ⲿ�ж���0
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;             //ʹ��
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   //�ж��¼�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�����ش���
	EXTI_Init(&EXTI_InitStructure);
	
	/******����Ƕ�������жϿ�����***********/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;  //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;         //�����ȼ�1
	NVIC_Init(&NVIC_InitStructure);
}

/*************************************************************
*Function Name  : EXTI0_IRQHandler
*Auther         : ����
*Version        : V2.0
*Date           : 2017-11-06
*Description    : ��̨����ϵĹ����������жϷ�������
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void EXTI0_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS 		       //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	
	EXTI_ClearFlag(EXTI_Line0);	   //���Lin0�ϵı�־λ
	climbSensorVal++;          	   //��̨���������λ�÷���������һ
	
#if SYSTEM_SUPPORT_OS 	           //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
}
