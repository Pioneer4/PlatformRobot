/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : motor.c
*Auther         : 张沁
*Version        : v1.0
*Date           : 2017-11-04
*Description    : 该文件包含了擂台机器人所有电机的初始化，以及
*                 机器人的运动控制。
*Function List  : 
*************************************************************/

#include "myinclude.h"

/*************************************************************
*Function Name  : AllMotorInit
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : 初始化机器人的所有电机 
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void AllMotorInit(void)    
{
	Edge4MotorInit();   /* 初始化边缘4个电机        */
	Mid2MotorInit();    /* 初始化中间2个电机        */
	ClimbMotorInit();   /* 初始化负责登台机构的电机 */
	ServoInit();		/* 初始化舵机               */
	ClimbSensorInit();  /* 初始化光电编码器引脚PE0  */
}


/*************************************************************
*Function Name  : Edge4MotorInit
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : 初始化边缘4个直流电机  
*				  TIM3用于生产PWM（10KHz）
*                 PC6~PC9 分别对应通道 CH1~CH4
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void Edge4MotorInit(void)   
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	  //TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	  //使能PORTC时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	      //速度100MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);                    //初始化PC6,PC7,PC8,PC9
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);   //GPIOC6复用为定时器3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);   //GPIOC7复用为定时器3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);   //GPIOC8复用为定时器3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);   //GPIOC9复用为定时器3
	
	TIM_TimeBaseStructure.TIM_Prescaler = 21 - 1;             //定时器分频 84M/21 = 4M
	TIM_TimeBaseStructure.TIM_Period = 400 - 1;               //自动重装载值 4M/400 = 10KHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse = 0;                        //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR4上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM3, ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);             //使能TIM3
}


/*************************************************************
*Function Name  : Mid2MotorInit
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : 初始化中间2个直流电机
*				  TIM4用于生产PWM（10KHz）
*                 PB6~PB9 分别对应通道 CH1~CH4
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void Mid2MotorInit(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	  //TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	  //使能PORTB时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	      //速度100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);                    //初始化PA0,PA1,PA2,PA3
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);   //GPIOB6复用为定时器4
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);   //GPIOB7复用为定时器4
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);   //GPIOB8复用为定时器4
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);   //GPIOB9复用为定时器4
	
	TIM_TimeBaseStructure.TIM_Prescaler = 21 - 1;             //定时器分频 84M/21 = 4M
	TIM_TimeBaseStructure.TIM_Period = 400 - 1;               //自动重装载值 4M/400 = 10KHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse = 0;                        //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM2在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM2在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM2在CCR4上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM4, ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);             //使能TIM4
}

/*************************************************************
*Function Name  : ClimbMotorInit
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : 初始化负责登台机构的电机
*				  TIM9用于生产PWM（10KHz）
*                 PE5~PE6 分别对应通道 CH1~CH2
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void ClimbMotorInit(void)   
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	  //TIM9时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	  //使能PORTE时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	      //速度100MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);                    //初始化PE5，PE6
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);   //GPIOE5复用为定时器9
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);   //GPIOE6复用为定时器9
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;             //定时器分频 168M/42 =  4M
	TIM_TimeBaseStructure.TIM_Period = 400 - 1;               //自动重装载值 4M/400 = 10KHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse = 0;                        //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM9在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM9在CCR2上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM9, ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM9, ENABLE);             //使能TIM9
}

 

/*************************************************************
*Function Name  : ServoInit
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : 初始化舵机SD5
*				  TIM2用于生产PWM（50Hz）
*                 PA0~PA3 分别对应通道 CH1~CH4
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void ServoInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	  //TIM2时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	  //使能PORTA时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	      //速度100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);                    //初始化PA0,PA1,PA2,PA3
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);   //GPIOA0复用为定时器5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);   //GPIOA1复用为定时器5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);   //GPIOA2复用为定时器5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);   //GPIOA3复用为定时器5
	
	TIM_TimeBaseStructure.TIM_Prescaler = 2800 - 1;           //定时器分频 84M/2800 = 30KHz
	TIM_TimeBaseStructure.TIM_Period = 100 - 1;               //自动重装载值 30K/100 = 300Hz
//	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;           //定时器分频 84M/8400 = 10KHz
//	TIM_TimeBaseStructure.TIM_Period = 200 - 1;               //自动重装载值 10K/200 = 50Hz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse = 0;                        //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR4上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM2, ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM2, ENABLE);             //使能TIM2	
}

/*************************************************************
*Function Name  : RobotMoveContrl
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : 控制机器人运行状态，不同状态运动的方向、速度不同
*Input          ：type 运动状态
* 				  @arg ROBOT_PATROL     (巡航模式        )   
*				  @arg ROBOT_ATTACK_F   (攻击状态：正前方)  
*				  @arg ROBOT_ATTACK_B   (攻击状态：正后方)   
*				  @arg ROBOT_ROTATION_L (自转状态：向左转)  
*				  @arg ROBOT_ROTATION_R (自转状态：向右转)
*				  @arg ROBOT_PUSH_F     (推木块  ：正前方)
*				  @arg ROBOT_PUSH_B     (推木块  ：正后方)
*				  @arg ROBOT_Ergent_B	(检测到边缘：后退)
*				  @arg ROBOT_Ergent_A 	(检测到边缘：前进)
*Output         ：
*Return         ：
*************************************************************/
void RobotMoveContrl(INT8U type) 
{
	switch (type)
	{
		case ROBOT_PATROL:       /* 巡航模式 */
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
		case ROBOT_ATTACK_F:     /* 攻击状态：正前方 */
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
		case ROBOT_ATTACK_B:     /* 攻击状态：正后方 */
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
		case ROBOT_ROTATION_L:     /* 自转状态：向左转 */
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
		case ROBOT_ROTATION_R:     /* 自转状态：向右转 */
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
		case ROBOT_PUSH_F:        /* 推木块：正前方 */
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
		case ROBOT_PUSH_B:        /* 推木块：正后方 */
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
		case ROBOT_Ergent_B:	  /* 检测到边缘：后退 */
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
		case ROBOT_Ergent_A:	  /* 检测到边缘：前进 */
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
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-04
*Description    : 1号舵机角度控制
*Input          ：servolAngleVal  @arg 0~30  
*                 占空比 = （servolAngleVal+1）/ 100
*Output         ：
*Return         ：
*************************************************************/
void Servo1Control(INT16U servolAngleVal)
{
	TIM_SetCompare1(TIM2, servolAngleVal);	//修改比较值，修改占空比
}


/*************************************************************
*Function Name  : ClimbMotorControl
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : 登台电机速度控制
*Input          ：climbVal  @arg 0~399  
*                 占空比 = （servolAngleVal+1）/ 399
*Output         ：
*Return         ：
*************************************************************/
void ClimbMotorControl(INT16U climbVal)
{
	TIM_SetCompare1(TIM9, climbVal);
	TIM_SetCompare2(TIM9, 0);
}

/*************************************************************
*Function Name  : SixMotorControl
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : 六路电机速度控制(仅登台使用)
*Input          ：climbVal  @arg 0~399  
*                 占空比 = （servolAngleVal+1）/ 399
*Output         ：
*Return         ：
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
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : 初始化光电编码器引脚PE0
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void ClimbSensorInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);   //使能AHB1始终
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //使能APB2始终
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/******开启EXTI0中断线控制器***********/
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;            //外部中断线0
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;             //使能
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   //中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//上升沿触发
	EXTI_Init(&EXTI_InitStructure);
	
	/******配置嵌套向量中断控制器***********/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;  //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;         //从优先级1
	NVIC_Init(&NVIC_InitStructure);
}

/*************************************************************
*Function Name  : EXTI0_IRQHandler
*Auther         : 张沁
*Version        : V2.0
*Date           : 2017-11-06
*Description    : 登台电机上的光电编码器的中断服务例程
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void EXTI0_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS 		       //如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	
	EXTI_ClearFlag(EXTI_Line0);	   //清除Lin0上的标志位
	climbSensorVal++;          	   //登台机构电机的位置反馈变量加一
	
#if SYSTEM_SUPPORT_OS 	           //如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
}
