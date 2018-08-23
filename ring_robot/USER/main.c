/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : main.c
*Auther         : 张沁
*Version        : v1.0
*Date           : 2017-10-31
*Description    : 华北五省擂台机器人代码模板（uCOS）
*                 系统频率设置为168MHz
*                 (由于考虑后期扩展传感器，该系统板引脚可能不足，
*                 所以考虑K60作为协处理器，通过串口40ms传输一次
*				  数据)
*Function List  : 
*History        : 
*************************************************************/

#define  MY_CPP_GLOBALS 

#include "myinclude.h"
#include "includes.h"
 
extern u16 USART_RX_STA;  //接收状态标记为0
extern u8 UART_RX_BUF[UART_REC_LEN];  //接收缓冲

/*
事件标志组：触摸屏相关
BIT0: 触摸屏是否开始扫描触摸点 
*/
OS_FLAG_GRP *flagTP; 

OS_EVENT *semDebug;   //信号量：负责Debug的调试任务可以运行


/********************************************
任务        ：MotorTask  机器人电机运转任务(对战模式下)
任务优先级  ：9
任务堆栈大小：256
任务堆栈    ：MotorTaskStk[MOTOR_STK_SIZE]
*********************************************/
#define MOTOR_TASK_PRIO  9
#define MOTOR_STK_SIZE  512
OS_STK MotorTaskStk[MOTOR_STK_SIZE];
void MotorTask(void *pdata);


/********************************************
任务        ：FightStaDecTask  机器人状态决策任务(对战模式下)
任务优先级  ：10
任务堆栈大小：512
任务堆栈    ：fightStaDecTaskStk[FIGHT_STA_DEC_STK_SIZE]
*********************************************/
#define FIGHT_STA_DEC_TASK_PRIO  10
#define FIGHT_STA_DEC_STK_SIZE  512
OS_STK fightStaDecTaskStk[FIGHT_STA_DEC_STK_SIZE];
void FightStaDecTask(void *pdata);


/********************************************
任务        ：PushStaDecTask  机器人状态决策任务(推木块模式下)
任务优先级  ：11
任务堆栈大小：512
任务堆栈    ：pushStaDecTaskStk[PUSH_STA_DEC_STK_SIZE]
*********************************************/
#define PUSH_STA_DEC_TASK_PRIO  11
#define PUSH_STA_DEC_STK_SIZE  512
OS_STK pushStaDecTaskStk[PUSH_STA_DEC_STK_SIZE];
void PushStaDecTask(void *pdata);


/********************************************
任务        ：ClimbTask  机器人上擂台任务
任务优先级  ：12
任务堆栈大小：256
任务堆栈    ：ClimbTaskStk[CLIMB_STK_SIZE]
*********************************************/
#define CLIMB_TASK_PRIO   12
#define CLIMB_STK_SIZE    256
OS_STK ClimbTaskStk[CLIMB_STK_SIZE];
void ClimbTask(void *pdata);


/********************************************
任务        ：DebugTask  调试任务
任务优先级  ：13
任务堆栈大小：256
任务堆栈    ：DebugTaskStk[DEBUG_STK_SIZE]
*********************************************/
#define DEBUG_TASK_PRIO    13
#define DEBUG_STK_SIZE    256
OS_STK DebugTaskStk[DEBUG_STK_SIZE];
void DebugTask(void *pdata);



/********************************************
任务        ：LcdShowTask  LCD显示屏任务
任务优先级  ：14
任务堆栈大小：256
任务堆栈    ：lcdShowTaskStk[LCD_SHOW_STK_SIZE]
*********************************************/
#define LCD_SHOW_TASK_PRIO 14    	  
#define LCD_SHOW_STK_SIZE  256		    	
OS_STK lcdShowTaskStk[LCD_SHOW_STK_SIZE];
void LcdShowTask(void *pdata);


/********************************************
任务        ：TP_Task    TP触摸屏任务
任务优先级  ：15
任务堆栈大小：256
任务堆栈    ：tpTaskStk[TP_STK_SIZE];
*********************************************/
#define TP_TASK_PRIO      15
#define TP_STK_SIZE  	  256
OS_STK tpTaskStk[TP_STK_SIZE];
void TP_Task(void *pdata);

/********************************************
任务        ：START 进行其它任务设置
任务优先级  ：16
任务堆栈大小：128
任务堆栈    ：startTaskSTK[START_STK_SIZE]
*********************************************/
#define START_TASK_PRIO      16
#define START_STK_SIZE  	128
OS_STK startTaskSTK[START_STK_SIZE];
void StartTask(void *pdata);


/*************************************************************
*Function Name  : IwdgTmrCallBack
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2016-12-06
*Description    : 定时器回调函数，用于喂狗和小灯的闪烁
*Input          : 1. void *ptmr
                  2. void *parg
				  没有传递的参数，则无需操作
*Output         :
*Return         :
*************************************************************/
OS_TMR *pTmrIwdg;
void IwdgTmrCallBack(void *ptmr, void *parg)
{
	
	if(GET_LED2_STATUS() == LED_STATUS_ON)
	{
        Led2Switch(OFF);
	}
	else
	{
		Led2Switch(ON);
	}
	
	IwdgFeed();  //喂狗
}


u8 err = 0;  //错误返回区

INT8U startFlag = 0; /* 开始标志 1：开始 0：结束 */

/* 用于处理接收K60传来的字符串 */
u8 *p_end1 = NULL;
u8 *p_end2 = NULL;

int main(void)
{
	p_speed1Select = &speed1Select;
	p_speed2Select = &speed2Select;
	p_receiveData  = &receiveData;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	delay_init(168);      //初始化延时函数
	UART1_Init(115200);   //初始化串口1
	LedInit();		      //初始化LED端口 
	LCD_Init();	          //初始化LCD	
	KeyInit();   	      //初始化按键
	
	OSInit();             //初始化uC/OS
	OSTaskCreate(StartTask, (void*)0, (OS_STK *)&startTaskSTK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	
	
	return 0;
}

/*************************************************************
*Function Name  : StartTask
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2016-12-05
*Description    : 开始任务 该任务用于建立了其它任务
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void StartTask(void *pdata)
{
	OS_CPU_SR cpu_sr = 0;
	
	pdata = pdata; 
	
	flagTP = OSFlagCreate(0, &err);    //创建事件标志组
	semDebug = OSSemCreate(0);         //创建信号量(Debug调试任务等待该信号量)
	
	OSStatInit();					   //初始化统计任务.这里会延时1秒钟左右
	
	OS_ENTER_CRITICAL();		       //进入临界区(无法被中断打断)    
	
	pTmrIwdg = OSTmrCreate(0, OS_TMR_CFG_TICKS_PER_SEC, OS_TMR_OPT_PERIODIC, (OS_TMR_CALLBACK)IwdgTmrCallBack, NULL, NULL, &err);
	
	if(OSTmrStart(pTmrIwdg, &err) == OS_TRUE)
	{
		printf("Iwdg Tmr start!\r\n");
		
	}
	else
	{
		printf("Iwdg Tmr err!");
	}
	
	OSTaskCreate(DebugTask,(void *)0,(OS_STK *)&DebugTaskStk[DEBUG_STK_SIZE-1],DEBUG_TASK_PRIO);
	OSTaskCreate(MotorTask,(void *)0,(OS_STK *)&MotorTaskStk[MOTOR_STK_SIZE-1],MOTOR_TASK_PRIO);
	OSTaskCreate(FightStaDecTask,(void *)0,(OS_STK *)&fightStaDecTaskStk[FIGHT_STA_DEC_STK_SIZE-1],FIGHT_STA_DEC_TASK_PRIO);
	OSTaskCreate(PushStaDecTask,(void *)0,(OS_STK *)&pushStaDecTaskStk[PUSH_STA_DEC_STK_SIZE-1],PUSH_STA_DEC_TASK_PRIO);
	OSTaskCreate(ClimbTask,(void *)0,(OS_STK *)&ClimbTaskStk[CLIMB_STK_SIZE-1],CLIMB_TASK_PRIO);
	OSTaskCreate(LcdShowTask,(void *)0,(OS_STK *)&lcdShowTaskStk[LCD_SHOW_STK_SIZE-1],LCD_SHOW_TASK_PRIO);
	OSTaskCreate(TP_Task,(void *)0,(OS_STK*)&tpTaskStk[TP_STK_SIZE-1],TP_TASK_PRIO);
	
	IwdgInit(4,1500);                 //IWDG的超时周期为3S
	OSTaskSuspend(START_TASK_PRIO);	  //挂起起始任务.
	OS_EXIT_CRITICAL();				  //退出临界区(可以被中断打断)
} 


/*************************************************************
*Function Name  : MotorTask
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : 对战模式下，电机的运行状态在此变化
*				  (每25ms运行一次)
*                 点击"Start"字样运行该任务
*                 点击"Stop" 字样挂起该任务
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void MotorTask(void *pdata)
{
	pdata = pdata;

	AllMotorInit();       			/* 初始化机器人上的电机系统 */
	
	Servo1Control(0);     			/* 舵机初始化位置          */
	
	OSTaskSuspend(OS_PRIO_SELF);    /* 挂起该任务              */
	
	while (1)
	{
		RobotMoveContrl(robotState);
		OSTimeDly(5);
	}
}

/*************************************************************
*Function Name  : FightStateDecideTask
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-02
*Description    : 对战模式下：实时检测并决策擂台机器人的运行状态
*                 (每25ms运行一次)
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void FightStaDecTask(void *pdata)
{
	pdata = pdata;
	
	INT32U randomNum = 0;         /* 随机数变量        		    */
	INT16U ergentTime = 0; 		  /* 紧急运动的时间变量	 	    */
	INT16U rotationErrTime = 0;   /* 自转状态的时间变量	 	    */
	INT16U patrolTime = 0;        /* 巡逻状态的时间变量	 	    */
	INT16U angleRandom = 0;       /* 随机旋转角度变量	 	    */
	
	RNG_Init();                   /* 初始化RNG随机数发生器       */
	
	OSTaskSuspend(OS_PRIO_SELF);  /* 挂起该任务          	    */
	
	robotState = ROBOT_PATROL;    /* 机器人的初试状态为巡逻状态  */
	
	while(1)
	{	
		/******存储K60所发送的光电对管检测状态、姿态传感器x,y,z轴角度等数据******/
		if (USART_RX_STA&0x8000)  /* 接收完成                   */
		{
			p_receiveData->photoelectricSensor = (INT32U)strtod((char*)UART_RX_BUF, (char**)&p_end1);
			p_receiveData->anglex = (INT16S)strtod((char*)p_end1, (char**)&p_end2);
			p_receiveData->angley = (INT16S)strtod((char*)p_end2, (char**)&p_end1);
			p_receiveData->anglez = (INT16S)strtod((char*)p_end1, NULL);
			memset(UART_RX_BUF, 0, UART_REC_LEN);
			USART_RX_STA = 0;     /* 接收完成标志位清0          */
		}
		
		/******光电对管检测后，返回机器人边缘警告、物体出现位置******/
		if      ( ElecSensorRead(1)  == 1 ) { sensorState =  SENSOR_WARN_FL;  }
		else if ( ElecSensorRead(2)  == 1 ) { sensorState =  SENSOR_WARN_FR;  }
		else if ( ElecSensorRead(3)  == 1 ) { sensorState =  SENSOR_WARN_BL;  }
		else if ( ElecSensorRead(4)  == 1 ) { sensorState =  SENSOR_WARN_BR;  }
		else if ( ElecSensorRead(6)  == 0 ) { sensorState =  SENSOR_F;        }
		else if ( ElecSensorRead(7)  == 0 ) { sensorState =  SENSOR_F;        }
		else if ( ElecSensorRead(10) == 0 ) { sensorState =  SENSOR_B;        }
		else if ( ElecSensorRead(11) == 0 ) { sensorState =  SENSOR_B;        }
		else if ( ElecSensorRead(5)  == 0 ) { sensorState =  SENSOR_FL;       }
		else if ( ElecSensorRead(8)  == 0 ) { sensorState =  SENSOR_FR;       }
		else if ( ElecSensorRead(9)  == 0 ) { sensorState =  SENSOR_BL;       }
		else if ( ElecSensorRead(12) == 0 ) { sensorState =  SENSOR_BR;       }
		else if ( ElecSensorRead(13) == 0 ) { sensorState =  SENSOR_L;        }
		else if ( ElecSensorRead(14) == 0 ) { sensorState =  SENSOR_L;        }
		else if ( ElecSensorRead(15) == 0 ) { sensorState =  SENSOR_R;        }
		else if ( ElecSensorRead(16) == 0 ) { sensorState =  SENSOR_R;        }
		else { sensorState = SENSOR_NONE; }
		
		/******根据光电对管检测后返回状态，决策机器人运行状态******/
		switch (sensorState)
		{
			case SENSOR_WARN_FL:  					  /* 边界警告：前方左侧        */
			{
				robotState = ROBOT_Ergent_B; 		  /* 状态切换：紧急后退        */
				robotRotationDir = SENSOR_WARN_FL;
				ergentTime = 0;    					  /* 紧急运动时间变量清0 	  */
				break;
			}
			case SENSOR_WARN_FR:  					  /* 边界警告：前方右侧 		  */
			{
				robotState = ROBOT_Ergent_B;  	      /* 状态切换：紧急后退 		  */
				robotRotationDir = SENSOR_WARN_FR;
				ergentTime = 0;    					  /* 紧急运动时间变量清0       */
				break;
			}
			case SENSOR_WARN_BL: 					  /* 边界警告：后方左侧        */
			{
				robotState = ROBOT_Ergent_A; 		  /* 状态切换：紧急前进 		  */
				robotRotationDir = SENSOR_WARN_BL;
				ergentTime = 0;    					  /* 紧急运动时间变量清0 	  */
				break;
			}
			case SENSOR_WARN_BR: 					  /* 边界警告：后方右侧		  */
			{
				robotState = ROBOT_Ergent_A; 		  /* 状态切换：紧急前进        */
				robotRotationDir = SENSOR_WARN_BR;
				ergentTime = 0;   					  /* 紧急运动时间变量清0 	  */
				break;
			}
			case SENSOR_F:       					  /* 物体出现：正前方          */
			{
				robotState = ROBOT_ATTACK_F; 		  /* 状态切换:攻击状态：正前方 */
				break;
			}
			case SENSOR_B:       					  /* 物体出现：正后方          */
			{
				robotState = ROBOT_ATTACK_B; 		  /* 状态切换:攻击状态：正后方 */
				break;
			}
			case  SENSOR_FL:  	                      /* 物体出现：前侧左方        */
			{
				robotState = ROBOT_ROTATION_L;        /* 状态切换：自转状态(向左)  */
				SetRotationAngle(TURN_LEFT, ANGLE_FL);/* 设置向左旋转角 			  */
				rotationErrTime = 0;				  /* 旋转时间变量清0 		  */
				break;
			}
			case  SENSOR_FR:  	 					  /* 物体出现：前侧右方 		  */
			{
				robotState = ROBOT_ROTATION_R;        /* 状态切换：自转状态(向右)  */
				SetRotationAngle(TURN_RIGHT,ANGLE_FR);/* 设置向右旋转角            */
				rotationErrTime = 0; 				  /* 旋转时间变量清0        	  */
				break;
			}
			case  SENSOR_BL:  	 					  /* 物体出现：后侧左方 		  */
			{
				robotState = ROBOT_ROTATION_R;        /* 状态切换：自转状态(向右)  */
				SetRotationAngle(TURN_RIGHT,ANGLE_BL);/* 设置向右旋转角 			  */
				rotationErrTime = 0; 				  /* 旋转时间变量清0			  */
				break;
			}
			case  SENSOR_BR:  	 					  /* 物体出现：后侧右方 	  	  */
			{
				robotState = ROBOT_ROTATION_L;        /* 状态切换：自转状态(向左)  */
				SetRotationAngle(TURN_LEFT, ANGLE_BR);/* 设置向左旋转角    	      */
				rotationErrTime = 0; 				  /* 旋转时间变量清0 	      */
				break;
			}
			case  SENSOR_L:  	 					  /* 物体出现：正左方 		  */
			{
				robotState = ROBOT_ROTATION_L;        /* 状态切换：自转状态(向左)  */
				SetRotationAngle(TURN_LEFT, ANGLE_L); /* 设置向左旋转角 		      */
				rotationErrTime = 0; 				  /* 旋转时间变量清0 		  */
				break;
			}
			case  SENSOR_R:  	 					  /* 物体出现：正左方          */
			{
				robotState = ROBOT_ROTATION_R;        /* 状态切换：自转状态(向右)  */
				SetRotationAngle(TURN_RIGHT, ANGLE_R);/* 设置向右旋转角 			  */
				rotationErrTime = 0; 				  /* 旋转时间变量清0 		  */
				break;
			}
			default: break;
		}
		
		switch (robotState)
		{
			/********************处于紧急后退状态********************/
			case ROBOT_Ergent_B: 
			{
				if (ergentTime < ERGENT_BACK_TIME) 					    /* 紧急后退时间未达到要求      				  */
				{
					ergentTime++;
				}
				else                              					    /* 后退时间达到要求后，进行状态切换和旋转角度设置 */
				{
					if (robotRotationDir == SENSOR_WARN_FL)			    /* 紧急后退后向右转 						      */
					{ 
		
						robotState = ROBOT_ROTATION_R;     			    /* 状态切换：自转状态(向右)					  */
						SetRotationAngle(TURN_RIGHT, ANGLE_ERGENT_BACK);/* 设置向右旋转角 							  */
						rotationErrTime = 0; 							/* 旋转时间变量清0					    	  */ 
					}
					else if (robotRotationDir == SENSOR_WARN_FR) 		/* 紧急后退后向左转 							  */
					{
						robotState = ROBOT_ROTATION_L;      			/* 状态切换：自转状态(向左) 					  */
						SetRotationAngle(TURN_LEFT, ANGLE_ERGENT_BACK);	/* 设置向左旋转角 							  */
						rotationErrTime = 0; 							/* 旋转时间变量清0 							  */
					}
				}
				break;
			}
			
			/********************处于紧急前进状态********************/
			case ROBOT_Ergent_A:   										/* 紧急前进时间未达到要求 					  */
			{
				if (ergentTime < ERGENT_AHEAD_TIME)
				{
					ergentTime++;
				}
				else             										/* 前进时间达到要求后，进行状态切换和旋转角度设置 */
				{
					if (robotRotationDir == SENSOR_WARN_BL) 			/* 紧急前进后向右转 							  */
					{
						robotState = ROBOT_ROTATION_R;     				/* 状态切换：自转状态(向右) 					  */
						SetRotationAngle(TURN_RIGHT, ANGLE_ERGENT_AHEAD);/* 设置向右旋转角 		   					  */
						rotationErrTime = 0; 					    	/* 旋转时间变量清0 							  */
					}
					else if (robotRotationDir == SENSOR_WARN_BR)		/* 紧急前进退后向左转 						  */
					{
						robotState = ROBOT_ROTATION_L;      			/* 状态切换：自转状态(向左) 					  */
						SetRotationAngle(TURN_LEFT, ANGLE_ERGENT_AHEAD);/* 设置向左旋转角 							  */
						rotationErrTime = 0; /* 旋转时间变量清0 */
					}
				}
				break;
			}
			
			/********************处于自转状态(向右)********************/
			case ROBOT_ROTATION_R:
			{
				if (RotationJudgeOk(TURN_RIGHT) == 0) 					/* 自转角度达到要求 							  */
				{
					robotState = ROBOT_PATROL; 							/* 状态切换：巡逻状态 						  */
					patrolTime = 0; 		   							/* 巡逻时间变量清0							  */
				}
				else if (rotationErrTime > ROTATION_ERR_TIME) 			/* 进入死转 									  */
				{
					robotState = ROBOT_PATROL; 							/* 状态切换：巡逻状态						 	  */
					patrolTime = 0;			   							/* 巡逻时间变量清0 							  */
				}
				else
				{
					rotationErrTime++;
				}
				break;
			}
			
			/********************处于自转状态(向左)********************/
			case ROBOT_ROTATION_L:
			{
				if (RotationJudgeOk(TURN_LEFT) == 0) 					/* 自转角度达到要求 							  */
				{
					robotState = ROBOT_PATROL; 							/* 状态切换：巡逻状态 						  */
					patrolTime = 0;			   							/* 巡逻时间变量清0 							  */
				}
				else if (rotationErrTime > ROTATION_ERR_TIME) 			/* 进入死转 								 	  */
				{
					robotState = ROBOT_PATROL; 							/* 状态切换：巡逻状态 						  */
					patrolTime = 0;			   							/* 巡逻时间变量清0 							  */
				}
				else
				{
					rotationErrTime++;
				}
				break;
			}
			
			/********************处于攻击状态：正前方********************/
			case ROBOT_ATTACK_F:
			{
				if ( (ElecSensorRead(6) != 0) && (ElecSensorRead(7) != 0) )/* 前方物体消失 							  */
				{
					robotState = ROBOT_PATROL;   						/* 状态切换：巡逻状态 						  */
					patrolTime = 0;			     						/* 巡逻时间变量清0   						  */
				}
				break;
			}
			
			/********************处于攻击状态：正后方********************/
			case ROBOT_ATTACK_B:
			{
				if ( (ElecSensorRead(10) != 0) && (ElecSensorRead(11) != 0) )/* 后方物体消失 						  */
				{
					robotState = ROBOT_PATROL;   						/* 状态切换：巡逻状态 						  */
					patrolTime = 0;			     						/* 巡逻时间变量清0    						  */
				}
				break;
			}
			
			/********************处于巡逻状态********************/
			case ROBOT_PATROL:
			{
				if (patrolTime == STATE_SWITCH_TIME) 					/* 达到状态自动切换时间 						  */
				{
					randomNum = RNG_Get_RandomRange(1, 16);
					if (randomNum == 8)
					{
						angleRandom = (INT16U)RNG_Get_RandomRange(ANGLE_RANDOM_MIN, ANGLE_RANDOM_MAX);
						robotState = ROBOT_ROTATION_R;      			/* 状态切换：自转状态(向右) 					  */
						SetRotationAngle(TURN_RIGHT, angleRandom);		/* 设置向右旋转角 							  */
						rotationErrTime = 0; 							/* 旋转时间变量清0 							  */
					}
					else if (randomNum == 16)
					{
						angleRandom = (INT16U)RNG_Get_RandomRange(ANGLE_RANDOM_MIN, ANGLE_RANDOM_MAX);
						robotState = ROBOT_ROTATION_L;      			/* 状态切换：自转状态(向左) 					  */
						SetRotationAngle(TURN_LEFT, angleRandom);		/* 设置向左旋转角 							  */
						rotationErrTime = 0; 							/* 旋转时间变量清0 							  */
					}
			    }
				else
				{
					patrolTime++;
				}
			}
			default: break;
		}
		
		OSTimeDly(10);  //每50ms运行一次
	}
}

/*************************************************************
*Function Name  : PushStaDecTask
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-02
*Description    : 推木块模式：实时检测并决策擂台机器人的运行状态
*                 (每25ms运行一次)
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void PushStaDecTask(void *pdata)
{
	pdata = pdata;
	
	//INT16U ergentTime = 0; 	  /* 紧急运动的时间变量	 	    */
	INT16U rotationErrTime = 0;   /* 自转状态的时间变量	 	    */
	
	OSTaskSuspend(OS_PRIO_SELF);  /* 挂起该任务          	    */
	
	robotState = ROBOT_PATROL;    /* 机器人的初试状态为巡逻状态  */
	
	while (1)
	{
		/******存储K60所发送的光电对管检测状态、姿态传感器x,y,z轴角度等数据******/
		if (USART_RX_STA&0x8000)  /* 接收完成                   */
		{
			p_receiveData->photoelectricSensor = (INT32U)strtod((char*)UART_RX_BUF, (char**)&p_end1);
			p_receiveData->anglex = (INT16S)strtod((char*)p_end1, (char**)&p_end2);
			p_receiveData->angley = (INT16S)strtod((char*)p_end2, (char**)&p_end1);
			p_receiveData->anglez = (INT16S)strtod((char*)p_end1, NULL);
			memset(UART_RX_BUF, 0, UART_REC_LEN);
			USART_RX_STA = 0;     /* 接收完成标志位清0          */
		}
		
		/******光电对管检测后，返回机器人边缘警告、物体出现位置******/
		if      ( ElecSensorRead(1)  == 1 ) { sensorState =  SENSOR_WARN_FL;  }
		else if ( ElecSensorRead(2)  == 1 ) { sensorState =  SENSOR_WARN_FR;  }
		else if ( ElecSensorRead(3)  == 1 ) { sensorState =  SENSOR_WARN_BL;  }
		else if ( ElecSensorRead(4)  == 1 ) { sensorState =  SENSOR_WARN_BR;  }
		else if ( ElecSensorRead(6)  == 0 ) { sensorState =  SENSOR_F;        }
		else if ( ElecSensorRead(7)  == 0 ) { sensorState =  SENSOR_F;        }
		else if ( ElecSensorRead(10) == 0 ) { sensorState =  SENSOR_B;        }
		else if ( ElecSensorRead(11) == 0 ) { sensorState =  SENSOR_B;        }
		else if ( ElecSensorRead(5)  == 0 ) { sensorState =  SENSOR_FL;       }
		else if ( ElecSensorRead(8)  == 0 ) { sensorState =  SENSOR_FR;       }
		else if ( ElecSensorRead(9)  == 0 ) { sensorState =  SENSOR_BL;       }
		else if ( ElecSensorRead(12) == 0 ) { sensorState =  SENSOR_BR;       }
		else if ( ElecSensorRead(13) == 0 ) { sensorState =  SENSOR_L;        }
		else if ( ElecSensorRead(14) == 0 ) { sensorState =  SENSOR_L;        }
		else if ( ElecSensorRead(15) == 0 ) { sensorState =  SENSOR_R;        }
		else if ( ElecSensorRead(16) == 0 ) { sensorState =  SENSOR_R;        }
		else { sensorState = SENSOR_NONE; }
		
		
		/******根据光电对管检测后返回状态，决策机器人运行状态******/
		switch (sensorState)
		{
			case SENSOR_WARN_FL:  					  /* 边界警告：前方左侧        */
			{
				robotState = ROBOT_Ergent_B; 		  /* 状态切换：紧急后退        */
				robotRotationDir = SENSOR_WARN_FL;
				//ergentTime = 0;    					  /* 紧急运动时间变量清0 	  */
				break;
			}
			case SENSOR_WARN_FR:  					  /* 边界警告：前方右侧 		  */
			{
				robotState = ROBOT_Ergent_B;  	      /* 状态切换：紧急后退 		  */
				robotRotationDir = SENSOR_WARN_FR;
				//ergentTime = 0;    					  /* 紧急运动时间变量清0       */
				break;
			}
			case SENSOR_WARN_BL: 					  /* 边界警告：后方左侧        */
			{
				robotState = ROBOT_Ergent_A; 		  /* 状态切换：紧急前进 		  */
				robotRotationDir = SENSOR_WARN_BL;
				//ergentTime = 0;    					  /* 紧急运动时间变量清0 	  */
				break;
			}
			case SENSOR_WARN_BR: 					  /* 边界警告：后方右侧		  */
			{
				robotState = ROBOT_Ergent_A; 		  /* 状态切换：紧急前进        */
				robotRotationDir = SENSOR_WARN_BR;
				//ergentTime = 0;   					  /* 紧急运动时间变量清0 	  */
				break;
			}
			case SENSOR_F:       					  /* 物体出现：正前方          */
			{
				robotState = ROBOT_ATTACK_F; 		  /* 状态切换:攻击状态：正前方 */
				break;
			}
			case SENSOR_B:       					  /* 物体出现：正后方          */
			{
				robotState = ROBOT_ATTACK_B; 		  /* 状态切换:攻击状态：正后方 */
				break;
			}
			case  SENSOR_FL:  	                      /* 物体出现：前侧左方        */
			{
				robotState = ROBOT_ROTATION_L;        /* 状态切换：自转状态(向左)  */
				SetRotationAngle(TURN_LEFT, ANGLE_FL);/* 设置向左旋转角 			  */
				rotationErrTime = 0;				  /* 旋转时间变量清0 		  */
				break;
			}
			case  SENSOR_FR:  	 					  /* 物体出现：前侧右方 		  */
			{
				robotState = ROBOT_ROTATION_R;        /* 状态切换：自转状态(向右)  */
				SetRotationAngle(TURN_RIGHT,ANGLE_FR);/* 设置向右旋转角            */
				rotationErrTime = 0; 				  /* 旋转时间变量清0        	  */
				break;
			}
			case  SENSOR_BL:  	 					  /* 物体出现：后侧左方 		  */
			{
				robotState = ROBOT_ROTATION_R;        /* 状态切换：自转状态(向右)  */
				SetRotationAngle(TURN_RIGHT,ANGLE_BL);/* 设置向右旋转角 			  */
				rotationErrTime = 0; 				  /* 旋转时间变量清0			  */
				break;
			}
			case  SENSOR_BR:  	 					  /* 物体出现：后侧右方 	  	  */
			{
				robotState = ROBOT_ROTATION_L;        /* 状态切换：自转状态(向左)  */
				SetRotationAngle(TURN_LEFT, ANGLE_BR);/* 设置向左旋转角    	      */
				rotationErrTime = 0; 				  /* 旋转时间变量清0 	      */
				break;
			}
			case  SENSOR_L:  	 					  /* 物体出现：正左方 		  */
			{
				robotState = ROBOT_ROTATION_L;        /* 状态切换：自转状态(向左)  */
				SetRotationAngle(TURN_LEFT, ANGLE_L); /* 设置向左旋转角 		      */
				rotationErrTime = 0; 				  /* 旋转时间变量清0 		  */
				break;
			}
			case  SENSOR_R:  	 					  /* 物体出现：正左方          */
			{
				robotState = ROBOT_ROTATION_R;        /* 状态切换：自转状态(向右)  */
				SetRotationAngle(TURN_RIGHT, ANGLE_R);/* 设置向右旋转角 			  */
				rotationErrTime = 0; 				  /* 旋转时间变量清0 		  */
				break;
			}
			default: break;
		}
		
		switch (robotState)
		{
			/********************处于紧急后退状态********************/
			case ROBOT_Ergent_B: 
			{
				OSTimeDly(20);  //100ms
				if (robotRotationDir == SENSOR_WARN_FL)			    /* 紧急后退后向右转 						      */
				{ 
	
					robotState = ROBOT_ROTATION_R;     			    /* 状态切换：自转状态(向右)					  */
					SetRotationAngle(TURN_RIGHT, ANGLE_ERGENT_BACK);/* 设置向右旋转角 							  */
					rotationErrTime = 0; 							/* 旋转时间变量清0					    	  */ 
				}
				else if (robotRotationDir == SENSOR_WARN_FR) 		/* 紧急后退后向左转 							  */
				{
					robotState = ROBOT_ROTATION_L;      			/* 状态切换：自转状态(向左) 					  */
					SetRotationAngle(TURN_LEFT, ANGLE_ERGENT_BACK);	/* 设置向左旋转角 							  */
					rotationErrTime = 0; 							/* 旋转时间变量清0 							  */
				}
				
				break;
			}
			
			/********************处于紧急前进状态********************/
			case ROBOT_Ergent_A:   										/* 紧急前进时间未达到要求 					  */
			{
				OSTimeDly(20);
				if (robotRotationDir == SENSOR_WARN_BL) 			/* 紧急前进后向右转 							  */
				{
					robotState = ROBOT_ROTATION_R;     				/* 状态切换：自转状态(向右) 					  */
					SetRotationAngle(TURN_RIGHT, ANGLE_ERGENT_AHEAD);/* 设置向右旋转角 		   					  */
					rotationErrTime = 0; 					    	/* 旋转时间变量清0 							  */
				}
				else if (robotRotationDir == SENSOR_WARN_BR)		/* 紧急前进退后向左转 						  */
				{
					robotState = ROBOT_ROTATION_L;      			/* 状态切换：自转状态(向左) 					  */
					SetRotationAngle(TURN_LEFT, ANGLE_ERGENT_AHEAD);/* 设置向左旋转角 							  */
					rotationErrTime = 0; 							/* 旋转时间变量清0 */
				}
				
				break;
			}
			
			/********************处于自转状态(向右)********************/
			case ROBOT_ROTATION_R:
			{
				if (RotationJudgeOk(TURN_RIGHT) == 0) 					/* 自转角度达到要求 							  */
				{
					robotState = ROBOT_PATROL; 							/* 状态切换：巡逻状态 						  */
				}
				else if (rotationErrTime > ROTATION_ERR_TIME) 			/* 进入死转 									  */
				{
					robotState = ROBOT_PATROL; 							/* 状态切换：巡逻状态						 	  */
				}
				else
				{
					rotationErrTime += 1;
				}
				break;
			}
			
			/********************处于自转状态(向左)********************/
			case ROBOT_ROTATION_L:
			{
				if (RotationJudgeOk(TURN_LEFT) == 0) 					/* 自转角度达到要求 							  */
				{
					robotState = ROBOT_PATROL; 							/* 状态切换：巡逻状态 						  */
				}
				else if (rotationErrTime > ROTATION_ERR_TIME) 			/* 进入死转 								 	  */
				{
					robotState = ROBOT_PATROL; 							/* 状态切换：巡逻状态 						  */
				}
				else
				{
					rotationErrTime++;
				}
				break;
			}
			default: break;
		}
		
		OSTimeDly(10);  //每50ms运行一次
	}
}

/*************************************************************
*Function Name  : ClimbTask
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : 负责机器人登上擂台
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void ClimbTask(void *pdata)
{
	pdata = pdata;
	
	INT8U i = 0;
	
	OSTaskSuspend(OS_PRIO_SELF);      /* 挂起该任务               */
	
	while (1)
	{
		#if 0
		for (i=0; i<100; i++)
		{
			OSTimeDly(1);
			Servo1Control(i);
		}
		
		OSTimeDly(400);
		
		for (i=100; i>0; i--)
		{
			OSTimeDly(1);
			Servo1Control(i);
		}
		modeSwitchFlag = 0;
		#endif
		

		climbSensorVal = 0;           /* 登台电机的位置反馈变量清0 */
		OSTimeDly(OS_TICKS_PER_SEC);  /* 等待1S      			  */ 		    
		ClimbMotorControl(80);        /* 登台电机运行 			  */
		SixMotorControl(240);         /* 六路电机运行			  */
		
		OSTimeDly(150);
		ClimbMotorControl(0);         /* 登台电机停转 			  */
		//OSTimeDly(130);  		      /* 等待600ms     			  */
		SixMotorControl(0);
		
		/* 往前走 */
		TIM_SetCompare1(TIM3, 40);
		TIM_SetCompare2(TIM3, 0);
		TIM_SetCompare3(TIM3, 40);
	    TIM_SetCompare4(TIM3, 0);
		for (i=100; i>0; i--)		  /* 放下攻击盘   			  */
		{
			OSTimeDly(1);
			Servo1Control(i);
		}
		modeSwitchFlag = 0;
		 
		/* 转圈 */
		TIM_SetCompare1(TIM3, 60);
		TIM_SetCompare2(TIM3, 0);
		TIM_SetCompare3(TIM3, 0);
	    TIM_SetCompare4(TIM3, 60);
		OSTimeDly(200);
		
		/* 前走 */
		TIM_SetCompare1(TIM3, 40);
		TIM_SetCompare2(TIM3, 0);
		TIM_SetCompare3(TIM3, 0);
	    TIM_SetCompare4(TIM3, 40);
		OSTimeDly(200);
		
		SixMotorControl(0);         /* 六路电机运行			  */
		
		if (startFlag == 1)
		{
			OSTaskResume(FIGHT_STA_DEC_TASK_PRIO);
			startFlag = 0;
		}
		
		OSTaskSuspend(OS_PRIO_SELF);  /* 挂起该任务               */
	}
}


/*************************************************************
*Function Name  : DebugTask
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : 机器人调试任务
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void DebugTask(void *pdata)
{
	pdata = pdata;
	
	INT8U errSemDebug;
	
	while (1)
	{
		OSSemPend(semDebug, 0, &errSemDebug);
		
		switch (modeSwitchFlag)
		{ 
			/******调试状态下：前进******/
			case 0x11:
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
			/******调试状态下：后退******/
			case 0x21:
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
			/******调试状态下：左转******/
			case 0x41:
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
			/******调试状态下：右转******/
			case 0x81:
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
			default: break;
		}
		
		OSTimeDly(OS_TICKS_PER_SEC);
		
		SixMotorControl(0);
		
		modeSwitchFlag &= 0x01;
		
		OSSemSet(semDebug, 0, &errSemDebug);      //...由于触摸屏未修复的BUG（虽然知道原因）
			                                      //...信号量会被会发出多次，在这里清除
	}

}

/*************************************************************
*Function Name  : LcdShowTask
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-10-31
*Description    : Lcd显示任务
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void LcdShowTask(void *pdata)
{
	pdata =  pdata; 
	INT8U pwmVariable = 0;  //存储所计算的占空比
	INT8U num = 0;
	char charBuff[8];      //字符串输出
	
	while (1)
	{
		#if 1 //climbSensorVal
		LCD_ShowString(112, 140, 24, 16, 16, (u8*)"   ");
		sprintf(charBuff, "Val:%3d", climbSensorVal);
		LCD_ShowString(80, 140, 56, 16, 16, (u8*)charBuff);
		#endif
		
		/*********Team Name、CPU使用率 显示界面***********/
		LCD_ShowString(10, 0, 88, 16, 16, (u8*)"Team: Tesla");
		LCD_ShowString(128, 0, 56, 16, 16, (u8*)"CPU:  %");
		LCD_ShowNum(160, 0, OSCPUUsage, 2, 16);
		
		/***************速度显示、修改界面*****************/
		/***边缘4个电机速度界面***/
		POINT_COLOR = WHITE;
		LCD_DrawRectangle(44,  36, 100, 60);
		LCD_DrawRectangle(104, 36, 160, 60);
		LCD_DrawRectangle(164, 36, 236, 60);
		POINT_COLOR = RED;
		LCD_Draw_Circle(22, 48, 18);
		LCD_ShowString(48,  40, 48, 16, 16, (u8*)"patrol");  //"patrol"字样
		LCD_ShowString(108, 40, 48, 16, 16, (u8*)"attack");  //"attack"字样
		LCD_ShowString(168, 40, 64, 16, 16, (u8*)"rotation");//"rotation"字样
		
		/***中间两个电机速度界面***/
		POINT_COLOR = WHITE;
		LCD_DrawRectangle(44,  104, 100, 128);
		LCD_DrawRectangle(104, 104, 160, 128);
		LCD_DrawRectangle(164, 104, 236, 128);
		POINT_COLOR = RED;
		LCD_Draw_Circle(22, 116, 18);
		LCD_ShowString(48,  108, 48, 16, 16, (u8*)"patrol");  //"patrol"字样
		LCD_ShowString(108, 108, 48, 16, 16, (u8*)"attack");  //"attack"字样
		LCD_ShowString(168, 108, 64, 16, 16, (u8*)"rotation");//"rotation"字样
		
		
		LCD_ShowString(10, 70, 48, 24, 24, (u8*)"save");      //"save"字样
		LCD_ShowString(73, 70, 48, 24, 24, (u8*)"edit");      //"edit"字样
		LCD_DrawLine(146, 82, 176, 82);                       //'+'号
		LCD_DrawLine(161, 67, 161, 97);                      
		LCD_DrawLine(200, 82, 230, 82);                       //'-'号
		
		LCD_ShowString(26, 40,  24, 16, 16, (u8*)"%");	      //'%'号 边缘4个
		LCD_ShowString(26, 108, 24, 16, 16, (u8*)"%");	      //'%'号 中间2个
		
		if (speedSwitchFlag & 0x01)       
		{
			LCD_DrawRectangle(44,  36,  100, 60);             //选中区域显示“正方框”
			pwmVariable = (INT8U)(p_speed1Select->patrol/4);
			sprintf(charBuff, "%2d", pwmVariable);
			LCD_ShowString(10, 40, 16, 16, 16, (u8*)charBuff);//显示测试速度			  
		}
		else if (speedSwitchFlag & 0x02)  
		{
			LCD_DrawRectangle(104,  36,  160, 60);
			pwmVariable = (INT8U)(p_speed1Select->attack/4);
			sprintf(charBuff, "%2d", pwmVariable);
			LCD_ShowString(10, 40, 16, 16, 16, (u8*)charBuff);
		}
		else if (speedSwitchFlag & 0x04)  
		{
			LCD_DrawRectangle(164,  36,  236, 60);
			pwmVariable = (INT8U)(p_speed1Select->rotation/4);
			sprintf(charBuff, "%2d", pwmVariable);
			LCD_ShowString(10, 40, 16, 16, 16, (u8*)charBuff);
		}
		else if (speedSwitchFlag & 0x08)  
		{
			LCD_DrawRectangle(44,  104,  100, 128);
			pwmVariable = (INT8U)(p_speed2Select->patrol/4);
			sprintf(charBuff, "%2d", pwmVariable);
			LCD_ShowString(10, 108, 16, 16, 16, (u8*)charBuff);
		}
		else if (speedSwitchFlag & 0x10)  
		{
			LCD_DrawRectangle(104,  104,  160, 128);
			pwmVariable = (INT8U)(p_speed2Select->attack/4);
			sprintf(charBuff, "%2d", pwmVariable);
			LCD_ShowString(10, 108, 16, 16, 16, (u8*)charBuff);
		}
		else if (speedSwitchFlag & 0x20)  
		{
			LCD_DrawRectangle(164,  104,  236, 128);
			pwmVariable = (INT8U)(p_speed2Select->rotation/4);
			sprintf(charBuff, "%2d", pwmVariable);
			LCD_ShowString(10, 108, 16, 16, 16, (u8*)charBuff);
		}
		
		/***********************姿态显示*************************/
		sprintf(charBuff, "x:%4d", p_receiveData->anglex);
		LCD_ShowString(92, 204, 48, 16, 16, (u8*)charBuff);
		sprintf(charBuff, "y:%4d", p_receiveData->angley);
		LCD_ShowString(92, 222, 48, 16, 16, (u8*)charBuff);
		sprintf(charBuff, "z:%4d", p_receiveData->anglez);
		LCD_ShowString(92, 240, 48, 16, 16, (u8*)charBuff);
		
		/*********机器人姿态、光电开关运行状态显示界面***********/
		/*机器人框架*/
		LCD_DrawLine(120, 184, 120, 199);
		LCD_DrawLine(120, 184, 126, 190);
		LCD_DrawLine(120, 184, 114, 190);
		
		LCD_DrawRectangle(70,  170,  170, 290);
		LCD_DrawRectangle(87,  199,  153, 261);
		/*四个负责边缘检测的光电开关*/
		LCD_Draw_Circle(87,  194, 4);
		LCD_Draw_Circle(153, 194, 4);
		LCD_Draw_Circle(87,  266, 4);
		LCD_Draw_Circle(153, 266, 4);
		/*四个负责前方的光电开关*/
		LCD_Draw_Circle(97 , 175, 4);
		LCD_Draw_Circle(113, 175, 4);
		LCD_Draw_Circle(127, 175, 4);
		LCD_Draw_Circle(142, 175, 4);
		/*四个负责后方的光电开关*/
		LCD_Draw_Circle(97 , 285, 4);
		LCD_Draw_Circle(113, 285, 4);
		LCD_Draw_Circle(127, 285, 4);
		LCD_Draw_Circle(142, 285, 4);
		/*两个负责左方的光电开关*/
		LCD_Draw_Circle(75 , 222, 4);;
		LCD_Draw_Circle(75 , 238, 4);
		/*两个负责边缘右方的光电开关*/
		LCD_Draw_Circle(165, 222, 4);
		LCD_Draw_Circle(165, 238, 4);
		
		/*画实心圆部分*/
		SensorAllClrCircle();
		for (num=1; num<=16; num++)
		{
			/*检测到有物体，则画实心圆*/
			if (ElecSensorRead(num) == 0) 
			{
				SensorShowCircle(num);
			}
		}
		
		/*********机器人控制按钮显示界面***********/
		LCD_DrawRectangle(48,  293, 110, 319);
		LCD_DrawRectangle(128, 293, 178, 319);
		LCD_ShowString( 50, 295, 60, 24, 24, (u8*)"Start");  /* "Start"字样  */
		LCD_ShowString(130, 295, 60, 24, 24, (u8*)"Stop");   /* "Stop"字样   */
		
		POINT_COLOR = WHITE;
		LCD_DrawRectangle(180, 170, 226, 192);
		LCD_DrawRectangle(180, 202, 226, 224);
		LCD_DrawRectangle(180, 234, 226, 256);
		LCD_DrawRectangle(180, 266, 226, 288);
		
		POINT_COLOR = LBBLUE;
		LCD_ShowString(183, 173, 40, 16, 16, (u8*)"Debug");  /* "Debug"字样  */
		LCD_ShowString(183, 205, 40, 16, 16, (u8*)"Climb");  /* "Climb"字样  */
		LCD_ShowString(183, 238, 40, 16, 16, (u8*)"Push");   /* "Push"字样   */
		LCD_ShowString(183, 270, 40, 16, 16, (u8*)"Fight");  /* "Fight"字样  */
		
		if     (modeSwitchFlag & 0x01)  { LCD_DrawRectangle(180, 170, 226, 192); }	
		else if(modeSwitchFlag & 0x02)  { LCD_DrawRectangle(180, 202, 226, 224); }	
		else if(modeSwitchFlag & 0x04)  { LCD_DrawRectangle(180, 234, 226, 256); }
		else if(modeSwitchFlag & 0x08)  { LCD_DrawRectangle(180, 266, 226, 288); }
		
		POINT_COLOR = WHITE;
		LCD_DrawRectangle(14, 170, 60, 192);
		LCD_DrawRectangle(14, 202, 60, 224);
		LCD_DrawRectangle(14, 234, 60, 256);
		LCD_DrawRectangle(14, 266, 60, 288);
		
		POINT_COLOR = BROWN;
		LCD_ShowString(17, 173, 40, 16, 16, (u8*)"Ahead");   /* "Ahead"字样  */
		LCD_ShowString(17, 205, 40, 16, 16, (u8*)"Back");    /* "Back"字样   */
		LCD_ShowString(17, 238, 40, 16, 16, (u8*)"TurnL");   /* "TurnL"字样  */
		LCD_ShowString(17, 270, 40, 16, 16, (u8*)"TurnR");   /* "TurnR"字样  */
		
		if     (modeSwitchFlag & 0x10)  { LCD_DrawRectangle(14, 170, 60, 192); }	
		else if(modeSwitchFlag & 0x20)  { LCD_DrawRectangle(14, 202, 60, 224); }	
		else if(modeSwitchFlag & 0x40)  { LCD_DrawRectangle(14, 234, 60, 256); }
		else if(modeSwitchFlag & 0x80)  { LCD_DrawRectangle(14, 266, 60, 288); }
		
		POINT_COLOR = RED;
		
		OSTimeDly(125);  //延时125个Ticks = 250ms 
	}
}


/*************************************************************
*Function Name  : TP_Task
*Auther         : 张沁
*Vertion        : v1.0
*Date           : 2017-10-31
*Description    : TP触摸屏任务
*Input          ：
*Output         ：
*Return         ：
*************************************************************/
void TP_Task(void *pdata)
{
	pdata =  pdata;
	u8 errTP_Task;
	INT8U i = 0;
	float speed1PatrolVar, speed1AttackVar, speed1RotationVar;
	float speed2PatrolVar, speed2AttackVar, speed2RotationVar;
	
	/*********************触摸屏初始化*********************/
    tp_dev.init();		  //触摸屏初始化
	POINT_COLOR=RED;      //设置字体为红色 
	if (tp_dev.touchtype != 0xFF)
	{
		LCD_ShowString(30, 130 ,200, 16, 16, (u8*)"Press KEY0 to Adjust");
	}
	Load_Drow_Dialog();   //清空屏幕并在右上角显示"CLR"
	
	/*读取保存在FLASH里面的速度参数*/
	SpeedGetData();  
	speed1PatrolVar   = p_speed1Select->patrol;
	speed1AttackVar   = p_speed1Select->attack;
	speed1RotationVar = p_speed1Select->rotation;
	
	speed2PatrolVar   = p_speed2Select->patrol;
	speed2AttackVar   = p_speed2Select->attack;
	speed2RotationVar = p_speed2Select->rotation;
	
	while(1)
	{
		OSFlagPend(flagTP, 0x01, OS_FLAG_WAIT_SET_ALL + OS_FLAG_CONSUME, 0, &errTP_Task);
		tp_dev.scan(0);
		if (tp_dev.x[0]>(lcddev.width-24) && tp_dev.y[0]<16)
		{
			Load_Drow_Dialog();            //清除
		}
		/*边缘四个电机*/
		else if (tp_dev.x[0]>44 && tp_dev.x[0]<100 && tp_dev.y[0]>36 && tp_dev.y[0]<60)
		{
			speedSwitchFlag &= 0x80;       //点击“patrol”字样
			speedSwitchFlag |= 0x01;
		}
		else if (tp_dev.x[0]>104 && tp_dev.x[0]<160 && tp_dev.y[0]>36 && tp_dev.y[0]<60)
		{
			speedSwitchFlag &= 0x80;       //点击“attack”字样
			speedSwitchFlag |= 0x02;
		}
		else if (tp_dev.x[0]>164 && tp_dev.x[0]<236 && tp_dev.y[0]>36 && tp_dev.y[0]<60)
		{
			speedSwitchFlag &= 0x80;       //点击“rotation”字样
			speedSwitchFlag |= 0x04;
		}
		/*中间两个电机*/
		else if (tp_dev.x[0]>44 && tp_dev.x[0]<100 && tp_dev.y[0]>104 && tp_dev.y[0]<128)
		{
			speedSwitchFlag &= 0x80;       //点击“patrol”字样(中间两个电机)
			speedSwitchFlag |= 0x08;
		}
		else if (tp_dev.x[0]>104 && tp_dev.x[0]<160 && tp_dev.y[0]>104 && tp_dev.y[0]<128)
		{
			speedSwitchFlag &= 0x80;       //点击“attack”字样(中间两个电机)
			speedSwitchFlag |= 0x10;
		}
		else if (tp_dev.x[0]>164 && tp_dev.x[0]<236 && tp_dev.y[0]>104 && tp_dev.y[0]<128)
		{
			speedSwitchFlag &= 0x80;       //点击“rotation”字样(中间两个电机)
			speedSwitchFlag |= 0x20;
		}
		else if (tp_dev.x[0]>10 && tp_dev.x[0]<58 && tp_dev.y[0]>70 && tp_dev.y[0]<94)
		{
			speedSwitchFlag &= 0x7F;       //点击“save”字样
			SpeedSaveData();               //向FLASH写入数据
		}
		else if (tp_dev.x[0]>73 && tp_dev.x[0]<121 && tp_dev.y[0]>70 && tp_dev.y[0]<94)
		{
			speedSwitchFlag |= 0x80;       //点击“eidt”字样
		}
		/******"+"字样 ******/
		else if ((speedSwitchFlag & 0x80) && tp_dev.x[0]>146 && tp_dev.x[0]<176 && tp_dev.y[0]>67 && tp_dev.y[0]<97)
		{
			switch (speedSwitchFlag)
			{
				case 0x81: 
				{
					speed1PatrolVar += (float)0.04; 
					if (speed1PatrolVar > 399) speed1PatrolVar = 399;
					p_speed1Select->patrol = (INT16U)speed1PatrolVar;
					break;
				}
				case 0x82: 
				{
					speed1AttackVar += (float)0.04; 
					if (speed1AttackVar > 399) speed1AttackVar = 399;
					p_speed1Select->attack = (INT16U)speed1AttackVar;
					break;
				}
				case 0x84: 
				{
					speed1RotationVar += (float)0.04; 
					if (speed1RotationVar > 399) speed1RotationVar = 399;
					p_speed1Select->rotation = (INT16U)speed1RotationVar;
					break;
				}
				case 0x88: 
				{
					speed2PatrolVar += (float)0.04; 
					if (speed2PatrolVar > 399) speed2PatrolVar = 399;
					p_speed2Select->patrol = (INT16U)speed2PatrolVar;
					break;
				}
				case 0x90: 
				{
					speed2AttackVar += (float)0.04; 
					if (speed2AttackVar > 399) speed2AttackVar = 399;
					p_speed2Select->attack = (INT16U)speed2AttackVar;
					break;
				}
				case 0xA0: 
				{
					speed2RotationVar += (float)0.04; 
					if (speed2RotationVar > 399) speed2RotationVar = 399;
					p_speed2Select->rotation = (INT16U)speed2RotationVar;
					break;
				}
				default: break;
			}
		}
		/******"-"字样 ******/
		else if ((speedSwitchFlag & 0x80) && tp_dev.x[0]>200 && tp_dev.x[0]<230 && tp_dev.y[0]>67 && tp_dev.y[0]<97)
		{
			switch (speedSwitchFlag)
			{
				case 0x81: 
				{
					speed1PatrolVar -= (float)0.04; 
					if (speed1PatrolVar < 0) speed1PatrolVar = 0;
					p_speed1Select->patrol = (INT16U)speed1PatrolVar;
					break;
				}
				case 0x82: 
				{
					speed1AttackVar -= (float)0.04; 
					if (speed1AttackVar < 0) speed1AttackVar = 0;
					p_speed1Select->attack = (INT16U)speed1AttackVar;
					break;
				}
				case 0x84: 
				{
					speed1RotationVar -= (float)0.04; 
					if (speed1RotationVar < 0) speed1RotationVar = 0;
					p_speed1Select->rotation = (INT16U)speed1RotationVar;
					break;
				}
				case 0x88: 
				{
					speed2PatrolVar -= (float)0.04; 
					if (speed2PatrolVar < 0) speed2PatrolVar = 0;
					p_speed2Select->patrol = (INT16U)speed2PatrolVar;
					break;
				}
				case 0x90: 
				{
					speed2AttackVar -= (float)0.04; 
					if (speed2AttackVar < 0) speed2AttackVar = 0;
					p_speed2Select->attack = (INT16U)speed2AttackVar;
					break;
				}
				case 0xA0: 
				{
					speed2RotationVar -= (float)0.04; 
					if (speed2RotationVar < 0) speed2RotationVar = 0;
					p_speed2Select->rotation = (INT16U)speed2RotationVar;
					break;
				}
				default: break;
			}
		}
		else if (tp_dev.x[0]>14 && tp_dev.x[0]<60 && tp_dev.y[0]>170 && tp_dev.y[0]<192)
		{
			if (modeSwitchFlag & 0x01)
			{
				modeSwitchFlag = 0x11;           /* 点击“Ahead”字样 				 */
				OSSemPost(semDebug);             /* 发送Debug信号量，调试开始任务运行 */
			}

		}
		else if (tp_dev.x[0]>14 && tp_dev.x[0]<60 && tp_dev.y[0]>202 && tp_dev.y[0]<224)
		{
			if (modeSwitchFlag & 0x01)
			{
				modeSwitchFlag = 0x21;           /* 点击“Back”字样  				 */
				OSSemPost(semDebug);             /* 发送Debug信号量，调试开始任务运行 */
			}
		}
		else if (tp_dev.x[0]>14 && tp_dev.x[0]<60 && tp_dev.y[0]>234 && tp_dev.y[0]<256)
		{
			if (modeSwitchFlag & 0x01)
			{
				modeSwitchFlag = 0x41;            /* 点击“TurnL”字样  				  */
				OSSemPost(semDebug);              /* 发送Debug信号量，调试开始任务运行 */
			}
		}
		else if (tp_dev.x[0]>14 && tp_dev.x[0]<60 && tp_dev.y[0]>266 && tp_dev.y[0]<288)
		{
			if (modeSwitchFlag & 0x01)
			{
				modeSwitchFlag = 0x81;            /* 点击“TurnR”字样  				  */
				OSSemPost(semDebug);              /* 发送Debug信号量，调试开始任务运行 */
			}
		}
		else if (tp_dev.x[0]>180 && tp_dev.x[0]<226 && tp_dev.y[0]>170 && tp_dev.y[0]<192)
		{
			modeSwitchFlag = 0x01;   		      /* 点击“Debug”字样			 */ 
		}
		else if (tp_dev.x[0]>180 && tp_dev.x[0]<226 && tp_dev.y[0]>202 && tp_dev.y[0]<224)
		{
			modeSwitchFlag = 0x02;   		      /* 点击“Climb”字样			 */
			OSTaskResume(CLIMB_TASK_PRIO);   
			OSTaskSuspend(FIGHT_STA_DEC_TASK_PRIO);
			OSTaskSuspend(PUSH_STA_DEC_TASK_PRIO);
			startFlag = 0;
		}
		else if (tp_dev.x[0]>180 && tp_dev.x[0]<226 && tp_dev.y[0]>234 && tp_dev.y[0]<256)
		{
			modeSwitchFlag = 0x04;   		      /* 点击“Push”字样              */
			OSTaskResume(PUSH_STA_DEC_TASK_PRIO); /* 开始运行PushStaDecTask       */
			OSTaskResume(MOTOR_TASK_PRIO);
			OSTaskSuspend(FIGHT_STA_DEC_TASK_PRIO);
			OSTaskSuspend(CLIMB_TASK_PRIO);
		}
		else if (tp_dev.x[0]>180 && tp_dev.x[0]<226 && tp_dev.y[0]>266 && tp_dev.y[0]<288)
		{
			modeSwitchFlag = 0x08;   		      /* 点击“Fight”字样			  */
			OSTaskResume(FIGHT_STA_DEC_TASK_PRIO);/* 开始运行FightStateDecideTask */
			OSTaskResume(MOTOR_TASK_PRIO);
			OSTaskSuspend(PUSH_STA_DEC_TASK_PRIO);
			OSTaskSuspend(CLIMB_TASK_PRIO);
		}
		else if (tp_dev.x[0]>48 && tp_dev.x[0]<110 && tp_dev.y[0]>293)
		{
			OSTaskResume(MOTOR_TASK_PRIO);       /* 点击“Start”字样  			 */
			OSTaskResume(CLIMB_TASK_PRIO);       /* 开始运行登台任务     	  	 */
			startFlag = 1;
			
		}
		else if (tp_dev.x[0]>128 && tp_dev.x[0]<178 && tp_dev.y[0]>293)
		{
			OSTaskSuspend(MOTOR_TASK_PRIO);      /* 点击“Stop”字样 			 */
			OSTaskSuspend(FIGHT_STA_DEC_TASK_PRIO);
			OSTaskSuspend(PUSH_STA_DEC_TASK_PRIO);
			OSTaskSuspend(CLIMB_TASK_PRIO);
			
			startFlag = 0;
			climbSensorVal = 0;
			modeSwitchFlag = 0;               
			SixMotorControl(0);
			ClimbMotorControl(0);
			for (i=0; i<100; i++)			      /* 抬起攻击盘   			  */
			{
			OSTimeDly(1);
			Servo1Control(i);
			}
		}
		else
		{
			TP_Draw_Big_Point(tp_dev.x[0], tp_dev.y[0], RED);
		}
	}
}
/*******************************代码草稿区*******************************/
#if 0

笔记：
u8 *p_end1 = NULL;
u8 *p_end2 = NULL;
把这个定义放在任务函数里面时，失去作用，不能正常获取字符串里面的内容 ？

	#if 0
	SetRotationAngle(TURN_LEFT, 90);
	printf("p_receiveData->setAngleZ = %f\r\n", p_receiveData->setAnglez);
	SetRotationAngle(TURN_RIGHT, 90);
	printf("p_receiveData->setAngleZ = %f\r\n", p_receiveData->setAnglez);
	#endif

		#if 0
		printf("\r\n/******begin******/\r\n");
		SetRotationAngle(TURN_RIGHT, 90);
	    printf("p_receiveData->setAngleZ = %f\r\n", p_receiveData->setAnglez);
		OSTimeDly(OS_TICKS_PER_SEC*5);
		printf("finally: %d", RotationJudgeOk(TURN_RIGHT));
		#endif
		
			#if 0
			printf("Sensor = %d\r\n", p_receiveData->photoelectricSensor);
			printf("anglex = %f\r\n", p_receiveData->anglex);
			printf("angley = %f\r\n", p_receiveData->angley);
			printf("anglez = %f\r\n", p_receiveData->anglez);
			#endif
			
		/*两个负责左方的光电开关*/
		LCD_Draw_Circle(75 , 215, 4);
		//LCD_Draw_Circle(75 , 230, 4);
		LCD_Draw_Circle(75 , 245, 4);
		/*两个负责边缘右方的光电开关*/
		LCD_Draw_Circle(165, 215, 4);
		//LCD_Draw_Circle(165, 230, 4);
		LCD_Draw_Circle(165, 245, 4);
		
		/***********************姿态显示*************************/
		sprintf(charBuff, "x:%f", 12);//p_receiveData->anglex);
		LCD_ShowString(92, 204, 56, 16, 16, (u8*)charBuff);
		sprintf(charBuff, "y:%f", p_receiveData->angley);
		LCD_ShowString(92, 222, 56, 16, 16, (u8*)charBuff);
		sprintf(charBuff, "z:%f", p_receiveData->anglez);
		LCD_ShowString(92, 240, 56, 16, 16, (u8*)charBuff);
		sprintf转换浮点数失效
#endif

