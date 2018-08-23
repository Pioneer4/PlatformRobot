/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : main.c
*Auther         : ����
*Version        : v1.0
*Date           : 2017-10-31
*Description    : ������ʡ��̨�����˴���ģ�壨uCOS��
*                 ϵͳƵ������Ϊ168MHz
*                 (���ڿ��Ǻ�����չ����������ϵͳ�����ſ��ܲ��㣬
*                 ���Կ���K60��ΪЭ��������ͨ������40ms����һ��
*				  ����)
*Function List  : 
*History        : 
*************************************************************/

#define  MY_CPP_GLOBALS 

#include "myinclude.h"
#include "includes.h"
 
extern u16 USART_RX_STA;  //����״̬���Ϊ0
extern u8 UART_RX_BUF[UART_REC_LEN];  //���ջ���

/*
�¼���־�飺���������
BIT0: �������Ƿ�ʼɨ�败���� 
*/
OS_FLAG_GRP *flagTP; 

OS_EVENT *semDebug;   //�ź���������Debug�ĵ��������������


/********************************************
����        ��MotorTask  �����˵����ת����(��սģʽ��)
�������ȼ�  ��9
�����ջ��С��256
�����ջ    ��MotorTaskStk[MOTOR_STK_SIZE]
*********************************************/
#define MOTOR_TASK_PRIO  9
#define MOTOR_STK_SIZE  512
OS_STK MotorTaskStk[MOTOR_STK_SIZE];
void MotorTask(void *pdata);


/********************************************
����        ��FightStaDecTask  ������״̬��������(��սģʽ��)
�������ȼ�  ��10
�����ջ��С��512
�����ջ    ��fightStaDecTaskStk[FIGHT_STA_DEC_STK_SIZE]
*********************************************/
#define FIGHT_STA_DEC_TASK_PRIO  10
#define FIGHT_STA_DEC_STK_SIZE  512
OS_STK fightStaDecTaskStk[FIGHT_STA_DEC_STK_SIZE];
void FightStaDecTask(void *pdata);


/********************************************
����        ��PushStaDecTask  ������״̬��������(��ľ��ģʽ��)
�������ȼ�  ��11
�����ջ��С��512
�����ջ    ��pushStaDecTaskStk[PUSH_STA_DEC_STK_SIZE]
*********************************************/
#define PUSH_STA_DEC_TASK_PRIO  11
#define PUSH_STA_DEC_STK_SIZE  512
OS_STK pushStaDecTaskStk[PUSH_STA_DEC_STK_SIZE];
void PushStaDecTask(void *pdata);


/********************************************
����        ��ClimbTask  ����������̨����
�������ȼ�  ��12
�����ջ��С��256
�����ջ    ��ClimbTaskStk[CLIMB_STK_SIZE]
*********************************************/
#define CLIMB_TASK_PRIO   12
#define CLIMB_STK_SIZE    256
OS_STK ClimbTaskStk[CLIMB_STK_SIZE];
void ClimbTask(void *pdata);


/********************************************
����        ��DebugTask  ��������
�������ȼ�  ��13
�����ջ��С��256
�����ջ    ��DebugTaskStk[DEBUG_STK_SIZE]
*********************************************/
#define DEBUG_TASK_PRIO    13
#define DEBUG_STK_SIZE    256
OS_STK DebugTaskStk[DEBUG_STK_SIZE];
void DebugTask(void *pdata);



/********************************************
����        ��LcdShowTask  LCD��ʾ������
�������ȼ�  ��14
�����ջ��С��256
�����ջ    ��lcdShowTaskStk[LCD_SHOW_STK_SIZE]
*********************************************/
#define LCD_SHOW_TASK_PRIO 14    	  
#define LCD_SHOW_STK_SIZE  256		    	
OS_STK lcdShowTaskStk[LCD_SHOW_STK_SIZE];
void LcdShowTask(void *pdata);


/********************************************
����        ��TP_Task    TP����������
�������ȼ�  ��15
�����ջ��С��256
�����ջ    ��tpTaskStk[TP_STK_SIZE];
*********************************************/
#define TP_TASK_PRIO      15
#define TP_STK_SIZE  	  256
OS_STK tpTaskStk[TP_STK_SIZE];
void TP_Task(void *pdata);

/********************************************
����        ��START ����������������
�������ȼ�  ��16
�����ջ��С��128
�����ջ    ��startTaskSTK[START_STK_SIZE]
*********************************************/
#define START_TASK_PRIO      16
#define START_STK_SIZE  	128
OS_STK startTaskSTK[START_STK_SIZE];
void StartTask(void *pdata);


/*************************************************************
*Function Name  : IwdgTmrCallBack
*Auther         : ����
*Vertion        : v1.0
*Date           : 2016-12-06
*Description    : ��ʱ���ص�����������ι����С�Ƶ���˸
*Input          : 1. void *ptmr
                  2. void *parg
				  û�д��ݵĲ��������������
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
	
	IwdgFeed();  //ι��
}


u8 err = 0;  //���󷵻���

INT8U startFlag = 0; /* ��ʼ��־ 1����ʼ 0������ */

/* ���ڴ������K60�������ַ��� */
u8 *p_end1 = NULL;
u8 *p_end2 = NULL;

int main(void)
{
	p_speed1Select = &speed1Select;
	p_speed2Select = &speed2Select;
	p_receiveData  = &receiveData;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	delay_init(168);      //��ʼ����ʱ����
	UART1_Init(115200);   //��ʼ������1
	LedInit();		      //��ʼ��LED�˿� 
	LCD_Init();	          //��ʼ��LCD	
	KeyInit();   	      //��ʼ������
	
	OSInit();             //��ʼ��uC/OS
	OSTaskCreate(StartTask, (void*)0, (OS_STK *)&startTaskSTK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	
	
	return 0;
}

/*************************************************************
*Function Name  : StartTask
*Auther         : ����
*Vertion        : v1.0
*Date           : 2016-12-05
*Description    : ��ʼ���� ���������ڽ�������������
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void StartTask(void *pdata)
{
	OS_CPU_SR cpu_sr = 0;
	
	pdata = pdata; 
	
	flagTP = OSFlagCreate(0, &err);    //�����¼���־��
	semDebug = OSSemCreate(0);         //�����ź���(Debug��������ȴ����ź���)
	
	OSStatInit();					   //��ʼ��ͳ������.�������ʱ1��������
	
	OS_ENTER_CRITICAL();		       //�����ٽ���(�޷����жϴ��)    
	
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
	
	IwdgInit(4,1500);                 //IWDG�ĳ�ʱ����Ϊ3S
	OSTaskSuspend(START_TASK_PRIO);	  //������ʼ����.
	OS_EXIT_CRITICAL();				  //�˳��ٽ���(���Ա��жϴ��)
} 


/*************************************************************
*Function Name  : MotorTask
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : ��սģʽ�£����������״̬�ڴ˱仯
*				  (ÿ25ms����һ��)
*                 ���"Start"�������и�����
*                 ���"Stop" �������������
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void MotorTask(void *pdata)
{
	pdata = pdata;

	AllMotorInit();       			/* ��ʼ���������ϵĵ��ϵͳ */
	
	Servo1Control(0);     			/* �����ʼ��λ��          */
	
	OSTaskSuspend(OS_PRIO_SELF);    /* ���������              */
	
	while (1)
	{
		RobotMoveContrl(robotState);
		OSTimeDly(5);
	}
}

/*************************************************************
*Function Name  : FightStateDecideTask
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-02
*Description    : ��սģʽ�£�ʵʱ��Ⲣ������̨�����˵�����״̬
*                 (ÿ25ms����һ��)
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void FightStaDecTask(void *pdata)
{
	pdata = pdata;
	
	INT32U randomNum = 0;         /* ���������        		    */
	INT16U ergentTime = 0; 		  /* �����˶���ʱ�����	 	    */
	INT16U rotationErrTime = 0;   /* ��ת״̬��ʱ�����	 	    */
	INT16U patrolTime = 0;        /* Ѳ��״̬��ʱ�����	 	    */
	INT16U angleRandom = 0;       /* �����ת�Ƕȱ���	 	    */
	
	RNG_Init();                   /* ��ʼ��RNG�����������       */
	
	OSTaskSuspend(OS_PRIO_SELF);  /* ���������          	    */
	
	robotState = ROBOT_PATROL;    /* �����˵ĳ���״̬ΪѲ��״̬  */
	
	while(1)
	{	
		/******�洢K60�����͵Ĺ��Թܼ��״̬����̬������x,y,z��Ƕȵ�����******/
		if (USART_RX_STA&0x8000)  /* �������                   */
		{
			p_receiveData->photoelectricSensor = (INT32U)strtod((char*)UART_RX_BUF, (char**)&p_end1);
			p_receiveData->anglex = (INT16S)strtod((char*)p_end1, (char**)&p_end2);
			p_receiveData->angley = (INT16S)strtod((char*)p_end2, (char**)&p_end1);
			p_receiveData->anglez = (INT16S)strtod((char*)p_end1, NULL);
			memset(UART_RX_BUF, 0, UART_REC_LEN);
			USART_RX_STA = 0;     /* ������ɱ�־λ��0          */
		}
		
		/******���Թܼ��󣬷��ػ����˱�Ե���桢�������λ��******/
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
		
		/******���ݹ��Թܼ��󷵻�״̬�����߻���������״̬******/
		switch (sensorState)
		{
			case SENSOR_WARN_FL:  					  /* �߽羯�棺ǰ�����        */
			{
				robotState = ROBOT_Ergent_B; 		  /* ״̬�л�����������        */
				robotRotationDir = SENSOR_WARN_FL;
				ergentTime = 0;    					  /* �����˶�ʱ�������0 	  */
				break;
			}
			case SENSOR_WARN_FR:  					  /* �߽羯�棺ǰ���Ҳ� 		  */
			{
				robotState = ROBOT_Ergent_B;  	      /* ״̬�л����������� 		  */
				robotRotationDir = SENSOR_WARN_FR;
				ergentTime = 0;    					  /* �����˶�ʱ�������0       */
				break;
			}
			case SENSOR_WARN_BL: 					  /* �߽羯�棺�����        */
			{
				robotState = ROBOT_Ergent_A; 		  /* ״̬�л�������ǰ�� 		  */
				robotRotationDir = SENSOR_WARN_BL;
				ergentTime = 0;    					  /* �����˶�ʱ�������0 	  */
				break;
			}
			case SENSOR_WARN_BR: 					  /* �߽羯�棺���Ҳ�		  */
			{
				robotState = ROBOT_Ergent_A; 		  /* ״̬�л�������ǰ��        */
				robotRotationDir = SENSOR_WARN_BR;
				ergentTime = 0;   					  /* �����˶�ʱ�������0 	  */
				break;
			}
			case SENSOR_F:       					  /* ������֣���ǰ��          */
			{
				robotState = ROBOT_ATTACK_F; 		  /* ״̬�л�:����״̬����ǰ�� */
				break;
			}
			case SENSOR_B:       					  /* ������֣�����          */
			{
				robotState = ROBOT_ATTACK_B; 		  /* ״̬�л�:����״̬������ */
				break;
			}
			case  SENSOR_FL:  	                      /* ������֣�ǰ����        */
			{
				robotState = ROBOT_ROTATION_L;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_LEFT, ANGLE_FL);/* ����������ת�� 			  */
				rotationErrTime = 0;				  /* ��תʱ�������0 		  */
				break;
			}
			case  SENSOR_FR:  	 					  /* ������֣�ǰ���ҷ� 		  */
			{
				robotState = ROBOT_ROTATION_R;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_RIGHT,ANGLE_FR);/* ����������ת��            */
				rotationErrTime = 0; 				  /* ��תʱ�������0        	  */
				break;
			}
			case  SENSOR_BL:  	 					  /* ������֣������ 		  */
			{
				robotState = ROBOT_ROTATION_R;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_RIGHT,ANGLE_BL);/* ����������ת�� 			  */
				rotationErrTime = 0; 				  /* ��תʱ�������0			  */
				break;
			}
			case  SENSOR_BR:  	 					  /* ������֣�����ҷ� 	  	  */
			{
				robotState = ROBOT_ROTATION_L;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_LEFT, ANGLE_BR);/* ����������ת��    	      */
				rotationErrTime = 0; 				  /* ��תʱ�������0 	      */
				break;
			}
			case  SENSOR_L:  	 					  /* ������֣����� 		  */
			{
				robotState = ROBOT_ROTATION_L;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_LEFT, ANGLE_L); /* ����������ת�� 		      */
				rotationErrTime = 0; 				  /* ��תʱ�������0 		  */
				break;
			}
			case  SENSOR_R:  	 					  /* ������֣�����          */
			{
				robotState = ROBOT_ROTATION_R;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_RIGHT, ANGLE_R);/* ����������ת�� 			  */
				rotationErrTime = 0; 				  /* ��תʱ�������0 		  */
				break;
			}
			default: break;
		}
		
		switch (robotState)
		{
			/********************���ڽ�������״̬********************/
			case ROBOT_Ergent_B: 
			{
				if (ergentTime < ERGENT_BACK_TIME) 					    /* ��������ʱ��δ�ﵽҪ��      				  */
				{
					ergentTime++;
				}
				else                              					    /* ����ʱ��ﵽҪ��󣬽���״̬�л�����ת�Ƕ����� */
				{
					if (robotRotationDir == SENSOR_WARN_FL)			    /* �������˺�����ת 						      */
					{ 
		
						robotState = ROBOT_ROTATION_R;     			    /* ״̬�л�����ת״̬(����)					  */
						SetRotationAngle(TURN_RIGHT, ANGLE_ERGENT_BACK);/* ����������ת�� 							  */
						rotationErrTime = 0; 							/* ��תʱ�������0					    	  */ 
					}
					else if (robotRotationDir == SENSOR_WARN_FR) 		/* �������˺�����ת 							  */
					{
						robotState = ROBOT_ROTATION_L;      			/* ״̬�л�����ת״̬(����) 					  */
						SetRotationAngle(TURN_LEFT, ANGLE_ERGENT_BACK);	/* ����������ת�� 							  */
						rotationErrTime = 0; 							/* ��תʱ�������0 							  */
					}
				}
				break;
			}
			
			/********************���ڽ���ǰ��״̬********************/
			case ROBOT_Ergent_A:   										/* ����ǰ��ʱ��δ�ﵽҪ�� 					  */
			{
				if (ergentTime < ERGENT_AHEAD_TIME)
				{
					ergentTime++;
				}
				else             										/* ǰ��ʱ��ﵽҪ��󣬽���״̬�л�����ת�Ƕ����� */
				{
					if (robotRotationDir == SENSOR_WARN_BL) 			/* ����ǰ��������ת 							  */
					{
						robotState = ROBOT_ROTATION_R;     				/* ״̬�л�����ת״̬(����) 					  */
						SetRotationAngle(TURN_RIGHT, ANGLE_ERGENT_AHEAD);/* ����������ת�� 		   					  */
						rotationErrTime = 0; 					    	/* ��תʱ�������0 							  */
					}
					else if (robotRotationDir == SENSOR_WARN_BR)		/* ����ǰ���˺�����ת 						  */
					{
						robotState = ROBOT_ROTATION_L;      			/* ״̬�л�����ת״̬(����) 					  */
						SetRotationAngle(TURN_LEFT, ANGLE_ERGENT_AHEAD);/* ����������ת�� 							  */
						rotationErrTime = 0; /* ��תʱ�������0 */
					}
				}
				break;
			}
			
			/********************������ת״̬(����)********************/
			case ROBOT_ROTATION_R:
			{
				if (RotationJudgeOk(TURN_RIGHT) == 0) 					/* ��ת�ǶȴﵽҪ�� 							  */
				{
					robotState = ROBOT_PATROL; 							/* ״̬�л���Ѳ��״̬ 						  */
					patrolTime = 0; 		   							/* Ѳ��ʱ�������0							  */
				}
				else if (rotationErrTime > ROTATION_ERR_TIME) 			/* ������ת 									  */
				{
					robotState = ROBOT_PATROL; 							/* ״̬�л���Ѳ��״̬						 	  */
					patrolTime = 0;			   							/* Ѳ��ʱ�������0 							  */
				}
				else
				{
					rotationErrTime++;
				}
				break;
			}
			
			/********************������ת״̬(����)********************/
			case ROBOT_ROTATION_L:
			{
				if (RotationJudgeOk(TURN_LEFT) == 0) 					/* ��ת�ǶȴﵽҪ�� 							  */
				{
					robotState = ROBOT_PATROL; 							/* ״̬�л���Ѳ��״̬ 						  */
					patrolTime = 0;			   							/* Ѳ��ʱ�������0 							  */
				}
				else if (rotationErrTime > ROTATION_ERR_TIME) 			/* ������ת 								 	  */
				{
					robotState = ROBOT_PATROL; 							/* ״̬�л���Ѳ��״̬ 						  */
					patrolTime = 0;			   							/* Ѳ��ʱ�������0 							  */
				}
				else
				{
					rotationErrTime++;
				}
				break;
			}
			
			/********************���ڹ���״̬����ǰ��********************/
			case ROBOT_ATTACK_F:
			{
				if ( (ElecSensorRead(6) != 0) && (ElecSensorRead(7) != 0) )/* ǰ��������ʧ 							  */
				{
					robotState = ROBOT_PATROL;   						/* ״̬�л���Ѳ��״̬ 						  */
					patrolTime = 0;			     						/* Ѳ��ʱ�������0   						  */
				}
				break;
			}
			
			/********************���ڹ���״̬������********************/
			case ROBOT_ATTACK_B:
			{
				if ( (ElecSensorRead(10) != 0) && (ElecSensorRead(11) != 0) )/* ��������ʧ 						  */
				{
					robotState = ROBOT_PATROL;   						/* ״̬�л���Ѳ��״̬ 						  */
					patrolTime = 0;			     						/* Ѳ��ʱ�������0    						  */
				}
				break;
			}
			
			/********************����Ѳ��״̬********************/
			case ROBOT_PATROL:
			{
				if (patrolTime == STATE_SWITCH_TIME) 					/* �ﵽ״̬�Զ��л�ʱ�� 						  */
				{
					randomNum = RNG_Get_RandomRange(1, 16);
					if (randomNum == 8)
					{
						angleRandom = (INT16U)RNG_Get_RandomRange(ANGLE_RANDOM_MIN, ANGLE_RANDOM_MAX);
						robotState = ROBOT_ROTATION_R;      			/* ״̬�л�����ת״̬(����) 					  */
						SetRotationAngle(TURN_RIGHT, angleRandom);		/* ����������ת�� 							  */
						rotationErrTime = 0; 							/* ��תʱ�������0 							  */
					}
					else if (randomNum == 16)
					{
						angleRandom = (INT16U)RNG_Get_RandomRange(ANGLE_RANDOM_MIN, ANGLE_RANDOM_MAX);
						robotState = ROBOT_ROTATION_L;      			/* ״̬�л�����ת״̬(����) 					  */
						SetRotationAngle(TURN_LEFT, angleRandom);		/* ����������ת�� 							  */
						rotationErrTime = 0; 							/* ��תʱ�������0 							  */
					}
			    }
				else
				{
					patrolTime++;
				}
			}
			default: break;
		}
		
		OSTimeDly(10);  //ÿ50ms����һ��
	}
}

/*************************************************************
*Function Name  : PushStaDecTask
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-02
*Description    : ��ľ��ģʽ��ʵʱ��Ⲣ������̨�����˵�����״̬
*                 (ÿ25ms����һ��)
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void PushStaDecTask(void *pdata)
{
	pdata = pdata;
	
	//INT16U ergentTime = 0; 	  /* �����˶���ʱ�����	 	    */
	INT16U rotationErrTime = 0;   /* ��ת״̬��ʱ�����	 	    */
	
	OSTaskSuspend(OS_PRIO_SELF);  /* ���������          	    */
	
	robotState = ROBOT_PATROL;    /* �����˵ĳ���״̬ΪѲ��״̬  */
	
	while (1)
	{
		/******�洢K60�����͵Ĺ��Թܼ��״̬����̬������x,y,z��Ƕȵ�����******/
		if (USART_RX_STA&0x8000)  /* �������                   */
		{
			p_receiveData->photoelectricSensor = (INT32U)strtod((char*)UART_RX_BUF, (char**)&p_end1);
			p_receiveData->anglex = (INT16S)strtod((char*)p_end1, (char**)&p_end2);
			p_receiveData->angley = (INT16S)strtod((char*)p_end2, (char**)&p_end1);
			p_receiveData->anglez = (INT16S)strtod((char*)p_end1, NULL);
			memset(UART_RX_BUF, 0, UART_REC_LEN);
			USART_RX_STA = 0;     /* ������ɱ�־λ��0          */
		}
		
		/******���Թܼ��󣬷��ػ����˱�Ե���桢�������λ��******/
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
		
		
		/******���ݹ��Թܼ��󷵻�״̬�����߻���������״̬******/
		switch (sensorState)
		{
			case SENSOR_WARN_FL:  					  /* �߽羯�棺ǰ�����        */
			{
				robotState = ROBOT_Ergent_B; 		  /* ״̬�л�����������        */
				robotRotationDir = SENSOR_WARN_FL;
				//ergentTime = 0;    					  /* �����˶�ʱ�������0 	  */
				break;
			}
			case SENSOR_WARN_FR:  					  /* �߽羯�棺ǰ���Ҳ� 		  */
			{
				robotState = ROBOT_Ergent_B;  	      /* ״̬�л����������� 		  */
				robotRotationDir = SENSOR_WARN_FR;
				//ergentTime = 0;    					  /* �����˶�ʱ�������0       */
				break;
			}
			case SENSOR_WARN_BL: 					  /* �߽羯�棺�����        */
			{
				robotState = ROBOT_Ergent_A; 		  /* ״̬�л�������ǰ�� 		  */
				robotRotationDir = SENSOR_WARN_BL;
				//ergentTime = 0;    					  /* �����˶�ʱ�������0 	  */
				break;
			}
			case SENSOR_WARN_BR: 					  /* �߽羯�棺���Ҳ�		  */
			{
				robotState = ROBOT_Ergent_A; 		  /* ״̬�л�������ǰ��        */
				robotRotationDir = SENSOR_WARN_BR;
				//ergentTime = 0;   					  /* �����˶�ʱ�������0 	  */
				break;
			}
			case SENSOR_F:       					  /* ������֣���ǰ��          */
			{
				robotState = ROBOT_ATTACK_F; 		  /* ״̬�л�:����״̬����ǰ�� */
				break;
			}
			case SENSOR_B:       					  /* ������֣�����          */
			{
				robotState = ROBOT_ATTACK_B; 		  /* ״̬�л�:����״̬������ */
				break;
			}
			case  SENSOR_FL:  	                      /* ������֣�ǰ����        */
			{
				robotState = ROBOT_ROTATION_L;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_LEFT, ANGLE_FL);/* ����������ת�� 			  */
				rotationErrTime = 0;				  /* ��תʱ�������0 		  */
				break;
			}
			case  SENSOR_FR:  	 					  /* ������֣�ǰ���ҷ� 		  */
			{
				robotState = ROBOT_ROTATION_R;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_RIGHT,ANGLE_FR);/* ����������ת��            */
				rotationErrTime = 0; 				  /* ��תʱ�������0        	  */
				break;
			}
			case  SENSOR_BL:  	 					  /* ������֣������ 		  */
			{
				robotState = ROBOT_ROTATION_R;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_RIGHT,ANGLE_BL);/* ����������ת�� 			  */
				rotationErrTime = 0; 				  /* ��תʱ�������0			  */
				break;
			}
			case  SENSOR_BR:  	 					  /* ������֣�����ҷ� 	  	  */
			{
				robotState = ROBOT_ROTATION_L;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_LEFT, ANGLE_BR);/* ����������ת��    	      */
				rotationErrTime = 0; 				  /* ��תʱ�������0 	      */
				break;
			}
			case  SENSOR_L:  	 					  /* ������֣����� 		  */
			{
				robotState = ROBOT_ROTATION_L;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_LEFT, ANGLE_L); /* ����������ת�� 		      */
				rotationErrTime = 0; 				  /* ��תʱ�������0 		  */
				break;
			}
			case  SENSOR_R:  	 					  /* ������֣�����          */
			{
				robotState = ROBOT_ROTATION_R;        /* ״̬�л�����ת״̬(����)  */
				SetRotationAngle(TURN_RIGHT, ANGLE_R);/* ����������ת�� 			  */
				rotationErrTime = 0; 				  /* ��תʱ�������0 		  */
				break;
			}
			default: break;
		}
		
		switch (robotState)
		{
			/********************���ڽ�������״̬********************/
			case ROBOT_Ergent_B: 
			{
				OSTimeDly(20);  //100ms
				if (robotRotationDir == SENSOR_WARN_FL)			    /* �������˺�����ת 						      */
				{ 
	
					robotState = ROBOT_ROTATION_R;     			    /* ״̬�л�����ת״̬(����)					  */
					SetRotationAngle(TURN_RIGHT, ANGLE_ERGENT_BACK);/* ����������ת�� 							  */
					rotationErrTime = 0; 							/* ��תʱ�������0					    	  */ 
				}
				else if (robotRotationDir == SENSOR_WARN_FR) 		/* �������˺�����ת 							  */
				{
					robotState = ROBOT_ROTATION_L;      			/* ״̬�л�����ת״̬(����) 					  */
					SetRotationAngle(TURN_LEFT, ANGLE_ERGENT_BACK);	/* ����������ת�� 							  */
					rotationErrTime = 0; 							/* ��תʱ�������0 							  */
				}
				
				break;
			}
			
			/********************���ڽ���ǰ��״̬********************/
			case ROBOT_Ergent_A:   										/* ����ǰ��ʱ��δ�ﵽҪ�� 					  */
			{
				OSTimeDly(20);
				if (robotRotationDir == SENSOR_WARN_BL) 			/* ����ǰ��������ת 							  */
				{
					robotState = ROBOT_ROTATION_R;     				/* ״̬�л�����ת״̬(����) 					  */
					SetRotationAngle(TURN_RIGHT, ANGLE_ERGENT_AHEAD);/* ����������ת�� 		   					  */
					rotationErrTime = 0; 					    	/* ��תʱ�������0 							  */
				}
				else if (robotRotationDir == SENSOR_WARN_BR)		/* ����ǰ���˺�����ת 						  */
				{
					robotState = ROBOT_ROTATION_L;      			/* ״̬�л�����ת״̬(����) 					  */
					SetRotationAngle(TURN_LEFT, ANGLE_ERGENT_AHEAD);/* ����������ת�� 							  */
					rotationErrTime = 0; 							/* ��תʱ�������0 */
				}
				
				break;
			}
			
			/********************������ת״̬(����)********************/
			case ROBOT_ROTATION_R:
			{
				if (RotationJudgeOk(TURN_RIGHT) == 0) 					/* ��ת�ǶȴﵽҪ�� 							  */
				{
					robotState = ROBOT_PATROL; 							/* ״̬�л���Ѳ��״̬ 						  */
				}
				else if (rotationErrTime > ROTATION_ERR_TIME) 			/* ������ת 									  */
				{
					robotState = ROBOT_PATROL; 							/* ״̬�л���Ѳ��״̬						 	  */
				}
				else
				{
					rotationErrTime += 1;
				}
				break;
			}
			
			/********************������ת״̬(����)********************/
			case ROBOT_ROTATION_L:
			{
				if (RotationJudgeOk(TURN_LEFT) == 0) 					/* ��ת�ǶȴﵽҪ�� 							  */
				{
					robotState = ROBOT_PATROL; 							/* ״̬�л���Ѳ��״̬ 						  */
				}
				else if (rotationErrTime > ROTATION_ERR_TIME) 			/* ������ת 								 	  */
				{
					robotState = ROBOT_PATROL; 							/* ״̬�л���Ѳ��״̬ 						  */
				}
				else
				{
					rotationErrTime++;
				}
				break;
			}
			default: break;
		}
		
		OSTimeDly(10);  //ÿ50ms����һ��
	}
}

/*************************************************************
*Function Name  : ClimbTask
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : ��������˵�����̨
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void ClimbTask(void *pdata)
{
	pdata = pdata;
	
	INT8U i = 0;
	
	OSTaskSuspend(OS_PRIO_SELF);      /* ���������               */
	
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
		

		climbSensorVal = 0;           /* ��̨�����λ�÷���������0 */
		OSTimeDly(OS_TICKS_PER_SEC);  /* �ȴ�1S      			  */ 		    
		ClimbMotorControl(80);        /* ��̨������� 			  */
		SixMotorControl(240);         /* ��·�������			  */
		
		OSTimeDly(150);
		ClimbMotorControl(0);         /* ��̨���ͣת 			  */
		//OSTimeDly(130);  		      /* �ȴ�600ms     			  */
		SixMotorControl(0);
		
		/* ��ǰ�� */
		TIM_SetCompare1(TIM3, 40);
		TIM_SetCompare2(TIM3, 0);
		TIM_SetCompare3(TIM3, 40);
	    TIM_SetCompare4(TIM3, 0);
		for (i=100; i>0; i--)		  /* ���¹�����   			  */
		{
			OSTimeDly(1);
			Servo1Control(i);
		}
		modeSwitchFlag = 0;
		 
		/* תȦ */
		TIM_SetCompare1(TIM3, 60);
		TIM_SetCompare2(TIM3, 0);
		TIM_SetCompare3(TIM3, 0);
	    TIM_SetCompare4(TIM3, 60);
		OSTimeDly(200);
		
		/* ǰ�� */
		TIM_SetCompare1(TIM3, 40);
		TIM_SetCompare2(TIM3, 0);
		TIM_SetCompare3(TIM3, 0);
	    TIM_SetCompare4(TIM3, 40);
		OSTimeDly(200);
		
		SixMotorControl(0);         /* ��·�������			  */
		
		if (startFlag == 1)
		{
			OSTaskResume(FIGHT_STA_DEC_TASK_PRIO);
			startFlag = 0;
		}
		
		OSTaskSuspend(OS_PRIO_SELF);  /* ���������               */
	}
}


/*************************************************************
*Function Name  : DebugTask
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-11-06
*Description    : �����˵�������
*Input          ��
*Output         ��
*Return         ��
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
			/******����״̬�£�ǰ��******/
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
			/******����״̬�£�����******/
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
			/******����״̬�£���ת******/
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
			/******����״̬�£���ת******/
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
		
		OSSemSet(semDebug, 0, &errSemDebug);      //...���ڴ�����δ�޸���BUG����Ȼ֪��ԭ��
			                                      //...�ź����ᱻ�ᷢ����Σ����������
	}

}

/*************************************************************
*Function Name  : LcdShowTask
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-10-31
*Description    : Lcd��ʾ����
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void LcdShowTask(void *pdata)
{
	pdata =  pdata; 
	INT8U pwmVariable = 0;  //�洢�������ռ�ձ�
	INT8U num = 0;
	char charBuff[8];      //�ַ������
	
	while (1)
	{
		#if 1 //climbSensorVal
		LCD_ShowString(112, 140, 24, 16, 16, (u8*)"   ");
		sprintf(charBuff, "Val:%3d", climbSensorVal);
		LCD_ShowString(80, 140, 56, 16, 16, (u8*)charBuff);
		#endif
		
		/*********Team Name��CPUʹ���� ��ʾ����***********/
		LCD_ShowString(10, 0, 88, 16, 16, (u8*)"Team: Tesla");
		LCD_ShowString(128, 0, 56, 16, 16, (u8*)"CPU:  %");
		LCD_ShowNum(160, 0, OSCPUUsage, 2, 16);
		
		/***************�ٶ���ʾ���޸Ľ���*****************/
		/***��Ե4������ٶȽ���***/
		POINT_COLOR = WHITE;
		LCD_DrawRectangle(44,  36, 100, 60);
		LCD_DrawRectangle(104, 36, 160, 60);
		LCD_DrawRectangle(164, 36, 236, 60);
		POINT_COLOR = RED;
		LCD_Draw_Circle(22, 48, 18);
		LCD_ShowString(48,  40, 48, 16, 16, (u8*)"patrol");  //"patrol"����
		LCD_ShowString(108, 40, 48, 16, 16, (u8*)"attack");  //"attack"����
		LCD_ShowString(168, 40, 64, 16, 16, (u8*)"rotation");//"rotation"����
		
		/***�м���������ٶȽ���***/
		POINT_COLOR = WHITE;
		LCD_DrawRectangle(44,  104, 100, 128);
		LCD_DrawRectangle(104, 104, 160, 128);
		LCD_DrawRectangle(164, 104, 236, 128);
		POINT_COLOR = RED;
		LCD_Draw_Circle(22, 116, 18);
		LCD_ShowString(48,  108, 48, 16, 16, (u8*)"patrol");  //"patrol"����
		LCD_ShowString(108, 108, 48, 16, 16, (u8*)"attack");  //"attack"����
		LCD_ShowString(168, 108, 64, 16, 16, (u8*)"rotation");//"rotation"����
		
		
		LCD_ShowString(10, 70, 48, 24, 24, (u8*)"save");      //"save"����
		LCD_ShowString(73, 70, 48, 24, 24, (u8*)"edit");      //"edit"����
		LCD_DrawLine(146, 82, 176, 82);                       //'+'��
		LCD_DrawLine(161, 67, 161, 97);                      
		LCD_DrawLine(200, 82, 230, 82);                       //'-'��
		
		LCD_ShowString(26, 40,  24, 16, 16, (u8*)"%");	      //'%'�� ��Ե4��
		LCD_ShowString(26, 108, 24, 16, 16, (u8*)"%");	      //'%'�� �м�2��
		
		if (speedSwitchFlag & 0x01)       
		{
			LCD_DrawRectangle(44,  36,  100, 60);             //ѡ��������ʾ��������
			pwmVariable = (INT8U)(p_speed1Select->patrol/4);
			sprintf(charBuff, "%2d", pwmVariable);
			LCD_ShowString(10, 40, 16, 16, 16, (u8*)charBuff);//��ʾ�����ٶ�			  
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
		
		/***********************��̬��ʾ*************************/
		sprintf(charBuff, "x:%4d", p_receiveData->anglex);
		LCD_ShowString(92, 204, 48, 16, 16, (u8*)charBuff);
		sprintf(charBuff, "y:%4d", p_receiveData->angley);
		LCD_ShowString(92, 222, 48, 16, 16, (u8*)charBuff);
		sprintf(charBuff, "z:%4d", p_receiveData->anglez);
		LCD_ShowString(92, 240, 48, 16, 16, (u8*)charBuff);
		
		/*********��������̬����翪������״̬��ʾ����***********/
		/*�����˿��*/
		LCD_DrawLine(120, 184, 120, 199);
		LCD_DrawLine(120, 184, 126, 190);
		LCD_DrawLine(120, 184, 114, 190);
		
		LCD_DrawRectangle(70,  170,  170, 290);
		LCD_DrawRectangle(87,  199,  153, 261);
		/*�ĸ������Ե���Ĺ�翪��*/
		LCD_Draw_Circle(87,  194, 4);
		LCD_Draw_Circle(153, 194, 4);
		LCD_Draw_Circle(87,  266, 4);
		LCD_Draw_Circle(153, 266, 4);
		/*�ĸ�����ǰ���Ĺ�翪��*/
		LCD_Draw_Circle(97 , 175, 4);
		LCD_Draw_Circle(113, 175, 4);
		LCD_Draw_Circle(127, 175, 4);
		LCD_Draw_Circle(142, 175, 4);
		/*�ĸ�����󷽵Ĺ�翪��*/
		LCD_Draw_Circle(97 , 285, 4);
		LCD_Draw_Circle(113, 285, 4);
		LCD_Draw_Circle(127, 285, 4);
		LCD_Draw_Circle(142, 285, 4);
		/*���������󷽵Ĺ�翪��*/
		LCD_Draw_Circle(75 , 222, 4);;
		LCD_Draw_Circle(75 , 238, 4);
		/*���������Ե�ҷ��Ĺ�翪��*/
		LCD_Draw_Circle(165, 222, 4);
		LCD_Draw_Circle(165, 238, 4);
		
		/*��ʵ��Բ����*/
		SensorAllClrCircle();
		for (num=1; num<=16; num++)
		{
			/*��⵽�����壬��ʵ��Բ*/
			if (ElecSensorRead(num) == 0) 
			{
				SensorShowCircle(num);
			}
		}
		
		/*********�����˿��ư�ť��ʾ����***********/
		LCD_DrawRectangle(48,  293, 110, 319);
		LCD_DrawRectangle(128, 293, 178, 319);
		LCD_ShowString( 50, 295, 60, 24, 24, (u8*)"Start");  /* "Start"����  */
		LCD_ShowString(130, 295, 60, 24, 24, (u8*)"Stop");   /* "Stop"����   */
		
		POINT_COLOR = WHITE;
		LCD_DrawRectangle(180, 170, 226, 192);
		LCD_DrawRectangle(180, 202, 226, 224);
		LCD_DrawRectangle(180, 234, 226, 256);
		LCD_DrawRectangle(180, 266, 226, 288);
		
		POINT_COLOR = LBBLUE;
		LCD_ShowString(183, 173, 40, 16, 16, (u8*)"Debug");  /* "Debug"����  */
		LCD_ShowString(183, 205, 40, 16, 16, (u8*)"Climb");  /* "Climb"����  */
		LCD_ShowString(183, 238, 40, 16, 16, (u8*)"Push");   /* "Push"����   */
		LCD_ShowString(183, 270, 40, 16, 16, (u8*)"Fight");  /* "Fight"����  */
		
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
		LCD_ShowString(17, 173, 40, 16, 16, (u8*)"Ahead");   /* "Ahead"����  */
		LCD_ShowString(17, 205, 40, 16, 16, (u8*)"Back");    /* "Back"����   */
		LCD_ShowString(17, 238, 40, 16, 16, (u8*)"TurnL");   /* "TurnL"����  */
		LCD_ShowString(17, 270, 40, 16, 16, (u8*)"TurnR");   /* "TurnR"����  */
		
		if     (modeSwitchFlag & 0x10)  { LCD_DrawRectangle(14, 170, 60, 192); }	
		else if(modeSwitchFlag & 0x20)  { LCD_DrawRectangle(14, 202, 60, 224); }	
		else if(modeSwitchFlag & 0x40)  { LCD_DrawRectangle(14, 234, 60, 256); }
		else if(modeSwitchFlag & 0x80)  { LCD_DrawRectangle(14, 266, 60, 288); }
		
		POINT_COLOR = RED;
		
		OSTimeDly(125);  //��ʱ125��Ticks = 250ms 
	}
}


/*************************************************************
*Function Name  : TP_Task
*Auther         : ����
*Vertion        : v1.0
*Date           : 2017-10-31
*Description    : TP����������
*Input          ��
*Output         ��
*Return         ��
*************************************************************/
void TP_Task(void *pdata)
{
	pdata =  pdata;
	u8 errTP_Task;
	INT8U i = 0;
	float speed1PatrolVar, speed1AttackVar, speed1RotationVar;
	float speed2PatrolVar, speed2AttackVar, speed2RotationVar;
	
	/*********************��������ʼ��*********************/
    tp_dev.init();		  //��������ʼ��
	POINT_COLOR=RED;      //��������Ϊ��ɫ 
	if (tp_dev.touchtype != 0xFF)
	{
		LCD_ShowString(30, 130 ,200, 16, 16, (u8*)"Press KEY0 to Adjust");
	}
	Load_Drow_Dialog();   //�����Ļ�������Ͻ���ʾ"CLR"
	
	/*��ȡ������FLASH������ٶȲ���*/
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
			Load_Drow_Dialog();            //���
		}
		/*��Ե�ĸ����*/
		else if (tp_dev.x[0]>44 && tp_dev.x[0]<100 && tp_dev.y[0]>36 && tp_dev.y[0]<60)
		{
			speedSwitchFlag &= 0x80;       //�����patrol������
			speedSwitchFlag |= 0x01;
		}
		else if (tp_dev.x[0]>104 && tp_dev.x[0]<160 && tp_dev.y[0]>36 && tp_dev.y[0]<60)
		{
			speedSwitchFlag &= 0x80;       //�����attack������
			speedSwitchFlag |= 0x02;
		}
		else if (tp_dev.x[0]>164 && tp_dev.x[0]<236 && tp_dev.y[0]>36 && tp_dev.y[0]<60)
		{
			speedSwitchFlag &= 0x80;       //�����rotation������
			speedSwitchFlag |= 0x04;
		}
		/*�м��������*/
		else if (tp_dev.x[0]>44 && tp_dev.x[0]<100 && tp_dev.y[0]>104 && tp_dev.y[0]<128)
		{
			speedSwitchFlag &= 0x80;       //�����patrol������(�м��������)
			speedSwitchFlag |= 0x08;
		}
		else if (tp_dev.x[0]>104 && tp_dev.x[0]<160 && tp_dev.y[0]>104 && tp_dev.y[0]<128)
		{
			speedSwitchFlag &= 0x80;       //�����attack������(�м��������)
			speedSwitchFlag |= 0x10;
		}
		else if (tp_dev.x[0]>164 && tp_dev.x[0]<236 && tp_dev.y[0]>104 && tp_dev.y[0]<128)
		{
			speedSwitchFlag &= 0x80;       //�����rotation������(�м��������)
			speedSwitchFlag |= 0x20;
		}
		else if (tp_dev.x[0]>10 && tp_dev.x[0]<58 && tp_dev.y[0]>70 && tp_dev.y[0]<94)
		{
			speedSwitchFlag &= 0x7F;       //�����save������
			SpeedSaveData();               //��FLASHд������
		}
		else if (tp_dev.x[0]>73 && tp_dev.x[0]<121 && tp_dev.y[0]>70 && tp_dev.y[0]<94)
		{
			speedSwitchFlag |= 0x80;       //�����eidt������
		}
		/******"+"���� ******/
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
		/******"-"���� ******/
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
				modeSwitchFlag = 0x11;           /* �����Ahead������ 				 */
				OSSemPost(semDebug);             /* ����Debug�ź��������Կ�ʼ�������� */
			}

		}
		else if (tp_dev.x[0]>14 && tp_dev.x[0]<60 && tp_dev.y[0]>202 && tp_dev.y[0]<224)
		{
			if (modeSwitchFlag & 0x01)
			{
				modeSwitchFlag = 0x21;           /* �����Back������  				 */
				OSSemPost(semDebug);             /* ����Debug�ź��������Կ�ʼ�������� */
			}
		}
		else if (tp_dev.x[0]>14 && tp_dev.x[0]<60 && tp_dev.y[0]>234 && tp_dev.y[0]<256)
		{
			if (modeSwitchFlag & 0x01)
			{
				modeSwitchFlag = 0x41;            /* �����TurnL������  				  */
				OSSemPost(semDebug);              /* ����Debug�ź��������Կ�ʼ�������� */
			}
		}
		else if (tp_dev.x[0]>14 && tp_dev.x[0]<60 && tp_dev.y[0]>266 && tp_dev.y[0]<288)
		{
			if (modeSwitchFlag & 0x01)
			{
				modeSwitchFlag = 0x81;            /* �����TurnR������  				  */
				OSSemPost(semDebug);              /* ����Debug�ź��������Կ�ʼ�������� */
			}
		}
		else if (tp_dev.x[0]>180 && tp_dev.x[0]<226 && tp_dev.y[0]>170 && tp_dev.y[0]<192)
		{
			modeSwitchFlag = 0x01;   		      /* �����Debug������			 */ 
		}
		else if (tp_dev.x[0]>180 && tp_dev.x[0]<226 && tp_dev.y[0]>202 && tp_dev.y[0]<224)
		{
			modeSwitchFlag = 0x02;   		      /* �����Climb������			 */
			OSTaskResume(CLIMB_TASK_PRIO);   
			OSTaskSuspend(FIGHT_STA_DEC_TASK_PRIO);
			OSTaskSuspend(PUSH_STA_DEC_TASK_PRIO);
			startFlag = 0;
		}
		else if (tp_dev.x[0]>180 && tp_dev.x[0]<226 && tp_dev.y[0]>234 && tp_dev.y[0]<256)
		{
			modeSwitchFlag = 0x04;   		      /* �����Push������              */
			OSTaskResume(PUSH_STA_DEC_TASK_PRIO); /* ��ʼ����PushStaDecTask       */
			OSTaskResume(MOTOR_TASK_PRIO);
			OSTaskSuspend(FIGHT_STA_DEC_TASK_PRIO);
			OSTaskSuspend(CLIMB_TASK_PRIO);
		}
		else if (tp_dev.x[0]>180 && tp_dev.x[0]<226 && tp_dev.y[0]>266 && tp_dev.y[0]<288)
		{
			modeSwitchFlag = 0x08;   		      /* �����Fight������			  */
			OSTaskResume(FIGHT_STA_DEC_TASK_PRIO);/* ��ʼ����FightStateDecideTask */
			OSTaskResume(MOTOR_TASK_PRIO);
			OSTaskSuspend(PUSH_STA_DEC_TASK_PRIO);
			OSTaskSuspend(CLIMB_TASK_PRIO);
		}
		else if (tp_dev.x[0]>48 && tp_dev.x[0]<110 && tp_dev.y[0]>293)
		{
			OSTaskResume(MOTOR_TASK_PRIO);       /* �����Start������  			 */
			OSTaskResume(CLIMB_TASK_PRIO);       /* ��ʼ���е�̨����     	  	 */
			startFlag = 1;
			
		}
		else if (tp_dev.x[0]>128 && tp_dev.x[0]<178 && tp_dev.y[0]>293)
		{
			OSTaskSuspend(MOTOR_TASK_PRIO);      /* �����Stop������ 			 */
			OSTaskSuspend(FIGHT_STA_DEC_TASK_PRIO);
			OSTaskSuspend(PUSH_STA_DEC_TASK_PRIO);
			OSTaskSuspend(CLIMB_TASK_PRIO);
			
			startFlag = 0;
			climbSensorVal = 0;
			modeSwitchFlag = 0;               
			SixMotorControl(0);
			ClimbMotorControl(0);
			for (i=0; i<100; i++)			      /* ̧�𹥻���   			  */
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
/*******************************����ݸ���*******************************/
#if 0

�ʼǣ�
u8 *p_end1 = NULL;
u8 *p_end2 = NULL;
����������������������ʱ��ʧȥ���ã�����������ȡ�ַ������������ ��

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
			
		/*���������󷽵Ĺ�翪��*/
		LCD_Draw_Circle(75 , 215, 4);
		//LCD_Draw_Circle(75 , 230, 4);
		LCD_Draw_Circle(75 , 245, 4);
		/*���������Ե�ҷ��Ĺ�翪��*/
		LCD_Draw_Circle(165, 215, 4);
		//LCD_Draw_Circle(165, 230, 4);
		LCD_Draw_Circle(165, 245, 4);
		
		/***********************��̬��ʾ*************************/
		sprintf(charBuff, "x:%f", 12);//p_receiveData->anglex);
		LCD_ShowString(92, 204, 56, 16, 16, (u8*)charBuff);
		sprintf(charBuff, "y:%f", p_receiveData->angley);
		LCD_ShowString(92, 222, 56, 16, 16, (u8*)charBuff);
		sprintf(charBuff, "z:%f", p_receiveData->anglez);
		LCD_ShowString(92, 240, 56, 16, 16, (u8*)charBuff);
		sprintfת��������ʧЧ
#endif

