/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : robot_cfg.h
*Auther         : ����
*Version        : v1.0
*Date           : 2017-11-03
*Description    : ��������ز��������ļ�
*Function List  : 
*************************************************************/

#ifndef _ROBOT_CFG_H
#define _ROBOT_CFG_H

/** �ٶ� **/
#define BOUND_WARN_SPEED_4   200   /* ��⵽�߽練���˶����ٶ�(��Ե4�����)						      */
#define BOUND_WARN_SPEED_2   200   /* ��⵽�߽練���˶����ٶ�(�м�2�����) 						  */
/** ʱ�� **/
#define ERGENT_BACK_TIME      2   /* ǰ���������Թܼ�⵽��Ե�󣬻����˽�������ʱ��(ÿ����λΪ25ms) */
#define ERGENT_AHEAD_TIME     2   /* ǰ���������Թܼ�⵽��Ե�󣬻����˽���ǰ��ʱ��(ÿ����λΪ25ms) */
#define ROTATION_ERR_TIME     40  /* ��ת״̬�µ���תʱ���޶�����ֹ��ת(ÿ����λΪ25ms) 			  */
#define STATE_SWITCH_TIME     40  /* (Ѳ��״̬��)״̬�Զ��л�ʱ��(ÿ����λΪ25ms)       			  */
/** �Ƕ� **/
#define ROTATION_ERR_RANGE    60  /* ��ת���60��                        						  */
#define ANGLE_ERGENT_BACK     30  /* �������˺������ת״̬����ת�Ƕ�    							  */
#define ANGLE_ERGENT_AHEAD    30  /* ����ǰ���������ת״̬����ת�Ƕ�    							  */
#define ANGLE_FL              30  /* ǰ���󷽳�����������ת�Ƕ�        							  */
#define ANGLE_FR              30  /* ǰ���ҷ�������������ת�Ƕ�         						  */
#define ANGLE_BL              30  /* ����󷽳�����������ת�Ƕ�         						  */
#define ANGLE_BR              30  /* ����ҷ�������������ת�Ƕ�         						  */
#define ANGLE_L               60  /* �󷽳�����������ת�Ƕ�        								  */ 
#define ANGLE_R               60  /* �ҷ�������������ת�Ƕ�       							      */ 
#define ANGLE_RANDOM_MIN      30  /* �����ת�Ƕȵ���Сֵ             							  */ 
#define ANGLE_RANDOM_MAX     100  /* �����ת�Ƕȵ���Сֵ             							  */ 
/* ��������⵽��״̬ */
#define SENSOR_WARN_FL   1     	  /* �߽羯�棺ǰ����� 											  */
#define SENSOR_WARN_FR   2		  /* �߽羯�棺ǰ���Ҳ� 											  */
#define SENSOR_WARN_BL   3		  /* �߽羯�棺����� 											  */
#define SENSOR_WARN_BR   4		  /* �߽羯�棺���Ҳ�   										  */
#define SENSOR_F         5        /* ������֣���ǰ��   											  */
#define SENSOR_FL        6		  /* ������֣�ǰ���� 											  */
#define SENSOR_FR        7	      /* ������֣�ǰ���ҷ� 											  */
#define SENSOR_B         8		  /* ������֣�����   											  */
#define SENSOR_BL        9        /* ������֣������ 											  */
#define SENSOR_BR       10		  /* ������֣�����ҷ� 											  */
#define SENSOR_L        11		  /* ������֣�����   											  */
#define SENSOR_R        12        /* ������֣����ҷ�   											  */
#define SENSOR_NONE     13        /* ���������   											  */

/* ��̨����������״̬ */
#define ROBOT_PATROL     1		/* Ѳ��ģʽ          */
#define ROBOT_ATTACK_F   2		/* ����״̬����ǰ��   */
#define ROBOT_ATTACK_B   3		/* ����״̬������   */
#define ROBOT_ROTATION_L 4		/* ��ת״̬������ת   */
#define ROBOT_ROTATION_R 5		/* ��ת״̬������ת   */
#define ROBOT_PUSH_F     6      /* ��ľ�飺  ��ǰ��   */
#define ROBOT_PUSH_B     7		/* ��ľ�飺  ����   */
#define ROBOT_Ergent_B   8      /* ��⵽��Ե������   */
#define ROBOT_Ergent_A   9		/* ��⵽��Ե��ǰ��   */

#endif
	 	   	  		 			 	    		   		 		 	 	 			 	    		   	 			 	  	 		 				 		  			 		 					 	  	  		      		  	   		      		  	 		 	      		   		 		  	 		 	      		  		  		  
