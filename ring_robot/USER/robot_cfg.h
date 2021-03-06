/*************************************************************
*  Open source but not allowed to modify(All rights reserved)
*File name      : robot_cfg.h
*Auther         : 张沁
*Version        : v1.0
*Date           : 2017-11-03
*Description    : 机器人相关参数配置文件
*Function List  : 
*************************************************************/

#ifndef _ROBOT_CFG_H
#define _ROBOT_CFG_H

/** 速度 **/
#define BOUND_WARN_SPEED_4   200   /* 检测到边界反向运动的速度(边缘4个电机)						      */
#define BOUND_WARN_SPEED_2   200   /* 检测到边界反向运动的速度(中间2个电机) 						  */
/** 时间 **/
#define ERGENT_BACK_TIME      2   /* 前侧两个光电对管检测到边缘后，机器人紧急后退时间(每个单位为25ms) */
#define ERGENT_AHEAD_TIME     2   /* 前侧两个光电对管检测到边缘后，机器人紧急前进时间(每个单位为25ms) */
#define ROTATION_ERR_TIME     40  /* 自转状态下的旋转时间限定，防止死转(每个单位为25ms) 			  */
#define STATE_SWITCH_TIME     40  /* (巡逻状态下)状态自动切换时间(每个单位为25ms)       			  */
/** 角度 **/
#define ROTATION_ERR_RANGE    60  /* 旋转误差60°                        						  */
#define ANGLE_ERGENT_BACK     30  /* 紧急后退后进入自转状态的旋转角度    							  */
#define ANGLE_ERGENT_AHEAD    30  /* 紧急前进后进入自转状态的旋转角度    							  */
#define ANGLE_FL              30  /* 前侧左方出现物体后的旋转角度        							  */
#define ANGLE_FR              30  /* 前侧右方出现物体后的旋转角度         						  */
#define ANGLE_BL              30  /* 后侧左方出现物体后的旋转角度         						  */
#define ANGLE_BR              30  /* 后侧右方出现物体后的旋转角度         						  */
#define ANGLE_L               60  /* 左方出现物体后的旋转角度        								  */ 
#define ANGLE_R               60  /* 右方出现物体后的旋转角度       							      */ 
#define ANGLE_RANDOM_MIN      30  /* 随机旋转角度的最小值             							  */ 
#define ANGLE_RANDOM_MAX     100  /* 随机旋转角度的最小值             							  */ 
/* 传感器检测到的状态 */
#define SENSOR_WARN_FL   1     	  /* 边界警告：前方左侧 											  */
#define SENSOR_WARN_FR   2		  /* 边界警告：前方右侧 											  */
#define SENSOR_WARN_BL   3		  /* 边界警告：后方左侧 											  */
#define SENSOR_WARN_BR   4		  /* 边界警告：后方右侧   										  */
#define SENSOR_F         5        /* 物体出现：正前方   											  */
#define SENSOR_FL        6		  /* 物体出现：前侧左方 											  */
#define SENSOR_FR        7	      /* 物体出现：前侧右方 											  */
#define SENSOR_B         8		  /* 物体出现：正后方   											  */
#define SENSOR_BL        9        /* 物体出现：后侧左方 											  */
#define SENSOR_BR       10		  /* 物体出现：后侧右方 											  */
#define SENSOR_L        11		  /* 物体出现：正左方   											  */
#define SENSOR_R        12        /* 物体出现：正右方   											  */
#define SENSOR_NONE     13        /* 无物体出现   											  */

/* 擂台机器人运行状态 */
#define ROBOT_PATROL     1		/* 巡航模式          */
#define ROBOT_ATTACK_F   2		/* 攻击状态：正前方   */
#define ROBOT_ATTACK_B   3		/* 攻击状态：正后方   */
#define ROBOT_ROTATION_L 4		/* 自转状态：向左转   */
#define ROBOT_ROTATION_R 5		/* 自转状态：向右转   */
#define ROBOT_PUSH_F     6      /* 推木块：  正前方   */
#define ROBOT_PUSH_B     7		/* 推木块：  正后方   */
#define ROBOT_Ergent_B   8      /* 检测到边缘：后退   */
#define ROBOT_Ergent_A   9		/* 检测到边缘：前进   */

#endif
	 	   	  		 			 	    		   		 		 	 	 			 	    		   	 			 	  	 		 				 		  			 		 					 	  	  		      		  	   		      		  	 		 	      		   		 		  	 		 	      		  		  		  
