#ifndef __motor_H
#define __motor_H

#include "main.h"

#define PWM_MAX 3600 //根据TIM4的ARR值设定

// 纯P调参起点：
// 1. 先用较保守的Kp把系统调到不过冲或轻微过冲
// 2. Ki/Kd先关闭，待P稳定后再逐步加入
#define Kp_L 50.0f //左电机P项
#define Ki_L 2.0f  //左电机I项
#define Kd_L 1.0f  //左电机D项

#define Kp_R 50.0f //右电机P项
#define Ki_R 2.0f  //右电机I项
#define Kd_R 1.0f  //右电机D项

// 电机控制相关函数
void Motor_Init(void);
void Set_Pwm(int motor_left, int motor_right);

// PID相关函数
int PID_Calc_Left(float target_speed, float current_speed);
int PID_Calc_Right(float target_speed, float current_speed);
void PID_Reset_Left(void);//重置左电机PID积分
void PID_Reset_Right(void);//重置右电机PID积分

#endif
