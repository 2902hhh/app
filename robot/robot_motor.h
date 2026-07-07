/*
 * robot_motor.h — 电机驱动模块 (L9110S + 硬件PWM调速)
 */

#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include "robot_car.h"

/* 初始化四路电机PWM端口 */
void motor_pwm_init(void);

/* 前进: 左右两轮同时正转, duty为PWM占空比计数值 */
void car_forward(unsigned short duty);

/* 后退: 左右两轮同时反转 */
void car_backward(unsigned short duty);

/* 左转: 左轮惰行, 右轮前进 (以左轮为轴心) */
void car_left(unsigned short duty);

/* 右转: 左轮前进, 右轮惰行 (以右轮为轴心) */
void car_right(unsigned short duty);

/* 刹车: 四路全高电平, 电机短路制动 */
void car_stop(void);

#endif /* ROBOT_MOTOR_H */
