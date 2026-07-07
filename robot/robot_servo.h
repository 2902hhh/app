/*
 * robot_servo.h — 舵机控制模块 (SG90, 安装在车头, 超声固定其上)
 *
 * 多线程架构: 舵机在独立线程中运行, 独占 GPIO2 输出 50Hz PWM
 * 控制线程通过 g_servo_frozen 标志控制舵机冻结/扫描
 */

#ifndef ROBOT_SERVO_H
#define ROBOT_SERVO_H

#include "robot_car.h"

/* 初始化舵机引脚 (归中), 由 servo_task 内部调用 */
void servo_init(void);

/* 设置舵机角度 (单次 20ms PWM 脉冲, hi_udelay 阻塞, 不可抢占) */
void servo_set_angle(unsigned int duty_us);

/* 舵机独立线程入口 (osPriorityAboveNormal, 独占 PWM 输出) */
void servo_task(void *param);

/* 获取当前舵机位置的描述标签 (用于 OLED 显示) */
const char* servo_get_position_label(void);

#endif /* ROBOT_SERVO_H */
