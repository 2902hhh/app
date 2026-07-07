/*
 * robot_ultrasonic.h — 超声波测距模块 (HC-SR04)
 */

#ifndef ROBOT_ULTRASONIC_H
#define ROBOT_ULTRASONIC_H

#include "robot_car.h"

/* 初始化超声波传感器GPIO (TRIG=输出, ECHO=输入) */
void ultrasonic_init(void);

/* 获取超声波测距值, 返回距离(cm), 超时/无回波返回 -1.0 */
float ultrasonic_get_distance(void);

#endif /* ROBOT_ULTRASONIC_H */
