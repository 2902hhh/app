/*
 * robot_tracking.h — 红外循迹模块 (TCRT5000 × 2)
 *
 * 两个传感器分布在黑线两侧:
 *   - 正常循迹: 两个传感器都在白色区域 (黑线在中间) → 前进
 *   - 车身偏左: 左传感器压到黑线 → 左黑右白 → 需要右转修正
 *   - 车身偏右: 右传感器压到黑线 → 左白右黑 → 需要左转修正
 */

#ifndef ROBOT_TRACKING_H
#define ROBOT_TRACKING_H

#include "robot_car.h"

/* 初始化循迹传感器引脚 (两个均为输入) */
void tracking_init(void);

/*
 * 读取左右循迹传感器状态
 * 参数:
 *   left_is_white  - [输出] 左侧: 1=白底, 0=黑线
 *   right_is_white - [输出] 右侧: 1=白底, 0=黑线
 */
void tracking_read(int *left_is_white, int *center_is_white, int *right_is_white);
int parking_button_pressed(void);

#endif /* ROBOT_TRACKING_H */
