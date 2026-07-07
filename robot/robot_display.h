/*
 * robot_display.h — OLED 显示模块 (SSD1306, I2C0, 128×64)
 *
 * 由控制线程在循环中周期性调用, 不在独立线程中运行
 */

#ifndef ROBOT_DISPLAY_H
#define ROBOT_DISPLAY_H

#include "robot_car.h"

/* 初始化 OLED (GPIO + I2C + SSD1306) */
void display_init(void);

/*
 * 刷新 OLED 显示内容 (单次全屏刷新, I2C 阻塞约 20-50ms)
 * 参数:
 *   distance     - 超声波测距值(cm), -1表示超时
 *   left_white   - 左循迹传感器: 1=白, 0=黑
 *   right_white  - 右循迹传感器: 1=白, 0=黑
 *   action       - 当前动作描述
 *   servo_label  - 舵机扫描方向标签
 */
void display_update(float distance, int left_white, int right_white,
                    const char *action, const char *servo_label);

#endif /* ROBOT_DISPLAY_H */
