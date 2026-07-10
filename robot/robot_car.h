/*
 * robot_car.h — 循迹小车公共头文件
 *
 * 包含: 引脚定义、系统常量、公共头文件引用
 * 硬件平台: Hi3861 (HiSpark Pegasus)
 *
 * 硬件连接一览:
 *   电机(L9110S):   GPIO0=左IN1(PWM3), GPIO1=左IN2(PWM4)
 *                   GPIO9=右IN1(PWM0), GPIO10=右IN2(PWM1)
 *   超声波(HC-SR04): GPIO7=TRIG(输出),  GPIO8=ECHO(输入)
 *   循迹(TCRT5000):  GPIO11=左传感器(输入), GPIO3=中间传感器(输入), GPIO12=右传感器(输入)
 *   停车解除按钮:     GPIO5=按钮输入(内部上拉, 按下为低电平)
 *   舵机(SG90):      GPIO2=控制信号(输出, 软件PWM)
 *   OLED(SSD1306):   GPIO13=I2C0_SDA, GPIO14=I2C0_SCL
 */

#ifndef ROBOT_CAR_H
#define ROBOT_CAR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_i2c.h"
#include "iot_errno.h"
#include "iot_watchdog.h"
#include "hi_io.h"
#include "hi_time.h"
#include "hi_pwm.h"

#include "ssd1306.h"

/* ============================================================================
 * 引脚定义
 * ============================================================================ */

/* ---- 电机驱动 L9110S ---- */
#define MOTOR_L_IN1     0       /* 左电机 IN1 (PWM3端口) */
#define MOTOR_L_IN2     1       /* 左电机 IN2 (PWM4端口) */
#define MOTOR_R_IN1     9       /* 右电机 IN1 (PWM0端口) */
#define MOTOR_R_IN2     10      /* 右电机 IN2 (PWM1端口) */

/* ---- 超声波测距 HC-SR04 ---- */
#define HCSR04_TRIG     7       /* 触发信号 (GPIO输出) */
#define HCSR04_ECHO     8       /* 回响信号 (GPIO输入) */

/* ---- 红外循迹 TCRT5000 (左/中/右三路) ---- */
#define TCRT5000_L      11      /* 左侧传感器 (输入) */
#define TCRT5000_C      3       /* 中间传感器 (输入) */
#define TCRT5000_R      12      /* 右侧传感器 (输入) */
#define PARK_RELEASE_BUTTON 5   /* 停车解除按钮，内部上拉，按下为低电平 */

/* ---- 舵机 SG90 (安装在车头, 超声固定其上) ---- */
#define SERVO_PIN       2       /* 舵机控制信号 (GPIO输出) */

/* ---- OLED SSD1306 I2C ---- */
#define OLED_I2C_BAUDRATE   (400 * 1000)  /* I2C0 快速模式 400kHz */

/* ============================================================================
 * PWM 电机调速参数
 *
 * Hi3861 PWM 端口映射:
 *   GPIO0  → HI_PWM_PORT_PWM3  (左IN1)
 *   GPIO1  → HI_PWM_PORT_PWM4  (左IN2)
 *   GPIO9  → HI_PWM_PORT_PWM0  (右IN1)
 *   GPIO10 → HI_PWM_PORT_PWM1  (右IN2)
 *
 * PWM 频率: f = 160MHz / PWM_FREQ ≈ 26.7kHz (>20kHz, 人耳不可闻)
 * 占空比  = duty / PWM_FREQ
 * duty范围: [1, 65535], duty >= PWM_FREQ 时输出恒高(100%)
 * ============================================================================ */

#define PWM_FREQ            6000    /* PWM分频值 */
#define SPEED_BRAKE         6000    /* 100%占空比 → 刹车(恒高) */
#define SPEED_OFF           1       /* ≈0%占空比 → 惰行(恒低) */
#define SPEED_FORWARD       3600    /* 前进占空比 ~33% (测试低速) */
#define SPEED_TURN          3300    /* 转弯占空比 ~23% (避免过冲) */

/* ============================================================================
 * 系统常量
 * ============================================================================ */

#define OBSTACLE_THRESHOLD_CM   10.0f   /* 障碍物判定距离 (厘米) */
#define HCSR04_TIMEOUT_US       38000UL /* 超声波超时 (微秒), ~2m量程 */
#define LINE_LOST_SEEK_US       6000000UL /* 三路全白后的短时寻线时间 */
#define LINE_FOLLOW_DELAY_MS    1      /* 主循环周期 (毫秒) */
#define PARK_BLACK_CONFIRM_CNT  8     /* 三路连续检测到黑色后停车的循环次数 */
#define SIDE_BLACK_PAIR_WINDOW_CNT 2   /* 左右边先后压黑的最大循环窗口 */
#define SPECIAL_RIGHT_TURN_CNT  5      /* 左右边压黑特殊右转的循环次数 */
#define BUTTON_DEBOUNCE_CNT      3     /* 按钮连续低电平确认次数 */
#define OLED_UPDATE_INTERVAL    5      /* OLED每N次循环刷新一次 */
#define SERVO_PERIOD_US         20000   /* 舵机PWM周期 20ms */
#define SERVO_SWEEP_STEP_US     50      /* 舵机每次扫描步进 (微秒) */
#define SERVO_MIN_US            1100     /* 舵机最小脉宽 (-90°) */
#define SERVO_MAX_US            1900    /* 舵机最大脉宽 (+90°) */
#define SERVO_CENTER_US         1500    /* 舵机居中脉宽 (0°) */

/* ============================================================================
 * 线程栈大小
 * ============================================================================ */

#define SERVO_STACK_SIZE    2048    /* 舵机线程栈 (仅需少量变量) */
#define CONTROL_STACK_SIZE  6144    /* 控制线程栈 (含超声波+OLED I2C) */

/* ============================================================================
 * 障碍物消抖参数
 * ============================================================================ */

#define OBSTACLE_DEBOUNCE_CNT   2    /* 连续N次检测到障碍物才触发停车 */
#define OBSTACLE_CLEAR_CNT      3    /* 连续N次检测不到障碍物才恢复 */

/* ============================================================================
 * 多线程共享状态 (volatile 全局变量)
 *
 * 双线程架构: ServoThread + ControlThread(含OLED)
 * 线程间通过 volatile 变量通信, 单核 Cortex-M 上对齐的 32-bit 读写是原子的
 * ============================================================================ */

/* 舵机状态 (舵机线程写入, 控制线程读取标签) */
extern volatile unsigned int g_servo_angle;     /* 当前舵机脉宽 (us) */
extern volatile int g_servo_dir;                /* 扫描方向: 1=右, -1=左 */

/* 舵机冻结标志 (控制线程写入, 舵机线程读取) */
extern volatile int g_servo_frozen;             /* 1=冻结舵机(有障碍) */

#endif /* ROBOT_CAR_H */
