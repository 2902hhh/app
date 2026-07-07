/*
 * robot_ultrasonic.c — 超声波测距模块实现 (HC-SR04)
 *
 * 工作原理:
 *   1. MCU 通过 TRIG 引脚发送 ≥10us 的高电平触发脉冲
 *   2. HC-SR04 自动发送 8 个 40kHz 的超声波脉冲
 *   3. 检测到回波后, ECHO 引脚输出高电平
 *   4. 高电平持续时间 = 超声波从发射到接收的往返时间
 *   5. 距离 = (高电平时间 × 0.034 cm/us) / 2
 *
 * 改进: 加入 38ms 超时保护 (对应约 2m 最大量程), 防止传感器故障时死循环
 *       看门狗在初始化时禁用一次, 不在每次测距时重复调用
 *
 * GPIO: TRIG=GPIO7(输出), ECHO=GPIO8(输入)
 */

#include "robot_ultrasonic.h"

/*
 * 初始化超声波传感器引脚
 * TRIG: 输出方向, 初始低电平
 * ECHO: 输入方向
 */
void ultrasonic_init(void)
{
    /* TRIG 引脚 → 输出, 低电平 */
    hi_io_set_func(HCSR04_TRIG, 0);
    IoTGpioSetDir(HCSR04_TRIG, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(HCSR04_TRIG, IOT_GPIO_VALUE0);

    /* ECHO 引脚 → 输入 */
    hi_io_set_func(HCSR04_ECHO, 0);
    IoTGpioSetDir(HCSR04_ECHO, IOT_GPIO_DIR_IN);
}

/*
 * 获取超声波测距值
 * 返回值: 距离 (厘米), 超时或超出量程返回 -1.0
 *
 * 注: 此函数包含 spin-wait 循环, 最多阻塞约 38ms
 */
float ultrasonic_get_distance(void)
{
    static unsigned long start_time = 0, echo_time = 0;
    float distance = 0.0f;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    unsigned long timeout_start;

    /* 发送 ≥10us 的触发脉冲 */
    IoTGpioSetOutputVal(HCSR04_TRIG, IOT_GPIO_VALUE1);
    hi_udelay(20);                                  /* 20us 高电平 */
    IoTGpioSetOutputVal(HCSR04_TRIG, IOT_GPIO_VALUE0);

    /* 等待 ECHO 回响信号, 带超时保护 */
    timeout_start = hi_get_us();
    while (1) {
        /* 超时检查: 约 2 米对应 38ms */
        if ((hi_get_us() - timeout_start) > HCSR04_TIMEOUT_US) {
            return -1.0f;                           /* 超时, 返回 -1 */
        }

        IoTGpioGetInputVal(HCSR04_ECHO, &value);

        /* 上升沿: 记录脉冲开始时间 */
        if (value == IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }

        /* 下降沿: 计算高电平持续时间, 结束测量 */
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            echo_time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }
    }

    /* 距离 = 时间 × 声速 / 2(往返) */
    distance = (float)echo_time * 0.034f / 2.0f;
    return distance;
}
