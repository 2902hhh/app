/*
 * robot_servo.c — 舵机控制模块实现 (SG90)
 *
 * SG90 舵机工作原理:
 *   - 需要 50Hz (周期20ms) 的 PWM 信号
 *   - 高电平脉宽决定角度:
 *       1100us → 左极限 (约 -20°)
 *       1500us → 居中 (0°, 朝正前方)
 *       1900us → 右极限 (约 +20°)
 *   - 每个脉冲周期 = 20ms (duty_us 高电平 + 余下低电平)
 *
 * 多线程架构:
 *   - servo_task() 在独立线程中运行 (osPriorityNormal)
 *   - 使用 hi_udelay() 忙等待产生 50Hz 软件 PWM
 *   - 每发送 2 个脉冲 (40ms) 后通过 osDelay(8) 让出 CPU
 *   - 控制线程通过 g_servo_frozen 标志控制扫描/冻结
 *   - 舵机角度 g_servo_angle 和方向 g_servo_dir 为全局共享变量
 *
 * GPIO: 舵机控制信号 = GPIO2 (输出)
 */

#include "robot_servo.h"

/*
 * 初始化舵机引脚
 * 设置 GPIO2 为输出模式, 舵机归中 (发送10个周期, 共200ms)
 */
void servo_init(void)
{
    int i;

    hi_io_set_func(SERVO_PIN, 0);
    IoTGpioSetDir(SERVO_PIN, IOT_GPIO_DIR_OUT);

    /* 发送10个脉冲将舵机稳定归中 (10 × 20ms = 200ms) */
    for (i = 0; i < 10; i++) {
        servo_set_angle(SERVO_CENTER_US);
    }
}

/*
 * 设置舵机角度 (发送单个 20ms PWM 周期)
 * 参数: duty_us - 高电平脉宽(微秒), 范围 1100~1900
 *
 * 此函数使用 hi_udelay() 忙等待, 单次调用约占用执行线程 20ms
 */
void servo_set_angle(unsigned int duty_us)
{
    IoTGpioSetDir(SERVO_PIN, IOT_GPIO_DIR_OUT);

    /* 高电平阶段: 持续 duty_us 微秒 */
    IoTGpioSetOutputVal(SERVO_PIN, IOT_GPIO_VALUE1);
    hi_udelay(duty_us);

    /* 低电平阶段: 补足 20ms 周期 */
    IoTGpioSetOutputVal(SERVO_PIN, IOT_GPIO_VALUE0);
    hi_udelay(SERVO_PERIOD_US - duty_us);
}

/*
 * 舵机独立线程 — 独占 GPIO2 输出 50Hz PWM 脉冲
 *
 * 线程优先级: osPriorityNormal (与控制线程同级)
 *
 * 执行逻辑:
 *   1. 初始化舵机 GPIO 并归中
 *   2. 循环: 更新角度 → 发2脉冲(40ms) → osDelay(8)让出CPU
 *   3. g_servo_frozen=1 时冻结角度, 不更新扫描方向
 *
 * 时序设计:
 *   - 2 个连续脉冲约占用 40ms
 *   - osDelay(8) 允许控制线程和低优先级传感器显示线程运行
 *   - 舵机 8ms 信号间隙约为周期的 16%, 略有抖动但可接受
 */
void servo_task(void *param)
{
    (void)param;

    /* 初始化舵机 (归中) */
    servo_init();

    while (1) {
        /* ---- 更新扫描角度 (无障碍时) ---- */
        if (!g_servo_frozen) {
            g_servo_angle += g_servo_dir * SERVO_SWEEP_STEP_US;

            /* 到达极限时反转扫描方向 */
            if (g_servo_angle >= SERVO_MAX_US) {
                g_servo_angle = SERVO_MAX_US;
                g_servo_dir = -1;                       /* 改为向左扫描 */
            } else if (g_servo_angle <= SERVO_MIN_US) {
                g_servo_angle = SERVO_MIN_US;
                g_servo_dir = 1;                        /* 改为向右扫描 */
            }
        }
        /* 注意: g_servo_frozen=1 时不更新角度, 舵机停在当前位置 */

        /* ---- 发送 2 个连续脉冲 (约 40ms) ---- */
        servo_set_angle(g_servo_angle);
        servo_set_angle(g_servo_angle);

        /* ---- 让出 CPU 给其他线程 (约 8ms) ---- */
        osDelay(8);
    }
}

/*
 * 获取当前舵机位置的描述标签 (用于 OLED 显示)
 *
 * 返回值:
 *   "SCAN>>" — 舵机偏右
 *   "<<SCAN" — 舵机偏左
 *   "CENTER" — 接近居中
 */
const char* servo_get_position_label(void)
{
    if (g_servo_angle >= SERVO_CENTER_US + 100) {
        return "SCAN>>";        /* 偏右, 向右扫描中 */
    } else if (g_servo_angle <= SERVO_CENTER_US - 100) {
        return "<<SCAN";        /* 偏左, 向左扫描中 */
    } else {
        return "CENTER";        /* 接近居中 */
    }
}
