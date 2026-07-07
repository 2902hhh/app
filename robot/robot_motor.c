/*
 * robot_motor.c — 电机驱动模块实现 (L9110S + Hi3861硬件PWM)
 *
 * L9110S 控制逻辑 (单路电机):
 *   IN1=H, IN2=L → 正转 (前进方向)
 *   IN1=L, IN2=H → 反转 (后退方向)
 *   IN1=L, IN2=L → 惰行 (惯性停止)
 *   IN1=H, IN2=H → 刹车 (短路制动)
 *
 * 调速方法: 对运转方向的IN引脚施加PWM, 另一引脚保持≈0%占空比
 *   前进时: IN1=占空比D, IN2≈0%  → 速度正比于D
 *   后退时: IN1≈0%, IN2=占空比D  → 速度正比于D
 *
 * PWM端口映射:
 *   GPIO0  → PWM3 (左IN1)    GPIO1  → PWM4 (左IN2)
 *   GPIO9  → PWM0 (右IN1)    GPIO10 → PWM1 (右IN2)
 */

#include "robot_motor.h"

/*
 * 初始化四路电机PWM
 * - 将GPIO复用为对应PWM功能
 * - 初始化各PWM端口
 * - 初始状态: 四路均输出≈0% (电机静止惰行)
 */
void motor_pwm_init(void)
{
    /* 全局PWM时钟源设为160MHz (对所有端口生效) */
    hi_pwm_set_clock(PWM_CLK_160M);

    /* GPIO0 → PWM3 (左电机 IN1) */
    hi_io_set_func(MOTOR_L_IN1, HI_IO_FUNC_GPIO_0_PWM3_OUT);
    hi_pwm_init(HI_PWM_PORT_PWM3);
    hi_pwm_start(HI_PWM_PORT_PWM3, SPEED_OFF, PWM_FREQ);

    /* GPIO1 → PWM4 (左电机 IN2) */
    hi_io_set_func(MOTOR_L_IN2, HI_IO_FUNC_GPIO_1_PWM4_OUT);
    hi_pwm_init(HI_PWM_PORT_PWM4);
    hi_pwm_start(HI_PWM_PORT_PWM4, SPEED_OFF, PWM_FREQ);

    /* GPIO9 → PWM0 (右电机 IN1) */
    hi_io_set_func(MOTOR_R_IN1, HI_IO_FUNC_GPIO_9_PWM0_OUT);
    hi_pwm_init(HI_PWM_PORT_PWM0);
    hi_pwm_start(HI_PWM_PORT_PWM0, SPEED_OFF, PWM_FREQ);

    /* GPIO10 → PWM1 (右电机 IN2) */
    hi_io_set_func(MOTOR_R_IN2, HI_IO_FUNC_GPIO_10_PWM1_OUT);
    hi_pwm_init(HI_PWM_PORT_PWM1);
    hi_pwm_start(HI_PWM_PORT_PWM1, SPEED_OFF, PWM_FREQ);
}

/*
 * 小车前进 — 左右两轮同时正转
 * 左电机: IN1=PWM(duty), IN2≈0%
 * 右电机: IN1=PWM(duty), IN2≈0%
 */
void car_forward(unsigned short duty)
{
    hi_pwm_start(HI_PWM_PORT_PWM3, duty, PWM_FREQ);      /* 左IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM4, SPEED_OFF, PWM_FREQ); /* 左IN2 */
    hi_pwm_start(HI_PWM_PORT_PWM0, duty, PWM_FREQ);      /* 右IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM1, SPEED_OFF, PWM_FREQ); /* 右IN2 */
}

/*
 * 小车后退 — 左右两轮同时反转
 * 左电机: IN1≈0%, IN2=PWM(duty)
 * 右电机: IN1≈0%, IN2=PWM(duty)
 */
void car_backward(unsigned short duty)
{
    hi_pwm_start(HI_PWM_PORT_PWM3, SPEED_OFF, PWM_FREQ);  /* 左IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM4, duty, PWM_FREQ);       /* 左IN2 */
    hi_pwm_start(HI_PWM_PORT_PWM0, SPEED_OFF, PWM_FREQ);  /* 右IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM1, duty, PWM_FREQ);       /* 右IN2 */
}

/*
 * 小车左转 — 左轮惰行, 右轮前进 (车身以左轮为轴心向左转)
 * 左电机: 两IN≈0% (惰行滑行)
 * 右电机: IN1=PWM(duty), IN2≈0%
 */
void car_left(unsigned short duty)
{
    hi_pwm_start(HI_PWM_PORT_PWM3, SPEED_OFF, PWM_FREQ);  /* 左IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM4, SPEED_OFF, PWM_FREQ);  /* 左IN2 (惰行) */
    hi_pwm_start(HI_PWM_PORT_PWM0, duty, PWM_FREQ);       /* 右IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM1, SPEED_OFF, PWM_FREQ);  /* 右IN2 */
}

/*
 * 小车右转 — 左轮前进, 右轮惰行 (车身以右轮为轴心向右转)
 * 左电机: IN1=PWM(duty), IN2≈0%
 * 右电机: 两IN≈0% (惰行滑行)
 */
void car_right(unsigned short duty)
{
    hi_pwm_start(HI_PWM_PORT_PWM3, duty, PWM_FREQ);       /* 左IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM4, SPEED_OFF, PWM_FREQ);  /* 左IN2 */
    hi_pwm_start(HI_PWM_PORT_PWM0, SPEED_OFF, PWM_FREQ);  /* 右IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM1, SPEED_OFF, PWM_FREQ);  /* 右IN2 (惰行) */
}

/*
 * 小车刹车 — 四路全高电平, L9110S内部短路制动
 * 四个IN引脚均输出100%占空比 = 恒高电平
 */
void car_stop(void)
{
    hi_pwm_start(HI_PWM_PORT_PWM3, SPEED_BRAKE, PWM_FREQ); /* 左IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM4, SPEED_BRAKE, PWM_FREQ); /* 左IN2 */
    hi_pwm_start(HI_PWM_PORT_PWM0, SPEED_BRAKE, PWM_FREQ); /* 右IN1 */
    hi_pwm_start(HI_PWM_PORT_PWM1, SPEED_BRAKE, PWM_FREQ); /* 右IN2 */
}
