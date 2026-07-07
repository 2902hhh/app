/*
 * robot_car.c — 循迹小车主控制程序 (双线程架构)
 *
 * 功能说明:
 *   - 舵机安装在车头, 带动超声波传感器持续左右扫描
 *   - 两个循迹传感器分布在黑线两侧, 正常时都检测到白色 (黑线在中间)
 *   - 前方检测到障碍物 (≤15cm) 时停车, 同时舵机冻结保持对准障碍物方向
 *   - 障碍物移开后舵机恢复扫描, 小车恢复循迹
 *   - OLED 实时显示小车运行状态 (在控制线程中刷新)
 *
 * 硬件平台: Hi3861 (HiSpark Pegasus)
 *
 * 双线程架构:
 *   ServoThread   (osPriorityNormal) — 独占舵机 PWM, 每 40ms 让出 8ms
 *   ControlThread (osPriorityNormal) — 传感器 + 循迹 + 电机 + OLED
 *
 * 线程间通信: volatile 全局变量 g_servo_frozen / g_servo_angle / g_servo_dir
 *
 * 模块文件:
 *   robot_car.h         — 公共定义 (引脚、常量、共享变量)
 *   robot_motor.h/.c    — 电机驱动 (L9110S + 硬件PWM)
 *   robot_ultrasonic.h/.c — 超声波测距 (HC-SR04)
 *   robot_servo.h/.c    — 舵机控制 (SG90, 独立线程)
 *   robot_tracking.h/.c — 循迹传感器 (TCRT5000 × 2)
 *   robot_display.h/.c  — OLED 显示 (SSD1306)
 */

#include "robot_motor.h"
#include "robot_ultrasonic.h"
#include "robot_servo.h"
#include "robot_tracking.h"
#include "robot_display.h"

/* ============================================================================
 * 多线程共享全局变量定义
 * ============================================================================ */

/* 舵机状态 (舵机线程写入, 控制线程读取标签) */
volatile unsigned int g_servo_angle = SERVO_CENTER_US;  /* 当前舵机脉宽 (us) */
volatile int g_servo_dir = 1;           /* 扫描方向: 1=向右, -1=向左 */

/* 舵机冻结标志 (控制线程写入, 舵机线程读取) */
volatile int g_servo_frozen = 0;        /* 1=冻结(有障碍), 0=正常扫描 */

/* ============================================================================
 * 循迹状态机 — 根据双传感器状态决策动作
 *
 * 传感器布局 (俯视图, 车头朝下):
 *        ┌─────────┐
 *        │  小车车身  │
 *   ┌────┴─────────┴────┐
 *   │ L=左传感器 R=右传感器│  ← 车头方向
 *   └───────────────────┘
 *     白     黑线    白
 *
 * 逻辑表:
 *   | 左传感器 | 右传感器 | 车身状态        | 动作      |
 *   |----------|----------|-----------------|-----------|
 *   | 白(HIGH) | 白(HIGH) | 居中, 黑线在中间  | 直行前进  |
 *   | 黑(LOW)  | 白(HIGH) | 偏左, 左轮压线    | 右转修正  |
 *   | 白(HIGH) | 黑(LOW)  | 偏右, 右轮压线    | 左转修正  |
 *   | 黑(LOW)  | 黑(LOW)  | 十字路口/宽黑线   | 直行前进  |
 *
 * 参数:
 *   left_white  - 左传感器: 1=白底, 0=黑线 (压线)
 *   right_white - 右传感器: 1=白底, 0=黑线 (压线)
 *
 * 返回值: 当前执行的动作描述字符串
 */
static const char* line_follow_step(int left_white, int right_white)
{
    if (left_white && right_white) {
        /* 情况1: 双白 → 黑线在两个传感器之间, 直行 */
        car_forward(SPEED_FORWARD);
        return "FORWARD";

    } else if (!left_white && right_white) {
        /* 情况2: 左黑右白 → 车身偏左, 左传感器压到黑线 → 右转修正 */
        //car_right(SPEED_TURN);
        car_left(SPEED_TURN);
        return "TURN L>";

    } else if (left_white && !right_white) {
        /* 情况3: 左白右黑 → 车身偏右, 右传感器压到黑线 → 左转修正 */
        //car_left(SPEED_TURN);
        car_right(SPEED_TURN);
        return "<TURN R";

    } else {
        /* 情况4: 双黑 → 可能是十字路口或宽黑线, 继续前进 */
        car_forward(SPEED_FORWARD);
        return "FORWARD";
    }
}

/* ============================================================================
 * 控制线程 — ControlThread
 *
 * 优先级: osPriorityNormal (与舵机线程同级, 协作调度)
 * 职责: 传感器读取 + 循迹决策 + 电机控制 + 障碍消抖 + OLED 刷新
 *
 * 障碍物消抖机制:
 *   - 需要连续 OBSTACLE_DEBOUNCE_CNT 次检测到障碍才触发停车
 *   - 需要连续 OBSTACLE_CLEAR_CNT 次检测不到障碍才恢复循迹
 *   - 避免超声波单次噪声读数导致误触发
 *
 * OLED 刷新: 每 OLED_UPDATE_INTERVAL 次循环刷新一次 (减少 I2C 开销)
 * ============================================================================ */

static void control_task(void *param)
{
    (void)param;

    /* ---- 状态变量 ---- */
    int loop_count = 0;                     /* 主循环计数器 */
    int left_white = 1;                     /* 左循迹: 1=白底, 0=黑线 */
    int right_white = 1;                    /* 右循迹: 1=白底, 0=黑线 */
    float distance = 0.0f;                  /* 超声波测距值 (cm) */
    const char *action = "INIT..";          /* 当前动作描述 */
    const char *servo_label = "CENTER";     /* 舵机标签 */
    int obstacle_waiting = 0;               /* 障碍等待标志 */
    int obs_count = 0;                      /* 障碍检测消抖计数 */
    int clear_count = 0;                    /* 障碍清除消抖计数 */

    printf("\r\n[Control] ====== XunJi Car Starting (2-Thread) ======\r\n");

    /* ---- 硬件初始化 ---- */

    /* 1. 初始化电机 PWM */
    motor_pwm_init();
    printf("[Control] Motor PWM init OK\r\n");

    /* 2. 初始化超声波传感器 */
    ultrasonic_init();
    printf("[Control] Ultrasonic init OK\r\n");

    /* 3. 初始化循迹传感器 */
    tracking_init();
    printf("[Control] Tracking sensors init OK\r\n");

    /* 4. 初始化 OLED 显示 */
    display_init();
    printf("[Control] OLED display init OK\r\n");

    /* 5. 禁用看门狗 (超声波测距中有 spin-wait) */
    IoTWatchDogDisable();
    printf("[Control] Watchdog disabled\r\n");

    /* 等待舵机线程完成初始化 (servo_init 约 200ms) */
    osDelay(300);

    /* ---- 启动画面 ---- */
    display_update(0.0f, 1, 1, "INIT..", "CENTER");
    osDelay(300);

    printf("[Control] Main loop started (period=%dms, threshold=%.0fcm, "
           "debounce=%d/%d)\r\n",
           LINE_FOLLOW_DELAY_MS, OBSTACLE_THRESHOLD_CM,
           OBSTACLE_DEBOUNCE_CNT, OBSTACLE_CLEAR_CNT);

    /* ========================================================================
     * 控制主循环
     *
     * 周期: 约 30ms + 舵机让出的 8ms 窗口
     * 每周期:
     *   1. 读取循迹传感器
     *   2. 读取超声波距离
     *   3. 障碍物消抖判断 → 设置 g_servo_frozen
     *   4. 循迹决策 → 电机控制
     *   5. 更新舵机标签
     *   6. 刷新 OLED (每 N 次循环)
     * ======================================================================== */

    while (1) {
        /* --- 1. 读取循迹传感器 --- */
        tracking_read(&left_white, &right_white);

        /* --- 2. 读取超声波距离 --- */
        distance = ultrasonic_get_distance();

        /* --- 3. 障碍物消抖检测 --- */
        if (distance > 0.0f && distance < OBSTACLE_THRESHOLD_CM) {
            /* 本次检测到障碍物 */
            clear_count = 0;                        /* 重置清除计数 */

            if (!obstacle_waiting) {
                obs_count++;                        /* 累加检测计数 */
                if (obs_count >= OBSTACLE_DEBOUNCE_CNT) {
                    /* 连续多次检测到 → 确认障碍物, 触发停车 */
                    car_stop();
                    obstacle_waiting = 1;
                    g_servo_frozen = 1;             /* 冻结舵机 */
                    obs_count = 0;
                    printf("[Control] !! OBSTACLE! dist:%.1f cm\r\n", distance);
                }
            }
            action = "OBSTACLE!";
        } else {
            /* 本次未检测到障碍物 (距离正常或超时) */
            obs_count = 0;                          /* 重置检测计数 */

            if (obstacle_waiting) {
                clear_count++;                      /* 累加清除计数 */
                if (clear_count >= OBSTACLE_CLEAR_CNT) {
                    /* 连续多次未检测到 → 确认障碍已清除, 恢复循迹 */
                    obstacle_waiting = 0;
                    g_servo_frozen = 0;             /* 恢复舵机扫描 */
                    clear_count = 0;
                    printf("[Control] Obstacle cleared, resuming\r\n");
                }
            }
        }

        /* --- 4. 循迹控制 (无障碍时执行) --- */
        if (!obstacle_waiting) {
            action = line_follow_step(left_white, right_white);
        }

        /* --- 5. 更新舵机标签 (供 OLED 显示) --- */
        if (obstacle_waiting) {
            servo_label = "HOLD!";
        } else {
            servo_label = servo_get_position_label();
        }

        /* --- 6. OLED 刷新 (节流: 每 N 次循环) --- */
        if (loop_count % OLED_UPDATE_INTERVAL == 0) {
            display_update(distance, left_white, right_white,
                          action, servo_label);
        }

        /* --- 主循环延迟 --- */
        osDelay(LINE_FOLLOW_DELAY_MS);
        loop_count++;
    }
}

/* ============================================================================
 * 业务入口 — RobotCarDemo
 *
 * 创建 2 个线程 (双线程架构):
 *   1. ServoThread   (osPriorityNormal) — 舵机 PWM, 40ms 脉冲 + 8ms 让出
 *   2. ControlThread (osPriorityNormal) — 传感器 + 循迹 + 电机 + OLED
 *
 * 两个线程同优先级, 协作调度:
 *   - 舵机线程 hi_udelay 忙等时不可抢占 (保证 PWM 时序)
 *   - 舵机线程 osDelay(8) 时控制线程获得 CPU
 *   - 控制线程 osDelay(30) 时舵机线程获得 CPU
 * ============================================================================ */

static void RobotCarDemo(void)
{
    osThreadAttr_t attr;

    printf("[RobotCarDemo] Creating 2 threads...\r\n");

    /* ---- 舵机线程 ---- */
    attr.name = "ServoThread";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = SERVO_STACK_SIZE;
    attr.priority = osPriorityNormal;
    if (osThreadNew(servo_task, NULL, &attr) == NULL) {
        printf("[RobotCarDemo] ERROR: Failed to create ServoThread!\r\n");
        return;
    }
    printf("[RobotCarDemo] ServoThread created (stack=%d)\r\n", SERVO_STACK_SIZE);

    /* ---- 控制线程 (含 OLED 刷新) ---- */
    attr.name = "ControlThread";
    attr.stack_size = CONTROL_STACK_SIZE;
    attr.priority = osPriorityNormal;
    if (osThreadNew(control_task, NULL, &attr) == NULL) {
        printf("[RobotCarDemo] ERROR: Failed to create ControlThread!\r\n");
        return;
    }
    printf("[RobotCarDemo] ControlThread created (stack=%d)\r\n", CONTROL_STACK_SIZE);

    printf("[RobotCarDemo] All 2 threads started OK\r\n");
}

/* 系统启动后自动执行 RobotCarDemo */
APP_FEATURE_INIT(RobotCarDemo);
