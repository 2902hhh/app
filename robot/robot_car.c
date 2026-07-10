/*
 * robot_car.c — 三路循迹小车主控制程序 (三线程架构)
 *
 * 功能说明:
 *   - 舵机安装在车头, 带动超声波传感器持续左右扫描
 *   - 左/中/右三个循迹传感器检测黑线, 中间传感器用于判断车辆是否居中
 *   - 前方检测到障碍物 (<10cm) 时停车, 同时舵机冻结保持对准障碍物方向
 *   - 障碍物移开后舵机恢复扫描, 小车恢复循迹
 *   - OLED 在低优先级线程中显示测距、循迹和车辆动作
 *
 * 硬件平台: Hi3861 (HiSpark Pegasus)
 *
 * 三线程架构:
 *   ServoThread        (osPriorityNormal)     — 产生舵机软件 PWM
 *   ControlThread      (osPriorityNormal)     — 循迹、避障状态判断和电机控制
 *   SensorDisplayThread(osPriorityBelowNormal)— 超声波测距和 OLED 刷新
 *
 * 线程间通信: volatile 全局变量 g_servo_frozen / g_servo_angle / g_servo_dir
 *
 * 模块文件:
 *   robot_car.h         — 公共定义 (引脚、常量、共享变量)
 *   robot_motor.h/.c    — 电机驱动 (L9110S + 硬件PWM)
 *   robot_ultrasonic.h/.c — 超声波测距 (HC-SR04)
 *   robot_servo.h/.c    — 舵机控制 (SG90, 独立线程)
 *   robot_tracking.h/.c — 循迹传感器 (TCRT5000 × 3) 与停车解除按钮
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

/* 测距线程写入, 控制线程读取；样本序号确保每次测距只参与一次消抖。 */
static volatile float g_distance_cm = -1.0f;
static volatile unsigned int g_distance_sample_seq = 0;

/* 控制线程写入, 显示线程读取；OLED允许显示短暂的非同步状态快照。 */
static volatile int g_display_left_white = 1;
static volatile int g_display_center_white = 1;
static volatile int g_display_right_white = 1;
static const char * volatile g_display_action = "INIT..";
static const char * volatile g_display_servo_label = "CENTER";

/* ============================================================================
 * 循迹状态机 — 根据左/中/右三个传感器状态决策动作
 *
 * 传感器布局 (俯视图, 车头朝下):
 *        ┌─────────┐
 *        │  小车车身  │
 *   ┌────┴─────────┴────┐
 *   │ L=左  C=中  R=右传感器│  ← 车头方向
 *   └───────────────────┘
 *       白   黑线   白
 *
 * 主要状态 (B=黑线, W=白底):
 *   W/B/W: 居中前进
 *   B/X/W: 执行实车校准后的左修正 (X表示任意状态)
 *   W/X/B: 执行实车校准后的右修正
 *   W/W/W: 按最近修正方向短时寻线, 超时停车
 *   B/B/B: 由控制线程的停车计数逻辑优先处理
 *   B/W/B: 由特殊右转逻辑优先处理
 *
 * 参数:
 *   left_white  - 左传感器: 1=白底, 0=黑线 (压线)
 *   center_white- 中传感器: 1=白底, 0=黑线 (压线)
 *   right_white - 右传感器: 1=白底, 0=黑线 (压线)
 *
 * 返回值: 当前执行的动作描述字符串
 */
static const char* line_follow_step(int left_white, int center_white, int right_white,
                                    int *last_turn_dir, unsigned long *lost_start_us)
{
    unsigned long now = hi_get_us();

    if (left_white && !center_white && right_white) {
        car_forward(SPEED_FORWARD);
        *lost_start_us = 0;
        return "FORWARD";

    } else if (!left_white && right_white) {
        *last_turn_dir = -1;
        *lost_start_us = 0;
        car_left(SPEED_TURN);
        return "TURN L>";

    } else if (left_white && !right_white) {
        *last_turn_dir = 1;
        *lost_start_us = 0;
        car_right(SPEED_TURN);
        return "<TURN R";

    } else if (!left_white && !center_white && !right_white) {
        *lost_start_us = 0;
        car_forward(SPEED_FORWARD);
        return "FORWARD";

    } else if (!left_white && center_white && !right_white) {
        *last_turn_dir = 1;
        *lost_start_us = 0;
        car_right(SPEED_TURN);
        return "<TURN R";

    } else if (left_white && center_white && right_white) {
        if (*lost_start_us == 0) {
            *lost_start_us = now;
        }

        if ((now - *lost_start_us) >= LINE_LOST_SEEK_US) {
            car_stop();
            return "LOST!";
        }

        if (*last_turn_dir < 0) {
            car_left(SPEED_TURN);
            return "SEEK L";
        } else if (*last_turn_dir > 0) {
            car_right(SPEED_TURN);
            return "SEEK R";
        } else {
            car_forward(SPEED_FORWARD);
            return "SEEK..";
        }

    } else {
        *lost_start_us = 0;
        car_forward(SPEED_FORWARD);
        return "FORWARD";
    }
}

/* ============================================================================
 * 控制线程 — ControlThread
 *
 * 优先级: osPriorityNormal
 * 职责: 循迹与按钮读取、障碍状态判断、循迹决策和电机控制
 *
 * 障碍物消抖机制:
 *   - 只在 SensorDisplayThread 发布新测距样本时更新计数
 *   - 连续 OBSTACLE_DEBOUNCE_CNT 个障碍样本才触发停车
 *   - 连续 OBSTACLE_CLEAR_CNT 个清除样本才恢复循迹
 *   - 避免超声波单次噪声读数导致误触发
 * ============================================================================ */

static void control_task(void *param)
{
    (void)param;

    /* ---- 状态变量 ---- */
    int left_white = 1;                     /* 左循迹: 1=白底, 0=黑线 */
    int center_white = 1;                   /* 中间循迹: 1=白底, 0=黑线 */
    int right_white = 1;                    /* 右循迹: 1=白底, 0=黑线 */
    float distance = -1.0f;                 /* 最近一次超声波测距值 (cm) */
    unsigned int last_distance_sample_seq = 0; /* 已处理的测距样本序号 */
    const char *action = "INIT..";          /* 当前动作描述 */
    const char *servo_label = "CENTER";     /* 舵机标签 */
    int obstacle_waiting = 0;               /* 障碍等待标志 */
    int obs_count = 0;                      /* 障碍检测消抖计数 */
    int clear_count = 0;                    /* 障碍清除消抖计数 */
    int last_turn_dir = 0;                  /* 最近一次修正方向: -1=左修正, 1=右修正 */
    unsigned long lost_start_us = 0;        /* 三路全白开始时间 */
    int all_black_count = 0;                /* 三路连续检测到黑色的循环次数 */
    int parked = 0;                         /* 停车标志，触发后保持停车 */
    int button_press_count = 0;              /* 停车解除按钮消抖计数 */
    int parking_rearm_wait = 0;              /* 离开全黑区域后才允许再次停车 */
    int left_black_age = SIDE_BLACK_PAIR_WINDOW_CNT + 1;
    int right_black_age = SIDE_BLACK_PAIR_WINDOW_CNT + 1;
    int special_right_turn_count = 0;

    printf("\r\n[Control] ====== XunJi Car Starting (3-Thread) ======\r\n");

    /* ---- 硬件初始化 ---- */

    /* 1. 初始化电机 PWM */
    motor_pwm_init();
    printf("[Control] Motor PWM init OK\r\n");

    /* 2. 初始化循迹传感器和停车解除按钮 */
    tracking_init();
    printf("[Control] Tracking sensors init OK\r\n");

    /* 3. 超声波测距存在最长 38ms 的轮询等待, 因此全局禁用看门狗 */
    IoTWatchDogDisable();
    printf("[Control] Watchdog disabled\r\n");

    /* 等待舵机线程完成初始化 (servo_init 约 200ms) */
    osDelay(300);

    printf("[Control] Main loop started (period=%dms, threshold=%.0fcm, "
           "debounce=%d/%d)\r\n",
           LINE_FOLLOW_DELAY_MS, OBSTACLE_THRESHOLD_CM,
           OBSTACLE_DEBOUNCE_CNT, OBSTACLE_CLEAR_CNT);

    /* ========================================================================
     * 控制主循环
     *
     * ControlThread 不执行超声波轮询或 OLED I2C 传输, 因而可以保持高频循迹。
     * 每周期:
     *   1. 读取循迹传感器
     *   2. 新测距样本到达时更新障碍物消抖状态
     *   3. 停车与循迹决策 → 电机控制
     *   4. 发布 OLED 状态快照
     * ======================================================================== */

    while (1) {
        /* --- 1. 读取循迹传感器 --- */
        tracking_read(&left_white, &center_white, &right_white);

        if (parked && parking_button_pressed()) {
            button_press_count++;
            if (button_press_count >= BUTTON_DEBOUNCE_CNT) {
                parked = 0;
                all_black_count = 0;
                button_press_count = 0;
                parking_rearm_wait = 1;
                last_turn_dir = 0;
                lost_start_us = 0;
            }
        } else {
            button_press_count = 0;
        }

        /* --- 2. 仅处理一次新发布的超声波样本 --- */
        {
            unsigned int sample_seq = g_distance_sample_seq;

            if (sample_seq != last_distance_sample_seq) {
                distance = g_distance_cm;
                last_distance_sample_seq = sample_seq;

                if (distance > 0.0f && distance < OBSTACLE_THRESHOLD_CM) {
                    clear_count = 0;

                    if (!obstacle_waiting) {
                        obs_count++;
                        if (obs_count >= OBSTACLE_DEBOUNCE_CNT) {
                            car_stop();
                            obstacle_waiting = 1;
                            g_servo_frozen = 1;
                            last_turn_dir = 0;
                            lost_start_us = 0;
                            all_black_count = 0;
                            obs_count = 0;
                            printf("[Control] !! OBSTACLE! dist:%.1f cm\r\n", distance);
                        }
                    }
                } else {
                    obs_count = 0;

                    if (obstacle_waiting) {
                        clear_count++;
                        if (clear_count >= OBSTACLE_CLEAR_CNT) {
                            obstacle_waiting = 0;
                            g_servo_frozen = 0;
                            last_turn_dir = 0;
                            lost_start_us = 0;
                            clear_count = 0;
                            printf("[Control] Obstacle cleared, resuming\r\n");
                        }
                    }
                }
            }
        }

        /* --- 3. 三路黑色停车检测与循迹控制 --- */
        if (parking_rearm_wait) {
            all_black_count = 0;
            if (left_white || center_white || right_white) {
                parking_rearm_wait = 0;
            }
        } else if (!obstacle_waiting && !parked) {
            if (!left_white && !center_white && !right_white) {
                all_black_count++;
                if (all_black_count >= PARK_BLACK_CONFIRM_CNT) {
                    parked = 1;
                    car_stop();
                    action = "PARKED";
                }
            } else {
                all_black_count = 0;
            }
        } else if (obstacle_waiting) {
            all_black_count = 0;
        }

        if (obstacle_waiting || parked) {
            left_black_age = SIDE_BLACK_PAIR_WINDOW_CNT + 1;
            right_black_age = SIDE_BLACK_PAIR_WINDOW_CNT + 1;
            special_right_turn_count = 0;
        } else {
            int all_black = (!left_white && !center_white && !right_white);
            int both_side_black_center_white = (!left_white && center_white && !right_white);

            if (!left_white) {
                left_black_age = 0;
            } else if (left_black_age <= SIDE_BLACK_PAIR_WINDOW_CNT) {
                left_black_age++;
            }

            if (!right_white) {
                right_black_age = 0;
            } else if (right_black_age <= SIDE_BLACK_PAIR_WINDOW_CNT) {
                right_black_age++;
            }

            if (!all_black &&
                (both_side_black_center_white ||
                 (left_black_age <= SIDE_BLACK_PAIR_WINDOW_CNT &&
                  right_black_age <= SIDE_BLACK_PAIR_WINDOW_CNT))) {
                special_right_turn_count = SPECIAL_RIGHT_TURN_CNT;
            }
        }

        if (obstacle_waiting) {
            action = "OBSTACLE!";
        } else if (parked) {
            car_stop();
            action = "PARKED";
        } else if (special_right_turn_count > 0) {
            special_right_turn_count--;
            last_turn_dir = 1;
            lost_start_us = 0;
            car_right(SPEED_TURN);
            action = "<TURN R";
        } else {
            action = line_follow_step(left_white, center_white, right_white,
                                      &last_turn_dir, &lost_start_us);
        }

        /* --- 4. 发布 OLED 使用的最新状态快照 --- */
        if (obstacle_waiting) {
            servo_label = "HOLD!";
        } else {
            servo_label = servo_get_position_label();
        }

        g_display_left_white = left_white;
        g_display_center_white = center_white;
        g_display_right_white = right_white;
        g_display_action = action;
        g_display_servo_label = servo_label;

        /* --- 主循环延迟 --- */
        osDelay(LINE_FOLLOW_DELAY_MS);
    }
}

/* ============================================================================
 * 传感器与显示线程 — SensorDisplayThread
 *
 * 优先级: osPriorityBelowNormal
 * 职责: 每 100ms 测距一次, 每 200ms 刷新一次 OLED。
 * 超声波轮询和 OLED I2C 传输均可能阻塞, 低优先级可让控制与舵机线程优先运行。
 * ============================================================================ */

static void sensor_display_task(void *param)
{
    unsigned long last_ultrasonic_us = 0;
    unsigned long last_display_us = 0;
    unsigned long now;
    float distance;
    int left_white;
    int center_white;
    int right_white;
    const char *action;
    const char *servo_label;

    (void)param;

    ultrasonic_init();
    printf("[SensorDisplay] Ultrasonic init OK\r\n");

    display_init();
    printf("[SensorDisplay] OLED display init OK\r\n");
    display_update(-1.0f, 1, 1, 1, "INIT..", "CENTER");

    while (1) {
        now = hi_get_us();
        if (last_ultrasonic_us == 0 ||
            (now - last_ultrasonic_us) >= ULTRASONIC_UPDATE_US) {
            last_ultrasonic_us = now;
            distance = ultrasonic_get_distance();
            g_distance_cm = distance;
            g_distance_sample_seq++;
        }

        now = hi_get_us();
        if (last_display_us == 0 || (now - last_display_us) >= OLED_UPDATE_US) {
            last_display_us = now;

            distance = g_distance_cm;
            left_white = g_display_left_white;
            center_white = g_display_center_white;
            right_white = g_display_right_white;
            action = g_display_action;
            servo_label = g_display_servo_label;

            display_update(distance, left_white, center_white, right_white,
                           action, servo_label);
        }

        osDelay(SENSOR_DISPLAY_DELAY_MS);
    }
}

/* ============================================================================
 * 业务入口 — RobotCarDemo
 *
 * 创建三个线程:
 *   1. ServoThread         (osPriorityNormal)      — 舵机软件 PWM
 *   2. SensorDisplayThread (osPriorityBelowNormal) — 超声波测距与 OLED
 *   3. ControlThread       (osPriorityNormal)      — 循迹决策与电机控制
 * ============================================================================ */

static void RobotCarDemo(void)
{
    osThreadAttr_t attr;

    printf("[RobotCarDemo] Creating 3 threads...\r\n");

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

    /* ---- 超声波与 OLED 线程 ---- */
    attr.name = "SensorDisplayThread";
    attr.stack_size = SENSOR_DISPLAY_STACK_SIZE;
    attr.priority = osPriorityBelowNormal;
    if (osThreadNew(sensor_display_task, NULL, &attr) == NULL) {
        printf("[RobotCarDemo] ERROR: Failed to create SensorDisplayThread!\r\n");
        return;
    }
    printf("[RobotCarDemo] SensorDisplayThread created (stack=%d)\r\n",
           SENSOR_DISPLAY_STACK_SIZE);

    /* ---- 高频控制线程 ---- */
    attr.name = "ControlThread";
    attr.stack_size = CONTROL_STACK_SIZE;
    attr.priority = osPriorityNormal;
    if (osThreadNew(control_task, NULL, &attr) == NULL) {
        printf("[RobotCarDemo] ERROR: Failed to create ControlThread!\r\n");
        return;
    }
    printf("[RobotCarDemo] ControlThread created (stack=%d)\r\n", CONTROL_STACK_SIZE);

    printf("[RobotCarDemo] All 3 threads started OK\r\n");
}

/* 系统启动后自动执行 RobotCarDemo */
APP_FEATURE_INIT(RobotCarDemo);
