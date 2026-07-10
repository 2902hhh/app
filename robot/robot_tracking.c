/*
 * robot_tracking.c — 红外循迹模块实现 (TCRT5000 × 2)
 *
 * TCRT5000 检测原理:
 *   - 包含红外发射管和接收管
 *   - 黑色表面: 红外光被吸收 → 接收管截止 → GPIO 读取低电平 (LOW=0)
 *   - 白色表面: 红外光被反射 → 接收管导通 → GPIO 读取高电平 (HIGH=1)
 *
 * 传感器布局 (俯视图):
 *          ┌─────────┐
 *          │  小车车身  │
 *   ┌──────┴─────────┴──────┐
 *   │ 左传感器      右传感器  │  ← 车头方向
 *   └───────────────────────┘
 *       ↑   ═══黑线═══   ↑
 *      (白)   (黑)    (白)
 *
 *   正常时两个传感器都在白色区域, 黑线在两者之间穿过
 *
 * GPIO: 左传感器=GPIO11, 中间传感器=GPIO3, 右传感器=GPIO12
 */

#include "robot_tracking.h"

/*
 * 初始化循迹传感器引脚
 * 两个引脚均配置为输入模式 (依赖外部上拉电阻)
 */
void tracking_init(void)
{
    /* 左侧 TCRT5000 → GPIO11 输入 */
    hi_io_set_func(TCRT5000_L, 0);
    IoTGpioSetDir(TCRT5000_L, IOT_GPIO_DIR_IN);

    /* 中间 TCRT5000 → GPIO3 输入 */
    hi_io_set_func(TCRT5000_C, 0);
    IoTGpioSetDir(TCRT5000_C, IOT_GPIO_DIR_IN);

    /* 右侧 TCRT5000 → GPIO12 输入 */
    hi_io_set_func(TCRT5000_R, 0);
    IoTGpioSetDir(TCRT5000_R, IOT_GPIO_DIR_IN);

    IoTGpioInit(PARK_RELEASE_BUTTON);
    hi_io_set_func(PARK_RELEASE_BUTTON, 0);
    IoTGpioSetDir(PARK_RELEASE_BUTTON, IOT_GPIO_DIR_IN);
    hi_io_set_pull(PARK_RELEASE_BUTTON, HI_IO_PULL_UP);
}

/*
 * 读取左右循迹传感器状态
 *
 * 参数:
 *   left_is_white  - [输出] 左侧传感器: 1=白色表面, 0=黑色表面(压线)
 *   right_is_white - [输出] 右侧传感器: 1=白色表面, 0=黑色表面(压线)
 *
 * 注意: 传感器在黑线上返回0, 在白底上返回1
 */
void tracking_read(int *left_is_white, int *center_is_white, int *right_is_white)
{
    IotGpioValue val;

    /* 读取右侧传感器 */
    IoTGpioGetInputVal(TCRT5000_R, &val);
    *left_is_white = (val == IOT_GPIO_VALUE1) ? 0 : 1;

    /* 读取中间传感器 */
    IoTGpioGetInputVal(TCRT5000_C, &val);
    *center_is_white = (val == IOT_GPIO_VALUE1) ? 0 : 1;

    /* 读取左侧传感器 */
    IoTGpioGetInputVal(TCRT5000_L, &val);
    *right_is_white = (val == IOT_GPIO_VALUE1) ? 0 : 1;
}

int parking_button_pressed(void)
{
    IotGpioValue val = IOT_GPIO_VALUE1;

    IoTGpioGetInputVal(PARK_RELEASE_BUTTON, &val);
    return (val == IOT_GPIO_VALUE0) ? 1 : 0;
}
