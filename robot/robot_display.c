/*
 * robot_display.c — OLED 显示模块实现 (SSD1306 + I2C0)
 *
 * 接口:
 *   I2C0: SDA=GPIO13, SCL=GPIO14
 *   速率: 400kHz (快速模式)
 *   从机地址: 0x3C
 *
 * 多线程架构:
 *   - display_task() 在独立低优先级线程中运行
 *   - 每 200ms 从全局共享变量读取最新状态并刷新屏幕
 *   - I2C 传输阻塞 (约 20-50ms) 不影响控制和舵机线程
 *
 * 屏幕布局 (128×64 像素, Font_7x10 字体):
 *   ┌──────────────────────────────┐
 *   │   XunJi Car v1.0            │  Y=0  标题
 *   │──────────────────────────────│  Y=11 分隔线
 *   │Dist: XX.X cm                │  Y=14 超声波测距
 *   │Line: L=WHT R=WHT           │  Y=25 传感器状态
 *   │Act: FORWARD                 │  Y=36 当前动作
 *   │Svo: <<SCAN                  │  Y=47 舵机扫描方向
 *   └──────────────────────────────┘
 */

#include "robot_display.h"

/*
 * 初始化 OLED 显示模块
 * 1. 配置 GPIO13/14 为 I2C0 SDA/SCL
 * 2. 初始化 I2C0 (400kHz)
 * 3. 初始化 SSD1306 控制器
 * 4. 清屏
 */
void display_init(void)
{
    /* GPIO 初始化 */
    IoTGpioInit(HI_IO_NAME_GPIO_13);
    IoTGpioInit(HI_IO_NAME_GPIO_14);

    /* 复用为 I2C0 功能 */
    hi_io_set_func(HI_IO_NAME_GPIO_13, HI_IO_FUNC_GPIO_13_I2C0_SDA);
    hi_io_set_func(HI_IO_NAME_GPIO_14, HI_IO_FUNC_GPIO_14_I2C0_SCL);

    /* 初始化 I2C0, 400kHz */
    IoTI2cInit(0, OLED_I2C_BAUDRATE);

    /* 等待 I2C 总线和 OLED 模块稳定 */
    usleep(20 * 1000);

    /* SSD1306 初始化并清屏 */
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

/*
 * 刷新 OLED 显示内容 (单次全屏刷新)
 *
 * 参数:
 *   distance    - 超声波测距值(cm), 负值表示超时/超出量程
 *   left_white  - 左循迹传感器: 1=白底, 0=黑线(压线)
 *   right_white - 右循迹传感器: 1=白底, 0=黑线(压线)
 *   action      - 当前动作的字符串描述
 *   servo_label - 舵机扫描方向标签
 */
void display_update(float distance, int left_white, int center_white, int right_white,
                    const char *action, const char *servo_label)
{
    char buf[22];  /* 128像素 / Font_7x10宽度(7px) ≈ 18字符 */

    /* 清空帧缓冲区, 准备重绘 */
    ssd1306_Fill(Black);

    /* ---- 第1行: 标题 (Y=0) ---- */
    ssd1306_SetCursor(22, 0);
    ssd1306_DrawString("XunJi Car v1.0", Font_7x10, White);

    /* ---- 分隔线 (Y=11) ---- */
    ssd1306_DrawLine(0, 11, 127, 11, White);

    /* ---- 第2行: 超声波距离 (Y=14) ---- */
    ssd1306_SetCursor(0, 14);
    if (distance < 0.0f) {
        ssd1306_DrawString("Dist: ---.- cm", Font_7x10, White);
    } else {
        snprintf(buf, sizeof(buf), "Dist: %.1f cm   ", distance);
        ssd1306_DrawString(buf, Font_7x10, White);
    }

    /* ---- 第3行: 循迹传感器状态 (Y=25) ---- */
    ssd1306_SetCursor(0, 25);
    snprintf(buf, sizeof(buf), "Line:L=%c C=%c R=%c",
             left_white   ? 'W' : 'B',
             center_white ? 'W' : 'B',
             right_white  ? 'W' : 'B');
    ssd1306_DrawString(buf, Font_7x10, White);

    /* ---- 第4行: 当前动作 (Y=36) ---- */
    ssd1306_SetCursor(0, 36);
    snprintf(buf, sizeof(buf), "Act: %s", action);
    ssd1306_DrawString(buf, Font_7x10, White);

    /* ---- 第5行: 舵机扫描方向 (Y=47) ---- */
    ssd1306_SetCursor(0, 47);
    snprintf(buf, sizeof(buf), "Svo: %s", servo_label);
    ssd1306_DrawString(buf, Font_7x10, White);

    /* 刷新到物理屏幕 */
    ssd1306_UpdateScreen();
}
