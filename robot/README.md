# Hi3861 三路循迹小车

这是一个运行在 Hi3861（HiSpark Pegasus）上的三路循迹小车程序。小车使用左、中、右三个 TCRT5000 模块识别黑线，通过 L9110S 驱动双路直流电机，并集成超声波避障、舵机扫描、OLED 状态显示和停车标记识别。

## 功能特性

- 三路红外循迹，中间传感器用于判断车辆是否居中
- 单边压线转向修正，保留最近一次修正方向用于脱线寻线
- 左右同时或短时间先后压黑时执行特殊右转
- 三路全黑连续确认后锁定停车
- GPIO5 按钮解除停车，离开全黑区域后才允许再次触发
- HC-SR04 超声波避障，障碍消失后自动恢复循迹
- SG90 舵机带动超声波模块左右扫描
- SSD1306 OLED 显示距离、三路循迹状态、车辆动作和舵机方向
- 循迹、电机控制与慢速外设分线程运行，减少测距和 I2C 刷新对循迹频率的影响

## 硬件组成

- Hi3861 / HiSpark Pegasus 开发板
- L9110S 双路电机驱动模块
- 两个直流减速电机及小车底盘
- 三个 TCRT5000 数字循迹模块
- HC-SR04 超声波测距模块
- SG90 舵机
- SSD1306 128×64 I2C OLED
- 一个常开按键
- 合适的电源、连接线和固定结构

所有模块必须共地。电机建议使用满足其电流需求的供电方案；连接超声波模块前，请确认开发板 GPIO 与模块 ECHO 引脚的电平兼容性。

## 引脚连接

| 功能 | Hi3861 GPIO | 方向或复用 |
| --- | ---: | --- |
| 左电机 IN1 | GPIO0 | PWM3 |
| 左电机 IN2 | GPIO1 | PWM4 |
| 右电机 IN1 | GPIO9 | PWM0 |
| 右电机 IN2 | GPIO10 | PWM1 |
| 左循迹模块 | GPIO11 | 输入 |
| 中间循迹模块 | GPIO3 | 输入 |
| 右循迹模块 | GPIO12 | 输入 |
| 停车解除按钮 | GPIO5 | 输入、内部上拉、按下接地 |
| 舵机信号 | GPIO2 | 软件 PWM 输出 |
| HC-SR04 TRIG | GPIO7 | 输出 |
| HC-SR04 ECHO | GPIO8 | 输入 |
| OLED SDA | GPIO13 | I2C0 SDA |
| OLED SCL | GPIO14 | I2C0 SCL |

> `tracking_read()` 中的左右与黑白映射来自实车校准，并非简单按 GPIO 名称直读。移植到其他车体时，应先通过 OLED 检查三路 `W/B` 状态，再决定是否调整映射。

## 循迹规则

程序内部使用 `1=白底`、`0=黑线`，OLED 使用 `W` 和 `B` 显示。传感器顺序统一写作 `左 / 中 / 右`。

| 传感器状态 | 含义 | 动作 |
| --- | --- | --- |
| `W / B / W` | 黑线位于中间，车辆居中 | 前进 |
| `B / X / W` | 左侧检测到黑线 | 按实车校准方向执行左修正 |
| `W / X / B` | 右侧检测到黑线 | 按实车校准方向执行右修正 |
| `B / W / B` | 左右为黑、中间为白 | 强制右转 |
| `W / W / W` | 三路脱离黑线 | 按最近修正方向寻线；无方向记忆时先前进 |
| `B / B / B` | 停车标记或宽黑线 | 连续确认后停车 |

其中 `X` 表示中间传感器状态不影响该条判断。左右两侧在两个控制循环内先后检测到黑线时，也会触发持续五个控制循环的特殊右转。

三路全白后，程序最多寻线 `6s`；仍未找到黑线时刹车并在 OLED 显示 `LOST!`。该时间可通过 `LINE_LOST_SEEK_US` 调整。

## 停车与避障

### 停车标记

三路传感器连续十个控制循环检测到黑色后，小车进入 `PARKED` 状态并保持刹车。按下 GPIO5 按钮，连续三个控制循环确认低电平后解除停车。解除后必须先离开全黑区域，停车检测才会重新启用。

### 超声波避障

超声波模块约每 `100ms` 测距一次：

- 连续两个样本小于 `10cm` 时确认障碍并停车
- 停车期间舵机保持当前方向
- 连续三个样本未检测到障碍后恢复扫描和循迹
- 测距超时返回无效距离，OLED 显示 `---.-`

OLED 约每 `200ms` 刷新一次。测距与显示位于低优先级线程中，不直接阻塞高频循迹控制循环。

## 软件架构

程序使用 CMSIS-RTOS2 创建三个线程：

| 线程 | 优先级 | 职责 |
| --- | --- | --- |
| `ServoThread` | `osPriorityNormal` | 产生 SG90 软件 PWM，更新扫描角度 |
| `ControlThread` | `osPriorityNormal` | 读取循迹与按钮、处理避障状态、控制电机 |
| `SensorDisplayThread` | `osPriorityBelowNormal` | 超声波测距和 OLED 刷新 |

主要源文件：

| 文件 | 作用 |
| --- | --- |
| `robot_car.c/.h` | 系统入口、状态判断、线程与参数定义 |
| `robot_motor.c/.h` | L9110S 与四路硬件 PWM 控制 |
| `robot_tracking.c/.h` | 三路循迹输入和停车解除按钮 |
| `robot_ultrasonic.c/.h` | HC-SR04 测距与超时保护 |
| `robot_servo.c/.h` | SG90 软件 PWM 和扫描逻辑 |
| `robot_display.c/.h` | SSD1306 OLED 页面绘制 |

`BUILD.gn` 中列出的文件才会参与 `robot_demo` 构建。目录内其他单模块示例文件用于早期硬件测试，不属于当前整车程序。

## 编译与烧录

本项目依赖 OpenHarmony WiFi-IoT/Hi3861 工程环境以及仓库中的 SSD1306 驱动。父目录 `applications/sample/wifi-iot/app/BUILD.gn` 需要启用：

```gn
lite_component("app") {
    features = [
        "robot:robot_demo",
        "ssd1306:oled_lib_app",
    ]
}
```

在 OpenHarmony 源码根目录选择 `wifiiot_hispark_pegasus` 产品并执行构建：

```bash
hb set
hb build -f
```

也可以使用已配置好的 DevEco Device Tool 完成编译、烧录和串口监视。具体固件路径与烧录方式以所使用的 Hi3861 工具链为准。

## 初次上车检查

1. 架空驱动轮上电，先确认 `car_forward()` 的两侧电机都向前转。
2. 分别用白底和黑线测试三个循迹模块，确认 OLED 的 `L/C/R` 与实际位置一致。
3. 将小车放在居中状态，确认 OLED 显示 `L=W C=B R=W`。
4. 手动让左侧和右侧压线，确认车辆修正方向符合实车安装。
5. 测试三路全黑停车及 GPIO5 按钮解除功能。
6. 最后测试超声波距离、障碍停车和舵机扫描。

## 参数调整

主要参数位于 `robot_car.h`：

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `SPEED_FORWARD` | `4000` | 前进 PWM 占空比计数 |
| `SPEED_TURN` | `3800` | 转向 PWM 占空比计数 |
| `OBSTACLE_THRESHOLD_CM` | `10.0` | 障碍判定距离 |
| `LINE_LOST_SEEK_US` | `6000000` | 全白寻线超时 |
| `PARK_BLACK_CONFIRM_CNT` | `10` | 全黑停车确认循环数 |
| `SIDE_BLACK_PAIR_WINDOW_CNT` | `2` | 左右先后压黑窗口 |
| `SPECIAL_RIGHT_TURN_CNT` | `5` | 特殊右转持续循环数 |

建议先校准传感器高度与阈值，再调整速度。循迹不稳定时先降低 `SPEED_FORWARD`，随后调整 `SPEED_TURN`；不要在未确认实车方向前交换 `car_left()`、`car_right()` 或左右传感器映射。

## OLED 状态说明

- `FORWARD`：前进
- `TURN L>` / `<TURN R`：循迹修正
- `SEEK L` / `SEEK R` / `SEEK..`：全白寻线
- `OBSTACLE!`：检测到障碍
- `PARKED`：停车标记已触发
- `LOST!`：寻线超时
- `SCAN>>` / `<<SCAN` / `CENTER` / `HOLD!`：舵机状态

## 许可证

本项目遵循仓库根目录 `LICENSE` 中对应目录的许可说明。OpenHarmony、HiHope 和 SSD1306 等第三方代码及构建文件保留其原始版权和许可证声明。
