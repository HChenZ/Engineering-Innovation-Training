# 三路 HC-SR04 超声波驱动

本驱动按收到的“新款 HC-SR04 / CS-100A”资料实现：

- 工作电压 3.0 V - 5.5 V，单模块工作电流约 5.3 mA；
- 标称量程 2 cm - 600 cm，探测精度约 0.1 cm + 1%；
- TRIG 至少给 10 us 高电平；
- ECHO 高电平宽度是超声波往返时间，距离按 `时间 * 340 m/s / 2` 换算；
- 超量程时 ECHO 可能出现约 66 ms 的长脉冲，驱动按无回波处理。

## 默认接线

| 逻辑位置 | TRIG | ECHO | 本驱动使用的 STM32 引脚 |
| --- | --- | --- | --- |
| 左侧 `ULTRASONIC_LEFT` | TRIG3 | ECHO3 | PA4 -> PA5 |
| 中间 `ULTRASONIC_CENTER` | TRIG2 | ECHO2 | PA10 -> PA11 |
| 右侧 `ULTRASONIC_RIGHT` | TRIG1 | ECHO1 | PA8 -> PA9 |

三个模块的 GND 必须和 STM32 共地。若实际线束左右相反，只需要交换
`BSP/Src/ultrasonic.c` 中 `ultrasonic_hw[]` 的 LEFT/RIGHT 两项。

### CubeMX 配置

三个 ECHO 引脚在 CubeMX 中都配置为 `GPXTI`、双边沿触发：

| ECHO 引脚 | EXTI 线 | 共用中断入口 |
| --- | --- | --- |
| PA5 / ECHO3 | EXTI5 | `EXTI9_5_IRQn` |
| PA9 / ECHO1 | EXTI9 | `EXTI9_5_IRQn` |
| PA11 / ECHO2 | EXTI11 | `EXTI15_10_IRQn` |

因此三个输入只需要两个 NVIC 向量。当前驱动在 `Ultrasonic_Init()` 中
清挂起标志、设置优先级并使能这两个向量；CubeMX 生成的 `MX_GPIO_Init()`
负责 GPIO 模式、`GPIO_NOPULL` 和双边沿 EXTI 属性。初始化顺序必须是：

```c
MX_GPIO_Init();
/* ...其它外设初始化... */
Ultrasonic_Init();
```

本工程采用“驱动使能 NVIC + `USER CODE` 中自定义 IRQHandler”的方式，
所以 CubeMX 的 NVIC 页不需要再勾选 EXTI9_5/EXTI15_10。若以后决定让
CubeMX 自动生成这两个 IRQHandler，需要先删除 `stm32f1xx_it.c` 中
`USER CODE BEGIN 1` 内的同名处理函数，避免重复定义。

### 供电电平

最简单的接法是三个模块都使用 **3.3 V**，这样 ECHO 输出不会超过 MCU
的 GPIO 电平，可以直接接 PA9、PA11、PA5。

如果模块使用 5 V 供电，ECHO 也可能输出约 5 V。不要直接假设所有
STM32 引脚都能承受 5 V；应分别确认数据手册中的 FT 标识，或给所有 ECHO
加电平转换/电阻分压。5 V 模块的 TRIG 高电平阈值也可能高于 3.3 V，
此时还应增加合适的电平转换。长线供电建议在模块附近再加 100 uF 和
0.1 uF 去耦电容。

## 软件用法

`Ultrasonic_Init()` 已经加入当前工程的 `main()`。主循环必须持续调用
`Ultrasonic_Process()`：

```c
#include "ultrasonic.h"

uint16_t center_mm;

Ultrasonic_Init();

while (1)
{
    Ultrasonic_Process();

    if (Ultrasonic_GetDistance(ULTRASONIC_CENTER, &center_mm))
    {
        /* center_mm 是最近三次有效采样的中值，单位 mm。 */
    }
}
```

一次取回三路状态：

```c
Ultrasonic_Reading reading[ULTRASONIC_SENSOR_COUNT];

Ultrasonic_GetAll(reading);

if (reading[ULTRASONIC_CENTER].valid)
{
    uint16_t center_mm = reading[ULTRASONIC_CENTER].filtered_mm;
    uint16_t raw_mm = reading[ULTRASONIC_CENTER].latest_mm;
    uint32_t age_ms = reading[ULTRASONIC_CENTER].age_ms;

    (void)center_mm;
    (void)raw_mm;
    (void)age_ms;
}
```

通过现有蓝牙串口周期性打印的调试示例：

```c
#include <stdio.h>
#include "bluetooth.h"
#include "ultrasonic.h"

static uint32_t next_print_tick;

while (1)
{
    Ultrasonic_Process();

    if ((HAL_GetTick() - next_print_tick) >= 500U)
    {
        Ultrasonic_Reading r[ULTRASONIC_SENSOR_COUNT];
        char line[64];

        next_print_tick = HAL_GetTick();
        Ultrasonic_GetAll(r);

        (void)snprintf(line, sizeof(line),
                       "L:%u C:%u R:%u\r\n",
                       (unsigned)r[ULTRASONIC_LEFT].filtered_mm,
                       (unsigned)r[ULTRASONIC_CENTER].filtered_mm,
                       (unsigned)r[ULTRASONIC_RIGHT].filtered_mm);
        Bluetooth_SendString(line);
    }
}
```

`ULTRASONIC_INVALID_MM`（65535）表示该路当前没有可用数据。不要只打印
`filtered_mm` 而不检查 `valid`，否则可能把“尚未测到”误当成远距离。

## 稳定性处理

驱动内部已经做了以下处理：

1. 三路按 CENTER -> LEFT -> RIGHT 顺序触发，不会同时发射 40 kHz 脉冲；
2. 每次测量结束到下一次触发之间保留 20 ms 保护间隔；
3. ECHO 用 EXTI 双边沿中断和 DWT 周期计数器计时，不依赖主循环轮询精度；
4. 启动等待 10 ms、回波最长等待 70 ms，超量程的约 66 ms 脉冲不会卡死程序；
5. 每路保存最近三次有效采样并取中值，偶发一个尖峰不会直接进入控制；
6. 连续两次无回波或超量程后，该路立即标记为无效；
7. 某路 ECHO 卡在高电平时会跳过该路继续扫描其它两路，避免整组停摆。

## 安装建议

- 中间模块朝向车体正前方；左右模块可按路径规划向两侧各偏约 30 度到
  45 度。模块波束角小于 15 度，角度越大，能看到的走廊侧壁范围越靠后，
  但也更容易漏掉细小障碍。
- 三个探头尽量安装在同一高度，并避开地面直接反射和车体自身的超声反射面。
- 模块之间不要紧贴。可以在探头之间加吸音泡沫或薄挡板，减少左右模块的
  直接串扰。
- 如果结构上三路仍会互相照到，可将 `ultrasonic.c` 中的
  `ULTRASONIC_GUARD_US` 从 20 ms 调到 30 ms - 50 ms，代价是刷新率下降。
- 标称 600 cm 是在平直、硬质反射面上的规格；斜向小目标、软质衣物、
  海绵和细杆的实际可靠距离会明显缩短，不能按最大值给避障逻辑留安全边界。
- 先用中心模块在 30 cm、100 cm、200 cm 的硬质平面做基准测试，再分别转动
  左右模块检查各自观测区域。最终避障阈值应在整车电池电压和实际安装结构
  下实测确定。
