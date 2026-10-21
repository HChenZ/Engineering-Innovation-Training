# 左右两路 HC-SR04 超声波驱动

当前软件只采样左、右两路超声波。原中间模块的 PA10/PA11 GPIO
和 EXTI 引脚配置仍保留在 CubeMX 生成的代码中，但不再触发或读取
中间模块数据。

## 默认接线

| 逻辑位置 | TRIG | ECHO | STM32 引脚 |
| --- | --- | --- | --- |
| 左侧 `ULTRASONIC_LEFT` | TRIG3 | ECHO3 | PA4 -> PA5 |
| 右侧 `ULTRASONIC_RIGHT` | TRIG1 | ECHO1 | PA8 -> PA9 |

原中间模块引脚为 TRIG2/ECHO2，对应 PA10/PA11。引脚配置保留，
当前不参与采样。

三个模块的 GND 必须与 STM32 共地。若模块使用 5 V 供电，应确认
ECHO 输出电平是否超过 MCU 的允许输入范围，必要时增加电平转换。

## 采样方式

驱动按 `LEFT -> RIGHT` 顺序触发，两路不会同时发射 40 kHz 超声波：

1. 每次测量结束到下一次触发之间保留 20 ms 保护间隔；
2. ECHO 用 EXTI 双边沿中断和 DWT 周期计数器计时；
3. 每路保存最近三次有效采样并取中值；
4. 连续两次无回波或超量程后，该路标记为无效；
5. 数据超过 `ULTRASONIC_STALE_MS` 后不可用于控制。

与三路采样相比，完整扫描少了一次测量和保护间隔，左右数据的刷新
频率更高。

## 软件用法

初始化顺序：

```c
MX_GPIO_Init();
/* ...其他外设初始化... */
Ultrasonic_Init();
```

主循环必须持续调用非阻塞调度函数：

```c
while (1)
{
    Ultrasonic_Process();
}
```

读取左右两路状态：

```c
Ultrasonic_Reading readings[ULTRASONIC_SENSOR_COUNT];
Ultrasonic_GetAll(readings);

if (readings[ULTRASONIC_LEFT].valid)
{
    uint16_t left_mm = readings[ULTRASONIC_LEFT].filtered_mm;
}

if (readings[ULTRASONIC_RIGHT].valid)
{
    uint16_t right_mm = readings[ULTRASONIC_RIGHT].filtered_mm;
}
```

数组顺序为 `LEFT`、`RIGHT`。无有效数据时 `valid == 0`，不能直接
把 `filtered_mm` 当作远距离。
