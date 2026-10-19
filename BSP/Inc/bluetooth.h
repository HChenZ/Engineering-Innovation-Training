#ifndef BLUETOOTH_H
#define BLUETOOTH_H

// 蓝牙模块（HC-05 / HC-06 / JDY-31 等）串口控制
//
// 硬件连接（模块 TX/RX 与单片机交叉）：
//   蓝牙模块 TXD -> PA3 (USART2_RX)   单片机接收
//   蓝牙模块 RXD -> PA2 (USART2_TX)   单片机发送
//   蓝牙模块 VCC -> 5V（HC-05/06 的底板带稳压，裸模块必须 3.3V！）
//   蓝牙模块 GND -> GND
// 串口参数：9600 8-N-1（模块出厂默认，别忘了一起改手机 APP 的波特率）
//
// 三阶段ASCII协议（数字单字节立即解析，不需要换行）：
//   '1' / 0x31 -> 学习（teach/t），只循迹；有参考圈长可自动终点停，否则发2结束
//   '2' / 0x32 -> 停车并保存（save），回OK MAP之后才能断电或比赛
//   '3' / 0x33 -> 比赛（start/s），必须已有地图；不自动学习
//   '0' / 0x30 -> 接收回调锁存急停，下一TIM1优先刹车并取消未保存学习，不覆盖旧图
//   注意：发送字符'1'，不是二进制字节0x01！
//   "map"   / "m"        -> 查询地图点数、间隔、模式和救援状态；无图回 "MAP NONE"
// 每收到一条命令，单片机回一行应答，方便手机端确认链路：
//   收到 start -> "OK START MAP\r\n"（无图回ERR NO MAP）
//   收到 stop  -> "OK STOP\r\n"
//   不认识的命令 -> "ERR\r\n"
//
// 用法：
//   init：MX_USART2_UART_Init() 之后调用 Bluetooth_Init()
//   主循环：while ((cmd = Bluetooth_GetCommand()) != BT_CMD_NONE) { ... }
//          （一个字节都不丢：接收在 USART2 中断里，主循环只是来取）

#include "main.h"
#include "usart.h"
#include <string.h>

/* 蓝牙命令 */
typedef enum
{
    BT_CMD_NONE = 0, /* 没有新命令 */
    BT_CMD_START,    /* 阶段3：比赛 */
    BT_CMD_STOP,     /* 停车 */
    BT_CMD_TEACH,    /* 阶段1：学习地图到RAM，见 odom.h */
    BT_CMD_SAVE,     /* 阶段2：先停车，再保存到Flash */
    BT_CMD_MAP,      /* 查询地图状态 */
    BT_CMD_UNKNOWN   /* 收到不认识的命令（可以回 ERR 提示） */
} BluetoothCmd;

/* 初始化：使能 USART2 中断并开始接收（MX_USART2_UART_Init 之后调用） */
void Bluetooth_Init(void);

/* 取一条命令（主循环里反复调用，直到返回 BT_CMD_NONE） */
BluetoothCmd Bluetooth_GetCommand(void);

/* ASCII '0'在接收回调直接锁存，TIM1据此优先刹车，不等OLED/文本命令处理。
 * Clear只由主循环在已设置停车请求后调用，并丢弃旧队列，防止迟到的start重启。 */
uint8_t Bluetooth_EmergencyStopLatched(void);
void Bluetooth_ClearEmergencyStop(void);
uint32_t Bluetooth_GetSaveSequence(void); /* ASCII '2'接收次数，TIM1可快速停车待存 */

/* 通过蓝牙回发字符串（调试/应答用） */
void Bluetooth_SendString(const char *str);

#endif

