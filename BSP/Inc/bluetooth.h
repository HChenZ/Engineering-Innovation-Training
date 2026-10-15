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
// 命令（大小写都行，结尾带不带 \r\n 都行）：
//   "start" / "s" / "1"  -> 启动：小车开始循迹 + 避障
//   "stop"  / "p" / "0"  -> 停车：立即刹车
// 每收到一条命令，单片机回一行应答，方便手机端确认链路：
//   收到 start -> "OK START\r\n"
//   收到 stop  -> "OK STOP\r\n"
//   不认识的命令 -> "ERR\r\n"
//
// 用法：
//   init：MX_USART2_UART_Init() 之后调用 Bluetooth_Init()
//   主循环：while ((cmd = Bluetooth_GetCommand()) != BT_CMD_NONE) { ... }
//          （一个字节都不丢：接收在 USART2 中断里，主循环只是来取）

#include "main.h"

/* 蓝牙命令 */
typedef enum
{
    BT_CMD_NONE = 0, /* 没有新命令 */
    BT_CMD_START,    /* 启动 */
    BT_CMD_STOP,     /* 停车 */
    BT_CMD_UNKNOWN   /* 收到不认识的命令（可以回 ERR 提示） */
} BluetoothCmd;

/* 初始化：使能 USART2 中断并开始接收（MX_USART2_UART_Init 之后调用） */
void Bluetooth_Init(void);

/* 取一条命令（主循环里反复调用，直到返回 BT_CMD_NONE） */
BluetoothCmd Bluetooth_GetCommand(void);

/* 通过蓝牙回发字符串（调试/应答用） */
void Bluetooth_SendString(const char *str);

#endif

