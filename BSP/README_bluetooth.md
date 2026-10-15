# 蓝牙启动控制（USART2 @ 9600）

给小车加一个蓝牙模块，手机 APP 发一条命令就能**启动 / 停车**。
上电后小车默认**待机不动**，收到 `start` 才开始循迹 + 避障，
这样比赛/调试时不用再靠拔电池来控制车什么时候跑。

## 一、硬件接线

| 蓝牙模块 | STM32 | 说明 |
| --- | --- | --- |
| TXD | **PA3**（USART2_RX） | 模块发 → 单片机收 |
| RXD | **PA2**（USART2_TX） | 单片机发 → 模块收 |
| VCC | 5V | HC-05/06 的底板自带稳压；**裸模块只能接 3.3V** |
| GND | GND | 必须共地 |
| EN/KEY | 悬空 | 不接就是正常透传模式 |

串口参数：**9600，8 位数据，无校验，1 位停止位（8-N-1）**，
无硬件流控，所以只要 TX/RX/GND 三根线。

> 模块出厂波特率一般就是 9600（HC-05 可能是 9600 或 38400）。
> 如果连不上，先把模块插到 USB-TTL 上用 AT 指令 `AT+UART?` 查一下，
> 改成 9600 最简单——单片机侧改波特率要动 `usart.c`，不如改模块。

## 二、命令

手机装任意"蓝牙串口助手"（如 蓝牙调试助手 / Serial Bluetooth Terminal），
连上模块后发下面的命令。**大小写都行，结尾带不带 `\r\n` 都行**
（代码里做了 30ms 静默判行，单字符和整串命令都能认）。

| 发出去的内容 | 作用 |
| --- | --- |
| `start` / `go` / `s` / `1` | 启动：小车开始循迹 + 避障 |
| `stop` / `p` / `0` | 停车：立即刹车，之后不再动 |

每收到一条命令，单片机会回一行应答，方便确认链路通：

```
收到 start    ->  OK START
收到 stop     ->  OK STOP
已经在跑再发   ->  OK RUNNING
不认识的命令   ->  ERR
```

## 三、车上的状态指示

| 状态 | LED（PC13） | OLED 第一行 |
| --- | --- | --- |
| 上电待机（还没收到 start） | 灭 | `STOP` + `WAIT CMD` |
| 运行中 | 亮 | `RUN` + `CMD:START` |
| 蓝牙停车后 | 灭 | `STOP` + `CMD:STOP` |

**8 路循迹全部压线（终点线）会触发终止停车**，此时 LED 灭、车停下；
想再跑一次，直接用蓝牙再发一次 `start` 即可（会清掉终止标志并复位 PID）。

## 四、代码放在哪

| 文件 | 内容 |
| --- | --- |
| `Core/Src/usart.c`、`Core/Inc/usart.h` | USART2 初始化（9600，PA2/PA3），CubeMX 风格 |
| `BSP/Src/bluetooth.c`、`BSP/Inc/bluetooth.h` | 中断接收、命令解析、应答、错误恢复 |
| `Core/Src/stm32f1xx_it.c` | `USART2_IRQHandler` → `HAL_UART_IRQHandler(&huart2)` |
| `Core/Src/main.c` | 主循环里处理 `start`/`stop`：`g_run` 控制跑/停 |

接收方式：**单字节中断接收 + 64 字节环形缓冲**，
主循环 `Bluetooth_GetCommand()` 取命令，不会因为主循环忙（OLED、测距）而丢字节。

## 五、和 CubeMX 的关系（重要）

`.ioc` 里已经加好了 USART2（异步、9600、PA2/PA3），重新生成代码不会丢配置。
但有两个地方**故意没用 CubeMX 的 NVIC 勾选**：

* USART2 的**中断使能**写在 `BSP/Src/bluetooth.c` 的 `Bluetooth_Init()` 里，
  优先级 2（低于 TIM1 的 1，保证 10ms 循迹周期优先）。
  这样 CubeMX 重新生成代码不会把它覆盖掉。
* 如果你以后在 CubeMX 里勾上了 "USART2 global interrupt"，重新生成后
  `stm32f1xx_it.c` 里会多出一个 `USART2_IRQHandler`，
  和手写的那份**重定义冲突**（编译报错）——把 `USER CODE BEGIN 1` 里手写的那份删掉即可。

另外 `stm32f1xx_hal_uart.c/.h`（HAL 驱动）和 `stm32f1xx_hal_conf.h` 里的
`HAL_UART_MODULE_ENABLED`、`cmake/stm32cubemx/CMakeLists.txt` 里的源文件列表
都已一并加好，CubeMX 重新生成时会保持一致。

## 六、第一次上电怎么试（建议按顺序）

1. 先不装车（或者把驱动轮架空），上电：OLED 显示 `STOP / WAIT CMD`，电机不动。
2. 手机连上模块，发 `1`，手机应立刻收到 `OK START`，OLED 变 `RUN`，LED 亮。
   收不到 `OK START` 就是链路问题：查 TX/RX 是否交叉、波特率是否 9600、模块是否已配对。
3. 发 `0`，应收到 `OK STOP`，电机刹车。
4. 放到地上发 `start`，正常循迹；用手挡住超声波应能看到避障动作。
5. 全程跑完压终点线后，车自己停下；再发 `start` 可以再跑一轮。

## 七、如果比赛要求"全自动运行"

本项目自己的避障规则笔记里写着"**非全自动运行直接不予参赛**"，
所以主循环顶部留了一个开关（`Core/Src/main.c` 的 `BT_START_GATE`）：

* `BT_START_GATE = 1`（默认）：上电待机，**必须**收到蓝牙 `start` 才跑，方便调试和发车；
* `BT_START_GATE = 0`：上电**直接开跑**，蓝牙只当紧急停车用，避免被裁判认为"遥控发车"。

比赛当天的发车方式按裁判口径选一个即可，改一个数字、重新编译就行。
