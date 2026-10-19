#include "bluetooth.h"

/* ==================== 接收缓冲区 ====================
 * 单生产者（USART2 中断）→ 单消费者（主循环）：
 * head 只在中断里动，tail 只在主循环里动，所以不用关中断保护。 */
#define BT_RX_BUF_SIZE 64U /* 必须是 2 的幂 */
#define BT_RX_BUF_MASK (BT_RX_BUF_SIZE - 1U)

static volatile uint8_t rx_buf[BT_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0; /* 中断写 */
static volatile uint16_t rx_tail = 0; /* 主循环读 */
static uint8_t rx_byte = 0;           /* HAL 单字节接收缓存 */
static volatile uint8_t emergency_stop = 0U;
static volatile uint32_t save_sequence = 0U;

/* ==================== 命令缓冲 ====================
 * 不要求手机 APP 一定发换行：收到最后一个字节后 BT_IDLE_MS 内没有新字节，
 * 就把已收到的内容当成一条完整命令解析。
 * 这样"单个字符"和"start\r\n"两类 APP 都能用，
 * 也不会出现把 "start" 里的 's'、'r' 当成单字符命令误触发的问题。 */
#define BT_LINE_MAX 16
#define BT_IDLE_MS 30U

static char line[BT_LINE_MAX];      /* 当前正在拼的一行命令 */
static uint8_t line_len = 0;
static uint32_t line_tick = 0;      /* 最后一个字节的到达时刻 */
static BluetoothCmd pending_cmd = BT_CMD_NONE;

/* 命令表：整条命令比较（转小写后），可在此处修改协议 */
static const struct
{
    const char *text;
    BluetoothCmd cmd;
} bt_cmd_table[] = {
    {"start", BT_CMD_START},
    {"s", BT_CMD_START},
    {"3", BT_CMD_START},
    {"stop", BT_CMD_STOP},
    {"p", BT_CMD_STOP},
    {"0", BT_CMD_STOP},
    {"teach", BT_CMD_TEACH},
    {"t", BT_CMD_TEACH},
    {"1", BT_CMD_TEACH},
    {"save", BT_CMD_SAVE},
    {"2", BT_CMD_SAVE},
    {"map", BT_CMD_MAP},
    {"m", BT_CMD_MAP},
};

/* 字符转小写 */
static char Bluetooth_ToLower(char c)
{
    if (c >= 'A' && c <= 'Z')
        return (char)(c - 'A' + 'a');

    return c;
}

/* 一整行命令 -> 命令码 */
static BluetoothCmd Bluetooth_Parse(const char *text)
{
    char lower[BT_LINE_MAX];
    uint8_t i = 0;

    while (text[i] != '\0' && i < BT_LINE_MAX - 1)
    {
        lower[i] = Bluetooth_ToLower(text[i]);
        i++;
    }
    lower[i] = '\0';

    for (i = 0; i < sizeof(bt_cmd_table) / sizeof(bt_cmd_table[0]); i++)
    {
        if (strcmp(lower, bt_cmd_table[i].text) == 0)
            return bt_cmd_table[i].cmd;
    }

    return BT_CMD_UNKNOWN;
}

/* 把环形缓冲里的字节搬进命令缓冲，凑够一行就解析 */
static void Bluetooth_Feed(void)
{
    while (rx_tail != rx_head)
    {
        uint8_t b = rx_buf[rx_tail];

        rx_tail = (uint16_t)((rx_tail + 1U) & BT_RX_BUF_MASK);
        line_tick = HAL_GetTick();

        /* 数字ASCII命令是独立单字节帧，立即生效；一次只返回一个命令，
         * 剩余字节留在环形缓冲中，不能像旧版一样被最后一条覆盖。 */
        if (b == '0' || (line_len == 0U && b >= '1' && b <= '3'))
        {
            const char digit[2] = {(char)b, '\0'};
            line_len = 0U; /* '0'也可打断半条未结束的文本命令 */
            pending_cmd = Bluetooth_Parse(digit);
            return;
        }

        if (b == '\r' || b == '\n')
        {
            /* 一行结束：解析（空行/多余换行直接忽略） */
            if (line_len > 0)
            {
                line[line_len] = '\0';
                pending_cmd = Bluetooth_Parse(line);
                line_len = 0;
                return;
            }
            continue;
        }

        if (b < 0x20 || b > 0x7E) /* 二进制命令不当作ASCII命令执行 */
            continue;

        if (line_len < BT_LINE_MAX - 1)
        {
            line[line_len++] = (char)b;
        }
        else
        {
            line_len = 0; /* 太长，多半是乱码，清掉重新收 */
        }
    }

    /* 不带换行的 APP：最后 1 个字节之后静默 BT_IDLE_MS 就当作一行收完 */
    if (line_len > 0 && (HAL_GetTick() - line_tick) > BT_IDLE_MS)
    {
        line[line_len] = '\0';
        pending_cmd = Bluetooth_Parse(line);
        line_len = 0;
    }
}

void Bluetooth_Init(void)
{
    rx_head = 0;
    rx_tail = 0;
    line_len = 0;
    line_tick = HAL_GetTick();
    pending_cmd = BT_CMD_NONE;
    emergency_stop = 0U;
    save_sequence = 0U;

    /* 打开中断接收：每收到 1 字节进一次回调，回调里自动续上 */
    HAL_UART_Receive_DMA(&huart2, &rx_byte, 1);
}

/* 串口接收完成回调（中断上下文）：收到 1 个字节 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != USART2)
        return;

    /* 与主循环解析独立：即使环形队列已满也必须收到急停。
     * 不在低优先级UART/DMA回调内写左右电机，避免打断TIM1造成半组输出。 */
    if (rx_byte == '0') emergency_stop = 1U;
    if (rx_byte == '2') save_sequence++;

    next = (uint16_t)((rx_head + 1U) & BT_RX_BUF_MASK);

    if (next != rx_tail) /* 缓冲满了就丢新数据（主循环没跟上才会发生） */
    {
        rx_buf[rx_head] = rx_byte;
        rx_head = next;
    }

    HAL_UART_Receive_DMA(huart, &rx_byte, 1); /* 继续收下一个字节 */
}

/* 串口错误回调（中断上下文）：溢出/帧错误会让 HAL 关掉接收中断，
 * 这里清标志并重新武装接收，否则一次干扰之后蓝牙就永久收不到数据 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
        return;

    /* ORE（接收溢出）是"阻塞型"错误：HAL 已经把接收中断关掉、状态复位，
     * 必须自己重新武装，否则一次溢出之后蓝牙就永久收不到数据。
     * 其它错误（噪声/帧错误）HAL 会继续收，下面的 Receive_IT 返回 BUSY，无副作用。 */
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
        __HAL_UART_CLEAR_OREFLAG(huart);

    HAL_UART_Receive_DMA(huart, &rx_byte, 1);
}

BluetoothCmd Bluetooth_GetCommand(void)
{
    BluetoothCmd cmd;

    Bluetooth_Feed();

    cmd = pending_cmd;
    pending_cmd = BT_CMD_NONE;

    return cmd;
}

uint8_t Bluetooth_EmergencyStopLatched(void) { return emergency_stop; }
uint32_t Bluetooth_GetSaveSequence(void) { return save_sequence; }

void Bluetooth_ClearEmergencyStop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* 当前已排队的学习/比赛命令不允许在急停后自动重放。 */
    rx_tail = rx_head;
    line_len = 0U;
    pending_cmd = BT_CMD_NONE;
    emergency_stop = 0U;
    __set_PRIMASK(primask);
}

void Bluetooth_SendString(const char *str)
{
    /* 只在主循环里调用。9600 波特率约 1ms/字节，50ms 超时足够发完一行；
     * 就算蓝牙没接上，超时返回也不会卡死程序 */
    HAL_UART_Transmit(&huart2, (const uint8_t *)str, (uint16_t)strlen(str), 50);
}
