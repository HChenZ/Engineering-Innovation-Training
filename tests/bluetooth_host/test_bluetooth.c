/* gcc -std=c11 -Wall -Wextra -Werror -Itests/bluetooth_host -IBSP/Inc
 *     tests/bluetooth_host/test_bluetooth.c -o <temp>/test_bluetooth.exe */
#include <assert.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"
UART_HandleTypeDef huart2 = {USART2, 0U};
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
int HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)h; (void)p; (void)n; return 0; }
int HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *p, uint16_t n, uint32_t timeout)
{ (void)h; (void)p; (void)n; (void)timeout; return 0; }

#include "../../BSP/Src/bluetooth.c"

static void Feed(const char *text)
{
    while (*text)
    {
        rx_byte = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(&huart2);
    }
}

int main(void)
{
    Bluetooth_Init();
    Feed("1"); assert(Bluetooth_GetCommand() == BT_CMD_TEACH);
    uint32_t old_save = Bluetooth_GetSaveSequence();
    Feed("2");
    assert(Bluetooth_GetSaveSequence() == old_save + 1U); /* visible before main parsing */
    assert(Bluetooth_GetCommand() == BT_CMD_SAVE);
    Feed("3"); assert(Bluetooth_GetCommand() == BT_CMD_START);
    Feed("0"); assert(Bluetooth_GetCommand() == BT_CMD_STOP);
    assert(Bluetooth_EmergencyStopLatched());
    Bluetooth_ClearEmergencyStop();
    assert(tick == 0U); /* digits do not need the 30ms text timeout */
    puts("PASS immediate ASCII 1/2/3/0 mapping");

    Feed("1230\r\n");
    assert(Bluetooth_GetCommand() == BT_CMD_TEACH);
    assert(Bluetooth_GetCommand() == BT_CMD_SAVE);
    assert(Bluetooth_GetCommand() == BT_CMD_START);
    assert(Bluetooth_GetCommand() == BT_CMD_STOP);
    assert(Bluetooth_GetCommand() == BT_CMD_NONE);
    puts("PASS commands are not overwritten when buffered together");

    Feed("teach\r\nSAVE\nstart\rstop\n");
    assert(Bluetooth_GetCommand() == BT_CMD_TEACH);
    assert(Bluetooth_GetCommand() == BT_CMD_SAVE);
    assert(Bluetooth_GetCommand() == BT_CMD_START);
    assert(Bluetooth_GetCommand() == BT_CMD_STOP);
    assert(Bluetooth_GetCommand() == BT_CMD_NONE);
    Feed("m"); assert(Bluetooth_GetCommand() == BT_CMD_NONE);
    tick += 31U;
    assert(Bluetooth_GetCommand() == BT_CMD_MAP);
    puts("PASS text aliases / CRLF / idle timeout");

    Feed("tea"); assert(Bluetooth_GetCommand() == BT_CMD_NONE);
    Feed("0"); assert(Bluetooth_GetCommand() == BT_CMD_STOP);
    tick += 31U;
    assert(Bluetooth_GetCommand() == BT_CMD_NONE);
    Feed("\x01\x02\x03"); assert(Bluetooth_GetCommand() == BT_CMD_NONE);
    puts("PASS stop interrupts partial text / binary bytes are not ASCII commands");

    Bluetooth_ClearEmergencyStop();
    Feed("3");
    for (unsigned i = 0U; i < BT_RX_BUF_SIZE + 10U; i++) Feed("x");
    Feed("0"); /* RX queue is full: emergency latch must still work before parsing. */
    assert(Bluetooth_EmergencyStopLatched());
    Bluetooth_ClearEmergencyStop();
    assert(!Bluetooth_EmergencyStopLatched());
    assert(Bluetooth_GetCommand() == BT_CMD_NONE); /* stale start was discarded */
    Feed("3"); assert(Bluetooth_GetCommand() == BT_CMD_START);
    puts("PASS stop before main parsing / full queue / no stale restart after stop");
    puts("ALL TESTS PASSED");
    return 0;
}
