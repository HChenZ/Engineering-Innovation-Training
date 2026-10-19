#include "oled_software_i2c.h"

void OLED_Init(void)
{
    HAL_Delay(200);

    OLED_WR_Byte(0xAE, OLED_CMD); //--turn off oled panel
    OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
    OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8, OLED_CMD); // Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00, OLED_CMD); //-not offset
    OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80, OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
    OLED_WR_Byte(0xF1, OLED_CMD); // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
    OLED_WR_Byte(0x40, OLED_CMD); // Set VCOM Deselect Level
    OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02, OLED_CMD); //
    OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
    OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
    OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD); /*display ON*/
}

void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); // 设置页地址（0~7）
        OLED_WR_Byte(0x00, OLED_CMD);     // 设置显示位置—列低地址
        OLED_WR_Byte(0x10, OLED_CMD);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(0, OLED_DATA);
    } // 更新显示
}

void OLED_ClearCharArea(uint8_t x, uint8_t y, uint8_t num_chars)
{
    uint8_t i, n;
    uint16_t end_x;

    if (y > 7)
        return;
    end_x = (uint16_t)x + (uint16_t)num_chars * 6;
    if (end_x > 128)
        num_chars = (128 - x) / 6;

    OLED_Set_Position(x, y);
    for (i = 0; i < num_chars; i++)
    {
        for (n = 0; n < 6; n++)
            OLED_WR_Byte(0x00, OLED_DATA); // 连续写0覆盖旧内容
    }
}

void OLED_ClearArea(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    uint8_t x, y;
    for (y = y1; y <= y2; y++)
    {
        OLED_Set_Position(x1, y);
        for (x = x1; x <= x2; x++)
            OLED_WR_Byte(0x00, OLED_DATA);
    }
}

void I2C_Start(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();

    OLED_SDA_Clear();
    OLED_SCL_Clear();
}

void I2C_Stop(void)
{
    OLED_SDA_Clear();
    OLED_SCL_Set();

    OLED_SDA_Set();
}

// 等待信号响应
void I2C_WaitAck(void) // 测数据信号的电平
{
    OLED_SDA_Set();

    OLED_SCL_Set();

    OLED_SCL_Clear();
}

// 写入一个字节数据
void Send_Byte(uint8_t dat)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_SCL_Clear(); // 将时钟信号设置为低电平
        if (dat & 0x80)   // 将 dat 的 8 位从最高位依次写入
            OLED_SDA_Set();
        else
            OLED_SDA_Clear();

        OLED_SCL_Set();

        OLED_SCL_Clear();
        dat <<= 1;
    }
}

// 向 SSD1306 写入一个字节
// mode: 数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    I2C_Start();
    Send_Byte(0x78);
    I2C_WaitAck();
    if (mode)
        Send_Byte(0x40);
    else
        Send_Byte(0x00);
    I2C_WaitAck();
    Send_Byte(dat);
    I2C_WaitAck();
    I2C_Stop();
}

// 坐标设置
void OLED_Set_Position(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f), OLED_CMD);
}

// 开启OLED显示
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
    OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}

// 关闭OLED显示
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
    OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}

// 在指定位置显示一个字符
// x:0~127
// y:0~63
// sizey:选择字体 6x8
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t c = 0, sizex = sizey / 2;
    uint16_t i = 0, size1;
    if (sizey == 8)
        size1 = 6;
    else
        size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);
    c = chr - ' '; // 得到偏移后的值
    OLED_Set_Position(x, y);
    for (i = 0; i < size1; i++)
    {
        if (i % sizex == 0 && sizey != 8)
            OLED_Set_Position(x, y++);
        else if (sizey == 8)
            OLED_WR_Byte(asc2_0806[c][i], OLED_DATA); // 6X8字号
        else
            return;
    }
}

// m^n函数
uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
        result *= m;
    return result;
}

// 显示数字
// x, y : 起点坐标
// num: 要显示的数字
// len: 数字的位数
// sizey: 字体大小
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)
{
    uint8_t t, temp, m = 0;
    uint8_t enshow = 0;
    if (sizey == 8)
        m = 2;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (sizey / 2 + m) * t, y, ' ', sizey);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (sizey / 2 + m) * t, y, temp + '0', sizey);
    }
}

void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t sizey)
{
    uint8_t t, temp, m = 0;
    uint8_t enshow = 0;
    uint32_t abs_num;
    uint8_t char_width;

    if (sizey == 8)
        m = 2;

    char_width = sizey / 2 + m;
    abs_num = (num < 0) ? (uint32_t)(-(int64_t)num) : (uint32_t)num;

    if (num < 0)
    {
        OLED_ShowChar(x, y, '-', sizey);
        x += char_width;
        if (len > 0)
            len--;
    }

    for (t = 0; t < len; t++)
    {
        temp = (abs_num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + char_width * t, y, ' ', sizey);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + char_width * t, y, temp + '0', sizey);
    }
}

void OLED_ShowFloatNum(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t sizey)
{
    uint8_t i, t, temp;
    uint8_t sign = 0;
    uint32_t total, int_part, dec_part;
    uint16_t div = 1;
    uint8_t char_width = 6; // 6x8字体宽度

    if (sizey != 8)
        return;
    if (int_len == 0 || int_len > 9)
        return;
    if (dec_len == 0 || dec_len > 6)
        return;

    /* 处理符号 */
    if (num < 0.0f)
    {
        sign = 1;
        num = -num;
    }

    /* 计算 10^dec_len */
    for (i = 0; i < dec_len; i++)
        div *= 10;

    /* 整体转为整数（四舍五入），避免多次浮点除法 */
    total = (uint32_t)(num * (float)div + 0.5f);

    /* 分离整数和小数 */
    int_part = total / div;
    dec_part = total % div;

    /* 显示符号 */
    if (sign)
    {
        OLED_ShowChar(x, y, '-', sizey);
        x += char_width;
    }

    /* 显示整数部分（复用OLED_ShowNum的前导零隐藏逻辑，右对齐） */
    uint8_t enshow = 0;
    for (t = 0; t < int_len; t++)
    {
        temp = (int_part / oled_pow(10, int_len - t - 1)) % 10;
        if (enshow == 0 && t < (int_len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + char_width * t, y, ' ', sizey);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + char_width * t, y, temp + '0', sizey);
    }

    /* 小数点 */
    OLED_ShowChar(x + char_width * int_len, y, '.', sizey);

    /* 小数部分（前导零必须显示） */
    for (t = 0; t < dec_len; t++)
    {
        temp = (dec_part / oled_pow(10, dec_len - t - 1)) % 10;
        OLED_ShowChar(x + char_width * (int_len + 1 + t), y, temp + '0', sizey);
    }
}


// 显示浮点数（紧凑模式，自动宽度，不补前导空格）
void OLED_ShowFloatNumCompact(uint8_t x, uint8_t y, float num, uint8_t dec_len, uint8_t sizey)
{
    uint8_t i, t, temp;
    uint8_t sign = 0;
    uint32_t total, int_part, dec_part;
    uint16_t div = 1;
    uint8_t char_width = 6;
    uint8_t int_len = 1;
    uint32_t temp_int;

    if (sizey != 8)
        return;
    if (dec_len > 6)
        return;

    if (num < 0.0f)
    {
        sign = 1;
        num = -num;
    }

    for (i = 0; i < dec_len; i++)
        div *= 10;
    total = (uint32_t)(num * (float)div + 0.5f);
    int_part = total / div;
    dec_part = total % div;

    /* 计算整数部分实际位数 */
    temp_int = int_part;
    if (temp_int == 0)
        int_len = 1;
    else
    {
        int_len = 0;
        while (temp_int > 0)
        {
            int_len++;
            temp_int /= 10;
        }
    }

    /* 显示符号 */
    if (sign)
    {
        OLED_ShowChar(x, y, '-', sizey);
        x += char_width;
    }

    /* 显示整数部分 */
    for (t = 0; t < int_len; t++)
    {
        temp = (int_part / oled_pow(10, int_len - t - 1)) % 10;
        OLED_ShowChar(x + char_width * t, y, temp + '0', sizey);
    }

    /* 小数点 */
    OLED_ShowChar(x + char_width * int_len, y, '.', sizey);

    /* 小数部分 */
    for (t = 0; t < dec_len; t++)
    {
        temp = (dec_part / oled_pow(10, dec_len - t - 1)) % 10;
        OLED_ShowChar(x + char_width * (int_len + 1 + t), y, temp + '0', sizey);
    }
}

// 显示一个字符串
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t sizey)
{
    uint8_t j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j++], sizey);
        if (sizey == 8)
            x += 6;
        else
            x += sizey / 2;
    }
}
