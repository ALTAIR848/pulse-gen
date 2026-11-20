/**
 * @file    oled.c
 * @brief   0.96寸 OLED (SSD1306) 驱动 - 4线软件SPI
 *          移植自核心板例程10：GPIO 由 MX_GPIO_Init 配置，此处不再重复初始化引脚
 */
#include "oled.h"
#include "oledfont.h"

/*
 * 显存格式：每个字节表示8个点, 128列x8页, 高位在上
 * g_oled_gram[0][0].bit0 即坐标(0,0)
 */
static uint8_t g_oled_gram[128][8];

/**
 * @brief  向OLED写入一个字节（软件SPI, MSB先行）
 * @param  data: 要写入的字节
 * @param  cmd:  0命令 / 1数据
 */
static void oled_wr_byte(uint8_t data, uint8_t cmd)
{
    uint8_t i;

    OLED_DC(cmd);
    OLED_CS(0);

    for (i = 0; i < 8; i++)
    {
        OLED_SCLK(0);

        if (data & 0x80)
        {
            OLED_SDIN(1);
        }
        else
        {
            OLED_SDIN(0);
        }

        OLED_SCLK(1);
        data <<= 1;
    }

    OLED_CS(1);
    OLED_DC(1);
}

/**
 * @brief  刷新显存到OLED
 */
void oled_refresh_gram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        oled_wr_byte(0xb0 + i, OLED_CMD);   /* 设置页地址(0~7) */
        oled_wr_byte(0x00, OLED_CMD);       /* 列低地址 */
        oled_wr_byte(0x10, OLED_CMD);       /* 列高地址 */

        for (n = 0; n < 128; n++)
        {
            oled_wr_byte(g_oled_gram[n][i], OLED_DATA);
        }
    }
}

/**
 * @brief  开启OLED显示
 */
void oled_display_on(void)
{
    oled_wr_byte(0X8D, OLED_CMD);
    oled_wr_byte(0X14, OLED_CMD);
    oled_wr_byte(0XAF, OLED_CMD);
}

/**
 * @brief  关闭OLED显示
 */
void oled_display_off(void)
{
    oled_wr_byte(0X8D, OLED_CMD);
    oled_wr_byte(0X10, OLED_CMD);
    oled_wr_byte(0XAE, OLED_CMD);
}

/**
 * @brief  清屏
 */
void oled_clear(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            g_oled_gram[n][i] = 0X00;
        }
    }

    oled_refresh_gram();
}

/**
 * @brief  画点
 * @param  x: 0~127, y: 0~63, dot: 1亮 0灭
 */
void oled_draw_point(uint8_t x, uint8_t y, uint8_t dot)
{
    uint8_t pos, bx, temp = 0;

    if (x > 127 || y > 63)
    {
        return;
    }

    pos = y / 8;
    bx = y % 8;
    temp = 1 << bx;

    if (dot)
    {
        g_oled_gram[x][pos] |= temp;
    }
    else
    {
        g_oled_gram[x][pos] &= ~temp;
    }
}

/**
 * @brief  区域填充, 需 x1<=x2, y1<=y2
 */
void oled_fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot)
{
    uint8_t x, y;

    for (x = x1; x <= x2; x++)
    {
        for (y = y1; y <= y2; y++)
        {
            oled_draw_point(x, y, dot);
        }
    }

    oled_refresh_gram();
}

/**
 * @brief  指定位置显示一个字符
 * @param  x: 0~127, y: 0~63, size: 12/16/24, mode: 0非叠加 1叠加
 */
void oled_show_char(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    const uint8_t *pfont = 0;
    uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);

    chr = chr - ' ';

    if (size == 12)
    {
        pfont = oled_asc2_1206[chr];
    }
    else if (size == 16)
    {
        pfont = oled_asc2_1608[chr];
    }
    else if (size == 24)
    {
        pfont = oled_asc2_2412[chr];
    }
    else
    {
        return;
    }

    for (t = 0; t < csize; t++)
    {
        temp = pfont[t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
            {
                oled_draw_point(x, y, mode);
            }
            else
            {
                oled_draw_point(x, y, !mode);
            }

            temp <<= 1;
            y++;

            if ((y - y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/**
 * @brief  m^n
 */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
    {
        result *= m;
    }

    return result;
}

/**
 * @brief  显示len位数字
 */
void oled_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                oled_show_char(x + (size / 2) * t, y, ' ', size, 1);
                continue;
            }
            else
            {
                enshow = 1;
            }
        }

        oled_show_char(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}

/**
 * @brief  显示字符串
 */
void oled_show_string(uint8_t x, uint8_t y, const char *p, uint8_t size)
{
    while ((*p <= '~') && (*p >= ' '))
    {
        if (x > (128 - (size / 2)))
        {
            x = 0;
            y += size;
        }

        if (y > (64 - size))
        {
            y = x = 0;
            oled_clear();
        }

        oled_show_char(x, y, *p, size, 1);
        x += size / 2;
        p++;
    }
}

/**
 * @brief  初始化OLED(SSD1306)
 * @note   引脚已由 MX_GPIO_Init 配置为推挽输出, 此处只做复位与寄存器初始化
 */
void oled_init(void)
{
    OLED_RST(0);
    HAL_Delay(100);
    OLED_RST(1);

    oled_wr_byte(0xAE, OLED_CMD);   /* 关闭显示 */
    oled_wr_byte(0xD5, OLED_CMD);   /* 设置时钟分频/振荡频率 */
    oled_wr_byte(80, OLED_CMD);
    oled_wr_byte(0xA8, OLED_CMD);   /* 设置驱动路数 */
    oled_wr_byte(0X3F, OLED_CMD);   /* 1/64 duty */
    oled_wr_byte(0xD3, OLED_CMD);   /* 设置显示偏移 */
    oled_wr_byte(0X00, OLED_CMD);
    oled_wr_byte(0x40, OLED_CMD);   /* 设置显示起始行 */
    oled_wr_byte(0x8D, OLED_CMD);   /* 电荷泵设置 */
    oled_wr_byte(0x14, OLED_CMD);   /* 开启电荷泵 */
    oled_wr_byte(0x20, OLED_CMD);   /* 内存地址模式 */
    oled_wr_byte(0x02, OLED_CMD);   /* 页地址模式 */
    oled_wr_byte(0xA1, OLED_CMD);   /* 段重定义, 0->127 */
    oled_wr_byte(0xC8, OLED_CMD);   /* COM扫描方向, 反向 */
    oled_wr_byte(0xDA, OLED_CMD);   /* COM硬件引脚配置 */
    oled_wr_byte(0x12, OLED_CMD);
    oled_wr_byte(0x81, OLED_CMD);   /* 对比度设置 */
    oled_wr_byte(0xEF, OLED_CMD);
    oled_wr_byte(0xD9, OLED_CMD);   /* 预充电周期 */
    oled_wr_byte(0xf1, OLED_CMD);
    oled_wr_byte(0xDB, OLED_CMD);   /* VCOMH电压倍率 */
    oled_wr_byte(0x30, OLED_CMD);
    oled_wr_byte(0xA4, OLED_CMD);   /* 正常显示(非全亮) */
    oled_wr_byte(0xA6, OLED_CMD);   /* 正常显示(非反色) */
    oled_wr_byte(0xAF, OLED_CMD);   /* 开启显示 */

    oled_clear();
}
