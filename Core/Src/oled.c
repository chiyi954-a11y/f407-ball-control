#include "oled.h"
#include "oled_font.h"
#include <stdbool.h>
#include <string.h>

/* ============================================================
 * 软件 I2C（PB6 = SCL，PB7 = SDA，开漏输出）
 * ============================================================ */
#define OLED_PORT       GPIOB
#define OLED_SCL_PIN    GPIO_PIN_6
#define OLED_SDA_PIN    GPIO_PIN_7

/* 半周期延时，时钟约 300~400 kHz；屏闪/花屏就加大这个值 */
#define OLED_I2C_DELAY_LOOPS    40U

/* 1 = 屏幕内容旋转 180°（装反了就把这个改成 0，或反过来） */
#define OLED_FLIP_180           1

static uint8_t oled_buf[OLED_PAGES][OLED_WIDTH];

static void OLED_I2C_Delay(void)
{
    for (volatile uint32_t i = 0; i < OLED_I2C_DELAY_LOOPS; i++)
    {
        __NOP();
    }
}

static void SCL_High(void) { HAL_GPIO_WritePin(OLED_PORT, OLED_SCL_PIN, GPIO_PIN_SET); }
static void SCL_Low(void)  { HAL_GPIO_WritePin(OLED_PORT, OLED_SCL_PIN, GPIO_PIN_RESET); }
static void SDA_High(void) { HAL_GPIO_WritePin(OLED_PORT, OLED_SDA_PIN, GPIO_PIN_SET); }
static void SDA_Low(void)  { HAL_GPIO_WritePin(OLED_PORT, OLED_SDA_PIN, GPIO_PIN_RESET); }

static void OLED_I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(OLED_PORT, &gpio);

    SCL_High();
    SDA_High();
}

static void I2C_Start(void)
{
    SDA_High();
    SCL_High();
    OLED_I2C_Delay();
    SDA_Low();
    OLED_I2C_Delay();
    SCL_Low();
    OLED_I2C_Delay();
}

static void I2C_Stop(void)
{
    SDA_Low();
    SCL_High();
    OLED_I2C_Delay();
    SDA_High();
    OLED_I2C_Delay();
}

static void I2C_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (byte & 0x80U) SDA_High();
        else              SDA_Low();

        OLED_I2C_Delay();
        SCL_High();
        OLED_I2C_Delay();
        SCL_Low();
        OLED_I2C_Delay();

        byte <<= 1;
    }

    /* 第 9 个时钟：释放 SDA，收 ACK（这里不判读） */
    SDA_High();
    OLED_I2C_Delay();
    SCL_High();
    OLED_I2C_Delay();
    SCL_Low();
    OLED_I2C_Delay();
}

static void OLED_WriteCmd(uint8_t cmd)
{
    I2C_Start();
    I2C_WriteByte(OLED_I2C_ADDR);
    I2C_WriteByte(0x00);        /* 命令 */
    I2C_WriteByte(cmd);
    I2C_Stop();
}

/* ============================================================
 * 初始化
 * ============================================================ */
void OLED_Init(void)
{
    OLED_I2C_GPIO_Init();
    HAL_Delay(100);

    OLED_WriteCmd(0xAE);        /* 关显示 */
    OLED_WriteCmd(0xD5);        /* 时钟分频 */
    OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8);        /* 复用率 1/64 */
    OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xD3);        /* 显示偏移 */
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40);        /* 起始行 0 */
    OLED_WriteCmd(0x8D);        /* 电荷泵 */
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(0x20);        /* 水平寻址 */
    OLED_WriteCmd(0x00);
#if OLED_FLIP_180
    OLED_WriteCmd(0xA0);        /* 左右方向：翻转显示 */
    OLED_WriteCmd(0xC0);        /* 上下方向：翻转显示 */
#else
    OLED_WriteCmd(0xA1);        /* 左右方向 */
    OLED_WriteCmd(0xC8);        /* 上下方向 */
#endif
    OLED_WriteCmd(0xDA);        /* COM 引脚配置 */
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81);        /* 对比度：调最大，户外/斜看更清楚 */
    OLED_WriteCmd(0xFF);
    OLED_WriteCmd(0xD9);        /* 预充电 */
    OLED_WriteCmd(0xF1);
    OLED_WriteCmd(0xDB);        /* VCOMH */
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0xA4);        /* 按 GDDRAM 显示 */
    OLED_WriteCmd(0xA6);        /* 正常显示 */
    OLED_WriteCmd(0x2E);        /* 关滚动 */
    OLED_WriteCmd(0xAF);        /* 开显示 */

    OLED_Clear();
    OLED_Refresh();
}

/* ============================================================
 * 显存操作
 * ============================================================ */
void OLED_Clear(void)
{
    memset(oled_buf, 0x00, sizeof(oled_buf));
}

void OLED_Fill(uint8_t data)
{
    memset(oled_buf, data, sizeof(oled_buf));
}

void OLED_ShowChar(uint8_t page, uint8_t x, char ch)
{
    if (page >= OLED_PAGES) return;

    /* 小写转大写，字库只到 0x5F */
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 32);
    if (ch < 0x20 || ch > 0x5F) ch = '?';

    uint8_t idx = (uint8_t)(ch - 0x20);

    for (uint8_t col = 0; col < 6; col++)
    {
        if ((uint16_t)x + col < OLED_WIDTH)
        {
            oled_buf[page][x + col] = OLED_F6x8[idx][col];
        }
    }
}

void OLED_ShowString(uint8_t page, uint8_t x, const char *str)
{
    if (str == NULL) return;

    while (*str != '\0')
    {
        if ((uint16_t)x + 6 > OLED_WIDTH) break;

        OLED_ShowChar(page, x, *str);
        x += 6;
        str++;
    }
}

void OLED_ShowNum(uint8_t page, uint8_t x, int32_t num, uint8_t width)
{
    char buf[12];
    uint8_t i;
    bool negative = (num < 0);
    uint32_t value = negative ? (uint32_t)(-num) : (uint32_t)num;

    if (width > 10) width = 10;
    if (width == 0) width = 1;

    i = width;
    buf[i] = '\0';

    do
    {
        buf[--i] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value > 0U && i > 0U);

    if (negative && i > 0U)
    {
        buf[--i] = '-';
    }

    while (i > 0U)
    {
        buf[--i] = ' ';
    }

    OLED_ShowString(page, x, buf);
}

/* ============================================================
 * 整屏刷新（8 页，每页 128 字节）
 * ============================================================ */
/* ============================================================
 * 2 倍字号（每个字符 12x16 像素，占两个 page）
 * 6x8 字模太小，侧看/远看认不出数字，调试信息一律用这个
 * ============================================================ */
static void OLED_BufPixel(uint8_t x, uint8_t y, uint8_t on)
{
    if ((x >= OLED_WIDTH) || (y >= (OLED_PAGES * 8U))) return;

    if (on != 0U) oled_buf[y >> 3][x] |= (uint8_t)(1U << (y & 7U));
    else          oled_buf[y >> 3][x] &= (uint8_t)(~(1U << (y & 7U)));
}

void OLED_ShowChar2x(uint8_t page, uint8_t x, char ch)
{
    uint8_t idx;
    uint8_t col;
    uint8_t row;

    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 32);
    if (ch < 0x20 || ch > 0x5F) ch = '?';

    idx = (uint8_t)(ch - 0x20);

    for (col = 0; col < 6U; col++)
    {
        uint8_t bits = OLED_F6x8[idx][col];

        for (row = 0; row < 8U; row++)
        {
            uint8_t on = (uint8_t)((bits >> row) & 1U);
            uint8_t px = (uint8_t)(x + 2U * col);
            uint8_t py = (uint8_t)(page * 8U + 2U * row);

            OLED_BufPixel(px,                py,                on);
            OLED_BufPixel((uint8_t)(px + 1U), py,                on);
            OLED_BufPixel(px,                (uint8_t)(py + 1U), on);
            OLED_BufPixel((uint8_t)(px + 1U), (uint8_t)(py + 1U), on);
        }
    }
}

void OLED_ShowString2x(uint8_t page, uint8_t x, const char *str)
{
    if (str == NULL) return;

    while (*str != '\0')
    {
        if ((uint16_t)x + 12U > OLED_WIDTH) break;

        OLED_ShowChar2x(page, x, *str);
        x += 12U;
        str++;
    }
}

void OLED_ShowNum2x(uint8_t page, uint8_t x, int32_t num, uint8_t width)
{
    char buf[12];
    uint8_t i;
    bool negative = (num < 0);
    uint32_t value = negative ? (uint32_t)(-num) : (uint32_t)num;

    if (width > 10U) width = 10U;
    if (width == 0U) width = 1U;

    i = width;
    buf[i] = '\0';

    do
    {
        buf[--i] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value > 0U && i > 0U);

    if (negative && i > 0U)
    {
        buf[--i] = '-';
    }

    while (i > 0U)
    {
        buf[--i] = ' ';
    }

    OLED_ShowString2x(page, x, buf);
}

void OLED_Refresh(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        OLED_WriteCmd((uint8_t)(0xB0 + page));
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);

        I2C_Start();
        I2C_WriteByte(OLED_I2C_ADDR);
        I2C_WriteByte(0x40);        /* 数据 */

        for (uint16_t x = 0; x < OLED_WIDTH; x++)
        {
            I2C_WriteByte(oled_buf[page][x]);
        }

        I2C_Stop();
    }
}
