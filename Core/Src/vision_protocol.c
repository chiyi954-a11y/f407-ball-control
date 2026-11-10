#include "vision_protocol.h"
#include "usart.h"
#include "dma.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 * 全局变量
 * ============================================================ */
__IO bool          vision_frame_ready = false;
__IO VisionFrame_t vision_frame;
__IO uint8_t       vision_rx_buf[VISION_BUF_LEN] = {0};
__IO uint16_t      vision_rx_len = 0;

/* ============================================================
 * 解析状态机
 * ============================================================ */
static VisionParserState_t parser_state = VSTATE_SYNC1;
static uint16_t parser_idx = 0;
static uint8_t  parser_type = 0;
static uint8_t  parser_length = 0;
static uint8_t  parser_buf[VISION_BUF_LEN] = {0};

/* ============================================================
 * 内部辅助
 * ============================================================ */
static uint8_t Vision_CalcChecksum(const uint8_t *data, uint16_t len)
{
    uint8_t cs = 0;
    for (uint16_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

static int16_t ReadInt16BE(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t ReadUint16BE(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static void ParseBasicTarget(const uint8_t *p, VisionTarget_Basic_t *t)
{
    t->cx         = ReadInt16BE(p + 0);
    t->cy         = ReadInt16BE(p + 2);
    t->width      = ReadUint16BE(p + 4);
    t->height     = ReadUint16BE(p + 6);
    t->tube_mm    = -1;   /* 老二进制帧里没有毫米位置 */
    t->confidence = 0;
}

static void ParseExtTarget(const uint8_t *p, VisionTarget_Ext_t *t)
{
    t->cx         = ReadInt16BE(p + 0);
    t->cy         = ReadInt16BE(p + 2);
    t->width      = ReadUint16BE(p + 4);
    t->height     = ReadUint16BE(p + 6);
    t->angle      = ReadInt16BE(p + 8);
    t->class_id   = p[10];
    t->confidence = p[11];
}

/* ============================================================
 * ASCII 行解析：$B,<px>,<py>,<mm>,<conf>#
 * ------------------------------------------------------------
 * K230 每帧写一行，字节流会在任意位置被 IDLE 中断切开，
 * 所以这里自己攒缓冲：看到 '$' 开始收，'#' 或换行结束。
 * ============================================================ */
#define ASCII_LINE_MAX   48

static char     ascii_line[ASCII_LINE_MAX];
static uint16_t ascii_len    = 0;
static bool     ascii_active = false;

/* 把 "B,123,456,250,100" 解析成四个整数，成功返回 true */
static bool Vision_ParseAsciiLine(const char *line, uint16_t len, VisionFrame_t *out)
{
    int32_t  v[4];
    int      n         = 0;
    int32_t  cur       = 0;
    bool     neg       = false;
    bool     have_digit= false;

    /* 只认 $B,... 这一种 */
    if (len < 4 || line[0] != 'B' || line[1] != ',') return false;

    /* i == len 时塞一个假逗号，收尾最后一个数 */
    for (uint16_t i = 2; i <= len; i++)
    {
        char c = (i < len) ? line[i] : ',';

        if (c == ',')
        {
            if (!have_digit || n >= 4) return false;
            v[n++] = neg ? -cur : cur;
            cur = 0; neg = false; have_digit = false;
        }
        else if ((c == '-') && !have_digit && !neg)
        {
            neg = true;
        }
        else if (c >= '0' && c <= '9')
        {
            cur = cur * 10 + (int32_t)(c - '0');
            if (cur > 100000) return false;     /* 明显是乱码，丢掉 */
            have_digit = true;
        }
        else
        {
            return false;
        }
    }

    if (n != 4) return false;

    memset(out, 0, sizeof(*out));
    out->type                  = VISION_BASIC;
    out->data.basic.cx         = (int16_t)v[0];
    out->data.basic.cy         = (int16_t)v[1];
    out->data.basic.width      = 0;
    out->data.basic.height     = 0;
    out->data.basic.tube_mm    = (int16_t)v[2];
    out->data.basic.confidence = (int16_t)v[3];
    return true;
}

/* 从字节流里攒行；攒够一行有效数据返回 true */
static bool Vision_ParseAsciiStream(const uint8_t *data, uint16_t len, VisionFrame_t *out)
{
    bool got = false;

    for (uint16_t i = 0; i < len; i++)
    {
        char c = (char)data[i];

        if (c == '$')
        {
            ascii_active = true;
            ascii_len    = 0;
            continue;
        }
        if (!ascii_active) continue;

        if (c == '#' || c == '\n' || c == '\r')
        {
            ascii_active = false;
            if (c == '#' || c == '\n')
            {
                if (Vision_ParseAsciiLine(ascii_line, ascii_len, out)) got = true;
            }
            ascii_len = 0;
            continue;
        }

        if (ascii_len >= (ASCII_LINE_MAX - 1))
        {
            ascii_active = false;       /* 太长，肯定不是合法行 */
            ascii_len    = 0;
            continue;
        }
        ascii_line[ascii_len++] = c;
    }

    return got;
}

/* ============================================================
 * 状态机解析
 * ============================================================ */
bool Vision_Parse(const uint8_t *data, uint16_t len, VisionFrame_t *out)
{
    if (data == NULL || out == NULL) return false;

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];

        switch (parser_state)
        {
            case VSTATE_SYNC1:
                if (byte == VISION_FRAME_SOF1) {
                    parser_buf[0] = byte;
                    parser_state = VSTATE_SYNC2;
                }
                break;

            case VSTATE_SYNC2:
                if (byte == VISION_FRAME_SOF2) {
                    parser_buf[1] = byte;
                    parser_state = VSTATE_TYPE;
                } else if (byte != VISION_FRAME_SOF1) {
                    parser_state = VSTATE_SYNC1;
                }
                break;

            case VSTATE_TYPE:
                parser_type = byte;
                parser_buf[2] = byte;
                parser_state = VSTATE_LENGTH;
                break;

            case VSTATE_LENGTH:
                parser_length = byte;
                if (parser_length > VISION_BUF_LEN - 5) {
                    parser_state = VSTATE_SYNC1;
                } else {
                    parser_buf[3] = byte;
                    parser_idx = 4;
                    parser_state = (parser_length > 0)
                                 ? VSTATE_PAYLOAD
                                 : VSTATE_CHECKSUM;
                }
                break;

            case VSTATE_PAYLOAD:
                parser_buf[parser_idx++] = byte;
                if ((parser_idx - 4) >= parser_length) {
                    parser_state = VSTATE_CHECKSUM;
                }
                break;

            case VSTATE_CHECKSUM:
            {
                uint8_t calc_cs = Vision_CalcChecksum(parser_buf, parser_idx);
                if (byte == calc_cs)
                {
                    memset(out, 0, sizeof(VisionFrame_t));

                    switch (parser_type)
                    {
                        case VISION_TYPE_BASIC:
                            if (parser_length >= 8) {
                                out->type = VISION_BASIC;
                                ParseBasicTarget(&parser_buf[4], &out->data.basic);
                            }
                            break;

                        case VISION_TYPE_EXTENDED:
                            if (parser_length >= 12) {
                                out->type = VISION_EXT;
                                ParseExtTarget(&parser_buf[4], &out->data.extended);
                            }
                            break;

                        case VISION_TYPE_MULTI:
                            if (parser_length >= 1) {
                                uint8_t count = parser_buf[4];
                                if (count > VISION_MAX_TARGETS)
                                    count = VISION_MAX_TARGETS;
                                out->type = VISION_MULTI;
                                out->data.multi.count = count;
                                for (uint8_t t = 0; t < count; t++) {
                                    ParseBasicTarget(
                                        &parser_buf[5 + t * 8],
                                        &out->data.multi.targets[t]);
                                }
                            }
                            break;

                        case VISION_TYPE_DEBUG_TEXT:
                        {
                            uint16_t copy_len = parser_length < (VISION_BUF_LEN - 5)
                                              ? parser_length
                                              : (VISION_BUF_LEN - 6);
                            out->type = VISION_DEBUG;
                            memcpy(out->data.debug_text, &parser_buf[4], copy_len);
                            out->data.debug_text[copy_len] = '\0';
                            break;
                        }

                        default:
                            break;
                    }

                    parser_state = VSTATE_SYNC1;
                    return true;
                }
                parser_state = VSTATE_SYNC1;
                break;
            }
        }
    }
    return false;
}

/* ============================================================
 * 公开 API
 * ============================================================ */

/**
  * @brief  启动 USART1 视觉接收（DMA + IDLE）
  */
void Vision_UART_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart1, (uint8_t *)vision_rx_buf, VISION_BUF_LEN);

    Vision_ResetParser();
}

/**
  * @brief  USART1 IDLE 中断里调用
  */
void Vision_OnRxIdle(uint16_t len)
{
    if (len == 0) return;

    VisionFrame_t tmp;

    /* 现在的 K230 发 ASCII 行，先试这个 */
    if (Vision_ParseAsciiStream((const uint8_t *)vision_rx_buf, len, &tmp))
    {
        vision_frame = tmp;
        vision_frame_ready = true;
        return;
    }

    /* 老二进制帧仍然兼容 */
    if (Vision_Parse((uint8_t *)vision_rx_buf, len, &tmp))
    {
        vision_frame = tmp;
        vision_frame_ready = true;
    }
}

/**
  * @brief  取一帧（非阻塞，读后清标志）
  */
bool Vision_GetFrame(VisionFrame_t *out)
{
    if (vision_frame_ready && out) {
        *out = vision_frame;
        vision_frame_ready = false;
        return true;
    }
    return false;
}

void Vision_ResetParser(void)
{
    parser_state  = VSTATE_SYNC1;
    parser_idx    = 0;
    parser_type   = 0;
    parser_length = 0;

    ascii_active  = false;
    ascii_len     = 0;

    vision_frame_ready = false;
    memset((void *)&vision_frame, 0, sizeof(vision_frame));
}

int Vision_TargetToString(const VisionFrame_t *f, char *buf, int maxlen)
{
    if (!f || !buf) return 0;

    switch (f->type)
    {
        case VISION_BASIC:
            return snprintf(buf, maxlen,
                "Basic: px=(%d,%d) mm=%d conf=%d",
                f->data.basic.cx, f->data.basic.cy,
                f->data.basic.tube_mm, f->data.basic.confidence);

        case VISION_EXT:
            return snprintf(buf, maxlen,
                "Ext: cx=%d cy=%d w=%u h=%u angle=%d cls=%u conf=%u%%",
                f->data.extended.cx, f->data.extended.cy,
                f->data.extended.width, f->data.extended.height,
                f->data.extended.angle,
                f->data.extended.class_id, f->data.extended.confidence);

        case VISION_MULTI:
        {
            int pos = snprintf(buf, maxlen, "Multi[%u]:", f->data.multi.count);
            for (uint8_t i = 0; i < f->data.multi.count && pos < maxlen - 1; i++) {
                pos += snprintf(buf + pos, maxlen - pos,
                    " T%d(%d,%d,%u,%u)",
                    i,
                    f->data.multi.targets[i].cx,
                    f->data.multi.targets[i].cy,
                    f->data.multi.targets[i].width,
                    f->data.multi.targets[i].height);
            }
            return pos;
        }

        case VISION_DEBUG:
            return snprintf(buf, maxlen, "Debug: %s", f->data.debug_text);

        case VISION_NONE:
        default:
            return snprintf(buf, maxlen, "No frame");
    }
}
