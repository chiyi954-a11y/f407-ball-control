#ifndef __VISION_PROTOCOL_H
#define __VISION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* ============================================================
 * K230 视觉协议
 * ------------------------------------------------------------
 * 【现在用的】ASCII 行，每帧一行，115200 8N1：
 *
 *      $B,<px>,<py>,<mm>,<conf>#
 *
 *   px,py  球心在 640x480 画面里的像素坐标
 *   mm     球沿试管的位置，单位毫米，0..250（0.1cm 分辨率）；-1 = 这一帧没算出来
 *   conf   置信度 x100，0..100
 *
 * 例：$B,566,242,250,100#  表示球在管子 0 刻度那一端
 *
 * 【兼容保留】原来的 13 字节二进制帧（大端）：
 *   [0] 0xAA  [1] 0x55  [2] 0x01  [3] 0x08
 *   [4..5] cx int16   [6..7] target int16   [8..11] 0   [12] XOR[0..11]
 *
 * 接线：K230 的 IO9 (UART1_TX) -> PA10 (USART1_RX)，共地
 * ============================================================ */

#define VISION_FRAME_SOF1       0xAA
#define VISION_FRAME_SOF2       0x55

#define VISION_TYPE_BASIC       0x01
#define VISION_TYPE_EXTENDED    0x02
#define VISION_TYPE_MULTI       0x03
#define VISION_TYPE_DEBUG_TEXT  0xFE

#define VISION_MAX_TARGETS      8
#define VISION_BUF_LEN          256

typedef struct {
    int16_t  cx;        /* 球心 X（像素） */
    int16_t  cy;        /* 球心 Y（像素）；二进制老协议里这一路是“目标 X” */
    uint16_t width;     /* 二进制老协议才有，ASCII 行里为 0 */
    uint16_t height;
    int16_t  tube_mm;   /* 球沿试管的位置（毫米，0..250）；-1 = 这一帧没算出来 */
    int16_t  confidence;/* 置信度 x100；二进制老协议里为 0 */
} VisionTarget_Basic_t;

typedef struct {
    int16_t  cx;
    int16_t  cy;
    uint16_t width;
    uint16_t height;
    int16_t  angle;
    uint8_t  class_id;
    uint8_t  confidence;
} VisionTarget_Ext_t;

typedef enum {
    VISION_NONE  = 0x00,
    VISION_BASIC = 0x01,
    VISION_EXT   = 0x02,
    VISION_MULTI = 0x03,
    VISION_DEBUG = 0xFE,
} VisionFrameType_t;

typedef struct {
    VisionFrameType_t type;
    union {
        VisionTarget_Basic_t basic;
        VisionTarget_Ext_t   extended;
        struct {
            uint8_t              count;
            VisionTarget_Basic_t targets[VISION_MAX_TARGETS];
        } multi;
        char debug_text[VISION_BUF_LEN - 4];
    } data;
} VisionFrame_t;

typedef enum {
    VSTATE_SYNC1,
    VSTATE_SYNC2,
    VSTATE_TYPE,
    VSTATE_LENGTH,
    VSTATE_PAYLOAD,
    VSTATE_CHECKSUM,
} VisionParserState_t;

extern __IO bool          vision_frame_ready;
extern __IO VisionFrame_t vision_frame;
extern __IO uint8_t       vision_rx_buf[VISION_BUF_LEN];
extern __IO uint16_t      vision_rx_len;

void Vision_UART_Init(void);
void Vision_OnRxIdle(uint16_t len);
bool Vision_Parse(const uint8_t *data, uint16_t len, VisionFrame_t *out);
bool Vision_GetFrame(VisionFrame_t *out);
void Vision_ResetParser(void);
int  Vision_TargetToString(const VisionFrame_t *f, char *buf, int maxlen);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_PROTOCOL_H */
