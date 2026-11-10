#ifndef INC_EMM_V5_H_
#define INC_EMM_V5_H_

#include "usart.h"      /* huart3 的声明 */
#include <stdbool.h>

#define EMM_ADDR              1       /* 驱动器地址 */
#define EMM_CHECKSUM          0x6B    /* Emm_V5 固定校验字节 */
#define EMM_PULSES_PER_REV    3200    /* 16 细分：一圈 3200 脉冲 */
#define EMM_RX_BUF_LEN        16      /* 应答缓冲区 */

/* 读取系统参数的功能码 */
typedef enum {
    S_VBUS  = 5,
    S_CBUS  = 6,
    S_CPHA  = 7,
    S_ENCO  = 8,
    S_CLKC  = 9,
    S_ENCL  = 10,
    S_CLKI  = 11,
    S_TPOS  = 12,
    S_SPOS  = 13,
    S_VEL   = 14,
    S_CPOS  = 15,
    S_PERR  = 16,
    S_VBAT  = 17,
    S_TEMP  = 18,
    S_FLAG  = 19,
    S_OFLAG = 20,
} SysParams_t;

/* 应答接收状态（USART3 + DMA + IDLE） */
extern volatile bool     emm_rx_frame_flag;
extern volatile uint8_t  emm_rx_buf[EMM_RX_BUF_LEN];
extern volatile uint16_t emm_rx_len;

/* 接收初始化 / 中断里调用 / 等应答 */
void Emm_V5_Rx_Init(void);
void Emm_V5_OnRxIdle(uint16_t len);
bool Emm_V5_WaitAck(uint32_t timeout_ms);

/* 运动控制 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                        uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                        uint8_t acc, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);

/* 回零 / 清零 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);

/* 读取参数 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);

/* 读回驱动器里的真实当前位置（脉冲）。成功返回 true。
 * 【为什么必须有这个】MCU 复位后 current_angle_pulses 从 0 开始算，
 * 可平台实际停在哪儿一点没变——于是"角度 0"每次上电都指向不同的物理位置，
 * LEVELOFF 也就跟着每次含义不同。读回真实位置才能让基准稳定。 */
bool Emm_V5_Read_CPos(int32_t *out);

/* 裸命令：只发 [addr][code][0x6B]，用来从上位机试探驱动器支持哪些命令码，
 * 不用为每一种猜测重新烧录一次固件。 */
void Emm_V5_Raw(uint8_t code);

#endif /* INC_EMM_V5_H_ */
