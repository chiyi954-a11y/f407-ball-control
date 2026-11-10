#include "emm_v5.h"

/* ============================================================
 * 接收状态
 * ============================================================ */
volatile bool     emm_rx_frame_flag = false;
volatile uint8_t  emm_rx_buf[EMM_RX_BUF_LEN] = {0};
volatile uint16_t emm_rx_len = 0;

/* ============================================================
 * 发送
 * ------------------------------------------------------------
 * 所有命令共用一块缓冲。发送前等上一条 DMA 完成，避免覆盖。
 * 115200 下 13 字节约 1.1 ms，5 ms 超时足够。
 * ============================================================ */
static uint8_t emm_tx_buf[EMM_RX_BUF_LEN] = {0};

static void Emm_Send(uint16_t len)
{
    uint32_t start = HAL_GetTick();

    /* 等上一条发送完成，免得覆盖还在用的 emm_tx_buf */
    while ((huart3.gState != HAL_UART_STATE_READY) &&
           ((HAL_GetTick() - start) < 10U))
    {
        /* 等 */
    }

    /* 发送失败要重试，不能像以前那样静默丢掉——
     * 闭环每帧都发指令，丢一条平台位置就差一截，而且永远追不回来 */
    while (HAL_UART_Transmit_DMA(&huart3, emm_tx_buf, len) != HAL_OK)
    {
        if ((HAL_GetTick() - start) >= 20U) break;
    }
}

/* ============================================================
 * 接收
 * ============================================================ */

/**
  * @brief  启动 USART3 接收（DMA + IDLE）
  * @note   USART3 全局中断已在 usart.c 的 MspInit 中使能
  */
void Emm_V5_Rx_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart3, (uint8_t *)emm_rx_buf, EMM_RX_BUF_LEN);
}

/**
  * @brief  USART3 IDLE 中断里调用
  */
void Emm_V5_OnRxIdle(uint16_t len)
{
    emm_rx_len = len;
    emm_rx_frame_flag = true;
}

/**
  * @brief  等一帧应答（带超时，不死等）
  * @retval true=收到 false=超时
  */
bool Emm_V5_WaitAck(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!emm_rx_frame_flag)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return false;
        }
    }

    emm_rx_frame_flag = false;
    return true;
}

/* ============================================================
 * 运动控制
 * ============================================================ */

/**
  * @brief  位置模式
  * @param  addr  电机地址
  * @param  dir   0=CW 1=CCW
  * @param  vel   速度 0~3000 RPM
  * @param  acc   加速度 0~255，0=直接启动
  * @param  clk   脉冲数（3200 = 一圈）
  * @param  raF   false=相对上一次目标，true=绝对位置
  * @param  snF   多机同步标志
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                        uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    emm_tx_buf[0]  = addr;
    emm_tx_buf[1]  = 0xFD;
    emm_tx_buf[2]  = dir;
    emm_tx_buf[3]  = (uint8_t)(vel >> 8);
    emm_tx_buf[4]  = (uint8_t)(vel);
    emm_tx_buf[5]  = acc;
    emm_tx_buf[6]  = (uint8_t)(clk >> 24);
    emm_tx_buf[7]  = (uint8_t)(clk >> 16);
    emm_tx_buf[8]  = (uint8_t)(clk >> 8);
    emm_tx_buf[9]  = (uint8_t)(clk);
    emm_tx_buf[10] = raF ? 1 : 0;
    emm_tx_buf[11] = snF ? 1 : 0;
    emm_tx_buf[12] = EMM_CHECKSUM;

    Emm_Send(13);
}

/**
  * @brief  速度模式（持续转，直到 Stop_Now）
  * @param  dir  0=CW 1=CCW
  * @param  vel  速度 0~3000 RPM
  * @param  acc  加速度 0~255，0=直接启动
  * @param  snF  多机同步标志
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                        uint8_t acc, bool snF)
{
    emm_tx_buf[0] = addr;
    emm_tx_buf[1] = 0xF6;
    emm_tx_buf[2] = dir;
    emm_tx_buf[3] = (uint8_t)(vel >> 8);
    emm_tx_buf[4] = (uint8_t)(vel);
    emm_tx_buf[5] = acc;
    emm_tx_buf[6] = snF ? 1 : 0;
    emm_tx_buf[7] = EMM_CHECKSUM;

    Emm_Send(8);
}

/**
  * @brief  立即停止
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    emm_tx_buf[0] = addr;
    emm_tx_buf[1] = 0xFE;
    emm_tx_buf[2] = 0x98;
    emm_tx_buf[3] = snF ? 1 : 0;
    emm_tx_buf[4] = EMM_CHECKSUM;

    Emm_Send(5);
}

/**
  * @brief  使能/失能
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    emm_tx_buf[0] = addr;
    emm_tx_buf[1] = 0xF3;
    emm_tx_buf[2] = 0xAB;
    emm_tx_buf[3] = state ? 1 : 0;
    emm_tx_buf[4] = snF ? 1 : 0;
    emm_tx_buf[5] = EMM_CHECKSUM;

    Emm_Send(6);
}

/**
  * @brief  触发回零
  * @param  o_mode  0=单圈就近回零 1=单圈方向回零 2=多圈无限位回零 3=多圈限位回零
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    emm_tx_buf[0] = addr;
    emm_tx_buf[1] = 0x9A;
    emm_tx_buf[2] = o_mode;
    emm_tx_buf[3] = snF ? 1 : 0;
    emm_tx_buf[4] = EMM_CHECKSUM;

    Emm_Send(5);
}

/**
  * @brief  设置当前位置为零点
  * @param  svF  true=存储到 Flash
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
    emm_tx_buf[0] = addr;
    emm_tx_buf[1] = 0x93;
    emm_tx_buf[2] = 0x88;
    emm_tx_buf[3] = svF ? 1 : 0;
    emm_tx_buf[4] = EMM_CHECKSUM;

    Emm_Send(5);
}

/**
  * @brief  当前位置清零（不改变零点标记）
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    emm_tx_buf[0] = addr;
    emm_tx_buf[1] = 0x0A;
    emm_tx_buf[2] = 0x6D;
    emm_tx_buf[3] = EMM_CHECKSUM;

    Emm_Send(4);
}

/**
  * @brief  读取系统参数
  * @note   驱动器有应答，可用 Emm_V5_WaitAck() 后读 emm_rx_buf
  */
void Emm_V5_Raw(uint8_t code)
{
    emm_tx_buf[0] = EMM_ADDR;
    emm_tx_buf[1] = code;
    emm_tx_buf[2] = EMM_CHECKSUM;

    Emm_Send(3);
}

bool Emm_V5_Read_CPos(int32_t *out)
{
    Emm_V5_Read_Sys_Params(EMM_ADDR, S_CPOS);

    if (!Emm_V5_WaitAck(50)) return false;

    /* 应答: addr, 0x1F, 0x2F, 数据..., 0x6B
     * 不同型号长度不一样：Emm42_V5 是 4 字节(32 位)，老款 2.6 是 2 字节。 */
    if (emm_rx_len < 7) return false;
    if (emm_rx_buf[1] != 0x1F || emm_rx_buf[2] != 0x2F) return false;

    if (out != NULL)
    {
        if (emm_rx_len >= 8)
        {
            *out = (int32_t)(((uint32_t)emm_rx_buf[3] << 24) |
                             ((uint32_t)emm_rx_buf[4] << 16) |
                             ((uint32_t)emm_rx_buf[5] << 8)  |
                              (uint32_t)emm_rx_buf[6]);
        }
        else
        {
            *out = (int32_t)(int16_t)(((uint16_t)emm_rx_buf[3] << 8) |
                                       (uint16_t)emm_rx_buf[4]);
        }
    }
    return true;
}

void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;

    emm_tx_buf[i++] = addr;
    emm_tx_buf[i++] = 0x1F;         /* 读参数命令 */

    switch (s)
    {
        case S_VBUS:  emm_tx_buf[i++] = 0x24; break;   /* 总线电压 */
        case S_CBUS:  emm_tx_buf[i++] = 0x26; break;   /* 总线电流 */
        case S_CPHA:  emm_tx_buf[i++] = 0x27; break;   /* 相电流 */
        case S_ENCO:  emm_tx_buf[i++] = 0x28; break;   /* 编码器值 */
        case S_CLKC:  emm_tx_buf[i++] = 0x29; break;   /* 时钟频率 */
        case S_ENCL:  emm_tx_buf[i++] = 0x2A; break;   /* 编码器线性值 */
        case S_CLKI:  emm_tx_buf[i++] = 0x2B; break;   /* 内部时钟 */
        case S_TPOS:  emm_tx_buf[i++] = 0x2C; break;   /* 目标位置 */
        case S_SPOS:  emm_tx_buf[i++] = 0x2D; break;   /* 实际位置 */
        case S_VEL:   emm_tx_buf[i++] = 0x2E; break;   /* 实际速度 */
        case S_CPOS:  emm_tx_buf[i++] = 0x2F; break;   /* 当前位置 */
        case S_PERR:  emm_tx_buf[i++] = 0x30; break;   /* 位置误差 */
        case S_VBAT:  emm_tx_buf[i++] = 0x31; break;   /* 电池电压 */
        case S_TEMP:  emm_tx_buf[i++] = 0x32; break;   /* 温度 */
        case S_FLAG:  emm_tx_buf[i++] = 0x3A; break;   /* 状态标志 */
        case S_OFLAG: emm_tx_buf[i++] = 0x3B; break;   /* 回零标志 */
        default: return;
    }

    emm_tx_buf[i++] = EMM_CHECKSUM;

    Emm_Send(i);
}
