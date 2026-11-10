/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include "vision_protocol.h"
#include "emm_v5.h"
#include "oled.h"
#include "ball_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EMM_VEL_RPM         20U     /* 位置模式转速：20 RPM ≈ 1067 脉冲/s */
#define TARGET_TUBE_MM      0      /* 目标位置：0 = 管子中心（= K230 屏上的 0cm） */
#define VISION_MM_OFFSET    -117   /* 视觉读数补偿(mm)：CX = K230读数 + 这个值。
                                     * 2026-11-08 重新标零后，K230 屏上从 "1~25cm" 变成
                                     * "-12~+12cm"，但串口发出来的还是 0~240 的绝对量。
                                     * 减 120 之后，CX 就和 K230 屏上的数一一对应（单位 mm）：
                                     * 左端 -120、中心 0、右端 +120。
                                     * 实测：左端 CX 144、0处 260、右端 367（偏移 +144 时），
                                     * 反推原始值 0 / 116 / 223，减 120 即得上面的值。
                                     * 用 -120 时球在中间读 -3，再补 3 得到 -117，中间读 0。 */
#define MOTOR_ENABLE        1      /* 0=只看不动（先验数据）；1=真闭环 */
#define MOTOR_DIR_INVERT    0       /* 极性开关。目前实测：正命令 -> 球往 +mm 走，
                                     * 与 ball_control.c 里 PD 项的符号一致，所以保持 0。
                                     * 要反转极性只改这一个宏（或 ball_control.c 里
                                     * command 最后的符号），不要在中间偷偷取反。 */
#define VISION_TIMEOUT_MS   300U    /* 超过这么久没帧，就把平台放平 */
#define DEBUG_REFRESH_MS    200U    /* 调试刷屏周期(ms)，5 Hz */
#define MOTOR_CALIB         0       /* 1=标定模式：把当前姿态存成驱动器零点（只做一次） */
#define MOTOR_TEST_ANGLE    0       /* 非0=开环测试：忽略视觉，平台直接停到这个角度(脉冲，可负) */
#define MOTOR_SWEEP         0       /* 1=开环扫角：0→151→0→-151 循环，测满行程到底能不能把球推动 */
#define OLED_ENABLE         1       /* 0=关掉屏幕刷新（每次要阻塞 25ms，会拖慢控制）；1=开 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static ballcontrol ctrl;                                /* 控制器全部状态，全局一份 */
static int32_t     current_angle_pulses = 0;            /* 已下发的绝对角度（脉冲） */
static float       target_tube_mm       = (float)TARGET_TUBE_MM; /* 目标位置(mm) */
/* 位置模式转速，上位机 MOTORRPM= 可在线改。
 * 20 RPM 时平台从 +120 转到 -120 要 4.6 秒，而球穿过整根管子只要 1.5 秒，
 * 刹车指令物理上来不及 —— 这是"怎么调都是极限环"的一个硬瓶颈。
 * 先给 60 RPM 试试（3200 脉冲/s），还嫌慢可以再往上加。 */
static float       host_motor_rpm       = 60.0f;
static uint32_t    last_frame_tick      = 0;            /* 最近一次收到有效帧的时刻 */
#if OLED_ENABLE
static uint32_t    debug_tick           = 0;            /* 调试刷屏计时 */
#endif
static uint32_t    vision_frames        = 0;            /* 已收帧数，用来判断 K230 通没通 */
static uint32_t    ctrl_frames          = 0;            /* 真正进了控制器的帧数，FPS 命令用它算控制频率 */
static bool        vision_lost          = true;         /* 是否已经进过超时保护 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 把控制器算出的角度真正发给电机。
 * MOTOR_ENABLE=0 时只记账不发指令，方便先验数据：屏上的 A 仍然跟着算出的角度走，
 * 但电机不动，球不会飞出去。 */
static void Platform_MoveTo(int32_t angle)
{
    int32_t delta = angle - current_angle_pulses;

    if (delta == 0) return;

#if MOTOR_ENABLE
    /* 方向极性：正角度 = dir 0。上板推一下球就知道对不对：
     * 球被拉回来 -> 对的；球越跑越远 -> 把 MOTOR_DIR_INVERT 改成 1 */
    uint8_t dir = (delta > 0) ? 0U : 1U;
#if MOTOR_DIR_INVERT
    dir ^= 1U;
#endif

    uint16_t vel = (uint16_t)((host_motor_rpm < 1.0f) ? 1.0f : host_motor_rpm);
    Emm_V5_Pos_Control(EMM_ADDR, dir, vel, 0,
                       (uint32_t)((delta > 0) ? delta : -delta), false, false);
#endif

    current_angle_pulses = angle;
}

/* 调试显示：128x64 用 12x16 的字最多放 4 行，只挑最关键的 4 个量 */
#if OLED_ENABLE
static void Debug_UpdateDisplay(void)
{
    OLED_Clear();

    OLED_ShowString2x(0, 0, "CX");      /* 滤波后的球位置(mm)，控制用的就是它 */
    OLED_ShowNum2x(0, 48, (int32_t)ctrl.position, 4);

    OLED_ShowString2x(2, 0, "TG");      /* 目标位置(mm) */
    OLED_ShowNum2x(2, 48, (int32_t)target_tube_mm, 4);

    OLED_ShowString2x(4, 0, "A");       /* 控制器算出的平台角度（脉冲） */
    OLED_ShowNum2x(4, 48, current_angle_pulses, 4);

    OLED_ShowString2x(6, 0, "V");       /* 球速(mm/s)，调参时看这个判断是不是冲过头 */
    OLED_ShowNum2x(6, 48, (int32_t)ctrl.speed_filter, 4);

    OLED_Refresh();
}
#endif

#if MOTOR_CALIB
/* 把驱动器的应答翻译成一句人话：状态 0x02 = 成功，0xE2 = 条件不满足 */
static void Motor_ShowAck(const char *title, bool got_reply, uint8_t status)
{
    OLED_Clear();
    OLED_ShowString2x(0, 0, title);

    if (!got_reply)
    {
        OLED_ShowString2x(2, 0, "NO REPLY");
        OLED_ShowString2x(4, 0, "CHECK WIRE");
    }
    else if (status == 0x02U)
    {
        OLED_ShowString2x(2, 0, "OK");
    }
    else if (status == 0xE2U)
    {
        OLED_ShowString2x(2, 0, "FAILED");
        OLED_ShowString2x(4, 0, "COND FAIL");
    }
    else
    {
        OLED_ShowString2x(2, 0, "ERR CODE");
        OLED_ShowNum2x(4, 0, (int32_t)status, 3);
    }

    OLED_Refresh();
}
#endif

/* =====================================================================
 * 上位机调参链路（USART2，PA2=TX / PA3=RX，115200 8N1）
 * ---------------------------------------------------------------------
 * 接线：STM32 PA2 ──> USB-TTL 的 RX
 *       STM32 PA3 <── USB-TTL 的 TX
 *       GND      ────  GND（必须共地）
 *
 * 命令（一行一条，回车/换行结尾，不分大小写）：
 *   KP=0.8          改一个参数，立刻生效，不用重编译
 *   GET             打印全部参数
 *   RST             恢复上电默认值
 *   STREAM=0 / 1    关 / 开遥测（默认开）
 *   HELP            命令提示
 *
 * 遥测（每 50ms 一行）：
 *   POS,TG,ERR,ANG  —— 当前坐标 / 目标坐标 / 误差(mm) / 平台角度(脉冲÷100)
 * ===================================================================== */
#define HOST_TELEM_MS    50U
#define HOST_LINE_MAX    48U

static uint8_t       host_rx_byte;               /* 中断里收的单字节 */
static char          host_line[HOST_LINE_MAX];   /* 攒一整行 */
static uint8_t       host_len;
static volatile bool host_line_ready;
static bool          host_stream_on = true;      /* 遥测默认开 */
static uint32_t      host_telem_tick;

typedef struct { const char *name; float *p; } host_param_t;

static const host_param_t host_params[] =
{
    { "KP",       &ctrl_param.kp            },
    { "KV",       &ctrl_param.kv            },
    { "FRICTION", &ctrl_param.friction      },
    { "MAXSPEED", &ctrl_param.max_speed     },
    { "KI",       &ctrl_param.ki            },
    { "POSF",     &ctrl_param.pos_filter    },
    { "SPEEDF",   &ctrl_param.speed_filter  },
    { "ACCEL",    &ctrl_param.profile_accel },
    { "MAXMEAS",  &ctrl_param.max_measured  },
    { "V0",       &ctrl_param.friction_v0   },
    { "DEADBAND", &ctrl_param.deadband      },
    { "IMAX",     &ctrl_param.i_max         },
    { "ISPEED",   &ctrl_param.i_speed       },
    { "KICKMAX",  &ctrl_param.kick_max      },
    { "KICKRATE", &ctrl_param.kick_rate     },
    { "STUCKMS",  &ctrl_param.stuck_ms      },
    { "MOVEEPS",  &ctrl_param.move_eps      },
    { "LIMIT",    &ctrl_param.angle_limit   },
    { "MAXSTEP",  &ctrl_param.max_step      },
    { "LEVELOFF", &ctrl_param.level_offset  },
    /* ---- 省一参考控制器 ---- */
    { "MODE",     &ctrl_param.mode          },
    { "REFACC",   &ctrl_param.ref_accel     },
    { "BRAKEBST", &ctrl_param.brake_boost   },
    { "BRKEN",    &ctrl_param.brake_enter   },
    { "BRKEX",    &ctrl_param.brake_exit    },
    { "IZONE",    &ctrl_param.i_zone        },
    { "IZSPEED",  &ctrl_param.i_zone_speed  },
    { "CBAND",    &ctrl_param.center_band   },
    { "CSPEED",   &ctrl_param.center_speed  },
    { "XBAND",    &ctrl_param.settle_band   },
    { "XSPEED",   &ctrl_param.settle_speed  },
    { "OUTGAIN",  &ctrl_param.out_gain      },
    /* ---- 借鉴 2026-H-balance（默认全关）---- */
    { "KDL",      &ctrl_param.kdl           },
    { "KD2",      &ctrl_param.kd2           },
    { "ASYM",     &ctrl_param.asym          },
    { "PREA",     &ctrl_param.pre_angle     },
    { "PREMS",    &ctrl_param.pre_ms        },
    /* 电机位置模式转速(RPM)，不在 ctrl_param 里，单独一个变量 */
    { "MOTORRPM", &host_motor_rpm           },
    /* 目标位置(mm)，在线改，免得每次换目标都要重烧 */
    { "TARGET",   &target_tube_mm           },
};

#define HOST_PARAM_CNT  (sizeof(host_params) / sizeof(host_params[0]))

/* 遥测用阻塞发送。一行 25 字节 @115200 只要 2.2ms，占 50ms 周期的 4%，
 * 完全承受得起。
 *
 * 【曾经改成 HAL_UART_Transmit_IT 非阻塞，已经回退】当时怀疑主循环有
 * 100ms 级停顿是被阻塞遥测拖的，实测站不住脚：Emm_Send 最坏 30ms、
 * 遥测 2.2ms，加起来也凑不出 100ms。而中断发送一旦因溢出/错误丢掉完成
 * 回调，huart2.gState 会永远卡在 BUSY_TX，之后连 HAL_UART_Transmit 的
 * 命令应答都直接返回 HAL_BUSY——整块板子对上位机"变哑"。为了 2ms 冒
 * 断链的险不值得。 */
static char host_telem_buf[80];

static void Host_Send(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 20);
}

/* 发 "标签+十进制有符号数+CRLF"，给 FPS / POS 这类诊断命令用 */
static void Host_SendUInt(const char *tag, int32_t v)
{
    char b[32];
    char tmp[12];
    int  n = 0;
    int  i = 0;

    while (tag[i] != '\0' && i < 16) b[i] = tag[i], i++;

    if (v < 0)
    {
        b[i++] = '-';
        /* 取绝对值。注意 INT32_MIN 直接取负会溢出，先转成 uint32 再取负 */
        v = (int32_t)(0U - (uint32_t)v);
    }

    if (v == 0) tmp[n++] = '0';
    while (v > 0) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n > 0 && i < 28) b[i++] = tmp[--n];

    b[i++] = '\r';
    b[i++] = '\n';
    b[i]   = '\0';
    Host_Send(b);
}

/* 【必须实现】串口一旦出现溢出(ORE)/噪声/帧错误，HAL 会在中断里主动把
 * 接收给关掉（UART_EndRxTransfer），之后板子就再也收不到任何命令了——
 * 现象就是"上电好好的，跑着跑着发 GET 没反应，必须按复位"。
 * 这里在错误回调里重新挂上接收，链路自己恢复。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        host_len = 0;
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart2, &host_rx_byte, 1);
    }
}

/* newlib-nano 的 printf 默认不带浮点支持，直接 %f 会链接失败或输出垃圾。
 * 这里手写格式化：整数 + 两位小数，看曲线足够。 */
static void Host_FmtFloat(char **p, float v)
{
    char     tmp[10];
    int      n = 0;
    int32_t  whole;
    int32_t  frac;

    if (v < 0.0f) { *(*p)++ = '-'; v = -v; }

    whole = (int32_t)v;
    frac  = (int32_t)((v - (float)whole) * 100.0f + 0.5f);
    if (frac > 99) frac = 99;

    if (whole == 0) tmp[n++] = '0';
    while (whole > 0) { tmp[n++] = (char)('0' + (whole % 10)); whole /= 10; }
    while (n > 0)     { *(*p)++ = tmp[--n]; }

    *(*p)++ = '.';
    *(*p)++ = (char)('0' + (frac / 10));
    *(*p)++ = (char)('0' + (frac % 10));
}

/* 一行遥测：POS,TG,ERR,ANG\n
 *
 *   I0 = 当前坐标 mm    (绝对位置，球实际在哪)
 *   I1 = 目标坐标 mm    (固定 120)
 *   I2 = 误差 mm        (I0 - I1)
 *   I3 = 平台角度       (脉冲 / 100，350 -> 3.50)
 *
 * 【VOFA+ 要开两张图】
 *   图A 只开 I0 + I1  -> Y 轴拖到 100~140，看球到没到目标线
 *   图B 只开 I2       -> 默认 -5~5 不用调，调参主要看这张
 */
static void Host_SendTelemetry(void)
{
    char *p = host_telem_buf;

    Host_FmtFloat(&p, ctrl.position);                         *p++ = ',';
    Host_FmtFloat(&p, target_tube_mm);                        *p++ = ',';
    Host_FmtFloat(&p, ctrl.position - target_tube_mm);        *p++ = ',';
    Host_FmtFloat(&p, (float)current_angle_pulses / 100.0f);  *p++ = '\n';

    HAL_UART_Transmit(&huart2, (uint8_t *)host_telem_buf,
                      (uint16_t)(p - host_telem_buf), 20);
}

static void Host_SendOneParam(uint8_t i)
{
    char        buf[40];
    char       *p = buf;
    const char *n = host_params[i].name;

    while (*n) *p++ = *n++;
    *p++ = '=';
    Host_FmtFloat(&p, *(host_params[i].p));
    *p++ = '\r'; *p++ = '\n';
    *p   = '\0';

    Host_Send(buf);
}

static void Host_ExecLine(char *s)
{
    char    *eq;
    uint8_t  i;

    /* 命令不分大小写 */
    for (i = 0; s[i] != '\0'; i++)
    {
        if (s[i] >= 'a' && s[i] <= 'z') s[i] = (char)(s[i] - 32);
    }

    if (strcmp(s, "GET") == 0)
    {
        for (i = 0; i < HOST_PARAM_CNT; i++) Host_SendOneParam(i);
        return;
    }
    if (strcmp(s, "HELP") == 0 || strcmp(s, "?") == 0)
    {
        Host_Send("NAME=VALUE | GET | RST | STREAM=0/1\r\n");
        return;
    }
    if (strcmp(s, "RST") == 0)
    {
        CtrlParam_Default();
        Host_Send("OK RST\r\n");
        return;
    }
    if (strcmp(s, "FPS") == 0)
    {
        /* 累计控制帧数。上位机隔一段时间问两次，差/时间就是真实控制频率。
         * 这个值决定了抖动能开到多快——控制帧跟不上，抖动就只是原地嗡嗡。 */
        Host_SendUInt("FPS=", (int32_t)ctrl_frames);
        return;
    }
    if (strcmp(s, "POS") == 0)
    {
        /* 驱动器里的真实位置 vs 程序自己记的角度。
         * 这两个数一旦对不上，就说明电机在失步——程序以为平台在动，
         * 实际它顶在机械限位上，之后所有控制都是假的。 */
        int32_t cpos = 0;
        bool    ok   = Emm_V5_Read_CPos(&cpos);

        Host_SendUInt("CPOS=", ok ? cpos : -99999);
        Host_SendUInt("ANG=",  current_angle_pulses);
        return;
    }
    if (strcmp(s, "DBG") == 0)
    {
        /* 读位置失败时，把驱动器原始应答整个倒出来看看：
         * 是没回、回了但不是 0x1F/0x2F、还是长度不对。 */
        uint8_t i;

        (void)Emm_V5_WaitAck(5);            /* 先清掉可能的残留帧 */
        Emm_V5_Read_Sys_Params(EMM_ADDR, S_CPOS);

        if (!Emm_V5_WaitAck(120)) { Host_Send("DBG TIMEOUT\r\n"); return; }

        Host_SendUInt("LEN=", (int32_t)emm_rx_len);
        for (i = 0; i < emm_rx_len && i < 12U; i++)
        {
            Host_SendUInt("B=", (int32_t)emm_rx_buf[i]);
        }
        return;
    }

    eq = strchr(s, '=');
    if (eq == NULL) { Host_Send("ERR SYNTAX\r\n"); return; }
    *eq = '\0';

    if (strcmp(s, "Q") == 0)
    {
        /* Q=54 -> 发命令码 0x36 并倒出驱动器应答。
         * 驱动器对不认识的命令会回 01 00 EE 6B，试码不用再烧固件。 */
        uint8_t i;
        uint8_t code = (uint8_t)atoi(eq + 1);

        (void)Emm_V5_WaitAck(5);
        Emm_V5_Raw(code);

        if (!Emm_V5_WaitAck(150)) { Host_Send("Q TIMEOUT\r\n"); return; }

        Host_SendUInt("LEN=", (int32_t)emm_rx_len);
        for (i = 0; i < emm_rx_len && i < 12U; i++)
        {
            Host_SendUInt("B=", (int32_t)emm_rx_buf[i]);
        }
        return;
    }

    if (strcmp(s, "STREAM") == 0)
    {
        host_stream_on = (atof(eq + 1) != 0.0);
        Host_Send(host_stream_on ? "OK STREAM=1\r\n" : "OK STREAM=0\r\n");
        return;
    }

    for (i = 0; i < HOST_PARAM_CNT; i++)
    {
        if (strcmp(s, host_params[i].name) == 0)
        {
            *(host_params[i].p) = (float)atof(eq + 1);
            Host_SendOneParam(i);
            return;
        }
    }

    Host_Send("ERR NAME\r\n");
}

/* HAL 的接收完成回调：每收一个字节进一次，攒够一行就置标志 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        char c = (char)host_rx_byte;

        if (c == '\r' || c == '\n')
        {
            /* 上一条还没被主循环取走，就直接覆盖：调参时以最新的命令为准，
             * 绝不能丢——丢了就会以为参数下发了，其实没生效。
             * 主循环取走时是关中断拷贝，不会读到半截。 */
            if (host_len > 0)
            {
                host_line[host_len] = '\0';
                host_line_ready     = true;
            }
            host_len = 0;
        }
        else if (host_len < (HOST_LINE_MAX - 1))
        {
            host_line[host_len++] = c;
        }
        else
        {
            host_len = 0;           /* 这行太长，整行丢掉 */
        }

        HAL_UART_Receive_IT(&huart2, &host_rx_byte, 1);
    }
}

static void Host_Init(void)
{
    host_len        = 0;
    host_line_ready = false;
    host_telem_tick = HAL_GetTick();

    HAL_UART_Receive_IT(&huart2, &host_rx_byte, 1);
    /* 【注意】这里不要发欢迎语。VOFA+ 的 FireWater 是按"第一帧"来定通道数的，
     * 一条不含逗号的文本会被当成 1 通道，后面 4 通道的遥测就全乱了。
     * 想确认链路通不通，发一个 HELP 看回显就行。 */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Vision_UART_Init();      /* USART1：K230 视觉帧（DMA + IDLE） */
  Emm_V5_Rx_Init();        /* USART3：驱动器应答（DMA + IDLE） */
  OLED_Init();             /* PB6/PB7 软件 I2C */

  /* ---- 1. 确认驱动器在线：读一次总线电压 ----
   * 【不能再 while(1) 死等】以前收不到应答就永久停在这里，连 USART2 遥测
   * 也一起没了，从电脑上看就是"板子彻底死了、一个字节都不回"，根本分不清
   * 是驱动器没供电还是 MCU 崩了。现在改成：边报警边重试，串口保持活着，
   * 上位机能一直看到 WARN EMM NO ACK，OLED 上也照常显示。 */
  (void)Emm_V5_WaitAck(20);                  /* 清掉可能残留的应答 */
  Emm_V5_Read_Sys_Params(EMM_ADDR, S_VBUS);
  {
      uint32_t warn_tick = HAL_GetTick();
      uint32_t wait_t0   = HAL_GetTick();

      /* 最多等 10 秒。等不到也继续往下跑：电机不动，但视觉和控制照常运算，
       * 上位机能看到 POS/ERR 在变，就能立刻区分"驱动器坏了"和"整块板子死了"。 */
      while (!Emm_V5_WaitAck(50) && (HAL_GetTick() - wait_t0) < 10000U)
      {
          if ((HAL_GetTick() - warn_tick) >= 500U)   /* 每 0.5s 报一次，别刷屏 */
          {
              warn_tick = HAL_GetTick();
              Host_Send("WARN EMM NO ACK\r\n");

              OLED_Clear();
              OLED_ShowString2x(0, 0, "EMM NO ACK");
              OLED_ShowString2x(2, 0, "USART3");
              OLED_ShowString2x(4, 0, "CHECK WIRE");
              OLED_Refresh();
          }
          Emm_V5_Read_Sys_Params(EMM_ADDR, S_VBUS);
      }
  }
  OLED_Clear();
  OLED_Refresh();

#if MOTOR_CALIB
  /* ---- 标定模式：把平台此刻的姿态存成驱动器零点（只做一次）----
   * 用法：断电 → 用手把平台摆到你要的水平位 → 上电 → 屏显 ZERO SAVED → 断电
   *      然后把 MOTOR_CALIB 改回 0 重新烧录 */
  (void)Emm_V5_WaitAck(20);
  Emm_V5_En_Control(EMM_ADDR, true, false);
  HAL_Delay(200);

  (void)Emm_V5_WaitAck(20);
  Emm_V5_Origin_Set_O(EMM_ADDR, true);      /* svF=true：写进驱动器，掉电不丢 */
  {
      bool    got = Emm_V5_WaitAck(500);
      uint8_t st  = (got && emm_rx_len >= 3U) ? (uint8_t)emm_rx_buf[2] : 0U;
      Motor_ShowAck("SAVE ZERO", got, st);
  }
  HAL_Delay(4000);

  OLED_Clear();
  OLED_ShowString2x(0, 0, "ZERO SAVED");
  OLED_ShowString2x(2, 0, "POWER OFF");
  OLED_ShowString2x(4, 0, "NOW");
  OLED_ShowString2x(6, 0, "CALIB=0");
  OLED_Refresh();
  while (1) { }
#endif

  /* ---- 2. 使能 ---- */
  (void)Emm_V5_WaitAck(20);
  Emm_V5_En_Control(EMM_ADDR, true, false);
  HAL_Delay(200);

  /* ---- 3. 上电回零：让平台回到机械零点，之后所有角度都以它为基准 ---- */
  (void)Emm_V5_WaitAck(20);
  Emm_V5_Origin_Trigger_Return(EMM_ADDR, 0, false);
  HAL_Delay(3000);                           /* 等回零走完，时长按机械行程量一下 */

  /* ---- 4. 回零后的位置记作 0° ---- */
  (void)Emm_V5_WaitAck(20);
  Emm_V5_Reset_CurPos_To_Zero(EMM_ADDR);
  HAL_Delay(50);

#if MOTOR_TEST_ANGLE != 0
  /* ---- 开环测试：平台固定到这个角度，不跑控制 ----
   * 用来单独确认“电机到底动没动”，跟视觉和参数全都无关。
   * 球放中间，上电后看球往哪边滚、滚不滚得动，然后断电改数值重烧。 */
  Platform_MoveTo((int32_t)(MOTOR_TEST_ANGLE));
  OLED_Clear();
  OLED_ShowString2x(0, 0, "OPEN LOOP");
  OLED_ShowNum2x(2, 0, (int32_t)(MOTOR_TEST_ANGLE), 4);
  OLED_Refresh();
  while (1) { }
#endif

#if MOTOR_SWEEP
  /* ---- 开环扫角：只用来看“平台到底动没动”，跟视觉和参数全都无关 ----
   * 屏上 OPEN 后面就是此刻命令的角度，每 2.5 秒换一个：
   *     0 → 76 → 151 → 0 → -76 → -151 → 循环
   * 盯着平台看，记下【命令到多少的时候平台开始真的抬起/放下】。
   * 如果一直到 151 都不动，就是驱动器/接线的问题；
   * 如果 0~76 之间有一段怎么变都平、然后突然抬起来，那段就是机械空程。 */
  {
      static const int32_t sweep[] = { 0, 151, 0, -151, 0 };
      uint32_t i;

      while (1)
      {
          for (i = 0; i < (sizeof(sweep) / sizeof(sweep[0])); i++)
          {
              uint32_t t0;
              Platform_MoveTo(sweep[i]);
              t0 = HAL_GetTick();

              while ((HAL_GetTick() - t0) < 4000U)
              {
                  OLED_Clear();
                  OLED_ShowString2x(0, 0, "OPEN");
                  OLED_ShowNum2x(0, 60, sweep[i], 4);
                  OLED_ShowString2x(2, 0, "M");
                  OLED_ShowNum2x(2, 48, Motor_ReadPos(), 4);
                  OLED_Refresh();
                  HAL_Delay(200);
              }
          }
      }
  }
#endif

  /* ---- 5. 控制层复位：和电机零点对齐 ---- */
  CtrlParam_Default();          /* 装载参数默认值（之后可由上位机在线改） */
  Host_Init();                  /* 打开 USART2 上位机链路 */
  current_angle_pulses = 0;
  last_frame_tick      = HAL_GetTick();
  BallControl_Init(&ctrl);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* ============= 1. 每收到一帧视觉数据跑一次控制 ============= */
    if (vision_frame_ready)
    {
        /* 帧在中断里写、主循环里读，拷贝时关一下中断，避免读到半帧 */
        __disable_irq();
        VisionFrame_t frame = vision_frame;
        vision_frame_ready  = false;
        __enable_irq();

        vision_frames++;    /* 只要收到一行就 +1，用来判断链路通没通 */

        /* tube_mm < 0 = 抓到了球但没算出管子坐标，这一帧不能用来控制 */
        if (frame.type == VISION_BASIC && frame.data.basic.tube_mm >= 0)
        {
            last_frame_tick = now;
            /* 注意参数顺序：目标在前、实测在后，都用毫米。
             * 实测值先加上视觉补偿，之后控制器里比较的就是真实位置了，
             * 屏上的 CX 显示的也是补偿后的值。 */
            int32_t desired = BallControl_Update(&ctrl,
                                                 target_tube_mm,
                                                 (float)(frame.data.basic.tube_mm
                                                         + VISION_MM_OFFSET),
                                                 now);
            ctrl_frames++;
            Platform_MoveTo(desired);
        }
    }

    /* ============= 2. 通信超时：长时间没帧就把平台放平，别拿过期坐标乱跑 ============= */
    if ((now - last_frame_tick) > VISION_TIMEOUT_MS)
    {
        if (!vision_lost)
        {
            vision_lost = true;
            BallControl_Reset(&ctrl);   /* 丢掉旧的 dt/位置，来帧后重新对齐 */
            /* 回水平 = 回到 LEVEL_OFFSET，不是 0 —— 0 只是编码器零点，
             * 平台真正水平的那个角度是 LEVEL_OFFSET。 */
            Platform_MoveTo((int32_t)ctrl_param.level_offset);
        }
    }
    else
    {
        vision_lost = false;
    }

    /* ============= 3. 上位机：执行命令 + 发遥测 =============
     * 【必须先把整行拷走再解析】Host_ExecLine 里每回一个应答都要
     * HAL_UART_Transmit 阻塞十几毫秒，这期间接收中断照常往 host_line
     * 里写下一条命令。以前是先清 host_line_ready 再把 host_line 传进去，
     * 于是解析到一半缓冲区被改掉，就读成了“别的参数名 + 别的数值”，
     * 表现为参数莫名其妙变成怪值（比如 DEADBAND 从 6 变成 60）。
     * 现在关中断拷一份出来，中断再改也不影响这次解析。 */
    if (host_line_ready)
    {
        static char cmd_buf[HOST_LINE_MAX];

        __disable_irq();
        strcpy(cmd_buf, host_line);
        host_line_ready = false;
        host_len        = 0;
        __enable_irq();

        Host_ExecLine(cmd_buf);
    }

    if (host_stream_on && (now - host_telem_tick) >= HOST_TELEM_MS)
    {
        host_telem_tick = now;
        Host_SendTelemetry();
    }

    /* ============= 4. 调试显示（阻塞约 20~30ms，OLED_ENABLE=0 时跳过） ============= */
#if OLED_ENABLE
    if ((now - debug_tick) >= DEBUG_REFRESH_MS)
    {
        debug_tick = now;
        Debug_UpdateDisplay();
    }
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: source line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
