#ifndef __BALL_CONTROL_H
#define __BALL_CONTROL_H

#include "main.h"

/* =====================================================================
 * 可调参数（运行时变量）
 * ---------------------------------------------------------------------
 * 原来这些是 #define，编译期就固化了，改一个数得重新编译烧录。
 * 现在改成变量，USART2 上位机可以边跑边改，看着曲线调。
 * 上电默认值在 ball_control.c 的 CtrlParam_Default() 里。
 * ===================================================================== */
typedef struct
{
    /* ---- 主调 ---- */
    float kp;              /* 位置增益 */
    float kv;              /* 速度增益（刹车） */
    float friction;        /* 静摩擦补偿 */
    float max_speed;       /* 限速上限 mm/s */
    float ki;

    /* ---- 一般不用动 ---- */
    float pos_filter;      /* 位置低通 */
    float speed_filter;    /* 速度低通 */
    float profile_accel;   /* 速度规划加速度 mm/s² */
    float max_measured;    /* 差分测速限幅 */
    float friction_v0;     /* 摩擦补偿淡出速度 */
    float deadband;        /* 中心死区 mm */
    float i_max;           /* 积分限幅 */
    float i_speed;         /* 积分泄放速度阈值 */
    float kick_max;        /* 顶一脚上限 */
    float kick_rate;       /* 顶力上涨速率 */
    float stuck_ms;        /* 卡住判定时间 */
    float move_eps;        /* 算作"动了"的位移 */
    float angle_limit;     /* 机械限位 */
    float max_step;        /* 每帧最大变化量 */
    float level_offset;    /* 平台水平补偿 */

    /* ---- 模式 ----
     * 0 = 早期自写的五项相加 PID
     * 1 = 2026 电赛 H 题江苏省一开源工程的"速度耦合轨迹控制器"（默认）
     *     来源：2026电赛H题-省一开源/F407bujin/Core/Src/main.c
     *     原工程用像素做单位，这里按 1mm ≈ 2.5px 换算成毫米。
     *     换算系数不确定，所以下面每个增益都能在线改，慢慢对。 */
    float mode;

    /* ---- 参考控制器参数（单位已换算成 mm）---- */
    float ref_accel;       /* 速度给定上升斜坡 mm/s²   原 300px/s² -> 120 */
    float brake_boost;     /* 刹车时速度环增益倍数      原 1.60 */
    float brake_enter;     /* 刹车距离 ≥ 这个×剩余距离 进刹车  原 0.85 */
    float brake_exit;      /* 刹车距离 < 这个×剩余距离 退刹车  原 0.55 */
    float i_zone;          /* 允许中心积分的误差范围 mm  原 25px -> 10 */
    float i_zone_speed;    /* 允许中心积分的速度上限 mm/s 原 35px/s -> 14 */
    float center_band;     /* 进入"稳定"的误差 mm       原 4px -> 1.6 */
    float center_speed;    /* 进入"稳定"的速度 mm/s     原 10px/s -> 4 */
    float settle_band;     /* 退出"稳定"的误差 mm       原 8px -> 3.2 */
    float settle_speed;    /* 退出"稳定"的速度 mm/s     原 25px/s -> 10 */
    float out_gain;        /* 输出总增益                原 1.50 */

    /* ---- 借鉴 2026-H-balance（github.com/good1example）---- */
    float kdl;             /* 线性速度阻尼 脉冲/(mm/s)，直接压在实测速度上 */
    float kd2;             /* 速度平方阻尼 脉冲/(mm/s)²，速度越快阻尼越大 */
    float asym;            /* 负向输出不对称系数，1.0 = 不补偿 */
    float pre_angle;       /* 卡住脱困的"预动作"初始偏置(脉冲)，0 = 关闭 */
    float pre_ms;          /* 预动作从 pre_angle 线性衰减到 0 的时长(ms) */
} ctrl_param_t;

extern ctrl_param_t ctrl_param;

void CtrlParam_Default(void);

typedef struct
{
uint8_t initialized;
uint32_t last_tick;
float position;
float last_position;
float speed_filter;
float speed_circle;
float speed_circle_celing;
float stop_distance;
float error;
float last_error;
float integral;
float kick;          /* 卡住时额外叠加的“顶一脚”力度(脉冲) */
float stuck_timer;   /* 已经“几乎没动”持续了多久(ms) */
float ref_pos;       /* 判断卡住用的参考位置(mm) */
float pre_timer;     /* 预动作已进行的时间(ms)；>= pre_ms 表示未激活 */
uint8_t brake_flag;
uint8_t settle_flag;
float command;
/* ---- 参考控制器专用状态 ---- */
float velocity_reference;  /* 速度给定(mm/s，带方向) */
float velocity_envelope;   /* 本帧允许的速度上限(带符号) */
float stopping_distance;   /* 刹车距离 mm */
}ballcontrol;

void BallControl_Init(ballcontrol  *controller);

int32_t BallControl_Update(ballcontrol *controller,float target_position,float measured_position,uint32_t tick);

void BallControl_Reset(ballcontrol *contorller);



















#endif