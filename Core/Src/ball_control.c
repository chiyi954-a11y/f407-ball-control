#include "ball_control.h"
#include <math.h>
#include <string.h>

/* =====================================================================
 * 试管滚球平衡控制
 * ---------------------------------------------------------------------
 * 输入：球沿试管的位置(mm)        输出：平台角度(脉冲, ±CTRL_ANGLE_LIMIT)
 *
 * 符号约定（2026-11-07 用完整轨迹实测确认）：
 *   命令为正 -> 平台正转 -> 球往 +mm（右）滚
 *   命令为负 -> 平台反转 -> 球往 -mm（左）滚
 *   所以"球在目标右边"时命令必须为负，位置项带负号。
 *
 * 主算法来自 2026 电赛 H 题江苏省一开源工程
 *   （2026电赛H题-省一开源/F407bujin/Core/Src/main.c 的 BallBalance_Update）
 * 原工程用像素，这里按 1mm ≈ 2.5px 换算成毫米；换算系数不精确，
 * 所有增益都能用上位机在线改（KP=0.3 这样），对着曲线慢慢对。
 * ===================================================================== */

/* =====================================================================
 * 可调参数 —— 上电默认值
 * ===================================================================== */

/* 下面的默认值 = 2026 电赛 H 题江苏省一开源工程的原始常量。
 * 原工程用"像素"做单位，本机视觉直接给毫米，按 CTRL_PX_PER_MM 换算：
 *   · 增益类（脉冲/px、脉冲/(px·s)）：× CTRL_PX_PER_MM
 *   · 距离/速度类（px、px/s、px/s²）：÷ CTRL_PX_PER_MM
 * 换算系数不确定，嫌整体反应太弱就把 CTRL_PX_PER_MM 调大后重烧，
 * 或者直接在线改 KP/KV/KI。 */

/* ---------------- 主调（从上往下） ---------------- */
/* ==========================================================================
 * 2026-11-08 深夜：参数已恢复成"省一开源 F407bujin"的原始值
 * （本地路径 2026电赛H题-省一开源/F407bujin/Core/Src/main.c，1px 按 1mm 移植）。
 * 唯一偏离：ANGLE_LIMIT 用 120 而不是 151，因为本机平台行程到 ±120 就到底了。
 * 下面每行注释里写的"原 xx"是更早一版的记录，现在 DEF_ 的值本身就是开源原值。
 * ======================================================================== */
#define DEF_KP               0.60f  /* 位置增益 脉冲/mm。2026-11-09 用户要求全部增益归零 */
#define DEF_KV                0.00f  /* 速度增益。2026-11-08 实测：这是极限环的主要推手
                                     * ——球还没跑起来时规划速度已经给到 MAXSPEED，
                                     * KV·(v_ref−v) 直接是一个大倾角，把球踹出去。
                                     * 改用 KDL 做真正的速度阻尼后，KV 置 0。 */
#define DEF_FRICTION          0.0f  /* 参考算法不用这一项，保留位 */
#define DEF_MAX_SPEED        180.0f  /* 球最高速度 mm/s */
#define DEF_KI                0.00f  /* 中心积分 脉冲/(mm·s)。2026-11-09 用户要求全部增益归零。 */
/* ---------------------------------------------------------------- */

/* ---------------- 参考控制器的其余参数 ---------------- */
#define DEF_POS_FILTER       0.90f  /* 位置低通。2026-11-08 实测：0.45 在 11fps 下滞后明显 */

#define DEF_SPEED_FILTER     0.80f  /* 速度低通。★★这是本轮最重要的一个改动★★
                                     * 0.25 在 10.5fps 下时间常数 ≈0.33s，在 2.2rad/s 的
                                     * 震荡频率上带来约 36° 相位滞后，加上视觉延迟，
                                     * 阻尼项被拖成了"负弹簧"—— KP≥0.3 就全量程来回甩
                                     * (实测 ±110 极限环)。改到 0.8 后四组末段峰峰全为 0。 */
#define DEF_PROFILE_ACCEL    220.0f  /* 速度规划 a mm/s² */
#define DEF_REF_ACCEL       300.0f  /* 速度给定斜坡 mm/s²   原 300px/s² -> 120 */
#define DEF_MAX_MEASURED    500.0f  /* 测速限幅 mm/s        原 500px/s -> 200 */
#define DEF_FRICTION_V0      0.0f   /* 保留位 */
#define DEF_DEADBAND          0.0f  /* 参考算法没有死区 */
#define DEF_I_MAX            0.0f  /* 积分限幅(脉冲)，开源原值。
                                     * 注意：本机卡点要 60~100 脉冲才顶得动，6 远远不够，
                                     * 但 2026-11-09 还原成开源地基时先按原值放回，
                                     * 让 KP/KDL/KI 的调整环境和用户调出好效果时一致。
                                     * 要加大时约束：I_MAX <= LIMIT - |LEVELOFF| - 10。 */
#define DEF_I_SPEED           0.0f  /* 保留位 */
#define DEF_KICK_MAX          0.0f  /* 卡住时抖一下的力度(脉冲)。0 = 关闭。
                                     * 2026-11-08 实测：KICKMAX 10~40 都会让球在几个势阱之间
                                     * 来回跳、把本来能收敛的系统踢成 ±110 极限环。
                                     * 卡点（+80/-6/-69）是管子的机械问题，靠抖顶不可靠。
                                     * 等视觉链路修好、机械问题解决后再单独评估是否打开。 */
#define DEF_KICK_RATE         0.0f  /* 保留位，暂未使用 */
#define DEF_STUCK_MS       1500.0f  /* 球几乎不动持续这么久(ms)就顶一脚 */
#define DEF_MOVE_EPS          2.0f  /* 位移小于这个(mm)算"没动"（视觉抖动约 1mm，0.6 会误判为"在动"） */
/* 原工程是 ±151 脉冲(±17°)。本机实测平台行程到 ±120 就到底了，
 * 151 会顶死并触发驱动器堵转。这里按 70，LEVELOFF 现在为 0，总角度就是 ±70，
 * 与 I_MAX 65 匹配（限幅必须 >= I_MAX，否则积分再大也被钳住推不动球）。 */
#define DEF_ANGLE_LIMIT     120.0f  /* 限幅钳的是"控制器输出 + LEVELOFF"，所以必须给
                                     * LEVELOFF 留余量：LEVELOFF=48 时若 LIMIT=90，
                                     * 往右只剩 42 脉冲的力，TARGET=40 根本到不了。
                                     * 平台机械行程实测约 ±120，115 留了一点安全余量。 */
#define DEF_MAX_STEP        30.0f  /* 每帧最多变化脉冲数。10 时平台从 -90 转到 +90 要 1.8 秒，
                                     * 球一次超调只跑 1 秒，反向刹车根本来不及。
                                     * 2026-11-08 实测：MAXSTEP 10 误差合计 48、30 -> 33、60 -> 54。
                                     * 30 最好，再快反而把球甩过头。 */

#define DEF_MODE              1.0f  /* 1 = 参考控制器（默认） */

#define DEF_BRAKE_BOOST       1.60f /* 刹车时速度增益倍数   原 1.60 */
#define DEF_BRAKE_ENTER       0.85f /* 刹车进入比例         原 0.85 */
#define DEF_BRAKE_EXIT        0.55f /* 刹车退出比例         原 0.55 */
#define DEF_I_ZONE          25.0f  /* 积分允许范围 mm      原 25px -> 15 -> 40 -> 200
                                     * 2026-11-08 实测：球经常停在离目标 40~90mm 的卡点上，
                                     * 40 的范围一出去积分就只衰减，等于永久丢权。放开到全管，
                                     * 由 I_MAX 50 兜住，不会无限累积。
                                     * 旧记录：15 时球偏到 -19 就永久丢权；40 能从 -19、+41 拉回。
                                     * "120 会累积到打满冲过目标"是标零前的老机械状态，已不适用。 */
#define DEF_I_ZONE_SPEED     35.0f  /* 积分允许速度 mm/s    原 35px/s -> 14 */
#define DEF_CENTER_BAND       4.0f  /* 进入稳定误差 mm      原 4px -> 1.6 */
#define DEF_CENTER_SPEED      10.0f  /* 进入稳定速度 mm/s    原 10px/s -> 4 */
#define DEF_SETTLE_BAND       8.0f  /* 退出稳定误差 mm      原 8px -> 3.2 */
#define DEF_SETTLE_SPEED     25.0f  /* 退出稳定速度 mm/s    原 25px/s -> 10 */
#define DEF_OUT_GAIN          1.00f /* 输出总增益           开源原值，本机 1.5 容易甩 */

/* ---- 借鉴 2026-H-balance（省一开源）----
 * 他们的做法：Kd 5.0 的强微分 + KFF2·v·|v| 速度平方前馈，
 * 负向输出再乘 0.75 补机械不对称，脱困靠一个线性衰减的"预动作"偏置。
 * 这里都做成在线参数，默认全关，验证过再打开。 */
#define DEF_KDL              0.17f  /* 线性速度阻尼，直接压在实测速度上。（2026-11-09 归零）
                                     * 配合 KV=0 后，这是唯一的阻尼项。
                                     * 2026-11-09 用户在 VOFA 上调定（此时 KI 已降到 0.01，
                                     * 不再需要 2026-11-08 那种 1.0 的强阻尼去压大积分）。 */
#define DEF_KD2               0.0f  /* 速度平方阻尼，0 = 关 */
#define DEF_ASYM              1.0f  /* 负向输出系数，1.0 = 关 */
#define DEF_PRE_ANGLE         0.0f  /* 预动作初始偏置(脉冲)，0 = 关 */
#define DEF_PRE_MS         2000.0f  /* 预动作衰减时长(ms) */

/* 平台水平补偿 = 让球在管子上不滚的那个倾角。
 * 2026-11-08 重新标定电机零点时，就是拿"球放稳"的姿态当零的，
 * 所以现在 LEVELOFF 的初值给 0，等实际测出来再改。
 * 旧值（标零前）是 28：实测 0 -> 球停 72，27 -> 113，28 -> 118，30 -> 126.5。
 * 【每次上电都要重测】这个只是初值。 */
#define DEF_LEVEL_OFFSET     52.0f
/* -------------------------------------------- */

ctrl_param_t ctrl_param;

void CtrlParam_Default(void)
{
    ctrl_param.mode         = DEF_MODE;

    ctrl_param.kp           = DEF_KP;
    ctrl_param.kv           = DEF_KV;
    ctrl_param.friction     = DEF_FRICTION;
    ctrl_param.max_speed    = DEF_MAX_SPEED;
    ctrl_param.ki           = DEF_KI;

    ctrl_param.pos_filter   = DEF_POS_FILTER;
    ctrl_param.speed_filter = DEF_SPEED_FILTER;
    ctrl_param.profile_accel = DEF_PROFILE_ACCEL;
    ctrl_param.max_measured = DEF_MAX_MEASURED;
    ctrl_param.friction_v0  = DEF_FRICTION_V0;
    ctrl_param.deadband     = DEF_DEADBAND;
    ctrl_param.i_max        = DEF_I_MAX;
    ctrl_param.i_speed      = DEF_I_SPEED;
    ctrl_param.kick_max     = DEF_KICK_MAX;
    ctrl_param.kick_rate    = DEF_KICK_RATE;
    ctrl_param.stuck_ms     = DEF_STUCK_MS;
    ctrl_param.move_eps     = DEF_MOVE_EPS;
    ctrl_param.angle_limit  = DEF_ANGLE_LIMIT;
    ctrl_param.max_step     = DEF_MAX_STEP;
    ctrl_param.level_offset = DEF_LEVEL_OFFSET;

    ctrl_param.ref_accel    = DEF_REF_ACCEL;
    ctrl_param.brake_boost  = DEF_BRAKE_BOOST;
    ctrl_param.brake_enter  = DEF_BRAKE_ENTER;
    ctrl_param.brake_exit   = DEF_BRAKE_EXIT;
    ctrl_param.i_zone       = DEF_I_ZONE;
    ctrl_param.i_zone_speed = DEF_I_ZONE_SPEED;
    ctrl_param.center_band  = DEF_CENTER_BAND;
    ctrl_param.center_speed = DEF_CENTER_SPEED;
    ctrl_param.settle_band  = DEF_SETTLE_BAND;
    ctrl_param.settle_speed = DEF_SETTLE_SPEED;
    ctrl_param.out_gain     = DEF_OUT_GAIN;

    ctrl_param.kdl          = DEF_KDL;
    ctrl_param.kd2          = DEF_KD2;
    ctrl_param.asym         = DEF_ASYM;
    ctrl_param.pre_angle    = DEF_PRE_ANGLE;
    ctrl_param.pre_ms       = DEF_PRE_MS;
}

/* 控制律里所有 CTRL_xxx 读的都是运行时变量 */
#define CTRL_MODE           (ctrl_param.mode)
#define CTRL_KP             (ctrl_param.kp)
#define CTRL_KV             (ctrl_param.kv)
#define CTRL_FRICTION       (ctrl_param.friction)
#define CTRL_MAX_SPEED      (ctrl_param.max_speed)
#define CTRL_KI             (ctrl_param.ki)

#define CTRL_POS_FILTER     (ctrl_param.pos_filter)
#define CTRL_SPEED_FILTER   (ctrl_param.speed_filter)
#define CTRL_PROFILE_ACCEL  (ctrl_param.profile_accel)
#define CTRL_MAX_MEASURED   (ctrl_param.max_measured)
#define CTRL_FRICTION_V0    (ctrl_param.friction_v0)
#define CTRL_DEADBAND       (ctrl_param.deadband)
#define CTRL_I_MAX          (ctrl_param.i_max)
#define CTRL_I_SPEED        (ctrl_param.i_speed)
#define CTRL_KICK_MAX       (ctrl_param.kick_max)
#define CTRL_KICK_RATE      (ctrl_param.kick_rate)
#define CTRL_STUCK_MS       (ctrl_param.stuck_ms)
#define CTRL_MOVE_EPS       (ctrl_param.move_eps)
#define CTRL_ANGLE_LIMIT    (ctrl_param.angle_limit)
#define CTRL_MAX_STEP       (ctrl_param.max_step)
#define CTRL_LEVEL_OFFSET   (ctrl_param.level_offset)

#define CTRL_REF_ACCEL      (ctrl_param.ref_accel)
#define CTRL_BRAKE_BOOST    (ctrl_param.brake_boost)
#define CTRL_BRAKE_ENTER    (ctrl_param.brake_enter)
#define CTRL_BRAKE_EXIT     (ctrl_param.brake_exit)
#define CTRL_I_ZONE         (ctrl_param.i_zone)
#define CTRL_I_ZONE_SPEED   (ctrl_param.i_zone_speed)
#define CTRL_CENTER_BAND    (ctrl_param.center_band)
#define CTRL_CENTER_SPEED   (ctrl_param.center_speed)
#define CTRL_SETTLE_BAND    (ctrl_param.settle_band)
#define CTRL_SETTLE_SPEED   (ctrl_param.settle_speed)
#define CTRL_OUT_GAIN       (ctrl_param.out_gain)

#define CTRL_KDL            (ctrl_param.kdl)
#define CTRL_KD2            (ctrl_param.kd2)
#define CTRL_ASYM           (ctrl_param.asym)
#define CTRL_PRE_ANGLE      (ctrl_param.pre_angle)
#define CTRL_PRE_MS         (ctrl_param.pre_ms)

void BallControl_Init(ballcontrol *controller)
{
    memset(controller, 0, sizeof(*controller));
    controller->initialized = 0;
    controller->command     = 0.0f;
    controller->pre_timer   = 1.0e9f;   /* 1e9 = 预动作未激活 */
}

void BallControl_Reset(ballcontrol *controller)
{
    BallControl_Init(controller);
}

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* =====================================================================
 * 参考控制器：速度耦合轨迹控制器
 * ---------------------------------------------------------------------
 *  ① 首帧只记初值
 *  ② dt，异常时用 0.033s
 *  ③ 位置低通 -> 差分测速 -> 速度低通
 *  ④ 稳定锁存（滞环）：进入 center_band/center_speed，退出 settle_band/settle_speed
 *  ⑤ 速度规划 v_max = √(2·a·(距离−中心带))，给定按 ref_accel 斜坡上升
 *  ⑥ 刹车：刹车距离 v²/(2a) ≥ enter×剩余距离 进入，< exit×剩余距离 退出，
 *     刹车时速度增益 ×brake_boost
 *  ⑦ 中心积分：只在 i_zone 内、速度<i_zone_speed、不刹车时累加
 *  ⑧ 合成 ×out_gain，±限幅，每帧最多变化 max_step
 *  ⑨ 锁存"稳定"时输出清零（平台回平）
 * ===================================================================== */
static int32_t BallControl_RefUpdate(ballcontrol *c,
                                     float target,
                                     float measured,
                                     uint32_t tick)
{
    float dt;
    float raw_v;
    float error;        /* 目标 − 当前，正 = 球在目标左边 */
    float distance;
    float direction;
    float v_toward;
    float allowed_speed;
    float v_error;
    float v_gain;
    float ref_step;
    float eff_d;
    float cmd;
    float change;

    /* ① 首帧 */
    if (c->initialized == 0U)
    {
        c->position        = measured;
        c->last_position   = measured;
        c->last_tick       = tick;
        c->last_error      = target - measured;
        c->initialized     = 1U;
        return 0;
    }

    /* ② dt
     * 【必须用真实 dt】参考工程按 30fps 写，dt>0.1 就当成异常换成 0.033。
     * 但本机视觉只有约 9fps，真实 dt≈0.11s，照搬那条会把测速放大 3.4 倍，
     * 速度环一直反向猛拽，表现为固定周期的来回甩。这里只在真正异常
     * （<10ms 重复帧 / >250ms 断流）时才兜底，其余用实测值。 */
    dt = (float)(tick - c->last_tick) * 0.001f;
    c->last_tick = tick;
    if (dt < 0.010f) dt = 0.010f;
    if (dt > 0.250f) dt = 0.250f;

    /* ③ 位置低通 + 差分测速 + 速度低通 */
    c->position += CTRL_POS_FILTER * (measured - c->position);

    raw_v = (c->position - c->last_position) / dt;
    c->last_position = c->position;
    raw_v = clampf(raw_v, -CTRL_MAX_MEASURED, CTRL_MAX_MEASURED);

    c->speed_filter += CTRL_SPEED_FILTER * (raw_v - c->speed_filter);

    /* ④ 误差 */
    error    = target - c->position;
    distance = fabsf(error);

    if (c->settle_flag == 0U)
    {
        if (distance <= CTRL_CENTER_BAND &&
            fabsf(c->speed_filter) <= CTRL_CENTER_SPEED)
        {
            c->settle_flag = 1U;
            c->integral    = 0.0f;
        }
    }
    else if (distance >= CTRL_SETTLE_BAND ||
             fabsf(c->speed_filter) >= CTRL_SETTLE_SPEED)
    {
        c->settle_flag = 0U;
    }

    if (error > 0.0f)      direction =  1.0f;
    else if (error < 0.0f) direction = -1.0f;
    else                   direction =  0.0f;

    /* ⑤ 速度规划 */
    eff_d = distance - CTRL_CENTER_BAND;
    if (eff_d < 0.0f) eff_d = 0.0f;
    allowed_speed = sqrtf(2.0f * CTRL_PROFILE_ACCEL * eff_d);
    allowed_speed = clampf(allowed_speed, 0.0f, CTRL_MAX_SPEED);
    c->velocity_envelope = direction * allowed_speed;

    if (direction == 0.0f)
    {
        c->velocity_reference = 0.0f;
    }
    else if ((c->velocity_reference * direction) < 0.0f)
    {
        c->velocity_reference = 0.0f;
    }
    else if (fabsf(c->velocity_reference) < allowed_speed)
    {
        ref_step = CTRL_REF_ACCEL * dt;
        if (ref_step > (allowed_speed - fabsf(c->velocity_reference)))
            ref_step = allowed_speed - fabsf(c->velocity_reference);
        c->velocity_reference += direction * ref_step;
    }
    else
    {
        c->velocity_reference = c->velocity_envelope;
    }

    /* ⑥ 刹车（带滞环） */
    v_toward = c->speed_filter * direction;
    if (v_toward > 0.0f)
        c->stopping_distance = (v_toward * v_toward) / (2.0f * CTRL_PROFILE_ACCEL);
    else
        c->stopping_distance = 0.0f;

    if (v_toward <= 0.0f)
    {
        c->brake_flag = 0U;
    }
    else if (c->brake_flag == 0U)
    {
        c->brake_flag = (uint8_t)(c->stopping_distance >= (CTRL_BRAKE_ENTER * distance));
    }
    else if (c->stopping_distance < (CTRL_BRAKE_EXIT * distance))
    {
        c->brake_flag = 0U;
    }

    v_error = c->velocity_reference - c->speed_filter;
    v_gain  = CTRL_KV;
    if (c->brake_flag != 0U) v_gain *= CTRL_BRAKE_BOOST;

    /* ⑦ 越过目标：积分砍到 1/4 */
    if ((error * c->last_error) < 0.0f)
        c->integral *= 0.25f;

    /* ⑦ 中心积分 */
    if (c->settle_flag == 0U &&
        distance < CTRL_I_ZONE &&
        fabsf(c->speed_filter) < CTRL_I_ZONE_SPEED &&
        c->brake_flag == 0U)
    {
        c->integral += CTRL_KI * error * dt;
        c->integral = clampf(c->integral, -CTRL_I_MAX, CTRL_I_MAX);
    }
    else
    {
        c->integral *= 0.92f;
    }

    /* ⑧ 合成 = P(位置) + 速度环 + 中心积分，单位是脉冲 */
    cmd = (CTRL_KP * error) + (v_gain * v_error) + c->integral;

    /* ⑧a 直接速度阻尼（借鉴 2026-H-balance 的 KFF·v + KFF2·v·|v|）
     * 【关键】上面 ⑤ 的速度项是 v_ref − v_measured，球按规划速度跑时它等于 0，
     * 没有任何阻尼作用。他们的做法是再叠一个直接压在实测速度上的项：
     *   KDL·v    —— 线性，低速也有阻尼
     *   KD2·v|v| —— 平方，只在球跑快时发力，不拖慢低速爬行
     * 两个都是 0 = 关闭。 */
    if (CTRL_KDL != 0.0f)
    {
        cmd -= CTRL_KDL * c->speed_filter;
    }
    if (CTRL_KD2 != 0.0f)
    {
        cmd -= CTRL_KD2 * c->speed_filter * fabsf(c->speed_filter);
    }

    /* ⑧b 卡住计时（KICKMAX 和 PRE_ANGLE 共用，只在这里推进一次）*/
    if ((CTRL_KICK_MAX > 0.0f || CTRL_PRE_ANGLE > 0.0f) &&
        CTRL_STUCK_MS > 0.0f && direction != 0.0f)
    {
        if (fabsf(c->position - c->ref_pos) > CTRL_MOVE_EPS)
        {
            c->ref_pos     = c->position;   /* 动了，重新计时 */
            c->stuck_timer = 0.0f;
            c->pre_timer   = 1.0e9f;        /* 动了就撤销预动作 */
        }
        else
        {
            c->stuck_timer += dt * 1000.0f;
        }
    }

    /* ⑧c 预动作脱困（借鉴 2026-H-balance 的 pre-action）
     * 他们的做法：叠一个偏置，然后让它【线性衰减到 0】，PID 全程在线。
     * 我们原来的方波踢（减10→加10→归零）之所以会振成极限环，
     * 就因为那是一个阶跃。这里改成：卡住后朝目标方向叠 PRE_ANGLE，
     * 在 PRE_MS 内线性衰减到 0；球一动立刻撤销。0 = 关闭。 */
    if (CTRL_PRE_ANGLE > 0.0f && CTRL_PRE_MS > 0.0f &&
        direction != 0.0f && c->settle_flag == 0U)
    {
        if (c->pre_timer < CTRL_PRE_MS)
        {
            float decay = 1.0f - (c->pre_timer / CTRL_PRE_MS);
            cmd += direction * CTRL_PRE_ANGLE * decay;
            c->pre_timer += dt * 1000.0f;
            if (c->pre_timer >= CTRL_PRE_MS)
            {
                c->pre_timer   = 1.0e9f;
                c->stuck_timer = 0.0f;
                c->ref_pos     = c->position;
            }
        }
        else if (c->stuck_timer >= CTRL_STUCK_MS)
        {
            c->pre_timer = 0.0f;            /* 开始一次新的预动作 */
        }
    }

    /* ⑧d 方波踢（旧方案，KICKMAX > 0 才启用；和预动作二选一）*/
    if (CTRL_KICK_MAX > 0.0f &&
        c->stuck_timer >= CTRL_STUCK_MS && c->settle_flag == 0U)
    {
        c->kick += dt;                          /* kick 复用成顶球计时器(秒) */
        if (c->kick < 0.60f)
        {
            cmd -= direction * CTRL_KICK_MAX;   /* 第一段：朝反方向顶 */
        }
        else if (c->kick < 1.60f)
        {
            cmd += direction * CTRL_KICK_MAX;   /* 第二段：朝目标方向顶 */
        }
        else
        {
            c->kick        = 0.0f;
            c->stuck_timer = 0.0f;
            c->ref_pos     = c->position;
        }
    }

    /* ⑨ 判定"已停稳"之后：平台回平、积分清零、速度给定清零、退出刹车。
     * 和参考工程一致——这里就是"输出清零"，不做任何保持。 */
    if (c->settle_flag != 0U)
    {
        cmd                   = 0.0f;
        c->integral           = 0.0f;
        c->velocity_reference = 0.0f;
        c->brake_flag         = 0U;
    }

    /* ⑧ 总增益：只整体放大，不改 P/速度环/积分的比例 */
    cmd *= CTRL_OUT_GAIN;

    /* ⑧e 机械不对称补偿（借鉴 2026-H-balance 的 pid_out<0 乘 0.75）
     * 管子往一边比另一边"顺"，同样的脉冲数会跑得更远，会表现为单侧过冲。
     * 1.0 = 不补偿，本机先按 1.0 烧。 */
    if (cmd < 0.0f && CTRL_ASYM != 1.0f)
    {
        cmd *= CTRL_ASYM;
    }

    /* ⑧ 机械限幅 */
    cmd = clampf(cmd, -CTRL_ANGLE_LIMIT, CTRL_ANGLE_LIMIT);

    /* ⑧ 变化率限幅：每帧最多改 MAXSTEP 脉冲，防止平台猛甩把球抛出去 */
    change = cmd - c->command;
    change = clampf(change, -CTRL_MAX_STEP, CTRL_MAX_STEP);
    c->command += change;
    c->command = clampf(c->command, -CTRL_ANGLE_LIMIT, CTRL_ANGLE_LIMIT);

    c->error      = error;
    c->last_error = error;

    return (int32_t)(c->command + CTRL_LEVEL_OFFSET);
}

/* =====================================================================
 * 早期自写的五项相加 PID（MODE=0，保留作为对照）
 * ===================================================================== */
static int32_t BallControl_PidUpdate(ballcontrol *controller,
                                     float target_position,
                                     float measured_position,
                                     uint32_t tick)
{
    float dt;
    float raw_speed;
    float error;
    float distance;
    float direction;
    float speed_ref;
    float v_toward;
    float fade;
    float command;
    float change;

    if (controller->initialized == 0)
    {
        controller->position      = measured_position;
        controller->last_position = measured_position;
        controller->ref_pos       = measured_position;
        controller->last_tick     = tick;
        controller->initialized   = 1;
        return 0;
    }

    dt = (float)(tick - controller->last_tick) * 0.001f;
    controller->last_tick = tick;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.050f) dt = 0.050f;

    controller->position += CTRL_POS_FILTER * (measured_position - controller->position);

    raw_speed = (controller->position - controller->last_position) / dt;
    raw_speed = clampf(raw_speed, -CTRL_MAX_MEASURED, CTRL_MAX_MEASURED);
    controller->last_position = controller->position;

    controller->speed_filter += CTRL_SPEED_FILTER * (raw_speed - controller->speed_filter);

    controller->error = controller->position - target_position;
    error    = controller->error;
    distance = fabsf(error);

    if (error > 0.0f)      direction =  1.0f;
    else if (error < 0.0f) direction = -1.0f;
    else                   direction =  0.0f;

    {
        float d = distance - CTRL_DEADBAND;
        if (d < 0.0f) d = 0.0f;
        speed_ref = -direction * sqrtf(2.0f * CTRL_PROFILE_ACCEL * d);
        speed_ref = clampf(speed_ref, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);
    }

    command = -CTRL_KP * error + CTRL_KV * (speed_ref - controller->speed_filter);

    if (distance > CTRL_DEADBAND)
    {
        v_toward = controller->speed_filter * direction;
        fade = 1.0f + v_toward / (CTRL_FRICTION_V0 > 0.0f ? CTRL_FRICTION_V0 : 1.0f);
        fade = clampf(fade, 0.0f, 1.0f);
        command += -direction * CTRL_FRICTION * fade;
    }

    if (distance > CTRL_DEADBAND)
    {
        float leak = 1.0f;
        controller->integral += CTRL_KI * (-error) * dt;
        if ((controller->speed_filter * direction) < -CTRL_I_SPEED) leak = 0.99f;
        controller->integral *= leak;
        controller->integral = clampf(controller->integral, -CTRL_I_MAX, CTRL_I_MAX);
    }
    else
    {
        controller->integral *= 0.85f;
    }

    if ((controller->last_error > 0.0f && error < 0.0f) ||
        (controller->last_error < 0.0f && error > 0.0f))
    {
        controller->integral *= 0.5f;
    }
    controller->last_error = error;

    command += controller->integral;

    command = clampf(command, -CTRL_ANGLE_LIMIT, CTRL_ANGLE_LIMIT);
    command += CTRL_LEVEL_OFFSET;

    change = command - controller->command;
    change = clampf(change, -CTRL_MAX_STEP, CTRL_MAX_STEP);
    controller->command += change;

    return (int32_t)controller->command;
}

int32_t BallControl_Update(ballcontrol *controller,
                           float target_position,
                           float measured_position,
                           uint32_t tick)
{
    if (CTRL_MODE > 0.5f)
        return BallControl_RefUpdate(controller, target_position, measured_position, tick);

    return BallControl_PidUpdate(controller, target_position, measured_position, tick);
}
