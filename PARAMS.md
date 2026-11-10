# 参数版本记录

改参数前先 commit，commit message 写上参数值，这里同步记一行。
切回某套参数：`git checkout <版本号> -- Core/Src/ball_control.c`

---

## 2026-11-10 · 全增益归零（当前）

```
KP 0     KV 0     KI 0     IMAX 0    KDL 0    KD2 0
KICKMAX 0  KICKRATE 0  PREA 0  FRICTION 0  V0 0  DEADBAND 0  ISPEED 0
LEVELOFF 52  LIMIT 120  MAXSTEP 30
POSF 0.90  SPEEDF 0.80  OUTGAIN 1.00  ASYM 1.00  MODE 1  MOTORRPM 60
```

表现：控制器输出恒为 0，平台只停在 LEVELOFF，球不受控。
用途：干净地基 / 重新测水平点。

---

## 2026-11-09 · 用户 VOFA 调定

```
KP 0.60   KDL 0.17   KI 0.01
KV 0.28   IMAX 6.0   LEVELOFF 52
POSF 0.45  SPEEDF 0.25  MAXSTEP 8  OUTGAIN 1.50  LIMIT 120
```

用户反馈：在这套地基上"效果很好"。
注意：同一组 KP/KDL/KI 换成 POSF 0.90 / SPEEDF 0.80 / MAXSTEP 30 / OUTGAIN 1.00
（2026-11-09 的同步版）后明显变差 —— 说明 KP 的观感强烈依赖这几项。

---

## 2026-11-08 · 本次实测最优（开源地基改造版）

```
KP 0.15   KV 0      KDL 1.0   KI 1.2   IMAX 85   IZONE 200
OUTGAIN 1.0  LIMIT 120  MAXSTEP 30  POSF 0.90  SPEEDF 0.80
LEVELOFF 52（当次实测）
```

实测 ±10mm 阶跃：`+10 → 落点 −1.9，in±10 94%`；`−10 → 落点 −9.0，峰峰 0.00，100%`。
已知限制：+24/+30 一带有机械卡点，右侧到不了 100%。

---

## 关键结论（别忘）

- `POSF 0.45 / SPEEDF 0.25` 在 11fps 下会全量程甩（POS −91↔+110）。要稳就得 0.90/0.80。
- `LEVELOFF` 每次上电都不同（见过 27 / 20 / −5 / 47 / 52），必须每把重测。
- 本机静摩擦要 60~100 脉冲才顶得动；`IMAX` 太小球就出不来。
  约束：`IMAX <= LIMIT - |LEVELOFF| - 10`。
