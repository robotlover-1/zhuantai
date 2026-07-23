"""
理论PWM曲线仿真 - 纯数值输出(无需matplotlib)
"""
import math

# ============== 用户设定参数 ==============
force     = 800      # F:  位置环输出限幅
force_F   = 3500     # R:  最终PWM限幅
time_c    = 140      # V:  最终匀速进孔速度
pulse_buf = 30000    # BS: 缓冲段起始脉冲数
time_buf  = 450      # BV: 缓冲段速度
pulse_low = 15000    # S:  慢速进孔脉冲数
SetPoint_distance = 39200  # 单工位移动脉冲数

# PID
PC  = 25
KP  = 0.150;  KI  = 0.005;  KD  = 0.015
SKP = 1.300;  SKI = 0.850;  SKD = 0.000

# 电机模型
K_motor = 0.25      # speed_ss = K_motor * PWM  (counts/5ms per PWM unit)
tau_motor = 0.050   # 时间常数(秒)

dt = 0.025
steps = int(3.0 / dt)

# 状态
pos = 0.0; speed = 0.0; speed_measured = 0.0
SumError_P = 0.0; LastError_P = 0.0; PrevError_P = 0.0
ActualValue_S = 0.0; LastError_S = 0.0; PrevError_S = 0.0
speed_buf = [0.0]*10; speed_buf_idx = 0
speed_filtered = 0.0; speed_filter_prev = 0.0

# 记录
records = []

for step in range(steps):
    t = step * dt
    if pos >= SetPoint_distance - 1 and t > 0.1:
        break

    # 位置PID
    error_p = SetPoint_distance - pos
    SumError_P += error_p
    SumError_P = max(-40000.0, min(40000.0, SumError_P))
    ActualValue_P = (KP * error_p + KI * SumError_P
                     + KD * (error_p - 2.0*LastError_P + PrevError_P))
    PrevError_P = LastError_P
    LastError_P = error_p

    motor_pwm_pos = int(ActualValue_P)
    if motor_pwm_pos >= 0:
        motor_pwm_pos = force if motor_pwm_pos >= force else (100 if motor_pwm_pos < 100 else motor_pwm_pos)
    else:
        motor_pwm_pos = -force if motor_pwm_pos <= -force else (-100 if motor_pwm_pos > -100 else motor_pwm_pos)

    # S曲线限速
    dist_remaining = SetPoint_distance - pos
    s_curve = None
    if dist_remaining < pulse_buf and pulse_buf > 0:
        ts = max(0.0, dist_remaining / pulse_buf)
        smoothed = ts*ts*(3.0 - 2.0*ts)
        s_curve = time_c + (time_buf - time_c) * smoothed
        ilimit = int(s_curve)
        if motor_pwm_pos > ilimit: motor_pwm_pos = ilimit
        if motor_pwm_pos < -ilimit: motor_pwm_pos = -ilimit

    SetPoint_S = float(motor_pwm_pos)

    # 速度PID
    Error_S = SetPoint_S - speed_measured
    ActualValue_S += (SKP*(Error_S - LastError_S) + SKI*Error_S
                      + SKD*(Error_S - 2.0*LastError_S + PrevError_S))
    ActualValue_S = max(-3000.0, min(3000.0, ActualValue_S))
    PrevError_S = LastError_S
    LastError_S = Error_S
    motor_pwm = int(ActualValue_S)
    if motor_pwm >= force_F: motor_pwm = force_F
    elif motor_pwm <= -force_F: motor_pwm = -force_F

    # 电机模型
    speed_ss = K_motor * motor_pwm
    speed = speed + (dt/tau_motor) * (speed_ss - speed)

    # 速度测量
    speed_buf[speed_buf_idx] = speed
    speed_buf_idx = (speed_buf_idx + 1) % 10
    if speed_buf_idx == 0:
        srt = sorted(speed_buf)
        temp = sum(srt[2:8])/6.0
        speed_filtered = 0.48*temp + 0.52*speed_filter_prev
        speed_filter_prev = speed_filtered
    speed_measured = speed_filtered if speed_filtered > 1e-6 else speed

    pos += speed * 5.0

    records.append((t*1000, motor_pwm, int(motor_pwm_pos), speed, SetPoint_S, pos,
                    s_curve if s_curve else 0, error_p))

# 输出CSV
print("time_ms,pwm,pos_pid_out,speed,speed_sp,pos,s_curve,error_p")
for r in records:
    print(f"{r[0]:.1f},{r[1]},{r[2]},{r[3]:.1f},{r[4]:.1f},{r[5]:.1f},{r[6]:.1f},{r[7]:.1f}")

print(f"\n=== 摘要 ===")
print(f"总步数: {len(records)}")
print(f"总时间: {records[-1][0]:.0f}ms")
print(f"最终PWM: {records[-1][1]}")
print(f"PWM最大值: {max(r[1] for r in records)}")
print(f"最终位置: {records[-1][5]:.1f}")

# 找S曲线开始点
s_start_idx = next((i for i,r in enumerate(records) if r[6] > 0), None)
if s_start_idx:
    print(f"S曲线开始: {records[s_start_idx][0]:.0f}ms, pos={records[s_start_idx][5]:.0f}")
print(f"S曲线shape: limit = 140 + 310 * smoothstep(dist/30000)")

# ASCII简易绘图
print("\n=== PWM ASCII曲线 ===")
width = 80
max_pwm = max(r[1] for r in records)
n = len(records)
for i in range(20):
    idx = int(i * n / 20)
    if idx >= n: break
    r = records[idx]
    bar_len = int(r[1] / max_pwm * width)
    s_mark = " |S" if r[6] > 0 else ""
    print(f"{r[0]:6.0f}ms PWM={r[1]:4d} {'#'*bar_len}{s_mark}")

# 关键阶段描述
print("\n=== 关键阶段分析 ===")
# Phase 1: ramp up
for i, r in enumerate(records):
    if r[1] >= 2500:
        print(f"Phase1: PWM升至2500+ @ {r[0]:.0f}ms, pos={r[5]:.0f}, speed={r[3]:.0f}")
        break

# Phase 2: cruise at full speed
for i, r in enumerate(records):
    if r[6] > 0:
        print(f"Phase2: S曲线开始 @ {r[0]:.0f}ms, pos={r[5]:.0f}, dist_remain={39200-r[5]:.0f}, PWM={r[1]}")
        break

# Phase 3: S-curve shapes
for i, r in enumerate(records):
    if r[6] > 0 and r[6] < 400:
        print(f"Phase3: S曲线中段(limit<400) @ {r[0]:.0f}ms, limit={r[6]:.0f}, PWM={r[1]}, pos={r[5]:.0f}")
        break

for i, r in enumerate(records):
    if r[6] > 0 and r[6] < 200:
        print(f"Phase3: S曲线末段(limit<200) @ {r[0]:.0f}ms, limit={r[6]:.0f}, PWM={r[1]}, pos={r[5]:.0f}")
        break

# Last
print(f"Phase4: 最终状态 @ {records[-1][0]:.0f}ms, PWM={records[-1][1]}, dist={39200-records[-1][5]:.0f}")
