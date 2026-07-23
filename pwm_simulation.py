"""
理论PWM曲线仿真 - 基于实际控制代码的数学模型
控制架构: 串级PID (位置环 → 速度环) + S曲线限速
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams['font.size'] = 10
rcParams['figure.dpi'] = 120

# ============== 用户设定参数 ==============
force     = 800      # F:  位置环输出限幅 (最大速度设定)
force_F   = 3500     # R:  最终PWM限幅 (最大占空比)
time_c    = 140      # V:  最终匀速进孔速度 (PWM单位)
pulse_buf = 30000    # BS: 缓冲段起始脉冲数 (S曲线范围)
time_buf  = 450      # BV: 缓冲段速度 (S曲线起点)
pulse_low = 15000    # S:  慢速进孔脉冲数 (显示用/备用)
pluse_ele = 38000    # E:  光电检测脉冲限制
SetPoint_distance = 39200  # 单工位移动脉冲数

# PID参数
PC  = 25             # 采样周期(ms)
KP  = 0.150          # 位置环P
KI  = 0.005          # 位置环I
KD  = 0.015          # 位置环D
SKP = 1.300          # 速度环P
SKI = 0.850          # 速度环I
SKD = 0.000          # 速度环D

# ============== 仿真参数 ==============
dt = 0.025           # PID周期 25ms (秒)
total_time = 3.0     # 最大仿真时间(秒)
steps = int(total_time / dt)

# ============== 电机模型参数 ==============
# 直流电机一阶模型: tau * d(speed)/dt + speed = K_motor * PWM
# speed 单位: 编码器脉冲/5ms (与speed_computer一致)
# 假设: PWM=3200时稳态速度≈800 counts/5ms → K_motor ≈ 0.25
K_motor = 0.25       # 稳态增益: speed_ss = K_motor * PWM
tau_motor = 0.050    # 电机时间常数(秒), 50ms

# ============== 仿真状态变量 ==============
pos = 0.0                     # 编码器位置(脉冲)
speed = 0.0                   # 电机速度(counts/5ms)
speed_measured = 0.0          # 测量速度(带滤波延迟)

# 位置PID状态
SumError_P = 0.0
LastError_P = 0.0
PrevError_P = 0.0

# 速度PID状态(增量式)
ActualValue_S = 0.0
LastError_S = 0.0
PrevError_S = 0.0

# 速度测量滤波器状态(模拟speed_computer中的滤波)
speed_buf = [0.0] * 10
speed_buf_idx = 0
speed_filtered = 0.0
speed_filter_prev = 0.0

# ============== 记录数组 ==============
time_arr = []
pwm_arr = []
pwm_pos_arr = []      # 位置PID原始输出
speed_arr = []
speed_sp_arr = []      # 速度设定(SetPoint_S)
pos_arr = []
s_curve_arr = []       # S曲线限幅值
error_p_arr = []       # 位置误差

# ============== 仿真主循环 ==============
for step in range(steps):
    t = step * dt

    # 检查是否到达目标
    if pos >= SetPoint_distance - 1 and t > 0.1:
        break

    # --- 位置PID (position式) ---
    error_p = SetPoint_distance - pos
    SumError_P += error_p
    SumError_P = max(-40000.0, min(40000.0, SumError_P))

    ActualValue_P = (KP * error_p
                     + KI * SumError_P
                     + KD * (error_p - 2.0 * LastError_P + PrevError_P))

    PrevError_P = LastError_P
    LastError_P = error_p

    motor_pwm_pos = int(ActualValue_P)

    # 位置环输出限幅 [100, force] 或 [-force, -100]
    if motor_pwm_pos >= 0:
        if motor_pwm_pos >= force:
            motor_pwm_pos = force
        elif motor_pwm_pos < 100:
            motor_pwm_pos = 100
    else:
        if motor_pwm_pos <= -force:
            motor_pwm_pos = -force
        elif motor_pwm_pos > -100:
            motor_pwm_pos = -100

    # --- S曲线距离限速 ---
    dist_remaining = SetPoint_distance - pos
    s_curve_limit = None
    if dist_remaining < pulse_buf and pulse_buf > 0:
        t_s = max(0.0, dist_remaining / pulse_buf)
        smoothed = t_s * t_s * (3.0 - 2.0 * t_s)  # smoothstep
        s_curve_limit = time_c + (time_buf - time_c) * smoothed
        ilimit = int(s_curve_limit)
        if motor_pwm_pos > ilimit:
            motor_pwm_pos = ilimit
        if motor_pwm_pos < -ilimit:
            motor_pwm_pos = -ilimit

    SetPoint_S = float(motor_pwm_pos)

    # --- 速度PID (增量式) ---
    Error_S = SetPoint_S - speed_measured
    ActualValue_S += (SKP * (Error_S - LastError_S)
                      + SKI * Error_S
                      + SKD * (Error_S - 2.0 * LastError_S + PrevError_S))

    # 速度环输出限幅 ±3000
    ActualValue_S = max(-3000.0, min(3000.0, ActualValue_S))
    PrevError_S = LastError_S
    LastError_S = Error_S

    motor_pwm = int(ActualValue_S)

    # 最终PWM限幅 ±force_F
    if motor_pwm >= force_F:
        motor_pwm = force_F
    elif motor_pwm <= -force_F:
        motor_pwm = -force_F

    # --- 电机物理模型 ---
    # 一阶惯性环节: tau * dv/dt + v = K_motor * PWM
    # 离散化: v[k+1] = v[k] + (dt/tau) * (K_motor * PWM - v[k])
    speed_ss = K_motor * motor_pwm
    alpha = dt / tau_motor
    speed = speed + alpha * (speed_ss - speed)

    # --- 速度测量模型 (模拟speed_computer的5ms采样+中值+低通滤波) ---
    # 每5ms记录一次增量, 25ms=5次记录
    # 简化: 用滑动平均 + 一阶低通模拟
    speed_buf[speed_buf_idx] = speed
    speed_buf_idx = (speed_buf_idx + 1) % 10

    # 中值滤波(排序取中间6个平均) + 一阶低通 q=0.48
    if speed_buf_idx == 0:  # 每10个采样(50ms)更新一次滤波值
        sorted_speeds = sorted(speed_buf)
        temp = sum(sorted_speeds[2:8]) / 6.0  # 中间6个平均
        speed_filtered = 0.48 * temp + 0.52 * speed_filter_prev
        speed_filter_prev = speed_filtered

    # 使用较简单的测量模型(上次滤波值)
    speed_measured = speed_filtered if speed_filtered > 0 else speed

    # --- 位置更新 ---
    # speed 是 counts/5ms, dt=25ms=5个5ms周期
    pos += speed * 5.0

    # --- 记录 ---
    time_arr.append(t * 1000.0)  # 转为ms
    pwm_arr.append(motor_pwm)
    pwm_pos_arr.append(ActualValue_P)
    speed_arr.append(speed)
    speed_sp_arr.append(SetPoint_S)
    pos_arr.append(pos)
    s_curve_arr.append(s_curve_limit if s_curve_limit is not None else np.nan)
    error_p_arr.append(error_p)

# 转为numpy数组
time_arr = np.array(time_arr)
pwm_arr = np.array(pwm_arr)
pwm_pos_arr = np.array(pwm_pos_arr)
speed_arr = np.array(speed_arr)
speed_sp_arr = np.array(speed_sp_arr)
pos_arr = np.array(pos_arr)
s_curve_arr = np.array(s_curve_arr)

print(f"仿真完成: {len(time_arr)} 步, 总时间 {time_arr[-1]:.0f}ms")
print(f"最终位置: {pos_arr[-1]:.1f} 脉冲 (目标: {SetPoint_distance})")
print(f"最终速度: {speed_arr[-1]:.1f} counts/5ms")
print(f"最终PWM:  {pwm_arr[-1]:.0f}")
print(f"PWM最大值: {pwm_arr.max():.0f}")
print(f"S曲线开始时间: dist=30000 at pos={SetPoint_distance-30000}")

# ============== 绘图 ==============
fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)

# 图1: PWM输出
ax = axes[0]
ax.plot(time_arr, pwm_arr, 'b-', linewidth=1.2, label='motor_pwm (最终输出)')
ax.axhline(y=force_F, color='darkred', linestyle='--', linewidth=0.8, alpha=0.7, label=f'force_F={force_F}')
ax.axhline(y=3000, color='orange', linestyle=':', linewidth=0.8, alpha=0.7, label='速度环内部限幅=3000')
ax.axhline(y=0, color='gray', linewidth=0.5)
ax.set_ylabel('PWM 占空比\n(0~3599)', fontsize=10)
ax.set_title('理论PWM曲线 — 串级PID + S曲线限速', fontsize=13, fontweight='bold')
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)

# 图2: 速度设定 vs 实际速度
ax = axes[1]
ax.plot(time_arr, speed_sp_arr, 'r-', linewidth=1.2, label='SetPoint_S (速度设定)')
ax.plot(time_arr, speed_arr, 'b-', linewidth=1.2, alpha=0.7, label='实际速度 (counts/5ms)')
ax.axhline(y=force, color='green', linestyle='--', linewidth=0.8, alpha=0.5, label=f'force(位置环上限)={force}')
ax.axhline(y=time_buf, color='orange', linestyle='--', linewidth=0.8, alpha=0.5, label=f'time_buf={time_buf}')
ax.axhline(y=time_c, color='purple', linestyle='--', linewidth=0.8, alpha=0.5, label=f'time_c={time_c}')
ax.set_ylabel('速度\n(counts/5ms)', fontsize=10)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)

# 图3: S曲线限幅
ax = axes[2]
ax.plot(time_arr, s_curve_arr, 'c-', linewidth=1.5, label='S-Curve Limit (smoothstep)')
ax.plot(time_arr, pwm_pos_arr, 'm-', linewidth=0.8, alpha=0.6, label='位置PID原始输出')
ax.axhline(y=force, color='green', linestyle='--', linewidth=0.8, alpha=0.5)
ax.fill_between(time_arr, 0, s_curve_arr, alpha=0.1, color='cyan')
ax.set_ylabel('限幅值', fontsize=10)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
# 标注S曲线区域
s_start = time_arr[~np.isnan(s_curve_arr)][0] if np.any(~np.isnan(s_curve_arr)) else 0
s_end = time_arr[~np.isnan(s_curve_arr)][-1] if np.any(~np.isnan(s_curve_arr)) else 0
ax.axvline(x=s_start, color='cyan', linestyle=':', linewidth=0.8)
ax.axvline(x=s_end, color='cyan', linestyle=':', linewidth=0.8)
if s_start > 0:
    ax.annotate(f'S曲线开始\n({s_start:.0f}ms)', xy=(s_start, 450),
                xytext=(s_start-100, 500), fontsize=8,
                arrowprops=dict(arrowstyle='->', color='gray'), color='gray')

# 图4: 位置 vs 时间
ax = axes[3]
ax.plot(time_arr, pos_arr, 'g-', linewidth=1.2, label='位置 (encoder counts)')
ax.axhline(y=SetPoint_distance, color='darkgreen', linestyle='--', linewidth=0.8, alpha=0.7, label=f'目标={SetPoint_distance}')
# 标注关键距离点
ax.axhline(y=SetPoint_distance - pulse_buf, color='orange', linestyle=':', linewidth=0.8, alpha=0.5)
ax.annotate(f'dist={pulse_buf}\n(S曲线起点)', xy=(s_start, SetPoint_distance-pulse_buf),
            fontsize=7, color='orange',
            xytext=(s_start+50, SetPoint_distance-pulse_buf-5000),
            arrowprops=dict(arrowstyle='->', color='orange', alpha=0.5))
ax.set_ylabel('位置\n(encoder脉冲)', fontsize=10)
ax.legend(loc='lower right', fontsize=8)
ax.grid(True, alpha=0.3)

# 图5: 位置误差
ax = axes[4]
ax.plot(time_arr, error_p_arr, 'purple', linewidth=1.0)
ax.set_ylabel('位置误差\n(脉冲)', fontsize=10)
ax.set_xlabel('时间 (ms)', fontsize=11)
ax.grid(True, alpha=0.3)
ax.axhline(y=30000, color='orange', linestyle=':', linewidth=0.8, alpha=0.5)
ax.annotate(f'误差={pulse_buf}', xy=(s_start, pulse_buf), fontsize=7, color='orange')

# 添加垂直参考线标记各阶段
if s_start > 0:
    for ax in axes:
        ax.axvline(x=s_start, color='gray', linestyle=':', linewidth=0.6, alpha=0.5)

plt.tight_layout()
plt.savefig('c:/Users/28935/Desktop/new/pwm_curve.png', dpi=150, bbox_inches='tight')
print("图片已保存: pwm_curve.png")
plt.show()
