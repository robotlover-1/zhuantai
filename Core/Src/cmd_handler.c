/**
 ****************************************************************************************************
 * @file        cmd_handler.c
 * @brief       串口命令处理函数实现
 * @note        从main.c中拆分出的所有命令处理函数
 ****************************************************************************************************
 */
#include "cmd_handler.h"
#include "app_util.h"
#include "flash_param.h"
#include "rtc.h"
#include "step_motor.h"
#include "usart.h"
#include "usart2.h"
#include "atim.h"
#include "delay.h"
#include "pid.h"

/******************************************************************************************/
/* 外部变量声明 */
/******************************************************************************************/
extern u8 receive_flag;
extern u8 printf_flag;
extern u8 run_flag;
extern u8 run_printf_flag;
extern int error;
extern int alarm;
extern int station_x;
extern int DIR;
extern int start;
extern int out;
extern int Ring;
extern int Ding;
extern int32_t force;
extern int32_t force_F;
extern int time_c;
extern int gravity_ff_amp;
extern int gravity_ff_phase;
extern int32_t pulse_low;
extern int32_t pulse_buf;
extern int time_buf;
extern int32_t pulse_out;
extern int32_t pluse_ele;
extern int loop1;
extern int loop2;
extern int PhotoPos;
extern float SetPoint_P;
extern float SetPoint_C;
extern float SetPluse;
extern float ActualValue_P;
extern int32_t motor_pwm;
extern int duzhuan_flag;
extern int32_t Encode_now;
extern float speed_m;
extern TIM_HandleTypeDef TIM4_Handler;
extern TIM_HandleTypeDef TIM5_Handler;
extern u8 time_run;
extern u8 time_out;
extern UART_HandleTypeDef UART4_Handler;
extern UART_HandleTypeDef UART5_Handler;
extern u8 USART_RX_BUF[];
extern u16 USART_RX_STA;
extern u8 USART_TX_BUF[];
extern u8 USART_TX_BUF_4[];
extern int32_t addr;
extern int32_t writeFlashData;
extern u8  g_eth_ip[4];
extern u8  g_eth_mask[4];
extern u8  g_eth_gw[4];
extern u16 g_eth_port;

/******************************************************************************************/
/* 参数设置类命令*/
/******************************************************************************************/
void cmd_set_time(u8 *buf, int len) /* TIME: 设置RTC时间, 格式: TIME:2026-07-11 14:30:00 */
{
    int y = 2026, mon = 1, d = 1, h = 0, min = 0, sec = 0;
    sscanf((char *)buf, "TIME:%d-%d-%d %d:%d:%d", &y, &mon, &d, &h, &min, &sec);
    if (y < 2025 || y > 2099) y = 2026;
    if (mon < 1 || mon > 12) mon = 1;
    if (d < 1 || d > 31) d = 1;
    if (h < 0 || h > 23) h = 0;
    if (min < 0 || min > 59) min = 0;
    if (sec < 0 || sec > 59) sec = 0;

    rtc_set_time(y, mon, d, h, min, sec);

    /* 保存所有参数到Flash (包含RTC时间备份) */
    param_save_all();

    {
        char ts[32];
        rtc_get_time_str(ts, sizeof(ts));
        printf("RTC时间已设置: %s\r\n", ts);
    }
}

void cmd_set_loop1(u8 *buf, int len) /* T: 大循环次数设置*/
{

    extern int loop1;
    loop1 = parse_uart_number(buf, len);
    printf("loop1:%d\r\n", loop1);
}

void cmd_set_loop2(u8 *buf, int len) /* H: 小循环次数设置 */
{

    extern int loop2;
    loop2 = parse_uart_number(buf, len);
    printf("loop2:%d\r\n", loop2);
}

void cmd_set_force(u8 *buf, int len) /* F: 运行最大速度 */
{
    force = parse_uart_number(buf, len);
    printf("运行中最大速度限制:%d\r\n", force);
    param_save_all();
}

void cmd_set_duty(u8 *buf, int len) /* R: 最大占空比 */
{
    force_F = parse_uart_number(buf, len);
    printf("最大占空比设置:%d\r\n", force_F);
    param_save_all();
}

void cmd_set_speed(u8 *buf, int len) /* V: 最后匀速进孔速度 */
{
    time_c = parse_uart_number(buf, len);
    printf("最后匀速进孔的速度:%d\r\n", time_c);
    param_save_all();
}

void cmd_set_pulse_low(u8 *buf, int len) /* S: 慢速进孔脉冲数 */
{
    pulse_low = parse_uart_number(buf, len);
    printf("慢速运行进孔脉冲数:%d\r\n", pulse_low);
    param_save_all();
}

void cmd_set_pulse_buf(u8 *buf, int len) /* BS: 缓冲段起始脉冲数 */
{
    int raw = 0;
    sscanf((char *)buf, "BS:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 50000) raw = 50000;
    pulse_buf = raw;
    printf("缓冲段起始脉冲数:%d (当前慢速段=%d)\r\n", pulse_buf, pulse_low);
    param_save_all();
}

void cmd_set_speed_buf(u8 *buf, int len) /* BV: 缓冲段速度 */
{
    int raw = 0;
    sscanf((char *)buf, "BV:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 1000) raw = 1000;
    time_buf = raw;
    printf("缓冲段速度:%d (进孔速度=%d, 最大速度=%d)\r\n", time_buf, time_c, force);
    param_save_all();
}

void cmd_set_ff_amp(u8 *buf, int len) /* FA: 重力前馈正弦幅值 */
{
    int raw = 0;
    sscanf((char *)buf, "FA:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 500) raw = 500;
    gravity_ff_amp = raw;
    printf("重力前馈幅值:%d (正弦波峰值PWM, 0=关闭)\r\n", gravity_ff_amp);
    param_save_all();
}

void cmd_set_ff_phase(u8 *buf, int len) /* FP: 重力前馈相位偏移 */
{
    int raw = 0;
    sscanf((char *)buf, "FP:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 195999) raw = 195999;
    gravity_ff_phase = raw;
    printf("重力前馈相位:%d (编码器脉冲, 0~195999, 对准'最陡上坡'孔位记enc)\r\n", gravity_ff_phase);
    param_save_all();
}

void cmd_set_photo_pos(u8 *buf, int len) /* P: 拍照孔位 */
{
    PhotoPos = parse_uart_number(buf, len);
    printf("拍照孔位:%d\r\n", PhotoPos);
}

void cmd_set_pluse_ele(u8 *buf, int len) /* E: 光电检测限制脉冲数 */
{
    pluse_ele = parse_uart_number(buf, len);
    printf("光电检测限制脉冲数:%d\r\n", pluse_ele);
    param_save_all();
}

void cmd_set_kp(u8 *buf, int len) /* KP: 位置环P增益 (整数×1000, 如 KP:30 = 0.030) */
{
    extern float g_pid_kp;
    int raw = 0;
    sscanf((char *)buf, "KP:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 3000) raw = 3000;
    g_pid_kp = raw * 0.001f;
    printf("位置环KP: %.3f\r\n", g_pid_kp);
    param_save_all();
}

void cmd_set_ki(u8 *buf, int len) /* KI: 位置环I增益 (整数×1000) */
{
    extern float g_pid_ki;
    int raw = 0;
    sscanf((char *)buf, "KI:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 1000) raw = 1000;
    g_pid_ki = raw * 0.001f;
    printf("位置环KI: %.3f\r\n", g_pid_ki);
    param_save_all();
}

void cmd_set_kd(u8 *buf, int len) /* KD: 位置环D增益 (整数×1000) */
{
    extern float g_pid_kd;
    int raw = 0;
    sscanf((char *)buf, "KD:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 1000) raw = 1000;
    g_pid_kd = raw * 0.001f;
    printf("位置环KD: %.3f\r\n", g_pid_kd);
    param_save_all();
}

void cmd_set_skp(u8 *buf, int len) /* SK: 速度环P增益 (整数×1000, 如 SK:3515 = 3.515) */
{
    extern float g_pid_skp;
    int raw = 0;
    sscanf((char *)buf, "SK:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 50000) raw = 50000;
    g_pid_skp = raw * 0.001f;
    printf("速度环S_KP: %.3f\r\n", g_pid_skp);
    param_save_all();
}

void cmd_set_ski(u8 *buf, int len) /* SI: 速度环I增益 (整数×1000, 如 SI:515 = 0.515) */
{
    extern float g_pid_ski;
    int raw = 0;
    sscanf((char *)buf, "SI:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 50000) raw = 50000;
    g_pid_ski = raw * 0.001f;
    printf("速度环S_KI: %.3f\r\n", g_pid_ski);
    param_save_all();
}

void cmd_set_skd(u8 *buf, int len) /* SD: 速度环D增益 (整数×1000, 如 SD:50 = 0.050) */
{
    extern float g_pid_skd;
    int raw = 0;
    sscanf((char *)buf, "SD:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 50000) raw = 50000;
    g_pid_skd = raw * 0.001f;
    printf("速度环S_KD: %.3f\r\n", g_pid_skd);
    param_save_all();
}

void cmd_set_pc(u8 *buf, int len) /* PC: PID采样周期(ms) */
{
    extern int g_pid_period_ms;
    int raw = 0;
    sscanf((char *)buf, "PC:%d", &raw);
    if (raw < 5) raw = 5;
    if (raw > 100) raw = 100;
    g_pid_period_ms = raw;
    printf("PID采样周期: %dms\r\n", g_pid_period_ms);
    param_save_all();
}

/******************************************************************************************/
/* 控制类命令*/
/******************************************************************************************/
void cmd_ele_test(void) /* ELE: 光电信号检测*/
{
    printf("光电信号 %d\r\n",
           HAL_GPIO_ReadPin(ST1_ELE_GPIO_PORT, ST1_ELE_GPIO_PIN));
}

void cmd_photo_test(void) /* OP: 拍照测试 */
{
    printf("进入拍照\r\n");
    trigger_photo();
}

void cmd_clc(void) /* CLC: 报警清除 */
{
    error = 0;
    alarm = 0;
}

void cmd_motor_init(void) /* D0: 电机初始化*/
{
    motor_init();
}

void cmd_hal_test(void) /* HAL: 霍尔位置检测*/
{
    station_x = read_hal();
    printf("station: %d\r\n", station_x);
    printf("HAL1: %d \r\n",
           HAL_GPIO_ReadPin(ST1_HAL1_GPIO_PORT, ST1_HAL1_GPIO_PIN));
    printf("HAL2: %d \r\n",
           HAL_GPIO_ReadPin(ST1_HAL2_GPIO_PORT, ST1_HAL2_GPIO_PIN));
    printf("HAL3: %d \r\n",
           HAL_GPIO_ReadPin(ST1_HAL3_GPIO_PORT, ST1_HAL3_GPIO_PIN));
}

void cmd_red(void) /* RED: 读取当前位置 */
{
    while (1)
    {
        printf("Pos: %d \r\n", Encode_now);
        delay_ms(50);
    }
}

void cmd_rtf(void) /* RTF: 正转 */
{
    step_motor_motion(1, 0);
    dcmotor_start();
}

void cmd_rtb(void) /* RTB: 反转 */
{
    step_motor_motion(1, 1);
}

void cmd_tim4_test(void) /* T4: TIM4计时测试(1Hz) - 已被T1-T5取代，保留T6备用 */
{

    extern u8 time_run;
    printf("T4已被工位测试取代，请用T6测试定时器\r\n");
}

void cmd_tim5_test(void) /* T5: TIM5超时测试(0.5s) - 已被T1-T5取代 */
{
    printf("T5已被工位测试取代，请用T6测试定时器\r\n");
}

void cmd_tim6_test(void) /* T6: TIM6 PID周期测试(1ms) */
{

    extern int k_loop;
    int cnt_start, cnt_end;
    printf("TIM6测试 2秒(1ms中断, PID每25ms循环)...\r\n");
    cnt_start = k_loop;
    delay_ms(2000);
    cnt_end = k_loop;
    printf("k_loop增量=%d (期望~80, 25ms×80=2000ms)\r\n", cnt_end - cnt_start);
}

/********************* 工位往返测试 T1-T5 *********************/
static void cmd_station_roundtrip(int base)
{
    int i, target;
    printf("工位%d往返测试开始\r\n", base);
    error = 0; run_flag = 0; run_printf_flag = 0;
    station_x = read_hal();
    if(station_x != base){
        char ts[32]; rtc_get_time_str(ts, sizeof(ts));
printf("[%s] 当前位置:%d, 需要先回到工位%d\r\n", ts, station_x, base);
        step_motor_LX(base, dir_jud(base, station_x));
        wait_motion_done();
    }

    for (i = 1; i <= 5; i++)
    {
        if (i == base) continue;
        target = i;
        if (error) { printf("出错停止\r\n"); break; }

        /* 从base到target */
        printf("→ 工位%d\n", target);
        station_x = read_hal();
        if (station_x < 1 || station_x > 5) { printf("HAL错误\r\n"); break; }
        DIR = dir_jud(target, station_x);
        step_motor_LX(target, DIR);
        wait_motion_done();
        delay_ms(10000);

        /* 从target回到base */
        if (error) { printf("出错停止\r\n"); break; }
        printf("← 工位%d\n", base);
        station_x = read_hal();
        if (station_x < 1 || station_x > 5) { printf("HAL错误\r\n"); break; }
        DIR = dir_jud(base, station_x);
        step_motor_LX(base, DIR);
        wait_motion_done();
        if (i < 5) delay_ms(10000);  /* 最后一次往返后不等待 */
    }

    printf("工位%d往返测试结束\r\n", base);
}

void cmd_test_1(void) { cmd_station_roundtrip(1); }

void cmd_test_2(void) { cmd_station_roundtrip(2); }

void cmd_test_3(void) { cmd_station_roundtrip(3); }

void cmd_test_4(void) { cmd_station_roundtrip(4); }

void cmd_test_5(void) { cmd_station_roundtrip(5); }

void cmd_auto_fp(void) /* AUTOFP: 自动标定重力前馈FP */
{
    extern TIM_HandleTypeDef g_timx_pwm_chy_handle;
    extern int32_t Encode_now;
    extern int gravity_ff_amp;
    extern int gravity_ff_phase;
    int i, best_station = 0;
    int32_t best_enc = 0, min_delta = 0x7FFFFFFF;
    int32_t start_enc[6] = {0}, delta[6] = {0};  /* 1-indexed */

    printf("=== AUTOFP 自动标定重力前馈相位 ===\r\n");

    /* 保存当前位置 */
    int save_station = read_hal();
    if (save_station < 1 || save_station > 5) {
        printf("错误: 无法读取当前孔位 (HAL=%d)\r\n", save_station);
        return;
    }
    printf("当前位置: 孔位%d, 即将遍历5个孔位...\r\n", save_station);

    /* 遍历5个孔位 */
    for (i = 1; i <= 5; i++) {
        int cur = read_hal();
        if (cur != i) {
            printf("→ 移动到孔位%d...\r\n", i);
            int d = dir_jud(i, cur);
            error = 0; run_flag = 0; run_printf_flag = 0;
            step_motor_LX(i, d);
            wait_motion_done();
            if (error) {
                printf("孔位%d移动失败, 跳过\r\n", i);
                continue;
            }
        }

        delay_ms(300);  /* 等电机稳定 */

        /* DC脉冲测试: PWM=2000, 200ms; 然后反向脉冲退回原位避免丢位置 */
        start_enc[i] = (int32_t)Encode_now;

        run_flag = 0;  /* 确保PID不干扰 */
        HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_4);
        dcmotor_start();

        /* 正向脉冲, 测量Δenc */
        motor_pwm_set(2000);
        delay_ms(200);
        motor_pwm_set(0);

        int32_t end_enc = (int32_t)Encode_now;
        delta[i] = end_enc - start_enc[i];
        if (delta[i] < 0) delta[i] = -delta[i];

        printf("孔位%d: enc=%d → %d, Δ=%d (前向)\r\n",
               i, (int)start_enc[i], (int)end_enc, (int)delta[i]);

        /* 反向脉冲退回去 (同样200ms, 避免偏离孔位) */
        motor_pwm_set(-2000);
        delay_ms(200);
        motor_pwm_set(0);
        dcmotor_stop();

        if (delta[i] < min_delta) {
            min_delta = delta[i];
            best_station = i;
            best_enc = start_enc[i];
        }
    }

    if (best_station == 0) {
        printf("错误: 无有效数据, 标定失败\r\n");
        return;
    }

    /* 计算FP */
    {
        int32_t enc_mod = best_enc % 196000;
        if (enc_mod < 0) enc_mod += 196000;
        gravity_ff_phase = enc_mod;
    }

    /* 如果幅值未设, 给默认值 */
    if (gravity_ff_amp == 0) {
        gravity_ff_amp = 80;
        printf("FA未设置, 自动设为: 80\r\n");
    }

    printf("--- 标定结果 ---\r\n");
    printf("最陡上坡孔位: %d, enc=%d, enc%%196000=%d\r\n",
           best_station, (int)best_enc, gravity_ff_phase);
    printf("各孔位Δenc: ");
    for (i = 1; i <= 5; i++) printf("%d:%d  ", i, (int)delta[i]);
    printf("\r\n");
    printf("FP: %d, FA: %d\r\n", gravity_ff_phase, gravity_ff_amp);

    param_save_all();

    /* 回到原位 */
    {
        int cur = read_hal();
        if (cur != save_station) {
            printf("← 回到孔位%d...\r\n", save_station);
            int d = dir_jud(save_station, cur);
            error = 0; run_flag = 0; run_printf_flag = 0;
            step_motor_LX(save_station, d);
            wait_motion_done();
        }
    }
    printf("=== AUTOFP 完成 ===\r\n");
}

/**************************************************************/
void cmd_tim3(void) /* TIM3: 保留旧命令, 实际调T4 */
{

    extern u8 time_run;
    HAL_TIM_Base_Start_IT(&TIM4_Handler);
    delay_ms(5000);
    HAL_TIM_Base_Stop_IT(&TIM4_Handler);
    printf("time_run: %d\r\n", time_run);
}

void cmd_ring(void) /* RING: 上升沿检测*/
{
    printf("上升沿检测测试！\r\n");
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void cmd_rts(void) /* RTS: 恢复初始位置 */
{
    SetPoint_P = 0;
    dcmotor_stop();
    motor_pwm = 0;
    motor_pwm_set(motor_pwm);
    run_flag = 0;
}

void cmd_dc_motor(u8 *buf, int len) /* DC:<pwm>,<dur_ms> 直接PWM输出(标定用) */
{
    extern TIM_HandleTypeDef g_timx_pwm_chy_handle;
    extern int32_t Encode_now;
    int pwm_val = 0, dur_ms = 0;

    sscanf((char *)buf, "DC:%d,%d", &pwm_val, &dur_ms);

    /* 安全检查 */
    if (dur_ms < 0)  dur_ms = 0;
    if (dur_ms > 10000) dur_ms = 10000;   /* 最长10秒 */
    if (pwm_val >  3500) pwm_val =  3500;
    if (pwm_val < -3500) pwm_val = -3500;

    printf("DC: PWM=%d, %dms, %s, 当前enc=%d\r\n",
           pwm_val >= 0 ? pwm_val : -pwm_val, dur_ms,
           pwm_val >= 0 ? "正转" : "反转", (int)Encode_now);

    if (dur_ms == 0) dur_ms = 100;  /* 默认100ms */

    /* 强制停止PID闭环，直接输出PWM */
    run_flag = 0;
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_4);
    dcmotor_start();
    motor_pwm_set((int32_t)pwm_val);

    delay_ms(dur_ms);

    motor_pwm_set(0);
    dcmotor_stop();
    printf("DC: 完成, 结束enc=%d\r\n", (int)Encode_now);
}

void cmd_res(void) /* RES: 串口更新程序 */
{
    writeFlashData = 1;
    param_save_all();
    HAL_NVIC_SystemReset();
}

void cmd_show_params(void) /* PARA: 显示所有参数 */
{
    extern float g_pid_kp;
    extern float g_pid_ki;
    extern float g_pid_kd;
    extern float g_pid_skp;
    extern float g_pid_ski;
    extern float g_pid_skd;
    extern int   g_pid_period_ms;
    printf("====== 当前参数 ======\r\n");
    printf("F(最大速度): %d\r\n", force);
    printf("R(最大占空比): %d\r\n", force_F);
    printf("V(匀速进孔速度): %d\r\n", time_c);
    printf("BS(缓冲段脉冲数): %d\r\n", pulse_buf);
    printf("BV(缓冲段速度): %d\r\n", time_buf);
    printf("S(慢速进孔脉冲数): %d\r\n", pulse_low);
    printf("E(光电检测限制): %d\r\n", pluse_ele);
    printf("P(拍照孔位): %d\r\n", PhotoPos);
    printf("T(大循环次数): %d\r\n", loop1);
    printf("H(小循环次数): %d\r\n", loop2);
    printf("--- PID参数 ---\r\n");
    printf("PC(采样周期): %dms\r\n", g_pid_period_ms);
    printf("KP(位置环P): %.3f  KI: %.3f  KD: %.3f\r\n", g_pid_kp, g_pid_ki, g_pid_kd);
    printf("SK(速度环P): %.3f  SI: %.3f  SD: %.3f\r\n", g_pid_skp, g_pid_ski, g_pid_skd);
    printf("--- 重力前馈 ---\r\n");
    printf("FA(正弦幅值): %d  FP(相位偏移): %d\r\n", gravity_ff_amp, gravity_ff_phase);
    printf("--- 网络配置 ---\r\n");
    printf("IP: %d.%d.%d.%d\r\n",
           g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);
    printf("MASK: %d.%d.%d.%d\r\n",
           g_eth_mask[0], g_eth_mask[1], g_eth_mask[2], g_eth_mask[3]);
    printf("GW: %d.%d.%d.%d\r\n",
           g_eth_gw[0], g_eth_gw[1], g_eth_gw[2], g_eth_gw[3]);
    printf("PORT: %d\r\n", g_eth_port);
    printf("====================\r\n");
    printf("Alarm: %d  Error: %d\r\n", alarm, error);
    printf("Encode: %d  RunFlag: %d\r\n", Encode_now, run_flag);
}

void cmd_alarm(void) /* ALARM: 查询报警代码 */
{
    printf("报警代码:%d\r\n", alarm);
}

void cmd_stat(void) /* STAT: 运动统计 */
{
    extern int undershoot_cnt;
    extern int overshoot_cnt;
    extern int passhole_cnt;
    printf("====== 运动统计 ======\r\n");
    printf("欠冲: %d\r\n", undershoot_cnt);
    printf("过冲: %d\r\n", overshoot_cnt);
    printf("过孔位: %d\r\n", passhole_cnt);
    printf("====================\r\n");
}

void cmd_stat_clear(void) /* STATC: 清除运动统计 */
{
    extern int undershoot_cnt;
    extern int overshoot_cnt;
    extern int passhole_cnt;
    undershoot_cnt = 0;
    overshoot_cnt = 0;
    passhole_cnt = 0;
    printf("运动统计已清除\r\n");
}

void cmd_cycle(void) /* CYCLE: 查询总转动计数 (只读, 不可清零, 用于耐久验证) */
{
    extern int32_t total_cycles;
    printf("总转动孔位计数: %d\r\n", total_cycles);
}

void cmd_save_cycle(void) /* SAVEC: 手动保存 total_cycles 到Flash (避免每次自动写Flash, 延长寿命) */
{
    extern int32_t total_cycles;
    param_save_all();
    printf("总转动计数已保存: %d\r\n", total_cycles);
}

/******************************************************************************************/
/* 工位跳转 */
/******************************************************************************************/
void cmd_goto_station(int station) /* L1-L5: 跳转到指定工位*/
{
    if (run_flag == 0)
    {
        station_x = read_hal();
        if (station_x == station)
        {
            printf("已在工位%d\r\n", station);
            return;
        }

        DIR = dir_jud(station, station_x);
        step_motor_LX(station, DIR);
    }
}

/******************************************************************************************/
/* 测试模式 */
/******************************************************************************************/
/**
 * @brief       gnn: 磨合测试(5站循环）
 * @note        5站间往返运动，每站间往返loop2次，大循环loop1次
 */

void cmd_gnn_test(void)
{

    extern u8 time_run;
    int j3 = 0, j4 = 0;
    USART_RX_STA = 0;
    USART_RX_BUF[0] = 0;
    time_run = 0;
    HAL_TIM_Base_Start_IT(&TIM4_Handler); // 开始磨合计时

    /* 5站间循环表格 */
    const uint8_t station_path[5] = {1, 2, 3, 4, 5};
    for (j4 = 0; j4 <= loop1; j4++)
    {
        if (error == 1)
        {
            HAL_TIM_Base_Stop_IT(&TIM4_Handler);
            break;
        }

        int s = 0;
        for (s = 0; s < 5; s++)
        {
            if (error == 1)
            {
                HAL_TIM_Base_Stop_IT(&TIM4_Handler);
                break;
            }

            /* 正转到下一站*/
            step_motor_motion(1, 0);
            wait_motion_done();
            delay_ms(500);

            /* 下一站与本站之间往返loop2次*/
            for (j3 = 0; j3 <= loop2; j3++)
            {
                if (error == 1)
                {
                    HAL_TIM_Base_Stop_IT(&TIM4_Handler);
                    break;
                }

                step_motor_motion(1, 0); // 正转
                wait_motion_done();
                delay_ms(500);
                if (error == 1)
                {
                    HAL_TIM_Base_Stop_IT(&TIM4_Handler);
                    break;
                }

                step_motor_motion(1, 1); // 反转
                wait_motion_done();
                delay_ms(500);
            }

            printf("Station %d & %d loop done: %d\r\n", station_path[s],
                   station_path[(s + 1) % 5], loop2);
        }

        printf("总循环次数：%d\r\n", j4);
    }

    HAL_TIM_Base_Stop_IT(&TIM4_Handler);
    printf("time_run: %d\r\n", time_run);
}

/**
 * @brief       goo: 自动测试
 * @note        loop2次往返拍照+IO输出
 */

void cmd_goo_test(void)
{
    int j3 = 0, j4 = 0, j5 = 0;
    int station_k1 = 0;
    for (j3 = 0; j3 <= loop1; j3++)
    {
        if (error == 1) break;
        station_k1 = read_hal();
        for (j5 = 1; j5 <= 5; j5++) // 工位循环
        {
            if (error == 1) break;
            if (station_k1 == j5)
            {
                // 如果在当前工位，跳过
            }
            else
            {
                for (j4 = 0; j4 <= loop2; j4++) // 工位往返
                {
                    if (error == 1) break;
                    station_k1 = read_hal();
                    DIR = dir_jud(j5, station_k1);
                    step_motor_LX(j5, DIR);
                    wait_motion_done();
                    delay_ms(500);
                    if (error == 1) break;
                    DIR = dir_jud(station_k1, j5);
                    step_motor_LX(station_k1, DIR);
                    wait_motion_done();
                    delay_ms(3000);
                    // PC5输出拍照信号(由trigger_photo内部完成)
                    trigger_photo();
                    delay_ms(2000);
                }
            }
        }
    }

    USART_RX_STA = 0;
}

/**
 * @brief       循环正转并拍照（在指定孔位触发拍照）
 */

static void gff_gmm_loop(int dir)
{
    DIR = dir;
    while (1)
    {
        step_motor_motion(1, dir);
        wait_motion_done();
        delay_ms(1000);
        station_x = read_hal();
        if (station_x == PhotoPos)
        {
            delay_ms(3000);
            trigger_photo();
            delay_ms(2000);
        }
    }
}

void cmd_gff_test(void) /* gff: 正转拍照测试 */
{
    gff_gmm_loop(1);
}

void cmd_gmm_test(void) /* gmm: 反转拍照测试 */
{
    gff_gmm_loop(0);
}

/********************* 波形采集 DBG *********************/
#define DBG_MAX  300

static int32_t dbg_pos[DBG_MAX];
static float   dbg_spd[DBG_MAX];
static int     dbg_cnt = 0;

void cmd_dbg_start(void) /* DBG: 采集一次正向运动的编码器数据 */
{
    printf("DBG 开始采集(250ms间隔, 最多%d点)...\r\n", DBG_MAX);
    error = 0; run_flag = 0; run_printf_flag = 0;
    if (ele_is_triggered() && run_printf_flag == 0 && error == 0 && run_flag == 0)
    {
        DIR = 0;
        motor_start_motion();
        SetPoint_C = Encode_now;
        SetPoint_P = Encode_now + 43000;
        run_flag = 1;
        dbg_cnt = 0;
        while (1)
        {
            if (dbg_cnt < DBG_MAX) {
                dbg_pos[dbg_cnt] = Encode_now;
                dbg_spd[dbg_cnt] = speed_m;
                dbg_cnt++;
            }

            delay_ms(25);  /* 25ms采样间隔，与PID同步 */
            if (run_printf_flag == 0) break;  /* 运动结束 */
            if (error) break;
        }

        /* 等待安全停稳 */
        delay_ms(200);
    }

    printf("DBG 采集结束, 共%d个点\r\n", dbg_cnt);
}

void cmd_dbg_dump(void) /* DUMP: 打印采集的数据(CSV格式) */
{
    int i;
    printf("=== DBG DATA (CSV) ===\n");
    printf("idx,pos,speed\n");
    for (i = 0; i < dbg_cnt; i++) {
        printf("%d,%d,%.0f\n", i, (int)dbg_pos[i], (double)dbg_spd[i]);
    }

    printf("=== DBG END ===\n");
}

void cmd_dp(u8 *buf, int len) /* DP: 定时打印PID状态 (ms) */
{
    extern int dbg_print_ms;
    int raw = 0;
    sscanf((char *)buf, "DP:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 10000) raw = 10000;
    dbg_print_ms = raw;
    if (dbg_print_ms > 0)
        printf("调试打印: 每%dms输出一次 PID状态 (DP:0 关闭)\r\n", dbg_print_ms);
    else
        printf("调试打印: 已关闭\r\n");
}

/**
 * @brief  IP: 设置本机IP地址
 *         格式: IP:192.168.1.100
 */
void cmd_set_ip(u8 *buf, int len)
{
    int a = 192, b = 168, c = 1, d = 100;
    (void)len;
    sscanf((char *)buf, "IP:%d.%d.%d.%d", &a, &b, &c, &d);
    if (a < 0 || a > 255) a = 192;
    if (b < 0 || b > 255) b = 168;
    if (c < 0 || c > 255) c = 1;
    if (d < 0 || d > 255) d = 100;
    g_eth_ip[0] = (u8)a;
    g_eth_ip[1] = (u8)b;
    g_eth_ip[2] = (u8)c;
    g_eth_ip[3] = (u8)d;
    printf("IP 已设置为: %d.%d.%d.%d (需发送RES复位生效)\r\n", a, b, c, d);
    param_save_all();
}

/**
 * @brief  MASK: 设置子网掩码
 *         格式: MASK:255.255.255.0
 */
void cmd_set_mask(u8 *buf, int len)
{
    int a = 255, b = 255, c = 255, d = 0;
    (void)len;
    sscanf((char *)buf, "MASK:%d.%d.%d.%d", &a, &b, &c, &d);
    g_eth_mask[0] = (u8)a;
    g_eth_mask[1] = (u8)b;
    g_eth_mask[2] = (u8)c;
    g_eth_mask[3] = (u8)d;
    printf("MASK 已设置为: %d.%d.%d.%d (需发送RES复位生效)\r\n", a, b, c, d);
    param_save_all();
}

/**
 * @brief  GW: 设置默认网关
 *         格式: GW:192.168.1.1
 */
void cmd_set_gw(u8 *buf, int len)
{
    int a = 192, b = 168, c = 1, d = 1;
    (void)len;
    sscanf((char *)buf, "GW:%d.%d.%d.%d", &a, &b, &c, &d);
    g_eth_gw[0] = (u8)a;
    g_eth_gw[1] = (u8)b;
    g_eth_gw[2] = (u8)c;
    g_eth_gw[3] = (u8)d;
    printf("GW 已设置为: %d.%d.%d.%d (需发送RES复位生效)\r\n", a, b, c, d);
    param_save_all();
}

/**
 * @brief  PORT: 设置TCP监听端口
 *         格式: PORT:5000
 */
void cmd_set_port(u8 *buf, int len)
{
    int port = 5000;
    (void)len;
    sscanf((char *)buf, "PORT:%d", &port);
    if (port < 1 || port > 65535) port = 5000;
    g_eth_port = (u16)port;
    printf("PORT 已设置为: %d (需发送RES复位生效)\r\n", port);
    param_save_all();
}

/******************************************************************************************/
/* 命令分发函数 */
/******************************************************************************************/
/**
 * @brief       串口命令分发
 * @param       buf: 接收缓冲区
 * @param       len: 数据长度
 * @retval      1-命令已处理 0-无法识别
 */

int cmd_dispatch(u8 *buf, int len)
{

    /* 参数设置类(X:num格式) */
    if (buf[0] == 'T' && buf[1] == 'I' && buf[2] == 'M' && buf[3] == 'E' && buf[4] == ':')
    {
        cmd_set_time(buf, len);
        return 1;
    }

    if (buf[0] == 'T' && buf[1] == ':')       /* T: 大循环次数*/
    {
        //printf("T:%.*s\r\n", len, buf);
        cmd_set_loop1(buf, len);
        return 1;
    }

    if (buf[0] == 'H' && buf[1] == ':')       /* H: 小循环次数 */
    {
        //printf("H:%.*s\r\n", len, buf);
        cmd_set_loop2(buf, len);
        return 1;
    }

    if (buf[0] == 'F' && buf[1] == ':')       /* F: 运行最大速度 */
    {
        //printf("F:%.*s\r\n", len, buf);
        cmd_set_force(buf, len);
        return 1;
    }

    if (buf[0] == 'R' && buf[1] == ':')       /* R: 最大占空比 */
    {
        //printf("R:%.*s\r\n", len, buf);
        cmd_set_duty(buf, len);
        return 1;
    }

    if (buf[0] == 'D' && buf[1] == 'C' && buf[2] == ':')   /* DC:<pwm>,<ms> 直接PWM输出 */
    {
        cmd_dc_motor(buf, len);
        return 1;
    }

    if (buf[0] == 'V' && buf[1] == ':')       /* V: 最后匀速进孔速度 */
    {
        //printf("V:%.*s\r\n", len, buf);
        cmd_set_speed(buf, len);
        return 1;
    }

    if (buf[0] == 'S' && buf[1] == ':')       /* S: 慢速进孔脉冲数 */
    {
        //printf("S:%.*s\r\n", len, buf);
        cmd_set_pulse_low(buf, len);
        return 1;
    }

    /* 网络参数设置 (IP/MASK/GW/PORT) */
    if (buf[0] == 'I' && buf[1] == 'P' && buf[2] == ':')
    {
        cmd_set_ip(buf, len);
        return 1;
    }
    if (buf[0] == 'M' && buf[1] == 'A' && buf[2] == 'S' && buf[3] == 'K' && buf[4] == ':')
    {
        cmd_set_mask(buf, len);
        return 1;
    }
    if (buf[0] == 'G' && buf[1] == 'W' && buf[2] == ':')
    {
        cmd_set_gw(buf, len);
        return 1;
    }
    if (buf[0] == 'P' && buf[1] == 'O' && buf[2] == 'R' && buf[3] == 'T' && buf[4] == ':')
    {
        cmd_set_port(buf, len);
        return 1;
    }

    if (buf[0] == 'P' && buf[1] == ':')       /* P: 拍照孔位 */
    {
        //printf("P:%.*s\r\n", len, buf);
        cmd_set_photo_pos(buf, len);
        return 1;
    }

    if (buf[0] == 'E' && buf[1] == ':')       /* E: 光电检测限制脉冲数 */
    {
        //printf("E:%.*s\r\n", len, buf);
        cmd_set_pluse_ele(buf, len);
        return 1;
    }

    /* 三段式缓冲段参数 */
    if (buf[0] == 'B' && buf[1] == 'S' && buf[2] == ':')    /* BS: 缓冲段起始脉冲数 */
    {
        cmd_set_pulse_buf(buf, len);
        return 1;
    }

    if (buf[0] == 'B' && buf[1] == 'V' && buf[2] == ':')    /* BV: 缓冲段速度 */
    {
        cmd_set_speed_buf(buf, len);
        return 1;
    }

    if (buf[0] == 'F' && buf[1] == 'A' && buf[2] == ':')    /* FA: 重力前馈正弦幅值 */
    {
        cmd_set_ff_amp(buf, len);
        return 1;
    }

    if (buf[0] == 'F' && buf[1] == 'P' && buf[2] == ':')    /* FP: 重力前馈相位偏移 */
    {
        cmd_set_ff_phase(buf, len);
        return 1;
    }

    /* PID参数设置类 (KP/KI/KD 整数×1000, SK/SI/SD 整数×100) */
    if (buf[0] == 'K' && buf[1] == 'P' && buf[2] == ':')    /* KP: 位置环P增益 */
    {
        cmd_set_kp(buf, len);
        return 1;
    }

    if (buf[0] == 'K' && buf[1] == 'I' && buf[2] == ':')    /* KI: 位置环I增益 */
    {
        cmd_set_ki(buf, len);
        return 1;
    }

    if (buf[0] == 'K' && buf[1] == 'D' && buf[2] == ':')    /* KD: 位置环D增益 */
    {
        cmd_set_kd(buf, len);
        return 1;
    }

    if (buf[0] == 'S' && buf[1] == 'K' && buf[2] == ':')    /* SK: 速度环P增益 */
    {
        cmd_set_skp(buf, len);
        return 1;
    }

    if (buf[0] == 'S' && buf[1] == 'I' && buf[2] == ':')    /* SI: 速度环I增益 */
    {
        cmd_set_ski(buf, len);
        return 1;
    }

    if (buf[0] == 'S' && buf[1] == 'D' && buf[2] == ':')    /* SD: 速度环D增益 */
    {
        cmd_set_skd(buf, len);
        return 1;
    }

    if (buf[0] == 'P' && buf[1] == 'C' && buf[2] == ':')    /* PC: PID采样周期 */
    {
        cmd_set_pc(buf, len);
        return 1;
    }

    /* 控制类命令(多字符命令 */
    if (buf[0] == 'E' && buf[1] == 'L' && buf[2] == 'E')
    {
        //printf("ELE:%.*s\r\n", len, buf);
        cmd_ele_test();
        return 1;
    }

    if (buf[0] == 'O' && buf[1] == 'P')
    {
        //printf("OP:%.*s\r\n", len, buf);
        cmd_photo_test();
        return 1;
    }

    if (buf[0] == 'C' && buf[1] == 'L' && buf[2] == 'C')
    {
        //printf("CLC:%.*s\r\n", len, buf);
        //printf("CLC:%.*s\r\n", len, buf);
        cmd_clc();
        return 1;
    }

    if (buf[0] == 'D' && buf[1] == '0')
    {
        //printf("D0:%.*s\r\n", len, buf);
        cmd_motor_init();
        return 1;
    }

    /* 测试类命令(多字符) */
    if (buf[0] == 'H' && buf[1] == 'A' && buf[2] == 'L')
    {
        //printf("HAL:%.*s\r\n", len, buf);
        cmd_hal_test();
        return 1;
    }

    if (buf[0] == 'R' && buf[1] == 'E' && buf[2] == 'D')
    {
        //printf("RED:%.*s\r\n", len, buf);
        cmd_red();
        return 1;
    }

    if (buf[0] == 'R' && buf[1] == 'T' && buf[2] == 'F')
    {
       // printf("RTF:%.*s\r\n", len, buf);
        cmd_rtf();
        return 1;
    }

    if (buf[0] == 'R' && buf[1] == 'T' && buf[2] == 'B')
    {
        //printf("RTB:%.*s\r\n", len, buf);
        cmd_rtb();
        return 1;
    }

    if (buf[0] == 'T' && buf[1] == 'I' && buf[2] == 'M' && buf[3] == '3')
    {
        //printf("TIM3:%.*s\r\n", len, buf);
        cmd_tim3();
        return 1;
    }

    if (buf[0] == 'T' && buf[1] == '1')
    {
        cmd_test_1();
        return 1;
    }

    if (buf[0] == 'T' && buf[1] == '2')
    {
        cmd_test_2();
        return 1;
    }

    if (buf[0] == 'T' && buf[1] == '3')
    {
        cmd_test_3();
        return 1;
    }

    if (buf[0] == 'T' && buf[1] == '4')
    {
        cmd_test_4();
        return 1;
    }

    if (buf[0] == 'T' && buf[1] == '5')
    {
        cmd_test_5();
        return 1;
    }

    if (buf[0] == 'A' && buf[1] == 'U' && buf[2] == 'T' && buf[3] == 'O' && buf[4] == 'F' && buf[5] == 'P')
    {
        cmd_auto_fp();
        return 1;
    }

    if (buf[0] == 'T' && buf[1] == '6')
    {
        cmd_tim6_test();
        return 1;
    }

    if (buf[0] == 'R' && buf[1] == 'I' && buf[2] == 'N' && buf[3] == 'G')
    {
        //printf("RING:%.*s\r\n", len, buf);
        cmd_ring();
        return 1;
    }

    if (buf[0] == 'R' && buf[1] == 'T' && buf[2] == 'S')
    {
        //printf("RTS:%.*s\r\n", len, buf);
        cmd_rts();
        return 1;
    }

    if (buf[0] == 'R' && buf[1] == 'E' && buf[2] == 'S')
    {
        //printf("RES:%.*s\r\n", len, buf);
        cmd_res();
        return 1;
    }

    if (buf[0] == 'A' && buf[1] == 'L' && buf[2] == 'A' &&
        buf[3] == 'R' && buf[4] == 'M')
    {
        //printf("ALARM:%.*s\r\n", len, buf);
        cmd_alarm();
        return 1;
    }

    if (buf[0] == 'S' && buf[1] == 'T' && buf[2] == 'A' && buf[3] == 'T')
    {
        if (buf[4] == 'C')      /* STATC: 清除运动统计 */
            cmd_stat_clear();
        else                     /* STAT:  运动统计 */
            cmd_stat();
        return 1;
    }

    if (buf[0] == 'S' && buf[1] == 'A' && buf[2] == 'V' && buf[3] == 'E' && buf[4] == 'C')
    {
        cmd_save_cycle();
        return 1;
    }

    if (buf[0] == 'C' && buf[1] == 'Y' && buf[2] == 'C' && buf[3] == 'L' && buf[4] == 'E')
    {
        cmd_cycle();
        return 1;
    }

    if (buf[0] == 'P' && buf[1] == 'A' && buf[2] == 'R' && buf[3] == 'A')
    {
        cmd_show_params();
        return 1;
    }

    /* 工位跳转 */
    if (buf[0] == 'L' && buf[1] >= '1' && buf[1] <= '5')
    {
        //printf("L%c:%.*s\r\n", buf[1], len, buf);
        cmd_goto_station(buf[1] - '0');
        return 1;
    }

    /* 测试模式 */
    if (buf[0] == 'g' && buf[1] == 'n' && buf[2] == 'n')
    {
        //printf("gnn:%.*s\r\n", len, buf);
        cmd_gnn_test();
        return 1;
    }

    if (buf[0] == 'g' && buf[1] == 'o' && buf[2] == 'o')
    {
        //printf("goo:%.*s\r\n", len, buf);
        cmd_goo_test();
        return 1;
    }

    if (buf[0] == 'g' && buf[1] == 'f' && buf[2] == 'f')
    {
        //printf("gff:%.*s\r\n", len, buf);
        cmd_gff_test();
        return 1;
    }

    if (buf[0] == 'g' && buf[1] == 'm' && buf[2] == 'm')
    {
        // printf("gmm:%.*s\r\n", len, buf);
        cmd_gmm_test();
        return 1;
    }

    if (buf[0] == 'D' && buf[1] == 'B' && buf[2] == 'G')
    {
        cmd_dbg_start();
        return 1;
    }

    if (buf[0] == 'D' && buf[1] == 'U' && buf[2] == 'M' && buf[3] == 'P')
    {
        cmd_dbg_dump();
        return 1;
    }

    if (buf[0] == 'D' && buf[1] == 'P' && buf[2] == ':')    /* DP: 定时打印PID状态 */
    {
        cmd_dp(buf, len);
        return 1;
    }

    return 0; /* 无法识别 */
}
