#include "cmd_handler.h"
#include "flash_param.h"
#include "rtc.h"
#include "step_motor.h"
#include "usart.h"
#include "usart2.h"
#include "atim.h"
#include "delay.h"
#include <stdio.h>
#include <stdlib.h>

/* ===== 外部变量声明 ===== */
extern u8 receive_flag;
extern u8 printf_flag;
extern u8 run_flag;
extern u8 error;
extern int station_x;
extern __IO uint32_t g_set_speed;
extern __IO uint32_t g_step_accel;
extern __IO uint32_t g_step_decel;
extern __IO uint16_t g_step1;
extern __IO uint16_t g_step2;
extern int step_pass;
extern int speed_rcc;
extern int pulse_cut;
extern int32_t pulse_compensate;
extern int time_delay_compensate;
extern int loop1;
extern int loop2;
extern int PL_T;
extern int PL_H;
extern int CS_T;
extern int CS_H;
extern int32_t total_cycles;
extern int32_t cycles_72;
extern int32_t writeFlashData;
extern u8 g_zero_flag;
extern u8 reset_flag;
extern u8 zero_flag;
extern int32_t addr;
extern int32_t step_all;
extern int32_t step_decel_2;
extern int32_t step_accel_2;
extern int32_t step_max;
extern int32_t step_max_1;
extern __IO uint32_t g_add_pulse_count;
extern __IO uint8_t  g_motion_sta;
extern int32_t step_count;
extern TIM_HandleTypeDef g_timx_pwm_chy_handle;
extern TIM_HandleTypeDef TIM5_Handler;
extern int time_out;
extern u8 time_run;
extern UART_HandleTypeDef UART4_Handler;
extern u16 USART_RX_STA;
extern u16 USART_RX_STA_4;
extern u8 USART_RX_BUF[];
extern u8 USART_RX_BUF_4[];
extern u8 USART_TX_BUF[];
extern u8 USART_TX_BUF_4[];

/* ===== 数字解析 ===== */
static int parse_uart_number(u8 *buf, int len)
{
    int datap = 0;
    for (int i = 2; i < len; i++) {
        if (buf[i] >= '0' && buf[i] <= '9') {
            datap = datap * 10 + (buf[i] - '0');
        }
    }
    return datap;
}

/* ===== 参数设置命令 ===== */
void cmd_set_time(u8 *buf, int len) {
    int y=2026, mon=1, d=1, h=0, min=0, sec=0;
    sscanf((char*)buf, "TIME:%d-%d-%d %d:%d:%d", &y, &mon, &d, &h, &min, &sec);
    if (y<2025||y>2099) y=2026; if (mon<1||mon>12) mon=1; if (d<1||d>31) d=1;
    if (h<0||h>23) h=0; if (min<0||min>59) min=0; if (sec<0||sec>59) sec=0;
    rtc_set_time(y, mon, d, h, min, sec);
    param_save_all();
    char ts[32]; rtc_get_time_str(ts, sizeof(ts));
    printf("RTC: %s\r\n", ts);
}

void cmd_set_g_speed(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 0) v = 0; if (v > 8000) v = 8000;
    g_set_speed = v;
    printf("g_set_speed: %d\r\n", v);
    param_save_all();
}

void cmd_set_g_accel(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    g_step_accel = v;
    printf("g_step_accel: %d\r\n", v);
    param_save_all();
}

void cmd_set_g_decel(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    g_step_decel = v;
    printf("g_step_decel: %d\r\n", v);
    param_save_all();
}

void cmd_set_g_step1(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    g_step1 = v;
    printf("g_step1: %d\r\n", v);
    param_save_all();
}

void cmd_set_g_step2(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    g_step2 = v;
    printf("g_step2: %d\r\n", v);
    param_save_all();
}

void cmd_set_step_pass(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    step_pass = 12000 / v;
    printf("step_pass: %d\r\n", step_pass);
    param_save_all();
}

void cmd_set_speed_rcc(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 1) v = 1; if (v > 200) v = 200;
    speed_rcc = 12000 / v;  /* rpm → delay 换算, 与 F1 一致 */
    printf("speed_rcc: %d\r\n", speed_rcc);
    param_save_all();
}

void cmd_set_pulse_cut(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 0) v = 0; if (v > 8000) v = 8000;
    pulse_cut = v;
    printf("pulse_cut: %d\r\n", v);
    param_save_all();
}

void cmd_set_pulse_comp(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 0) v = 0; if (v > 8000) v = 8000;
    pulse_compensate = v;
    printf("pulse_compensate: %d\r\n", v);
    param_save_all();
}

void cmd_set_time_delay(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    time_delay_compensate = v;
    printf("time_delay_compensate: %d\r\n", v);
    param_save_all();
}

void cmd_set_loop1(u8 *buf, int len) {
    loop1 = parse_uart_number(buf, len);
    printf("loop1: %d\r\n", loop1);
    param_save_all();
}

void cmd_set_loop2(u8 *buf, int len) {
    loop2 = parse_uart_number(buf, len);
    printf("loop2: %d\r\n", loop2);
    param_save_all();
}

void cmd_set_pl_t(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 0) v = 0;
    PL_T = v;
    printf("PL_T: %d\r\n", PL_T);
    param_save_all();
}

void cmd_set_pl_h(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 0) v = 0;
    PL_H = v;
    printf("PL_H: %d\r\n", PL_H);
    param_save_all();
}

void cmd_set_cs_t(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 0) v = 0;
    CS_T = v;
    printf("CS_T: %d\r\n", CS_T);
    param_save_all();
}

void cmd_set_cs_h(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 0) v = 0;
    CS_H = v;
    printf("CS_H: %d\r\n", CS_H);
    param_save_all();
}

/* 等待运动完成(g_motion_sta==0), 20秒超时强制复位, 防止卡死 */
static void wait_motion_done(void) {
    uint32_t t = 0;
    while (1) {
        if (g_motion_sta == 0) break;
        if (++t >= 20000) {
            HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
            HAL_TIM_Base_Stop_IT(&TIM5_Handler);
            run_flag = 0;
            g_motion_sta = 0;
            step_count = 0;
            printf("运动超时\r\n");
            break;
        }
        delay_ms(1);
    }
}

/* 运动完成计数: total_cycles++, 每1000次保存, 清除printf_flag避免重复统计
 * (goo/gnn阻塞主循环时主循环的统计不会执行, 需在测试内部统计) */
static void count_cycle(void) {
    total_cycles++;
    if (total_cycles % 1000 == 0) param_save_all();
    printf_flag = 0;
}

/* ===== 控制命令 ===== */
void cmd_zero(void) {
    if (reset_flag == 0 && receive_flag == 0) {
        error = 0;
        alarm_set(0);   /* 清除报警输出 */
        /* 触发回零: PC0 RESET(低) → Q5导通 → CN4第7脚拉低 → 外部回零 */
        HAL_GPIO_WritePin(ST1_ZERO_GPIO_PORT, ST1_ZERO_GPIO_PIN, GPIO_PIN_RESET);
        /* 回零触发后固定延时1秒等待外部回零完成 */
        delay_ms(1000);
        /* 释放回零: PC0 SET(高) → Q5截止 → 不再下拉CN4第7脚 */
        HAL_GPIO_WritePin(ST1_ZERO_GPIO_PORT, ST1_ZERO_GPIO_PIN, GPIO_PIN_SET);
        if (error != 0) return;
        if (reset_flag != 0) return;   /* 急停中断打断回零 */
        /* 正向补偿 */
        level_speed = 7;
        create_t_ctrl_param(pulse_compensate, g_step1, g_step_accel, g_step2,
                            g_step_decel, g_set_speed);
        delay_ms(50);
        wait_motion_done();
        if (reset_flag != 0) return;   /* 急停中断打断补偿运动 */
        /* 反向补偿 */
        level_speed = 7;
        create_t_ctrl_param(-pulse_compensate, g_step1, g_step_accel, g_step2,
                            g_step_decel, g_set_speed);
        delay_ms(50);
        wait_motion_done();
        if (reset_flag != 0) return;   /* 急停中断打断补偿运动 */
        zero_flag = 1;
        g_zero_flag = 1;
        receive_flag = 0;
        delay_ms(100);
        printf("P1\r\n");
        station_x = 1;
    }
}

void cmd_set_station1(void) {
    zero_flag = 1; g_zero_flag = 1; receive_flag = 0;
    printf("P1\r\n");
    station_x = 1;
}

void cmd_ele_test(void) {
    printf("HOME: %d\r\n", HAL_GPIO_ReadPin(ST1_ZERO1_GPIO_PORT, ST1_ZERO1_GPIO_PIN));
}

/* ZT1: 手动PC0拉低(ZERO RESET), 检查是否触发回零 */
void cmd_zero_test_low(void) {
    HAL_GPIO_WritePin(ST1_ZERO_GPIO_PORT, ST1_ZERO_GPIO_PIN, GPIO_PIN_RESET);
    printf("PC0 LOW\r\n");
}

/* ZT0: 手动PC0拉高(ZERO SET), 检查是否触发回零 */
void cmd_zero_test_high(void) {
    HAL_GPIO_WritePin(ST1_ZERO_GPIO_PORT, ST1_ZERO_GPIO_PIN, GPIO_PIN_SET);
    printf("PC0 HIGH\r\n");
}

/* 触发拍照: PC5输出500ms高电平 + UART4发"OP"指令 (与Desktop\new一致) */
void trigger_photo(void) {
    // HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, GPIO_PIN_SET);
    // delay_ms(500);
    // HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, GPIO_PIN_RESET);

    USART_TX_BUF_4[0] = 'O';
    USART_TX_BUF_4[1] = 'P';
    HAL_UART_Transmit(&UART4_Handler, (uint8_t *)USART_TX_BUF_4, 2, 1000);
    /* 等待发送完成, 100ms超时防止TC异常卡死 */
    uint32_t tc_timeout = 100;
    while (__HAL_UART_GET_FLAG(&UART4_Handler, UART_FLAG_TC) != SET) {
        if (--tc_timeout == 0) break;
        delay_ms(1);
    }
}

void cmd_photo_test(void) {
    printf("拍照\r\n");
    trigger_photo();
}

void cmd_clc(void) {
    time_out = 0; error = 0; g_zero_flag = 0;
    alarm_set(0);   /* 清除报警输出 */
}

void cmd_rtf(void) {
    if (receive_flag == 0 && reset_flag == 0) {
        step_motor_motion(1, 1);
        delay_ms(50);
        while (1) { if (g_motion_sta == 0) break; }
        if (reset_flag == 0) {   /* 急停中断打断运动时不更新工位 */
            station_x++;
            if (station_x > 5) station_x = 1;
        }
    }
}

void cmd_rtb(void) {
    if (receive_flag == 0 && reset_flag == 0) {
        step_motor_motion(1, 0);
        delay_ms(50);
        while (1) { if (g_motion_sta == 0) break; }
        if (reset_flag == 0) {   /* 急停中断打断运动时不更新工位 */
            station_x--;
            if (station_x == 0) station_x = 5;
        }
    }
}

/* 停止运动 (RTS 命令已移除, 仅供 gnn 测试中止调用) */
static void cmd_rts(void) {
    HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
    time_out = 0;
    g_motion_sta = 0;
    step_count = 0;
    receive_flag = 0;
    g_zero_flag = 0;
    printf_flag = 0;
}

void cmd_res(void) {
    writeFlashData = 1;
    param_save_all();              /* 先保存参数(整扇区擦除, 升级标志变0xFF) */
    write_update_flag(1);          /* 固件升级标志=1, 复位后bootloader进入升级模式 (对齐F1) */
    HAL_NVIC_SystemReset();
}

void cmd_io_test(void) {
    HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, GPIO_PIN_SET);
    delay_ms(500);
    HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, GPIO_PIN_RESET);
}

void cmd_hal_test(void) {
    int hal = read_hal();
    printf("HAL station: %d\r\n", hal);
    printf("HAL1:%d HAL2:%d HAL3:%d\r\n",
        HAL_GPIO_ReadPin(ST1_HAL1_GPIO_PORT, ST1_HAL1_GPIO_PIN),
        HAL_GPIO_ReadPin(ST1_HAL2_GPIO_PORT, ST1_HAL2_GPIO_PIN),
        HAL_GPIO_ReadPin(ST1_HAL3_GPIO_PORT, ST1_HAL3_GPIO_PIN));
}

void cmd_query_station(void) {
    if (g_zero_flag == 1) {
        printf("P%d\r\n", station_x);
    }
}

/* 路径统一为最短路径(L1); 补偿与否由 motion_dispatch 按 B 开关决定 */
static void motion_path(int station_y, int station_z) {
    step_motor_L1(station_y, station_z);
}

void cmd_goto_station(int station) {
    if (g_zero_flag == 1 && receive_flag == 0 && reset_flag == 0) {
        time_out = 0;
        HAL_TIM_Base_Start_IT(&TIM5_Handler);
        motion_path(station, station_x);
        station_x = station;
    }
}

void cmd_cycle(void) {
    printf("total_cycles: %d\r\n", total_cycles);
}

/* CC: 清零 total_cycles 并保存到 Flash */
void cmd_cycle_clr(void) {
    total_cycles = 0;
    param_save_all();
    printf("total_cycles: 0\r\n");
}

void cmd_show_params(void) {
    char ts[32]; rtc_get_time_str(ts, sizeof(ts));
    printf("====== PARAMETERS ======\r\n");
    printf("RTC: %s\r\n", ts);
    printf("g_set_speed(D:): %d\r\n", (int)g_set_speed);
    printf("g_step_accel(J:): %d\r\n", (int)g_step_accel);
    printf("g_step_decel(L:): %d\r\n", (int)g_step_decel);
    printf("g_step1(M:): %d\r\n", (int)g_step1);
    printf("g_step2(N:): %d\r\n", (int)g_step2);
    printf("step_pass(X:): %d\r\n", step_pass);
    printf("speed_rcc(Z:): %d\r\n", speed_rcc);
    printf("pulse_cut(P:): %d\r\n", pulse_cut);
    printf("pulse_compensate(C:): %d\r\n", (int)pulse_compensate);
    printf("time_delay_compensate(Q:): %d\r\n", time_delay_compensate);
    printf("loop1(T:): %d\r\n", loop1);
    printf("loop2(H:): %d\r\n", loop2);
    printf("PL_T: %d\r\n", PL_T);
    printf("PL_H: %d\r\n", PL_H);
    printf("CS_T: %d\r\n", CS_T);
    printf("CS_H: %d\r\n", CS_H);
    printf("total_cycles: %d\r\n", total_cycles);
    printf("station_x: %d  error: %d\r\n", station_x, error);
    printf("--- 网络配置 ---\r\n");
    printf("IP: %d.%d.%d.%d\r\n",
           g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);
    printf("MASK: %d.%d.%d.%d\r\n",
           g_eth_mask[0], g_eth_mask[1], g_eth_mask[2], g_eth_mask[3]);
    printf("GW: %d.%d.%d.%d\r\n",
           g_eth_gw[0], g_eth_gw[1], g_eth_gw[2], g_eth_gw[3]);
    printf("PORT: %d\r\n", (int)g_eth_port);
    printf("=======================\r\n");
}

/* ===== 测试模式 (移植F1 goo/gnn) ===== */
void cmd_gnn_test(int loop1_val, int loop2_val) {
    int j3 = 0, j4 = 0, j5 = 0;
    int station_to;
    int abort_test = 0;
    printf("疲劳测试开始\r\n");
    delay_ms(1000);
    for (j3 = 0; j3 <= loop1_val && !abort_test; j3++) {
        j4 = 0;
        station_to = station_x;
        j5 = 1;
        for (j5 = 1; j5 <= 5 && !abort_test; j5++) {
            if (station_to == j5) continue;
            for (j4 = 0; j4 <= loop2_val && !abort_test; j4++) {
                printf("L%d", station_to);
                printf("L%d\r\n", j5);
                HAL_TIM_Base_Start_IT(&TIM5_Handler);
                motion_path(j5, station_to);
                {
                    uint32_t wt = 0;
                    while (1) {
                        if (receive_flag == 0) break;
                        if (USART_RX_STA & 0x8000) { abort_test = 1; cmd_rts(); break; }
                        if (++wt >= 20000) {
                            HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                            HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                            run_flag = 0; g_motion_sta = 0; step_count = 0;
                            printf("运动超时\r\n"); break;
                        }
                        delay_ms(1);
                    }
                }
                if (abort_test) break;
                count_cycle();
                delay_ms(500);
                printf("L%d", j5);
                printf("L%d\r\n", station_to);
                HAL_TIM_Base_Start_IT(&TIM5_Handler);
                motion_path(station_to, j5);
                {
                    uint32_t wt = 0;
                    while (1) {
                        if (receive_flag == 0) break;
                        if (USART_RX_STA & 0x8000) { abort_test = 1; cmd_rts(); break; }
                        if (++wt >= 20000) {
                            HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                            HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                            run_flag = 0; g_motion_sta = 0; step_count = 0;
                            printf("运动超时\r\n"); break;
                        }
                        delay_ms(1);
                    }
                }
                if (abort_test) break;
                count_cycle();
                delay_ms(2000);
            }
        }
        if (abort_test) break;
        /* 切换到下一站继续测试 */
        step_motor_motion(1, 1);
        delay_ms(50);
        {
            uint32_t wt = 0;
            while (1) {
                if (receive_flag == 0) break;
                if (USART_RX_STA & 0x8000) { abort_test = 1; cmd_rts(); break; }
                if (++wt >= 20000) {
                    HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    run_flag = 0; g_motion_sta = 0; step_count = 0;
                    printf("运动超时\r\n"); break;
                }
                delay_ms(1);
            }
        }
        count_cycle();
        station_x++;
        if (station_x > 5) station_x = 1;
    }
    printf("疲劳测试结束\r\n");
}

void cmd_goo_test(int loop1_val, int loop2_val) {
    int j3 = 0, j4 = 0, j5 = 0;
    int station_to;
    printf("自动测试开始\r\n");
    for (j3 = 0; j3 <= loop1_val; j3++) {
        station_to = station_x;
        for (j5 = 1; j5 <= 5; j5++) {
            if (station_to == j5) continue;
            for (j4 = 0; j4 <= loop2_val; j4++) {
                motion_path(j5, station_to);
                wait_motion_done();
                count_cycle();
                delay_ms(500);
                motion_path(station_to, j5);
                wait_motion_done();
                count_cycle();
                delay_ms(3000);
                trigger_photo();
                delay_ms(2000);
            }
        }
    }
    printf("自动测试结束\r\n");
}

/* PL: 疲劳测试(gnn), 预设T/H, 约12小时 */
void cmd_pl(void) {
    printf("PL: fatigue test T:%d H:%d (~12h)\r\n", PL_T, PL_H);
    cmd_gnn_test(PL_T, PL_H);
}

/* CS: 自动测试(goo), 预设T/H, 约12小时 */
void cmd_cs(void) {
    printf("CS: auto test T:%d H:%d (~12h)\r\n", CS_T, CS_H);
    cmd_goo_test(CS_T, CS_H);
}

/* ===== 网络参数设置命令 ===== */
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

void cmd_set_mask(u8 *buf, int len)
{
    int a = 255, b = 255, c = 255, d = 0;
    (void)len;
    sscanf((char *)buf, "MASK:%d.%d.%d.%d", &a, &b, &c, &d);
    if (a < 0) a = 0; if (a > 255) a = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;
    if (c < 0) c = 0; if (c > 255) c = 255;
    if (d < 0) d = 0; if (d > 255) d = 255;
    g_eth_mask[0] = (u8)a;
    g_eth_mask[1] = (u8)b;
    g_eth_mask[2] = (u8)c;
    g_eth_mask[3] = (u8)d;
    printf("MASK 已设置为: %d.%d.%d.%d (需发送RES复位生效)\r\n", a, b, c, d);
    param_save_all();
}

void cmd_set_gw(u8 *buf, int len)
{
    int a = 192, b = 168, c = 1, d = 1;
    (void)len;
    sscanf((char *)buf, "GW:%d.%d.%d.%d", &a, &b, &c, &d);
    if (a < 0 || a > 255) a = 192;
    if (b < 0 || b > 255) b = 168;
    if (c < 0 || c > 255) c = 1;
    if (d < 0 || d > 255) d = 1;
    g_eth_gw[0] = (u8)a;
    g_eth_gw[1] = (u8)b;
    g_eth_gw[2] = (u8)c;
    g_eth_gw[3] = (u8)d;
    printf("GW 已设置为: %d.%d.%d.%d (需发送RES复位生效)\r\n", a, b, c, d);
    param_save_all();
}

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

/* ===== 命令分发 ===== */
int cmd_dispatch(u8 *buf, int len)
{
    if (len < 1) return 0;

    /* TIME: 设置RTC */
    if (buf[0]=='T' && buf[1]=='I' && buf[2]=='M' && buf[3]=='E' && buf[4]==':')
    { cmd_set_time(buf, len); return 1; }

    /* 参数设置类 (X:num 格式) */
    if (buf[0]=='D' && buf[1]==':') { cmd_set_g_speed(buf, len); return 1; }
    if (buf[0]=='J' && buf[1]==':') { cmd_set_g_accel(buf, len); return 1; }
    if (buf[0]=='L' && buf[1]==':') { cmd_set_g_decel(buf, len); return 1; }
    if (buf[0]=='M' && buf[1]==':') { cmd_set_g_step1(buf, len); return 1; }
    if (buf[0]=='N' && buf[1]==':') { cmd_set_g_step2(buf, len); return 1; }
    if (buf[0]=='X' && buf[1]==':') { cmd_set_step_pass(buf, len); return 1; }
    if (buf[0]=='Z' && buf[1]==':') { cmd_set_speed_rcc(buf, len); return 1; }
    if (buf[0]=='P' && buf[1]==':') { cmd_set_pulse_cut(buf, len); return 1; }
    if (buf[0]=='C' && buf[1]==':') { cmd_set_pulse_comp(buf, len); return 1; }
    if (buf[0]=='Q' && buf[1]==':') { cmd_set_time_delay(buf, len); return 1; }
    if (buf[0]=='T' && buf[1]==':') { cmd_set_loop1(buf, len); return 1; }
    if (buf[0]=='H' && buf[1]==':') { cmd_set_loop2(buf, len); return 1; }
    if (buf[0]=='P' && buf[1]=='L' && buf[2]=='T' && buf[3]==':') { cmd_set_pl_t(buf, len); return 1; }
    if (buf[0]=='P' && buf[1]=='L' && buf[2]=='H' && buf[3]==':') { cmd_set_pl_h(buf, len); return 1; }
    if (buf[0]=='C' && buf[1]=='S' && buf[2]=='T' && buf[3]==':') { cmd_set_cs_t(buf, len); return 1; }
    if (buf[0]=='C' && buf[1]=='S' && buf[2]=='H' && buf[3]==':') { cmd_set_cs_h(buf, len); return 1; }

    /* 控制类 (多字符命令) */
    if (buf[0]=='D' && buf[1]=='0') { cmd_zero(); return 1; }
    if (buf[0]=='Z' && buf[1]=='T' && buf[2]=='1') { cmd_zero_test_low(); return 1; }
    if (buf[0]=='Z' && buf[1]=='T' && buf[2]=='0') { cmd_zero_test_high(); return 1; }
    if (buf[0]=='D' && buf[1]=='1') { cmd_set_station1(); return 1; }
    if (buf[0]=='E' && buf[1]=='L' && buf[2]=='E') { cmd_ele_test(); return 1; }
    if (buf[0]=='O' && buf[1]=='P') { cmd_photo_test(); return 1; }
    if (buf[0]=='C' && buf[1]=='L' && buf[2]=='C') { cmd_clc(); return 1; }
    if (buf[0]=='R' && buf[1]=='T' && buf[2]=='F') { cmd_rtf(); return 1; }
    if (buf[0]=='R' && buf[1]=='T' && buf[2]=='B') { cmd_rtb(); return 1; }
    if (buf[0]=='R' && buf[1]=='E' && buf[2]=='S') { cmd_res(); return 1; }
    if (buf[0]=='I' && buf[1]=='O') { cmd_io_test(); return 1; }
    if (buf[0]=='H' && buf[1]=='A' && buf[2]=='L') { cmd_hal_test(); return 1; }
    if (buf[0]=='C' && buf[1]=='Y' && buf[2]=='C' && buf[3]=='L' && buf[4]=='E') { cmd_cycle(); return 1; }
    if (buf[0]=='C' && buf[1]=='C') { cmd_cycle_clr(); return 1; }
    if (buf[0]=='P' && buf[1]=='A' && buf[2]=='R' && buf[3]=='A') { cmd_show_params(); return 1; }

    /* 网络参数设置 (IP/MASK/GW/PORT) */
    if (buf[0]=='I' && buf[1]=='P' && buf[2]==':') { cmd_set_ip(buf, len); return 1; }
    if (buf[0]=='M' && buf[1]=='A' && buf[2]=='S' && buf[3]=='K' && buf[4]==':') { cmd_set_mask(buf, len); return 1; }
    if (buf[0]=='G' && buf[1]=='W' && buf[2]==':') { cmd_set_gw(buf, len); return 1; }
    if (buf[0]=='P' && buf[1]=='O' && buf[2]=='R' && buf[3]=='T' && buf[4]==':') { cmd_set_port(buf, len); return 1; }

    /* 工位跳转 */
    if (buf[0]=='L' && buf[1]>='1' && buf[1]<='5') { cmd_goto_station(buf[1]-'0'); return 1; }
    if (buf[0]=='L' && buf[1]=='?') { cmd_query_station(); return 1; }

    /* 测试模式 */
    if (buf[0]=='g' && buf[1]=='n' && buf[2]=='n') { cmd_gnn_test(loop1, loop2); return 1; }
    if (buf[0]=='g' && buf[1]=='o' && buf[2]=='o') { cmd_goo_test(loop1, loop2); return 1; }
    if (buf[0]=='P' && buf[1]=='L') { cmd_pl(); return 1; }
    if (buf[0]=='C' && buf[1]=='S') { cmd_cs(); return 1; }

    return 0;
}
