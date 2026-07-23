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
extern int error;
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
extern int32_t g_add_pulse_count;
extern __IO uint8_t  g_motion_sta;
extern int32_t step_count;
extern TIM_HandleTypeDef g_timx_pwm_chy_handle;
extern TIM_HandleTypeDef TIM5_Handler;
extern u8 time_out;
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
    int datap = 0, i;
    u32 t1;
    for (i = len - 2; i >= 2; i--) {
        if (buf[i] >= '0' && buf[i] <= '9') {
            t1 = 1;
            int j;
            for (j = 0; j < (len - 2 - i); j++) t1 *= 10;
            datap += (buf[i] - '0') * t1;
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
}

void cmd_set_g_decel(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    g_step_decel = v;
    printf("g_step_decel: %d\r\n", v);
}

void cmd_set_g_step1(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    g_step1 = v;
    printf("g_step1: %d\r\n", v);
}

void cmd_set_g_step2(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    g_step2 = v;
    printf("g_step2: %d\r\n", v);
}

void cmd_set_step_pass(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    step_pass = 12000 / v;
    printf("step_pass: %d\r\n", step_pass);
}

void cmd_set_speed_rcc(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    if (v < 0) v = 0; if (v > 200) v = 200;
    speed_rcc = v;
    printf("speed_rcc: %d(rpm)\r\n", v);
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
}

void cmd_set_loop1(u8 *buf, int len) {
    loop1 = parse_uart_number(buf, len);
    printf("loop1: %d\r\n", loop1);
}

void cmd_set_loop2(u8 *buf, int len) {
    loop2 = parse_uart_number(buf, len);
    printf("loop2: %d\r\n", loop2);
}

/* ===== 控制命令 ===== */
void cmd_zero(void) {
    if (reset_flag == 0) {
        if (receive_flag == 0) {
            HAL_GPIO_WritePin(ST1_ZERO_GPIO_PORT, ST1_ZERO_GPIO_PIN, 1);
            delay_ms(1000);
            HAL_GPIO_WritePin(ST1_ZERO_GPIO_PORT, ST1_ZERO_GPIO_PIN, 0);
            while (1) {
                if (HAL_GPIO_ReadPin(ST1_END_GPIO_PORT, ST1_END_GPIO_PIN) == 1) break;
            }
        }
        delay_ms(100);
        level_speed = 7;
        create_t_ctrl_param(pulse_compensate, g_step1, g_step_accel, g_step2, g_step_decel, g_set_speed);
        delay_ms(50);
        while (1) { if (g_motion_sta == 0) break; }
        level_speed = 7;
        create_t_ctrl_param(-pulse_compensate, g_step1, g_step_accel, g_step2, g_step_decel, g_set_speed);
        delay_ms(50);
        while (1) { if (g_motion_sta == 0) break; }
        zero_flag = 1; g_zero_flag = 1; receive_flag = 0;
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
    printf("光电: %d\r\n", HAL_GPIO_ReadPin(ST1_END_GPIO_PORT, ST1_END_GPIO_PIN));
}

void cmd_photo_test(void) {
    printf("拍照\r\n");
    HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, 1);
    delay_ms(500);
    HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, 0);
}

void cmd_clc(void) {
    time_out = 0; error = 0; g_zero_flag = 0;
}

void cmd_rtf(void) {
    if (receive_flag == 0 && reset_flag == 0) {
        step_motor_motion(1, 1);
        delay_ms(50);
        while (1) { if (g_motion_sta == 0) break; }
        station_x++;
        if (station_x > 5) station_x = 1;
    }
}

void cmd_rtb(void) {
    if (receive_flag == 0 && reset_flag == 0) {
        step_motor_motion(1, 0);
        delay_ms(50);
        while (1) { if (g_motion_sta == 0) break; }
        station_x--;
        if (station_x == 0) station_x = 5;
    }
}

void cmd_rts(void) {
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
    param_save_all();
    HAL_NVIC_SystemReset();
}

void cmd_io_test(void) {
    HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, 1);
    delay_ms(500);
    HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, 0);
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
        USART_TX_BUF[0] = 'P';
        USART_TX_BUF[1] = station_x + '0';
        HAL_UART_Transmit(&UART4_Handler, (uint8_t*)USART_TX_BUF, 2, 1000);
        while (__HAL_UART_GET_FLAG(&UART4_Handler, UART_FLAG_TC) != SET);
        printf("\r\n");
    }
}

void cmd_goto_station(int station) {
    if (g_zero_flag == 1 && receive_flag == 0 && reset_flag == 0) {
        time_out = 0;
        HAL_TIM_Base_Start_IT(&TIM5_Handler);
        step_motor_L2(station, station_x);
        station_x = station;
    }
}

void cmd_cycle(void) {
    printf("total_cycles: %d\r\n", total_cycles);
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
    printf("total_cycles: %d\r\n", total_cycles);
    printf("station_x: %d  error: %d\r\n", station_x, error);
    printf("=======================\r\n");
}

/* ===== 测试模式 (移植F1 goo/gnn) ===== */
void cmd_gnn_test(void) {
    int j3 = 0, j4 = 0, j5 = 0;
    int station_to;
    printf("疲劳测试开始\r\n");
    delay_ms(1000);
    for (j3 = 0; j3 <= loop1; j3++) {
        j4 = 0;
        station_to = station_x;
        j5 = 1;
        for (j5 = 1; j5 <= 5; j5++) {
            if (station_to == j5) continue;
            for (j4 = 0; j4 <= loop2; j4++) {
                printf("L%d", station_to);
                printf("L%d\r\n", j5);
                HAL_TIM_Base_Start_IT(&TIM5_Handler);
                step_motor_L2(j5, station_to);
                while (1) { if (receive_flag == 0) break; }
                delay_ms(500);
                printf("L%d", j5);
                printf("L%d\r\n", station_to);
                HAL_TIM_Base_Start_IT(&TIM5_Handler);
                step_motor_L2(station_to, j5);
                while (1) { if (receive_flag == 0) break; }
                delay_ms(2000);
            }
        }
        /* 切换到下一站继续测试 */
        step_motor_motion(1, 1);
        delay_ms(50);
        while (1) { if (receive_flag == 0) break; }
        station_x++;
        if (station_x > 5) station_x = 1;
    }
    printf("疲劳测试结束\r\n");
}

void cmd_goo_test(void) {
    int j3 = 0, j4 = 0, j5 = 0;
    int station_to;
    printf("自动测试开始\r\n");
    for (j3 = 0; j3 <= loop1; j3++) {
        station_to = station_x;
        for (j5 = 1; j5 <= 5; j5++) {
            if (station_to == j5) continue;
            for (j4 = 0; j4 <= loop2; j4++) {
                step_motor_L2(j5, station_to);
                while (1) { if (g_motion_sta == 0) break; }
                delay_ms(500);
                step_motor_L2(station_to, j5);
                while (1) { if (g_motion_sta == 0) break; }
                delay_ms(3000);
                HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, 1);
                delay_ms(10);
                HAL_GPIO_WritePin(ST1_IO_GPIO_PORT, ST1_IO_GPIO_PIN, 0);
                delay_ms(2000);
            }
        }
    }
    printf("自动测试结束\r\n");
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

    /* 控制类 (多字符命令) */
    if (buf[0]=='D' && buf[1]=='0') { cmd_zero(); return 1; }
    if (buf[0]=='D' && buf[1]=='1') { cmd_set_station1(); return 1; }
    if (buf[0]=='E' && buf[1]=='L' && buf[2]=='E') { cmd_ele_test(); return 1; }
    if (buf[0]=='O' && buf[1]=='P') { cmd_photo_test(); return 1; }
    if (buf[0]=='C' && buf[1]=='L' && buf[2]=='C') { cmd_clc(); return 1; }
    if (buf[0]=='R' && buf[1]=='T' && buf[2]=='F') { cmd_rtf(); return 1; }
    if (buf[0]=='R' && buf[1]=='T' && buf[2]=='B') { cmd_rtb(); return 1; }
    if (buf[0]=='R' && buf[1]=='T' && buf[2]=='S') { cmd_rts(); return 1; }
    if (buf[0]=='R' && buf[1]=='E' && buf[2]=='S') { cmd_res(); return 1; }
    if (buf[0]=='I' && buf[1]=='O') { cmd_io_test(); return 1; }
    if (buf[0]=='H' && buf[1]=='A' && buf[2]=='L') { cmd_hal_test(); return 1; }
    if (buf[0]=='C' && buf[1]=='Y' && buf[2]=='C' && buf[3]=='L' && buf[4]=='E') { cmd_cycle(); return 1; }
    if (buf[0]=='P' && buf[1]=='A' && buf[2]=='R' && buf[3]=='A') { cmd_show_params(); return 1; }

    /* 工位跳转 */
    if (buf[0]=='L' && buf[1]>='1' && buf[1]<='5') { cmd_goto_station(buf[1]-'0'); return 1; }
    if (buf[0]=='L' && buf[1]=='?') { cmd_query_station(); return 1; }

    /* 测试模式 */
    if (buf[0]=='g' && buf[1]=='n' && buf[2]=='n') { cmd_gnn_test(); return 1; }
    if (buf[0]=='g' && buf[1]=='o' && buf[2]=='o') { cmd_goo_test(); return 1; }

    return 0;
}
