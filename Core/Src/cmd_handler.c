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

/******************************************************************************************/
/* 参数设置类命令*/
/******************************************************************************************/
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

void cmd_set_skp(u8 *buf, int len) /* SK: 速度环P增益 (整数×100, 如 SK:350 = 3.50) */
{
    extern float g_pid_skp;
    int raw = 0;
    sscanf((char *)buf, "SK:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 5000) raw = 5000;
    g_pid_skp = raw * 0.01f;
    printf("速度环S_KP: %.2f\r\n", g_pid_skp);
    param_save_all();
}

void cmd_set_ski(u8 *buf, int len) /* SI: 速度环I增益 (整数×100) */
{
    extern float g_pid_ski;
    int raw = 0;
    sscanf((char *)buf, "SI:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 5000) raw = 5000;
    g_pid_ski = raw * 0.01f;
    printf("速度环S_KI: %.2f\r\n", g_pid_ski);
    param_save_all();
}

void cmd_set_skd(u8 *buf, int len) /* SD: 速度环D增益 (整数×100) */
{
    extern float g_pid_skd;
    int raw = 0;
    sscanf((char *)buf, "SD:%d", &raw);
    if (raw < 0) raw = 0;
    if (raw > 5000) raw = 5000;
    g_pid_skd = raw * 0.01f;
    printf("速度环S_KD: %.2f\r\n", g_pid_skd);
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
        printf("当前位置:%d, 需要先回到工位%d\r\n", station_x, base);
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
    printf("SK(速度环P): %.2f  SI: %.2f  SD: %.2f\r\n", g_pid_skp, g_pid_ski, g_pid_skd);
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
        SetPoint_P = Encode_now + 39200;
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
