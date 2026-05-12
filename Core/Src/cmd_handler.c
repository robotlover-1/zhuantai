/**
 ****************************************************************************************************
 * @file        cmd_handler.c
 * @brief       串口命令处理函数实现
 * @note        从main.c中拆分出的所有命令处理函�?
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
extern int32_t pulse_out;
extern int32_t pluse_ele;
extern int loop1;
extern int loop2;
extern int PhotoPos;
extern float SetPoint_P;
extern float ActualValue_P;
extern int32_t motor_pwm;
extern int duzhuan_flag;
extern int32_t Encode_now;

extern TIM_HandleTypeDef TIM4_Handler;
extern TIM_HandleTypeDef TIM5_Handler;
extern TIM_HandleTypeDef TIM4_Handler;

extern UART_HandleTypeDef UART4_Handler;
extern USART_HandleTypeDef UART1_Handler;

extern u8 USART_RX_BUF[];
extern u16 USART_RX_STA;
extern u8 USART_TX_BUF[];
extern u8 USART_TX_BUF_4[];
extern int32_t addr;
extern int32_t writeFlashData;

/******************************************************************************************/
/* 参数设置类命�?*/
/******************************************************************************************/

void cmd_set_loop1(u8 *buf, int len) /* T: 大循环次数设�?*/
{
    extern int loop1;
    loop1 = parse_uart_number(buf, len);
    printf("loop1:%d\r\n", loop1);
}

void cmd_set_loop2(u8 *buf, int len) /* H: 速度设置 */
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

/******************************************************************************************/
/* 控制类命�?*/
/******************************************************************************************/

void cmd_ele_test(void) /* ELE: 光电信号检�?*/
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

void cmd_motor_init(void) /* D0: 电机初始�?*/
{
    motor_init();
}

void cmd_hal_test(void) /* HAL: 霍尔位置检�?*/
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

void cmd_tim3(void) /* TIM3: TIM4计时测试 */
{
    extern u8 time_run;
    HAL_TIM_Base_Start_IT(&TIM4_Handler);
    delay_ms(5000);
    HAL_TIM_Base_Stop_IT(&TIM4_Handler);
    printf("time_run: %d\r\n", time_run);
}

void cmd_ring(void) /* RING: 上升沿检�?*/
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

void cmd_alarm(void) /* ALARM: 查询报警代码 */
{
    printf("报警代码:%d\r\n", alarm);
}

/******************************************************************************************/
/* 工位跳转 */
/******************************************************************************************/

void cmd_goto_station(int station) /* L1-L5: 跳转到指定工�?*/
{
    if (run_flag == 0)
    {
        station_x = read_hal();
        DIR = dir_jud(station, station_x);
        step_motor_LX(station, DIR);
    }
}

/******************************************************************************************/
/* 测试模式 */
/******************************************************************************************/

/**
 * @brief       gnn: 磨合测试�?站循环）
 * @note        5站间往返运动，每站间往返loop2次，大循环loop1�?
 */
void cmd_gnn_test(void)
{
    extern u8 time_run;
    int j3 = 0, j4 = 0;

    USART_RX_STA = 0;
    USART_RX_BUF[0] = 0;
    time_run = 0;
    HAL_TIM_Base_Start_IT(&TIM4_Handler); // 开始磨合计�?

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

            /* 正转到下一�?*/
            step_motor_motion(1, 0);
            wait_motion_done();
            delay_ms(500);

            /* 下一站与本站之间往返loop2�?*/
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
 * @note        loop2次往�?拍照+IO输出
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
                for (j4 = 0; j4 <= loop2; j4++) // 工位往�?
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

/******************************************************************************************/
/* 命令分发函数 */
/******************************************************************************************/

/**
 * @brief       串口命令分发
 * @param       buf: 接收缓冲�?
 * @param       len: 数据长度
 * @retval      1-命令已处�? 0-无法识别
 */
int cmd_dispatch(u8 *buf, int len)
{
    /* 参数设置�?(X:num格式) */
    if (buf[0] == 'T' && buf[1] == ':')       /* T: 大循环次�?*/
    {
        //printf("T:%.*s\r\n", len, buf);
        cmd_set_loop1(buf, len);
        return 1;
    }
    if (buf[0] == 'H' && buf[1] == ':')       /* H: 速度设置 */
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

    /* 控制类命�?(多字符命�? */
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

    /* 测试类命�?(多字�? */
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

    return 0; /* 无法识别 */
}
