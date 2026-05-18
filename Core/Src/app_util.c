/**
 ****************************************************************************************************
 * @file        app_util.c
 * @brief       应用层通用工具函数实现
 ****************************************************************************************************
 */
#include "app_util.h"
#include "step_motor.h"
#include "usart.h"
#include "usart2.h"
#include "atim.h"
#include "delay.h"
#include "pid.h"

/******************************************************************************************/
/* 外部变量声明 */
/******************************************************************************************/
extern u8 run_printf_flag;
extern u8 run_flag;
extern u8 printf_flag;
extern int error;
extern int alarm;
extern int duzhuan_flag;
extern u8 time_out;
extern TIM_HandleTypeDef TIM5_Handler;

/******************************************************************************************/
/* 报警/状态指示灯 */
/******************************************************************************************/

/**
 * @brief       报警IO闪烁（ALARM引脚输出50ms脉冲）
 */
void alarm_pulse(void)
{
    HAL_GPIO_WritePin(ST1_ALARM_GPIO_PORT, ST1_ALARM_GPIO_PIN, GPIO_PIN_SET);
    delay_ms(50);
    HAL_GPIO_WritePin(ST1_ALARM_GPIO_PORT, ST1_ALARM_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief       完成IO脉冲（DONE引脚输出100ms脉冲）
 */
void done_pulse(void)
{
    HAL_GPIO_WritePin(ST1_DONE_GPIO_PORT, ST1_DONE_GPIO_PIN, GPIO_PIN_SET);
    delay_ms(100);
    HAL_GPIO_WritePin(ST1_DONE_GPIO_PORT, ST1_DONE_GPIO_PIN, GPIO_PIN_RESET);
}

/******************************************************************************************/
/* 光电传感器 */
/******************************************************************************************/

/**
 * @brief       光电传感器消抖读取
 * @retval      1-已触发, 0-未触发
 */
int ele_is_triggered(void)
{
    if (HAL_GPIO_ReadPin(ST1_ELE_GPIO_PORT, ST1_ELE_GPIO_PIN) == 0)
    {
        delay_ms(10);
        if (HAL_GPIO_ReadPin(ST1_ELE_GPIO_PORT, ST1_ELE_GPIO_PIN) == 0)
        {
            return 1;
        }
    }
    return 0;
}

/******************************************************************************************/
/* 电机安全控制 */
/******************************************************************************************/

/**
 * @brief       完整的安全停止序列（紧急/错误路径）
 * @note        停止PWM, 清除堵转标志, 清运行标志, 停止TIM5超时定时器
 */
void motor_safe_stop(void)
{
    dcmotor_stop();
    duzhuan_flag = 0;
    motor_pwm_set(0);
    run_flag = 0;
    run_printf_flag = 0;
    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
}

/**
 * @brief       正常停止（成功到达路径）
 * @note        安全停止 + 置位printf_flag表示可以打印结果
 */
void motor_stop_normal(void)
{
    dcmotor_stop();
    duzhuan_flag = 0;
    motor_pwm_set(0);
    printf_flag = 1;
    run_flag = 0;
    run_printf_flag = 0;
    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
}

/**
 * @brief       标准电机启动序列
 * @note        清零time_out, 启动TIM5超时定时器, 初始化PID, 启动电机
 */
void motor_start_motion(void)
{
    extern int DIR;
    time_out = 0;
    HAL_TIM_Base_Start_IT(&TIM5_Handler);
    pid_init();
    run_printf_flag = 1;
    /* 启动两个PWM通道，方向由motor_pwm_set通过CCR控制 */
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_4);
    dcmotor_start();
    duzhuan_flag = 1;
}

/******************************************************************************************/
/* 运行中异常检测 */
/******************************************************************************************/

/**
 * @brief       24V断电检测
 * @retval      1-断电(已自动停止电机), 0-正常
 */
int check_24v_loss(void)
{
    if (HAL_GPIO_ReadPin(ST1_IN2_GPIO_PORT, ST1_IN2_GPIO_PIN) == 0)
    {
        printf("24V断电!\r\n");
        motor_safe_stop();
        error = 1;
        return 1;
    }
    return 0;
}

/**
 * @brief       运动超时检测
 * @param       alarm_code: 报警代码
 * @param       msg:        报警信息
 * @retval      1-超时(已自动停止电机), 0-未超时
 */
int check_motion_timeout(int alarm_code, const char *msg)
{
    if (time_out > 8)
    {
        motor_safe_stop();
        error = 1;
        alarm = alarm_code;
        printf("%s\r\n", msg);
        alarm_pulse();
        return 1;
    }
    return 0;
}

/******************************************************************************************/
/* 电机运动等待 */
/******************************************************************************************/

/**
 * @brief       等待电机运行结束（忙等待run_printf_flag变为0）
 */
void wait_motion_done(void)
{
    while (run_printf_flag)
    {
        if (error == 1)
        {
            break;
        }
    }
}

/**
 * @brief       等待运行结束或出错
 * @retval      0-正常结束, 1-出错
 */
int wait_motion_done_or_error(void)
{
    while (1)
    {
        if (run_printf_flag == 0)
        {
            return 0;
        }
        if (error == 1)
        {
            return 1;
        }
    }
}

/******************************************************************************************/
/* 串口工具 */
/******************************************************************************************/

extern u32 tpingg(u32 tmp); // 10的幂次方

/**
 * @brief       解析串口接收的数字参数
 * @param       buf: 串口接收缓冲区
 * @param       len: 数据长度
 * @retval      解析出的整数值
 */
int parse_uart_number(u8 *buf, int len)
{
    int i = 0, t1 = 0, t2 = 0, datap = 0;
    i = len - 3;                        // 减去命令前缀('X:')长度
    len = len - 2;
    while (len--)                       // 位数递减
    {
        t1 = tpingg(len);               // 取对应位的基数(10^n)
        t2 = buf[i - len + 2] - '0';    // ASCII转数字
        datap += t1 * t2;
    }
    return datap;
}

extern UART_HandleTypeDef UART4_Handler;
extern u8 USART_TX_BUF_4[];

/**
 * @brief       触发拍照（PC5输出 + UART4发"OP"指令）
 * @note        PC5(3.3V推挽)输出500ms高电平作为拍照信号
 */
void trigger_photo(void)
{
    /* PC5输出3.3V拍照信号 */
    HAL_GPIO_WritePin(ST1_PHOTO_GPIO_PORT, ST1_PHOTO_GPIO_PIN, GPIO_PIN_SET);
    delay_ms(500);
    HAL_GPIO_WritePin(ST1_PHOTO_GPIO_PORT, ST1_PHOTO_GPIO_PIN, GPIO_PIN_RESET);

    /* UART4发送"OP"拍照指令 */
    USART_TX_BUF_4[0] = 'O';
    USART_TX_BUF_4[1] = 'P';
    HAL_UART_Transmit(&UART4_Handler, (uint8_t *)USART_TX_BUF_4, 2, 1000);
    while (__HAL_UART_GET_FLAG(&UART4_Handler, UART_FLAG_TC) != SET)
        ; // 等待发送结束
}

/******************************************************************************************/
/* Flash工具 */
/******************************************************************************************/

/**
 * @brief       保存参数到Flash
 * @param       addr: Flash地址
 * @param       data: 要保存的数据
 */
void flash_save_param(uint32_t addr, int32_t data)
{
    uint32_t SectorError = 0;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_SECTORS;
    f.Sector = FLASH_SECTOR_7;
    f.NbSectors = 1;
    f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&f, &SectorError);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data);
    HAL_FLASH_Lock();
}

/******************************************************************************************/
/* 工位验证 */
/******************************************************************************************/

/**
 * @brief       工位到达验证
 * @param       hal_start: 起始霍尔工位值(1-5)
 * @param       hal_end:   目标霍尔工位值(1-5)
 * @retval      1-验证通过, 0-位置错误
 * @note        正确路径: 正转1->5,2->1,3->2,4->3,5->4 / 反转1->2,2->3,3->4,4->5,5->1
 */
int check_station_valid(int hal_start, int hal_end)
{
    /* 正转时：1->5, 2->1, 3->2, 4->3, 5->4 */
    if (hal_start == 1 && hal_end == 5) return 1;
    if (hal_start == 2 && hal_end == 1) return 1;
    if (hal_start == 3 && hal_end == 2) return 1;
    if (hal_start == 4 && hal_end == 3) return 1;
    if (hal_start == 5 && hal_end == 4) return 1;

    /* 反转时：1->2, 2->3, 3->4, 4->5, 5->1 */
    if (hal_start == 1 && hal_end == 2) return 1;
    if (hal_start == 2 && hal_end == 3) return 1;
    if (hal_start == 3 && hal_end == 4) return 1;
    if (hal_start == 4 && hal_end == 5) return 1;
    if (hal_start == 5 && hal_end == 1) return 1;

    return 0;
}

/**
 * @brief       工位验证并输出结果（完成脉冲或报警）
 * @param       hal_start: 起始霍尔工位值(1-5)
 * @param       hal_end:   目标霍尔工位值(1-5)
 * @note        验证通过则打印工位并输出完成脉冲，失败则报警
 */
void handle_station_verify(int hal_start, int hal_end)
{
    if (check_station_valid(hal_start, hal_end))
    {
        printf("P%d\r\n", hal_end);
        done_pulse();
    }
    else
    {
        alarm = 9;
        printf("工位验证失败!\r\n");
        alarm_pulse();
    }
}
