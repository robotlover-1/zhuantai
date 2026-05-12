/**
 ****************************************************************************************************
 * @file        cmd_handler.h
 * @brief       串口命令处理函数声明
 ****************************************************************************************************
 */
#ifndef __CMD_HANDLER_H
#define __CMD_HANDLER_H

#include "sys.h"

/* 命令处理函数 - 参数设置类 */
void cmd_set_loop1(u8 *buf, int len);    /* T: 大循环次数设置 */
void cmd_set_loop2(u8 *buf, int len);    /* H: 速度设置 */
void cmd_set_force(u8 *buf, int len);    /* F: 运行最大速度 */
void cmd_set_duty(u8 *buf, int len);     /* R: 最大占空比 */
void cmd_set_speed(u8 *buf, int len);    /* V: 最后匀速进孔速度 */
void cmd_set_pulse_low(u8 *buf, int len);/* S: 慢速进孔脉冲数 */
void cmd_set_photo_pos(u8 *buf, int len);/* P: 拍照孔位 */
void cmd_set_pluse_ele(u8 *buf, int len);/* E: 光电检测限制脉冲数 */

/* 命令处理函数 - 控制类 */
void cmd_ele_test(void);                 /* ELE: 光电信号检测 */
void cmd_photo_test(void);               /* OP:  拍照测试 */
void cmd_clc(void);                      /* CLC: 报警清除 */
void cmd_motor_init(void);               /* D0:  电机初始化 */
void cmd_hal_test(void);                 /* HAL: 霍尔位置检测 */
void cmd_red(void);                      /* RED: 读取当前位置 */
void cmd_rtf(void);                      /* RTF: 正转 */
void cmd_rtb(void);                      /* RTB: 反转 */
void cmd_tim3(void);                     /* TIM3: TIM4计时测试 */
void cmd_ring(void);                     /* RING: 上升沿检测 */
void cmd_rts(void);                      /* RTS: 恢复初始位置 */
void cmd_res(void);                      /* RES: 串口更新程序 */
void cmd_alarm(void);                    /* ALARM: 查询报警代码 */

/* 命令处理函数 - 工位跳转 */
void cmd_goto_station(int station);      /* L1-L5: 跳转到指定工位 */

/* 命令处理函数 - 测试模式 */
void cmd_gnn_test(void);                 /* gnn: 磨合测试(5站循环) */
void cmd_goo_test(void);                 /* goo: 自动测试 */
void cmd_gff_test(void);                 /* gff: 正转拍照测试 */
void cmd_gmm_test(void);                 /* gmm: 反转拍照测试 */

/* 命令分发 */
int cmd_dispatch(u8 *buf, int len);      /* 串口命令分发入口 */

#endif
