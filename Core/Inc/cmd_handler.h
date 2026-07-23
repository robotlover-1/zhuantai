#ifndef __CMD_HANDLER_H
#define __CMD_HANDLER_H

#include "sys.h"

/* 参数设置类 */
void cmd_set_time(u8 *buf, int len);        /* TIME: 设置RTC时间 */
void cmd_set_g_speed(u8 *buf, int len);     /* D: 最大速度 g_set_speed */
void cmd_set_g_accel(u8 *buf, int len);     /* J: 第一段加速度 g_step_accel */
void cmd_set_g_decel(u8 *buf, int len);     /* L:num 第一段减速度 g_step_decel */
void cmd_set_g_step1(u8 *buf, int len);     /* M: 加速段步数 g_step1 */
void cmd_set_g_step2(u8 *buf, int len);     /* N: 减速段步数 g_step2 */
void cmd_set_step_pass(u8 *buf, int len);   /* X: 过孔速度 step_pass */
void cmd_set_speed_rcc(u8 *buf, int len);   /* Z: 回零速度 speed_rcc */
void cmd_set_pulse_cut(u8 *buf, int len);   /* P: 工位间脉冲数 pulse_cut */
void cmd_set_pulse_comp(u8 *buf, int len);  /* C: 补偿脉冲数 pulse_compensate */
void cmd_set_time_delay(u8 *buf, int len);  /* Q: 补偿延时 time_delay_compensate */
void cmd_set_loop1(u8 *buf, int len);       /* T: 大循环次数 loop1 */
void cmd_set_loop2(u8 *buf, int len);       /* H: 小循环次数 loop2 */

/* 控制类 */
void cmd_zero(void);                        /* D0: 回零 */
void cmd_set_station1(void);                /* D1: 设当前位置为工位1 */
void cmd_ele_test(void);                    /* ELE: 光电信号检测 */
void cmd_photo_test(void);                  /* OP: 拍照测试 */
void cmd_clc(void);                         /* CLC: 清除报警 */
void cmd_rtf(void);                         /* RTF: 正转一个工位 */
void cmd_rtb(void);                         /* RTB: 反转一个工位 */
void cmd_rts(void);                         /* RTS: 停止 */
void cmd_res(void);                         /* RES: 软复位 */
void cmd_io_test(void);                     /* IO: IO输出测试 */
void cmd_hal_test(void);                    /* HAL: 霍尔传感器检测 (调试) */
void cmd_goto_station(int station);         /* L1-L5: 跳转到指定工位 */
void cmd_query_station(void);               /* L?: 查询当前工位 */

/* 测试模式 */
void cmd_gnn_test(void);                    /* gnn: 疲劳测试 */
void cmd_goo_test(void);                     /* goo: 自动测试 */

/* 查询类 (新增) */
void cmd_cycle(void);                       /* CYCLE: 查询总转动孔位计数 */
void cmd_show_params(void);                 /* PARA: 显示全部参数 */

/* 命令分发 */
int cmd_dispatch(u8 *buf, int len);
#endif
