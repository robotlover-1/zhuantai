#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include "sys.h"

/******************************************************************************************/
/* F407 平式步进电机 GPIO 宏 */
/******************************************************************************************/

/* 电机使能 — PC1 (24V输出1, 开漏) */
#define ST1_EN_GPIO_PORT                  GPIOC
#define ST1_EN_GPIO_PIN                   GPIO_PIN_1
#define ST1_EN_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 方向控制 — PA11 (GPIO推挽) */
#define ST1_DIR_GPIO_PORT                  GPIOA
#define ST1_DIR_GPIO_PIN                   GPIO_PIN_11
#define ST1_DIR_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/* 回零气缸 — PC0 (24V输出, 开漏) */
#define ST1_ZERO_GPIO_PORT                  GPIOC
#define ST1_ZERO_GPIO_PIN                   GPIO_PIN_0
#define ST1_ZERO_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 回零完成检测 — PF8 (HOME, 双向光耦输入) */
#define ST1_ZERO1_GPIO_PORT                  GPIOF
#define ST1_ZERO1_GPIO_PIN                  GPIO_PIN_8
#define ST1_ZERO1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* 到位/光电检测 — PF9 (平式光电, 双向光耦输入) */
#define ST1_END_GPIO_PORT                  GPIOF
#define ST1_END_GPIO_PIN                   GPIO_PIN_9
#define ST1_END_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* IO/拍照输出 — PC5 (推挽输出) */
#define ST1_IO_GPIO_PORT                  GPIOC
#define ST1_IO_GPIO_PIN                   GPIO_PIN_5
#define ST1_IO_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 到位脉冲输出 — PC4 (推挽输出) */
#define ST1_DONE_GPIO_PORT                  GPIOC
#define ST1_DONE_GPIO_PIN                   GPIO_PIN_4
#define ST1_DONE_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 辅助输出 — PC2 (24V输出2, 开漏) */
#define ST1_AUX_GPIO_PORT                   GPIOC
#define ST1_AUX_GPIO_PIN                    GPIO_PIN_2
#define ST1_AUX_GPIO_CLK_ENABLE()           do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 霍尔传感器（仅调试用） */
#define ST1_HAL1_GPIO_PORT                  GPIOE
#define ST1_HAL1_GPIO_PIN                   GPIO_PIN_5
#define ST1_HAL1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ST1_HAL2_GPIO_PORT                  GPIOE
#define ST1_HAL2_GPIO_PIN                   GPIO_PIN_6
#define ST1_HAL2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ST1_HAL3_GPIO_PORT                  GPIOC
#define ST1_HAL3_GPIO_PIN                   GPIO_PIN_13
#define ST1_HAL3_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/******************************************************************************************/
/* IO 操作宏 */
#define ST1_DIR(x)   do{ x ? \
                      HAL_GPIO_WritePin(ST1_DIR_GPIO_PORT , ST1_DIR_GPIO_PIN , GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(ST1_DIR_GPIO_PORT , ST1_DIR_GPIO_PIN , GPIO_PIN_RESET); \
                  }while(0)

#define ST1_EN(x)   do{ x ? \
                      HAL_GPIO_WritePin(ST1_EN_GPIO_PORT , ST1_EN_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(ST1_EN_GPIO_PORT , ST1_EN_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)

/******************************************************************************************/
/* F1 速度斜坡结构体 (完整保留) */
typedef struct
{
     uint8_t  run_state;                    /* 电机运转状态 */
     uint8_t  dir;                          /* 电机运转方向 */
     int32_t  step_delay;                   /* 下一个脉冲周期 */
     int32_t step_accel1;                   /* 加速1点 */
     int32_t step_accel2;                   /* 加速2点 */
     int32_t step_run;                      /* 匀速运行步数 */
     int32_t step_decel1;                   /* 减速1点 */
     int32_t step_decel2;                   /* 减速2点 */
     int32_t step_accel2_delay;             /* 加速2延时 */
     int32_t step_decel2_delay;             /* 减速2延时 */
     int32_t step_accel1_delay;             /* 加速1延时 */
     int32_t step_decel1_delay;             /* 减速1延时 */
} speedRampData;

enum STA
{
    STOP = 0, ACCEL, ACCEL2, ACCEL3, ACCEL4, ACCEL5,
    DECEL, DECEL2, DECEL3, DECEL4, DECEL5,
    RUN, RUN2, RUN3, RUN4, RUN5
};

enum DIR_ENUM
{
 CCW = 0,   /* 逆时针 (反转) */
 CW         /* 顺时针 (正转) */
};

/******************************************************************************************/
/* 外部接口 */
extern speedRampData g_srd;
extern __IO int32_t  g_step_position;
extern __IO uint8_t  g_motion_sta;
extern __IO uint32_t g_add_pulse_count;
extern int32_t step_count;
extern u8 receive_flag;
extern u8 printf_flag;
extern int time_out;
extern int step_pass;
extern int32_t pulse_compensate;
extern int speed_rcc;
extern int level_speed;
extern int pulse_cut;
extern int32_t step_decel_2;
extern int32_t step_accel_2;
extern int32_t step_max;
extern int32_t step_max_1;
extern int32_t step_all;
extern u8 zero_flag;
extern u8 zero_run;
extern u8 run_flag;
extern u8 g_zero_flag;
extern u8 reset_flag;
extern int32_t cycles_72;
extern int32_t total_cycles;
extern __IO uint32_t g_set_speed;
extern __IO uint32_t g_step_accel;
extern __IO uint32_t g_step_decel;
extern __IO uint16_t g_step_angle;
extern __IO uint16_t g_step1;
extern __IO uint16_t g_step2;
extern int time_delay_compensate;
extern int station_x;

/* 函数声明 */
void step_motor_init(void);
void create_t_ctrl_param(int32_t step, int32_t step1, int32_t accel, int32_t step2, int32_t decel, int32_t speed);
void step_motor_motion(int num, int dir);
void step_motor_motion_compensate(int num, int dir);
void step_motor_L1(int station_y, int station_z);
void step_motor_L2(int station_y, int station_z);
void motor_run_all(int loop, int station_to);
u8 CheckRtrig(uint16_t GPIO_Pin);
u8 CheckPtrig(uint16_t GPIO_Pin);
int read_hal(void);   /* 霍尔读取 (调试用) */
#endif
