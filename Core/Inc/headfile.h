//
// Created by admin on 2026/6/17.
//

#ifndef F407_VACUUM_CLEANER_HEADFILE_H
#define F407_VACUUM_CLEANER_HEADFILE_H
#include "main.h"
#include <math.h>
#include <stdio.h>
#include "tim.h"
// 状态枚举
typedef enum {//BOW状态枚举
    BOW_STATE_IDLE,
    BOW_STATE_FORWARD,
    BOW_STATE_TURN_LEFT,
    BOW_STATE_TURN_RIGHT,
    BOW_STATE_LATERAL,
    BOW_STATE_BACK_TURN,
    BOW_STATE_DONE
} BOW_State;

// 状态函数类型（无参数无返回值）
typedef void (*StateFunc)(void);

//在定时器里使用状态机
typedef enum {//顶层状态机枚举
    STATE_IDLE,           // 空闲
    STATE_MOVE_FORWARD,   // 直行（固定距离或连续）
    STATE_TURN_LEFT,           // 转向（目标角度）
    STATE_TURN_RIGHT,
    STATE_AVOID,          // 避障（后退、旋转等）
    STATE_BOW_CLEAN       // 弓字形清扫（子状态机内部实现）
} RobotState_t;

typedef struct {//顶层状态机
    void (*enter)(void);    // 进入这个状态时要做什么
    void (*update)(void);   // 在定时器中断里周期做什么
} StateAction;

typedef struct {//子状态机
    StateFunc on_enter;   // 进入状态时执行
    StateFunc on_update;  // 状态更新时执行
} BOW_StateHandler;


extern BOW_StateHandler BOW_State_table[];//BOW任务表

extern BOW_State BOW_Current_State;//BOW状态

extern RobotState_t Current_State;//顶层状态机状态





extern volatile int64_t last_left_encoder_cnt;



extern volatile float l_actual_rpm;

extern volatile uint8_t l_current_duty;

extern volatile uint8_t r_current_duty;

extern int32_t r_current_cnt;

extern int32_t l_current_cnt;

extern float sync_target_rpm;

extern volatile int64_t last_left_encoder_cnt;

extern volatile int64_t last_right_encoder_cnt;





void PID_Right_Setup(float speed);

void PID_Left_Setup(float speed);

void PID_Base_Start(void);

void PID_Base_Stop(void);











#endif //F407_VACUUM_CLEANER_HEADFILE_H