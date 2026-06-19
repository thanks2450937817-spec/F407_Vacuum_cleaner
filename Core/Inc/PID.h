//
// Created by admin on 2026/6/1.
//

#ifndef F407_VACUUM_CLEANER_PID_H
#define F407_VACUUM_CLEANER_PID_H
#include <stdint.h>
#include "headfile.h"
#define STOP_PULSE_THRESHOLD  50   // 剩余脉冲小于 50 时认为到达（约 6.6mm）
#define TURN_PULSE_THRESHOLD  20
#define MIN_DUTY_FOR_RUN       5    // 能保证转动的下限占空比
#define Turn_Need_Pluse 986  //左（右）转90°所需脉冲数
//位置PID参数
#define PKP 0.2f
#define PKI 0.08
#define PKD 0
#define TARGET_DISTANCE 1000.0f  //单位mm，目标行进距离
//速度PID参数
#define SKP 0.2f
#define SKI 0.08
#define SKD 0.15
#define TARGET 1000.0f  //目标转速
#define WHEEL_Perimeter 204.2f //轮子周长
#define WHEEL_Diameter 65.0f //轮子直径
typedef struct {
    float Kp, Ki, Kd;
    float target;           // 目标转速 (RPM)
    float e_prev;           // e(k-1)
    float e_prev2;          // e(k-2)
    float delta;            // 本次计算的增量 (Δu)
    // 可选：积分限幅参数等
    float out_min, out_max; // Δu 限幅（例如 -20, 20）
} PID_TypeDef;
// 位置 PID 结构体
typedef struct {
    float Kp, Ki, Kd;
    float target_pulse;      // 目标脉冲数
    float target_distance;
    float current_pulse;     // 当前脉冲数（平均值）
    float error;
    float prev_error;
    float integral;
    float integral_limit;
    float output_speed;      // 输出：期望转速 (RPM)
    uint8_t stop_flg;
} PosPID_TypeDef;



extern PosPID_TypeDef pos_pid_left;
extern PosPID_TypeDef pos_pid_right;

// 速度环（增量式）
extern PID_TypeDef speed_pid_left;
extern PID_TypeDef speed_pid_right;





extern volatile float l_actual_rpm;

extern volatile uint8_t l_current_duty;

extern volatile uint8_t r_current_duty;

extern int32_t r_current_cnt;

extern int32_t l_current_cnt;

extern volatile uint8_t use_sync_speed;

extern float sync_target_rpm;


void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float target);

void PosPID_Init(PosPID_TypeDef *pid, float Kp, float Ki, float Kd);

float PosPID_Update(PosPID_TypeDef *pid, float current_pulse, float dt);

void PID_Right_Setup(float speed);

void PID_Left_Setup(float speed);

void PID_Base_Start(void);

void PID_Base_Stop(void);







#endif //F407_VACUUM_CLEANER_PID_H