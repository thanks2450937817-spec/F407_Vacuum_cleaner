//
// Created by admin on 2026/6/1.
//
#include "main.h"
#include "tim.h"
#include "PID.h"
#include <inttypes.h>
#include <stdio.h>
#include <math.h>


// 位置环
PosPID_TypeDef pos_pid_left;
PosPID_TypeDef pos_pid_right;

// 速度环（增量式）
PID_TypeDef speed_pid_left;
PID_TypeDef speed_pid_right;

volatile int64_t last_left_encoder_cnt = 0;//上次读取的脉冲值
volatile int64_t last_right_encoder_cnt = 0;
volatile float l_actual_rpm = 0.0f;
volatile float r_actual_rpm = 0.0f;
const float PULSES_PER_REV = 1544.0f;   // 轮子一圈 1544 脉冲

volatile uint8_t l_current_duty = 50;   // 初始占空比 50%
volatile uint8_t r_current_duty = 50;   // 初始占空比 50%

int32_t r_current_cnt = 0;
int32_t l_current_cnt = 0;

volatile uint8_t use_sync_speed = 0;   // 1: 直行同步模式，使用公共目标转速
float sync_target_rpm = 0.0f;          // 同步后的期望转速



void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float target) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = target;
    pid->e_prev = 0;
    pid->e_prev2 = 0;
    pid->delta = 0;
    pid->out_min = -2.0f;
    pid->out_max = 2.0f;
}

float PID_Update_Incremental(PID_TypeDef *pid, float feedback) {
    float error = pid->target - feedback;
    // 增量计算
    float delta = pid->Kp * (error - pid->e_prev)
                + pid->Ki * error
                + pid->Kd * (error - 2*pid->e_prev + pid->e_prev2);
    // 限幅
    if (delta > pid->out_max) delta = pid->out_max;
    if (delta < pid->out_min) delta = pid->out_min;
    // 更新历史误差
    pid->e_prev2 = pid->e_prev;
    pid->e_prev = error;
    pid->delta = delta;
    return delta;
}



//位置PID初始化，外环
void PosPID_Init(PosPID_TypeDef *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
    //pid->target_distance = TARGET_DISTANCE;
    if (pid->target_distance != 0) {
        pid->target_pulse = pid->target_distance*7.56f;
    }
    else pid->target_pulse = 0;
    pid->current_pulse = 0;
    pid->error = 0; pid->prev_error = 0; pid->integral = 0;
    pid->integral_limit = 2500.0f;
    pid->output_speed = 0;
    pid->stop_flg = 0;
}

float PosPID_Update(PosPID_TypeDef *pid, float current_pulse, float dt) {//位置PID
    pid->current_pulse = current_pulse;
    pid->error = pid->target_pulse - pid->current_pulse;//目标脉冲减去当前脉冲
    //printf("pid->target_pulse:%d\r\n",(int)pid->target_pulse);
    //printf("pid->current_pulse:%ld\r\n",(long)pid->current_pulse);
    //printf("pid->error: %d\r\n",(int)pid->error);
    if (pid->error <= STOP_PULSE_THRESHOLD && pid->error >= -STOP_PULSE_THRESHOLD) {
        pid->error = 0.0f;
        pid->stop_flg = 1;
        //printf("pid->stop_flg:%d\r\n",(int)pid->stop_flg);
    }
    //printf("pid_error:%d\r\n",(int)pid->error);

    pid->integral += pid->Ki * pid->error * dt;
    //printf("pid_integral:%d\r\n",(int)pid->integral);
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float derivative = pid->Kd * (pid->error - pid->prev_error) / dt;
    //printf("derivative:%d\r\n",(int)derivative);
    pid->output_speed = pid->Kp * pid->error + pid->integral + derivative;
    //printf("pid_output_speed:%d\r\n",(int)pid->output_speed);
    // 限幅期望速度（例如最大 500 RPM）
    if (pid->output_speed > 150 ) pid->output_speed = 150;
    if (pid->output_speed < -150) pid->output_speed = -150;
    pid->prev_error = pid->error;
    return pid->output_speed;

}

void PID_Left_Setup(float speed) {
    l_current_cnt = __HAL_TIM_GET_COUNTER(&htim2);//左轮
    r_current_cnt = __HAL_TIM_GET_COUNTER(&htim5);//右轮
    //printf("l_current_cnt = %ld\r\n", l_current_cnt);

    int64_t l_delta_encoder = l_current_cnt - last_left_encoder_cnt;//左轮
    last_left_encoder_cnt = l_current_cnt;
    //printf("last_left_encoder_cnt = %ld\r\n", (long)last_left_encoder_cnt);

    float l_rps = (float)l_delta_encoder / PULSES_PER_REV / SPEED_CHECK_TIME;//左轮
    l_actual_rpm = l_rps * 60.0f;
    float L_Actual_rpm = l_rps * 60.0f;
    //printf("l_actual_rpm: %d\r\n",(int)L_Actual_rpm);//左轮转速

    speed_pid_left.target = speed;//位置环设定目标转速

    //printf("l_target_rpm:%d\r\n",(int)speed_pid_left.target); //目标转速

    //2. 调用增量式 PID，得到占空比增量 Δu
    float l_delta_duty = PID_Update_Incremental(&speed_pid_left, L_Actual_rpm);//增量式速度环
        // 3. 更新当前占空比
    int16_t l_duty = l_current_duty + (int16_t)l_delta_duty;
    if (l_duty > 100) l_duty = 100;
    else if (l_duty < 0) l_duty = 0;
    l_current_duty = (uint8_t)l_duty;

    if (BOW_Current_State == BOW_STATE_FORWARD) {

        if (pos_pid_left.stop_flg == 0)
        {
            if (l_current_duty > 100) l_current_duty = 100;
            else if (l_current_duty < 5) l_current_duty = 5;
        }
        else{
            l_current_duty = 0;
        }
        // 4. 设置 PWM 占空比（假设正转，方向固定）
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, l_current_duty);
    }
    else if (BOW_Current_State == BOW_STATE_TURN_LEFT) {
        if (pos_pid_left.stop_flg == 0) {
            if (l_current_duty > 10
                ) l_current_duty = 10;
            else if (l_current_duty < MIN_DUTY_FOR_RUN) l_current_duty = MIN_DUTY_FOR_RUN;
        }
        else {
            l_current_duty = 0;
        }
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, l_current_duty);
    }
    else if (BOW_Current_State == BOW_STATE_TURN_RIGHT) {
        if (pos_pid_left.stop_flg == 0) {
            if (l_current_duty > 10
                ) l_current_duty = 10;
            else if (l_current_duty < MIN_DUTY_FOR_RUN) l_current_duty = MIN_DUTY_FOR_RUN;
        }
        else {
            l_current_duty = 0;
        }
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, l_current_duty);
    }
}

void PID_Right_Setup(float speed) {
    r_current_cnt = __HAL_TIM_GET_COUNTER(&htim5);//右轮
    l_current_cnt = __HAL_TIM_GET_COUNTER(&htim2);//左轮
    //printf("r_current_cnt = %ld\r\n", r_current_cnt);
    int64_t r_delta_encoder = r_current_cnt - last_right_encoder_cnt;
    last_right_encoder_cnt = r_current_cnt;
    //printf("last_right_encoder_cnt = %ld\r\n", (long)last_right_encoder_cnt);
    float r_rps = (float)r_delta_encoder / PULSES_PER_REV / SPEED_CHECK_TIME;
    r_actual_rpm = r_rps * 60.0f;
    float R_Actual_rpm = r_rps * 60.0f;
    //printf("r_actual_rpm: %d\r\n",(int)R_Actual_rpm);
    speed_pid_right.target = speed;//位置环设定目标转速
    //printf("r_target_rpm:%d\r\n",(int)speed_pid_right.target); //目标转速
    //2. 调用增量式 PID，得到占空比增量 Δu
    float r_delta_duty = PID_Update_Incremental(&speed_pid_right, R_Actual_rpm);
    int16_t r_duty = r_current_duty + (int16_t)r_delta_duty;
    if (r_duty > 100) r_duty = 100;
    else if (r_duty < 0) r_duty = 0;
    r_current_duty = (uint8_t)r_duty;
    if (BOW_Current_State == BOW_STATE_FORWARD) {
        // 3. 更新当前占空比
        if (pos_pid_right.stop_flg == 0) {
            if (r_current_duty > 100) r_current_duty = 100;
            else if (r_current_duty < 5) r_current_duty = 5;
        }
        else {
            r_current_duty = 0;
        }
        // 4. 设置 PWM 占空比（假设正转，方向固定）
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, r_current_duty);
    }
    else if (BOW_Current_State == BOW_STATE_TURN_LEFT) {
        if (pos_pid_right.stop_flg == 0) {
             if (r_current_duty > 10) r_current_duty = 10;
             else if (r_current_duty < MIN_DUTY_FOR_RUN) r_current_duty = MIN_DUTY_FOR_RUN;
         }
        else {
             r_current_duty = 0;
        }
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, r_current_duty);
    }
    else if (BOW_Current_State == BOW_STATE_TURN_RIGHT) {
        if (pos_pid_right.stop_flg == 0) {
            if (r_current_duty > 10) r_current_duty = 10;
            else if (r_current_duty < MIN_DUTY_FOR_RUN) r_current_duty = MIN_DUTY_FOR_RUN;
        }
        else {
            r_current_duty = 0;
        }
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, r_current_duty);
    }
}

void PID_Base_Start(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void PID_Base_Stop(void) {
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

}

void Speed_Loop_Forward(void) {
    PID_Right_Setup(sync_target_rpm);
    PID_Left_Setup(sync_target_rpm);
}
//备份
// void PID_Left_Setup(float speed) {
//     l_current_cnt = __HAL_TIM_GET_COUNTER(&htim2);//左轮
//     r_current_cnt = __HAL_TIM_GET_COUNTER(&htim5);//右轮
//     printf("l_current_cnt = %ld\r\n", l_current_cnt);
//
//     int64_t l_delta_encoder = l_current_cnt - last_left_encoder_cnt;//左轮
//     last_left_encoder_cnt = l_current_cnt;
//     //printf("last_left_encoder_cnt = %ld\r\n", (long)last_left_encoder_cnt);
//
//     float l_rps = (float)l_delta_encoder / PULSES_PER_REV / 0.01f;//左轮
//     l_actual_rpm = l_rps * 60.0f;
//     float L_Actual_rpm = l_rps * 60.0f;
//     printf("l_actual_rpm: %d\r\n",(int)L_Actual_rpm);//左轮转速
//
//     speed_pid_left.target = speed;//位置环设定目标转速
//
//     printf("l_target_rpm:%d\r\n",(int)speed_pid_left.target); //目标转速
//
//     //2. 调用增量式 PID，得到占空比增量 Δu
//     float l_delta_duty = PID_Update_Incremental(&speed_pid_left, L_Actual_rpm);//增量式速度环
//         // 3. 更新当前占空比
//     int16_t l_duty = l_current_duty + (int16_t)l_delta_duty;
//     if (l_duty > 100) l_duty = 100;
//     else if (l_duty < 0) l_duty = 0;
//     l_current_duty = (uint8_t)l_duty;
//
//     if (Current_State == STATE_MOVE_FORWARD) {
//
//         if (pos_pid_left.stop_flg == 0)
//         {
//             if (l_current_duty > 100) l_current_duty = 100;
//             else if (l_current_duty < 10) l_current_duty = 10;
//         }
//         else{
//             l_current_duty = 0;
//         }
//         // 4. 设置 PWM 占空比（假设正转，方向固定）
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
//         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, l_current_duty);
//     }
//     else if (Current_State == STATE_TURN_LEFT) {
//         if (pos_pid_left.stop_flg == 0) {
//             if (l_current_duty > 15
//                 ) l_current_duty = 15;
//             else if (l_current_duty < 8) l_current_duty = 8;
//         }
//         else {
//             l_current_duty = 0;
//             if (pos_pid_right.stop_flg == 1) {
//                 pos_pid_left.stop_flg = 0;
//                 Current_State = STATE_IDLE;
//             }
//         }
//
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
//         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, l_current_duty);
//     }
//     else if (Current_State == STATE_TURN_RIGHT) {
//         if (pos_pid_left.stop_flg == 0) {
//             if (l_current_duty > 15
//                 ) l_current_duty = 15;
//             else if (l_current_duty < 8) l_current_duty = 8;
//         }
//         else {
//             l_current_duty = 0;
//             if (pos_pid_right.stop_flg == 1) {
//                 pos_pid_left.stop_flg = 0;
//                 Current_State = STATE_IDLE;
//             }
//         }
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
//         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, l_current_duty);
//     }
// }
//
// void PID_Right_Setup(float speed) {
//     r_current_cnt = __HAL_TIM_GET_COUNTER(&htim5);//右轮
//     l_current_cnt = __HAL_TIM_GET_COUNTER(&htim2);//左轮
//     printf("r_current_cnt = %ld\r\n", r_current_cnt);
//     int64_t r_delta_encoder = r_current_cnt - last_right_encoder_cnt;
//     last_right_encoder_cnt = r_current_cnt;
//     //printf("last_right_encoder_cnt = %ld\r\n", (long)last_right_encoder_cnt);
//     float r_rps = (float)r_delta_encoder / PULSES_PER_REV / 0.01f;
//     r_actual_rpm = r_rps * 60.0f;
//     float R_Actual_rpm = r_rps * 60.0f;
//     printf("r_actual_rpm: %d\r\n",(int)R_Actual_rpm);
//     speed_pid_right.target = speed;//位置环设定目标转速
//     printf("r_target_rpm:%d\r\n",(int)speed_pid_right.target); //目标转速
//     //2. 调用增量式 PID，得到占空比增量 Δu
//     float r_delta_duty = PID_Update_Incremental(&speed_pid_right, R_Actual_rpm);
//     int16_t r_duty = r_current_duty + (int16_t)r_delta_duty;
//     if (r_duty > 100) r_duty = 100;
//     else if (r_duty < 0) r_duty = 0;
//     r_current_duty = (uint8_t)r_duty;
//     if (Current_State == STATE_MOVE_FORWARD) {
//         // 3. 更新当前占空比
//         if (pos_pid_right.stop_flg == 0) {
//             if (r_current_duty > 100) r_current_duty = 100;
//             else if (r_current_duty < 10) r_current_duty = 10;
//         }
//         else {
//             r_current_duty = 0;
//         }
//         // 4. 设置 PWM 占空比（假设正转，方向固定）
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
//         __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, r_current_duty);
//     }
//     else if (Current_State == STATE_TURN_LEFT) {
//         if (pos_pid_right.stop_flg == 0) {
//              if (r_current_duty > 15) r_current_duty = 15;
//              else if (r_current_duty < 8) r_current_duty = 8;
//          }
//         else {
//              r_current_duty = 0;
//              if (pos_pid_left.stop_flg == 1) {
//                  pos_pid_right.stop_flg = 0;
//                  Current_State = STATE_IDLE;
//              }
//         }
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
//         __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, r_current_duty);
//     }
//     else if (Current_State == STATE_TURN_RIGHT) {
//         if (pos_pid_right.stop_flg == 0) {
//             if (r_current_duty > 15) r_current_duty = 15;
//             else if (r_current_duty < 8) r_current_duty = 8;
//         }
//         else {
//             r_current_duty = 0;
//             if (pos_pid_left.stop_flg == 1) {
//                 pos_pid_right.stop_flg = 0;
//                 Current_State = STATE_IDLE;
//             }
//         }
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
//         __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, r_current_duty);
//     }
// }




// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim->Instance == TIM6) {
//
//         switch (Current_State) {
//             case STATE_IDLE:
//                 break;
//
//             case STATE_MOVE_FORWARD:
//
//                 // 2. 位置环独立计算期望转速
//                 float vL = PosPID_Update(&pos_pid_left,  l_current_cnt, 0.01f);
//                 float vR = PosPID_Update(&pos_pid_right, r_current_cnt, 0.01f);
//
//                 // 3. 同步：取绝对值较小的那个（跟随慢侧）
//                 sync_target_rpm = (fabs(vL) < fabs(vR)) ? vL : vR;
//                 use_sync_speed = 1;
//
//                 PID_Left_Setup(sync_target_rpm);
//                 PID_Right_Setup(sync_target_rpm);
//                 break;
//             case STATE_TURN_LEFT:
//                 PID_Right_Setup(PosPID_Update(&pos_pid_right, r_current_cnt, 0.01f));
//                 PID_Left_Setup(PosPID_Update(&pos_pid_left,  l_current_cnt, 0.01f));
//                 break;
//             case STATE_TURN_RIGHT:
//                 PID_Right_Setup(PosPID_Update(&pos_pid_right, r_current_cnt, 0.01f));
//                 PID_Left_Setup(PosPID_Update(&pos_pid_left,  l_current_cnt, 0.01f));
//                 break;
//
//         }
//
//
//     }
// }















