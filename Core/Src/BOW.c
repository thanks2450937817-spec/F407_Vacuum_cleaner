//
// Created by admin on 2026/6/15.
//

#include "BOW.h"


void BOW_Idle_Enter(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    last_left_encoder_cnt = 0;//将上次状态获取的编码值清0
    last_right_encoder_cnt = 0;
    printf("BOW Current State: Idle\n");
}

void BOW_Idle_Update(void) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
}

void BOW_Forward_Enter(void) {
    static uint8_t counter = 0;//这里应该只初始化一次
    last_left_encoder_cnt = 0;//将上次状态获取的编码值清0
    last_right_encoder_cnt = 0;
    counter = counter % 2;
    if (counter == 0) {//第一次进入
        pos_pid_left.target_distance = 1000;
        pos_pid_right.target_distance = 1000;
        printf("Current State = MOVE_FORWARD\r\n");


    }
    else {//第二次进入，说明左转了，要平移一个身位
        pos_pid_left.target_distance = 300;
        pos_pid_right.target_distance = 300;
    }
    counter++;
    pos_pid_left.stop_flg = 0;
    pos_pid_right.stop_flg = 0;
    PosPID_Init(&pos_pid_left,PKP,PKI,PKD);
    PosPID_Init(&pos_pid_right,PKP,PKI,PKD);
    PID_Base_Start();

}

void BOW_Forward_Update(void) {
    float vL = PosPID_Update(&pos_pid_left,  l_current_cnt, 0.01f);
    float vR = PosPID_Update(&pos_pid_right, r_current_cnt, 0.01f);
    static uint8_t counter = 0;
    counter = counter % 4;
    //同步：取绝对值较小的那个（跟随慢侧）
    sync_target_rpm = (fabs(vL) < fabs(vR)) ? vL : vR;
    PID_Right_Setup(sync_target_rpm);
    PID_Left_Setup(sync_target_rpm);
    if (pos_pid_left.stop_flg == 1 && pos_pid_right.stop_flg == 1) {
        if (counter <= 1) {
            BOW_Change_State(BOW_STATE_TURN_LEFT);
        }
        else BOW_Change_State(BOW_STATE_TURN_RIGHT);
        counter++;
    }
}

void BOW_Turn_Right_Enter(void) {
    // 重置位置环（清零积分和误差历史）
    PosPID_Init(&pos_pid_left, PKP, PKI, PKD);
    PosPID_Init(&pos_pid_right, PKP, PKI, PKD);
    last_left_encoder_cnt = 0;//将上次状态获取的编码值清0
    last_right_encoder_cnt = 0;
    pos_pid_left.target_pulse  = 986.0f;
    pos_pid_right.target_pulse = -986.0f;

    printf("Target l Pulse = %d\r\n",(int)pos_pid_left.target_pulse);
    printf("Target r Pulse = %d\r\n",(int)pos_pid_right.target_pulse);
    // HAL_Delay(1000);

    // 重置速度环
    PID_Init(&speed_pid_left,  SKP, SKI, SKD, 0);
    PID_Init(&speed_pid_right, SKP, SKI, SKD, 0);

    // 清零编码器
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    r_current_cnt = __HAL_TIM_GET_COUNTER(&htim5);
    l_current_cnt = __HAL_TIM_GET_COUNTER(&htim2);
    // 确保占空比初始值合理（转向时用小占空比防止冲出）
    l_current_duty = 10;
    r_current_duty = 10;
    PID_Base_Start();
    printf("Current State = STATE_TURN_RIGHT\r\n");
}

void BOW_Turn_Right_Update(void) {
    PID_Right_Setup(PosPID_Update(&pos_pid_right, r_current_cnt, 0.01f));
    PID_Left_Setup(PosPID_Update(&pos_pid_left,  l_current_cnt, 0.01f));
    if (pos_pid_left.stop_flg == 1 && pos_pid_right.stop_flg == 1) {
        BOW_Change_State(BOW_STATE_FORWARD);
    }
}

void BOW_Turn_Left_Enter(void){
    // 重置位置环（清零积分和误差历史）
    PosPID_Init(&pos_pid_left, PKP, PKI, PKD);
    PosPID_Init(&pos_pid_right, PKP, PKI, PKD);
    last_left_encoder_cnt = 0;//将上次状态获取的编码值清0
    last_right_encoder_cnt = 0;

    pos_pid_left.target_pulse  = -986.0f;   // 左轮反转
    pos_pid_right.target_pulse =  986.0f;   // 右轮正转

    printf("Target l Pulse = %d\r\n",(int)pos_pid_left.target_pulse);
    printf("Target r Pulse = %d\r\n",(int)pos_pid_right.target_pulse);
    // HAL_Delay(1000);

    // 重置速度环
    PID_Init(&speed_pid_left,  SKP, SKI, SKD, 0);
    PID_Init(&speed_pid_right, SKP, SKI, SKD, 0);

    // 清零编码器
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    r_current_cnt = __HAL_TIM_GET_COUNTER(&htim5);
    l_current_cnt = __HAL_TIM_GET_COUNTER(&htim2);
    // 确保占空比初始值合理（转向时用小占空比防止冲出）
    l_current_duty = 10;
    r_current_duty = 10;
    PID_Base_Start();
    printf("Current State = STATE_TURN_LEFT\r\n");
}

void BOW_Turn_Left_Update(void) {
    PID_Right_Setup(PosPID_Update(&pos_pid_right, r_current_cnt, 0.01f));
    PID_Left_Setup(PosPID_Update(&pos_pid_left,  l_current_cnt, 0.01f));
    if (pos_pid_left.stop_flg == 1 && pos_pid_right.stop_flg == 1) {
        BOW_Change_State(BOW_STATE_FORWARD);
    }
}
BOW_State BOW_Current_State = BOW_STATE_IDLE;//初始状态

BOW_StateHandler BOW_State_table[] = {
    [BOW_STATE_IDLE] = {BOW_Idle_Enter,BOW_Idle_Update},
    [BOW_STATE_FORWARD] = {BOW_Forward_Enter,BOW_Forward_Update},
    [BOW_STATE_TURN_LEFT] = {BOW_Turn_Left_Enter,BOW_Turn_Left_Update},
    [BOW_STATE_TURN_RIGHT] = {BOW_Turn_Right_Enter,BOW_Turn_Right_Update}
};

void BOW_Change_State(BOW_State new_state) {
    BOW_Current_State = new_state;
    BOW_State_table[new_state].on_enter();   // 调用新状态的进入函数（做初始化）
}

























