//
// Created by admin on 2026/6/14.
//

#include "BOW.h"
#include "headfile.h"
#include "pid.h"
#include "State_machine.h"
RobotState_t Current_State = STATE_IDLE;//当前状态
void Idle_Enter(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    //printf("Current State: Idle\n");
}

void Idle_Update(void) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
}

///这些实际上都是测试函数指针用的，在真正的状态机中这些函数不使用----------------------------------
void Forward_Enter(void) {
    pos_pid_left.stop_flg = 0;
    pos_pid_right.stop_flg = 0;
    last_left_encoder_cnt = 0;//将上次状态获取的编码值清0
    last_right_encoder_cnt = 0;
    pos_pid_left.target_distance = 1000;
    pos_pid_right.target_distance = 1000;
    PID_Base_Start();
    //printf("Current State = STATE_MOVE_FORWARD\r\n");
}

void Forward_Update(void) {

    float vL = PosPID_Update(&pos_pid_left,  l_current_cnt, 0.01f);
    float vR = PosPID_Update(&pos_pid_right, r_current_cnt, 0.01f);

    //同步：取绝对值较小的那个（跟随慢侧）
    sync_target_rpm = (fabs(vL) < fabs(vR)) ? vL : vR;
    use_sync_speed = 1;
    PID_Right_Setup(sync_target_rpm);
    PID_Left_Setup(sync_target_rpm);
    if (pos_pid_left.stop_flg == 1 && pos_pid_right.stop_flg == 1) {
        Change_State(STATE_IDLE);
    }
}
///-----------------------------------------------------------------------------------

void BOW_Enter(void) {
    //printf("Current State = BOW_Enter\r\n");
    BOW_Change_State(BOW_STATE_FORWARD);
}

void BOW_Update(void) {
    BOW_State_table[BOW_Current_State].on_update();
}


//这是关键，这是状态与任务的对应表
StateAction state_table[] = {
    [STATE_IDLE] = {Idle_Enter,Idle_Update},
    [STATE_MOVE_FORWARD] = {Forward_Enter,Forward_Update},
    [STATE_BOW_CLEAN] = {BOW_Enter,BOW_Update},
};

void Change_State(RobotState_t new_state) {
    Current_State = new_state;
    state_table[new_state].enter();   // 调用新状态的进入函数（做初始化）
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        static  uint8_t counter = 0;
        if (Current_State == STATE_BOW_CLEAN && BOW_Current_State == BOW_STATE_FORWARD) {
            Speed_Loop_Forward();
        }
        counter++;
        if (counter >= 10) {
            counter = 0;
            state_table[Current_State].update();
            printf("%d,%d,%d,%d\n", (int)sync_target_rpm, (int)l_actual_rpm, (int)r_actual_rpm);
            // printf("%d,%d,%d\n", (int)pos_pid_left.current_pulse, (int)pos_pid_left.target_pulse, (int)pos_pid_left.error);
            // printf("%d,%d\n", (int)pos_pid_left.output_speed, (int)pos_pid_left.error);
        }
    }
}


char* mystrcpy(char *dest, const char *src) {
    char* temp = dest;
    while ((*dest++ = *src++) != '\0');
    return temp;
}


