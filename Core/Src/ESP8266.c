#include "ESP8266.h"

#include "PID.h"
#include "tim.h"
#include "usart.h"
#include "State_machine.h"

void ESP8266_SendString(char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

uint8_t ESP8266_SendCommand(char *cmd, char *expected, uint32_t timeout_ms) {
    uint8_t rx_buffer[100] = {0};  // 存放回复的缓冲区

    ESP8266_SendString(cmd);       // 发送指令

    // 阻塞接收，直到收到数据或超时
    HAL_UART_Receive(&huart1, rx_buffer, sizeof(rx_buffer)-1, timeout_ms);

    // 检查回复中是否包含期望的字符串
    if(strstr((char*)rx_buffer, expected) != NULL) {
        return 1;  // 成功
    }
    return 0;      // 失败
}

void ESP_8266_Init(void) {
    ESP8266_SendCommand("AT\r\n", "OK", 1000);
    ESP8266_SendCommand("ATE0\r\n", "OK", 500);
    ESP8266_SendCommand("AT+CWMODE=3\r\n", "OK", 1000);
    ESP8266_SendCommand("AT+CIPMUX=1\r\n", "OK", 1000);
    ESP8266_SendCommand("AT+CIPSERVER=1,5000\r\n", "OK", 1000);


    printf("--- ESP8266 Init Start ---\r\n");

    printf("1. Disable echo...");
    if(ESP8266_SendCommand("ATE0\r\n", "OK", 500)) printf("OK\r\n");
    else printf("FAIL\r\n");

    printf("2. Set WiFi mode...");
    if(ESP8266_SendCommand("AT+CWMODE=3\r\n", "OK", 1000)) printf("OK\r\n");
    else printf("FAIL\r\n");

    printf("3. Enable multiple connections...");
    if(ESP8266_SendCommand("AT+CIPMUX=1\r\n", "OK", 1000)) printf("OK\r\n");
    else printf("FAIL\r\n");

    printf("4. Start TCP Server on port 5000...");
    if(ESP8266_SendCommand("AT+CIPSERVER=1,5000\r\n", "OK", 1000)) printf("OK\r\n");
    else printf("FAIL\r\n");

    printf("--- Server Ready ---\r\n");
}

#define RX_BUF_SIZE 256
uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t rx_ready = 0;   // 一帧数据接收完成标志
uint8_t rx_len = 0;


/**
  * @brief 空闲中断接收完成回调（HAL库自动调用）
  * @param huart: 触发中断的串口句柄
  * @param Size: 本次接收到的数据字节数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1 && Size > 0)
    {
        rx_buf[Size] = '\0';        // 添加字符串结束符，确保printf等函数正确处理
        rx_ready = 1;               // 通知主循环：有一帧完整数据待处理
        rx_len = Size;
        printf("okCallback\r\n");
        // 立即重启接收，准备接收下一帧
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf, RX_BUF_SIZE);
    }
}
typedef enum {
    CMD_LED_OFF = 0,
    CMD_LED_ON,
    CMD_Advance_1m,
    CMD_Advance_STOP,
    CMD_Turn_Left,
    CMD_Turn_Right,
    CMD_BOW_CLEAN,
    CMD_UNKNOW
}cmd_t;

static cmd_t parse_cmd(const char *buf) {
    if (buf != NULL) {
        // if (strstr((char*)buf, "LED_OFF")) {
        //     return CMD_LED_OFF;
        // }
        // if (strstr((char*)buf, "LED_ON")) {
        //     return CMD_LED_ON;
        // }
        if (strstr((char*)buf, "ADVANCE_1m")) {
            return CMD_Advance_1m;
        }
        if (strstr((char*)buf, "Advance_STOP")) {
            return CMD_Advance_STOP;
        }
        // if (strstr((char*)buf, "Turn_Left")) {
        //     return CMD_Turn_Left;
        // }
        // if (strstr((char*)buf, "Turn_Right")) {
        //     return CMD_Turn_Right;
        // }
        if (strstr((char*)buf, "BOW_CLEAN")) {
            return CMD_BOW_CLEAN;
        }

    }
    return CMD_UNKNOW;
}







void Signal_decoding() {
    //@brief 手机发送的信息解码
    if (!rx_ready) return;
    rx_ready = 0;
    rx_buf[rx_len] = '\0';   // 方便用字符串函数
    cmd_t cmd = parse_cmd((char*)rx_buf);

    switch (cmd) {
        case CMD_LED_OFF:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            printf("LED OFF\r\n");
            break;
        case CMD_LED_ON:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
            printf("LED ON\r\n");
            break;
        case CMD_Advance_1m:
            Change_State(STATE_MOVE_FORWARD);//这是针对函数指针所做的
            printf("Current State = STATE_MOVE_FORWARD\r\n");
            break;
        case CMD_Advance_STOP:
            PID_Base_Stop();
            break;
        case CMD_Turn_Left:
            // 重置位置环（清零积分和误差历史）
            PosPID_Init(&pos_pid_left, PKP, PKI, PKD);
            PosPID_Init(&pos_pid_right, PKP, PKI, PKD);

            pos_pid_left.target_pulse  = -986.0f;   // 左轮反转
            pos_pid_right.target_pulse =  986.0f;   // 右轮正转

            printf("Target l Pulse = %d\r\n",(int)pos_pid_left.target_pulse);
            printf("Target r Pulse = %d\r\n",(int)pos_pid_right.target_pulse);
            HAL_Delay(1000);

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
            Current_State = STATE_TURN_LEFT;
            break;
        case CMD_Turn_Right:
            // 重置位置环（清零积分和误差历史）
            PosPID_Init(&pos_pid_left, PKP, PKI, PKD);
            PosPID_Init(&pos_pid_right, PKP, PKI, PKD);

            pos_pid_left.target_pulse  = 986.0f;
            pos_pid_right.target_pulse = -986.0f;

            printf("Target l Pulse = %d\r\n",(int)pos_pid_left.target_pulse);
            printf("Target r Pulse = %d\r\n",(int)pos_pid_right.target_pulse);
            HAL_Delay(1000);

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
            Current_State = STATE_TURN_RIGHT;
            break;

        case CMD_BOW_CLEAN:
            Change_State(STATE_BOW_CLEAN);
            break;
    }

    // 清空缓冲，准备接收下一次数据
    memset(rx_buf, 0, RX_BUF_SIZE);

}




/**
  * @brief LED控制函数（保持原样）
  */
void LED_Control(uint8_t state)
{
    if(state)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // 低电平点亮
    else
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);    // 高电平熄灭
}






















