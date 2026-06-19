//电机控制部分
#include "moto.h"





void Action_mode(char *mode,uint8_t speed)
{
    if (strcmp(mode,"Advance") == 0)//如果mode指向的内容是“Advance”
    {
        //左轮前进
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        //右轮前进
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        TIM3->CCR1 = speed;
        TIM4->CCR1 = speed;
        printf("Action mode:Advance,Speed:%d",speed);
    }
    else if (strcmp(mode,"Back_off") == 0)
    {
        //左轮后退
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
        //右轮后退
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        TIM3->CCR1 = speed;
        TIM4->CCR1 = speed;
        printf("Action mode:Back_off,Speed:%d",speed);
    }
    else if (strcmp(mode,"Turn_left") == 0)
    {
        //左轮停止
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        //右轮前进
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        TIM3->CCR1 = speed;
        TIM4->CCR1 = speed;
        printf("Action mode:Turn_left,Speed:%d",speed);
    }
    else if (strcmp(mode,"Turn_right") == 0)
    {
        //左轮前进
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        //右轮停止
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        TIM3->CCR1 = speed;
        TIM4->CCR1 = speed;
        printf("Action mode:Turn_right,Speed:%d",speed);
    }
    else if (strcmp(mode,"Stop") == 0 || speed == 0) {
        speed = 0;
        TIM3->CCR1 = speed;
        TIM4->CCR1 = speed;
        printf("Action mode:Stop,Speed:%d",speed);
    }
}





















