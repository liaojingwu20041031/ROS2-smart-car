#include "bmq.h"
#include "tim.h"

/**
  * @brief  读取编码器计数值
  * @param  TIMX: 编码器对应的定时器(2或者3)
  * @retval 相对上次读取编码器的计数值
  */
int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;
    if(TIMX == 2)
    {
        Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
    }
    else if(TIMX == 3)
    {
        Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim3);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
    else
    {
        Encoder_TIM = 0;
    }
    return Encoder_TIM;
}

