#include "motor.h"
#include "tim.h"
#include "gpio.h"

/* 左右轮 PID 内部状态 */
static float left_error_last = 0.0f;//左轮上次误差
static float left_error_prev = 0.0f;//左轮上上次误差
static float left_pwm_out = 0.0f;//左轮PID输出

static float right_error_last = 0.0f;//右轮上次误差
static float right_error_prev = 0.0f;//右轮上上次误差
static float right_pwm_out = 0.0f;//右轮PID输出


/**
  * @brief  电机初始化
  */
void Motor_Init(void)
{
    //开启定时器2,3的编码器模式
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    
    //开启定时器4的PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    
    //开启定时器1的1ms中断
    HAL_TIM_Base_Start_IT(&htim1);
}

/**
  * @brief  设置电机PWM
  * @param  motor_left: 左电机PWM值，带符号
  * @param  motor_right: 右电机PWM值，带符号
  */
void Set_Pwm(int motor_left, int motor_right)
{
    //处理左电机 (假设对应AO1和AO2，配合左电机PWM通道 TIM4_CH1或CH2)
    //根据实际接线，调整通道和引脚：这里以左电机接A通道，右电机接B通道为例
    if(motor_left < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        motor_left = -motor_left;
    }
    else if(motor_left > 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }
    
    //处理右电机 (配合BO1和BO2)
    if(motor_right < 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        motor_right = -motor_right;
    }
    else if(motor_right > 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    
    //限幅
    if(motor_left > PWM_MAX) motor_left = PWM_MAX;
    if(motor_right > PWM_MAX) motor_right = PWM_MAX;
    
    //输出PWM (假设TIM4 CH1对应左电机, TIM4 CH2对应右电机，如果对应反了在这更改即可)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_left);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, motor_right);
}


/**
  * @brief  计算左电机增量式PID
  * @param  target_speed: 目标速度
  * @param  current_speed: 当前速度
  */
int PID_Calc_Left(float target_speed, float current_speed)
{
    float error = target_speed - current_speed;
    
    // 死区：当目标速度为0时，若实际速度非常小(比如噪声)，直接输出0，使电机彻底安静
    if(target_speed == 0 && error > -1.0f && error < 1.0f)
    {
        left_pwm_out = 0;
        left_error_last = 0;
        left_error_prev = 0;
        return 0;
    }

    left_pwm_out += Kp_L * (error - left_error_last) + Ki_L * error + Kd_L * (error - 2.0f * left_error_last + left_error_prev);
    
    //反积分饱和（Anti-Windup）限幅
    if(left_pwm_out > PWM_MAX) left_pwm_out = PWM_MAX;
    if(left_pwm_out < -PWM_MAX) left_pwm_out = -PWM_MAX;

    left_error_prev = left_error_last;
    left_error_last = error;
    
    return (int)left_pwm_out;
}

/**
  * @brief  计算右电机增量式PID
  * @param  target_speed: 目标速度
  * @param  current_speed: 当前速度
  */
int PID_Calc_Right(float target_speed, float current_speed)
{
    float error = target_speed - current_speed;
    
    if(target_speed == 0 && error > -1.0f && error < 1.0f)
    {
        right_pwm_out = 0;
        right_error_last = 0;
        right_error_prev = 0;
        return 0;
    }

    right_pwm_out += Kp_R * (error - right_error_last) + Ki_R * error + Kd_R * (error - 2.0f * right_error_last + right_error_prev);
    
    if(right_pwm_out > PWM_MAX) right_pwm_out = PWM_MAX;
    if(right_pwm_out < -PWM_MAX) right_pwm_out = -PWM_MAX;

    right_error_prev = right_error_last;
    right_error_last = error;

    return (int)right_pwm_out;
}

/**
  * @brief  清空左轮PID历史项
  */
void PID_Reset_Left(void)
{
    left_error_last = 0.0f;
    left_error_prev = 0.0f;
    left_pwm_out = 0.0f;
}

/**
  * @brief  清空右轮PID历史项
  */
void PID_Reset_Right(void)
{
    right_error_last = 0.0f;
    right_error_prev = 0.0f;
    right_pwm_out = 0.0f;
}

