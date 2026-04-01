/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "bmq.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile float Speed_Left = 0;// 左电机当前速度（RPM、转/分钟）
volatile float Speed_Right = 0;// 右电机当前速度（RPM、转/分钟）
volatile float Target_Speed_Left = 0;  // 设定左电机目标速度（RPM、转/分钟）
volatile float Target_Speed_Right = 0; // 设定右电机目标速度（RPM、转/分钟）
volatile int PWM_Left = 0;// 左电机PWM输出值（占空比）
volatile int PWM_Right = 0;// 右电机PWM输出值（占空比）
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART2_FRAME_LEN              7U // 串口帧长度
#define UART2_FRAME_HEAD1            0xAAU // 串口帧头1
#define UART2_FRAME_HEAD2            0x55U // 串口帧头2
#define WHEEL_DIAMETER_MM            54.0f // 轮子直径（mm）
#define WHEEL_CIRCUMFERENCE_MM       (WHEEL_DIAMETER_MM * 3.1415926f) // 轮子周长（mm）
#define LEFT_SPEED_SIGN              1.0f // 根据电机接线调整正负，确保正值为前进，负值为后退
#define RIGHT_SPEED_SIGN             1.0f // 根据电机接线调整正负，确保正值为前进，负值为后退
#define UART2_SEND_PERIOD_MS         15U // 串口发送周期（ms）
#define REVERSE_RELEASE_RPM          8.0f // 反向前判定已停稳的速度阈值（RPM）
#define REVERSE_FILTER_NEW_RATIO     0.7f // 反向过零时提高瞬时速度占比
#define NORMAL_FILTER_NEW_RATIO      0.3f // 正常状态下一阶滤波瞬时速度占比

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t uart2_rx_byte = 0;// 串口接收字节
static uint8_t uart2_rx_buf[UART2_FRAME_LEN] = {0};// 串口接收缓冲区
static uint8_t uart2_rx_index = 0;// 串口接收索引
volatile int16_t uart2_target_left_mm_s = 0;// 设定左电机目标速度（毫米/秒）
volatile int16_t uart2_target_right_mm_s = 0;// 设定右电机目标速度（毫米/秒）
volatile float uart2_target_left_rpm = 0;// 左轮目标转速（RPM），收到串口命令时预先换算好
volatile float uart2_target_right_rpm = 0;// 右轮目标转速（RPM），收到串口命令时预先换算好
volatile int16_t uart2_feedback_left_mm_s = 0;// 左电机反馈速度（毫米/秒）
volatile int16_t uart2_feedback_right_mm_s = 0;// 右电机反馈速度（毫米/秒）
volatile uint8_t uart2_send_flag = 0;// 串口发送标志位
volatile uint8_t left_reverse_brake_flag = 0;// 左轮反向前先刹停标志
volatile uint8_t right_reverse_brake_flag = 0;// 右轮反向前先刹停标志

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t UART2_CalcChecksum(const uint8_t *buf);// 计算校验和
static float MmpsToRpm(int16_t mm_s);// 毫米/秒转RPM
static int16_t RpmToMmps(float rpm);// RPM转毫米/秒
static float AbsFloat(float value);// 取绝对值

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 计算前6字节累加和，低8位作为校验 */
static uint8_t UART2_CalcChecksum(const uint8_t *buf)
{
    uint16_t sum = 0;
    uint8_t i = 0;

    for (i = 0; i < (UART2_FRAME_LEN - 1U); i++)
    {
        sum += buf[i];
    }

    return (uint8_t)sum;
}

/* RDK 发来的是放大1000倍后的整数速度，本质就是 mm/s */
static float MmpsToRpm(int16_t mm_s)// 毫米/秒--->RPM
{
    return ((float)mm_s * 60.0f) / WHEEL_CIRCUMFERENCE_MM;
}

/* 把电机当前 RPM 换算成 mm/s，再按 short 回传给 RDK */
static int16_t RpmToMmps(float rpm)// RPM--->毫米/秒
{
    float mm_s = (rpm * WHEEL_CIRCUMFERENCE_MM) / 60.0f;

    if (mm_s > 32767.0f)
    {
        mm_s = 32767.0f;
    }
    else if (mm_s < -32768.0f)
    {
        mm_s = -32768.0f;
    }

    if (mm_s >= 0.0f)
    {
        return (int16_t)(mm_s + 0.5f);
    }

    return (int16_t)(mm_s - 0.5f);
}

static float AbsFloat(float value)
{
    if (value >= 0.0f)
    {
        return value;
    }

    return -value;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)// 串口接收完成回调函数
{
    if (huart->Instance == USART2)
    {
        /* 按字节收 7 字节协议帧，逻辑尽量写直白一些 */
        if (uart2_rx_index == 0U)
        {
            if (uart2_rx_byte == UART2_FRAME_HEAD1)
            {
                uart2_rx_buf[0] = uart2_rx_byte;
                uart2_rx_index = 1U;
            }
        }
        else if (uart2_rx_index == 1U)
        {
            if (uart2_rx_byte == UART2_FRAME_HEAD2)
            {
                uart2_rx_buf[1] = uart2_rx_byte;
                uart2_rx_index = 2U;
            }
            else if (uart2_rx_byte == UART2_FRAME_HEAD1)
            {
                uart2_rx_buf[0] = uart2_rx_byte;
                uart2_rx_index = 1U;
            }
            else
            {
                uart2_rx_index = 0U;
            }
        }
        else
        {
            uart2_rx_buf[uart2_rx_index++] = uart2_rx_byte;

            if (uart2_rx_index >= UART2_FRAME_LEN)
            {
                uart2_rx_index = 0U;

                if (uart2_rx_buf[6] == UART2_CalcChecksum(uart2_rx_buf))
                {
                    int16_t left_mm_s = (int16_t)(((uint16_t)uart2_rx_buf[2] << 8) | uart2_rx_buf[3]);
                    int16_t right_mm_s = (int16_t)(((uint16_t)uart2_rx_buf[4] << 8) | uart2_rx_buf[5]);

                    /* 新命令和上一条命令方向相反时，先清空PID历史项，再走刹停后反向 */
                    if (((uart2_target_left_mm_s > 0) && (left_mm_s < 0)) || ((uart2_target_left_mm_s < 0) && (left_mm_s > 0)))
                    {
                        left_reverse_brake_flag = 1U;
                        PID_Reset_Left();
                    }

                    if (((uart2_target_right_mm_s > 0) && (right_mm_s < 0)) || ((uart2_target_right_mm_s < 0) && (right_mm_s > 0)))
                    {
                        right_reverse_brake_flag = 1U;
                        PID_Reset_Right();
                    }

                    uart2_target_left_mm_s = left_mm_s;
                    uart2_target_right_mm_s = right_mm_s;
                    uart2_target_left_rpm = LEFT_SPEED_SIGN * MmpsToRpm(left_mm_s);
                    uart2_target_right_rpm = RIGHT_SPEED_SIGN * MmpsToRpm(right_mm_s);
                }
            }
        }

        (void)HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)// 串口错误回调函数
{
    if (huart->Instance == USART2)
    {
        uart2_rx_index = 0U;// 接收帧索引重置
        (void)HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1U);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1U);// 开启接收中断
  Motor_Init();// 初始化电机

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //printf("%.2f,%.2f\n", Speed_Left, Speed_Right);
    /* 发送节拍由 TIM1 控制，负责发串口，避免在中断里阻塞 */
    if (uart2_send_flag != 0U)// 发送标志位
    {
        uint8_t tx_buf[UART2_FRAME_LEN];
        uint16_t left_mm_s = (uint16_t)uart2_feedback_left_mm_s;
        uint16_t right_mm_s = (uint16_t)uart2_feedback_right_mm_s;

        uart2_send_flag = 0U;// 发送标志位重置

        tx_buf[0] = UART2_FRAME_HEAD1;
        tx_buf[1] = UART2_FRAME_HEAD2;
        tx_buf[2] = (uint8_t)(left_mm_s >> 8);
        tx_buf[3] = (uint8_t)left_mm_s;
        tx_buf[4] = (uint8_t)(right_mm_s >> 8);
        tx_buf[5] = (uint8_t)right_mm_s;
        tx_buf[6] = UART2_CalcChecksum(tx_buf);

        (void)HAL_UART_Transmit(&huart2, tx_buf, UART2_FRAME_LEN, 5U);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


// 转换系数 = 60 / (122880 * 0.01) = 0.048828125
const float PULSE_TO_RPM = 0.048828125f;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        static uint8_t T1 = 0;
        static uint8_t T2 = 0;
        // static long long T3 = 0;// 用于速度变化检测
        /* TIM1 是 1ms 中断，这里累加出 10ms 控制节拍 */
        T1++;
        if (T1 >= 10U)
        {
            T1 = 0U;
            int encoder_left = Read_Encoder(2);
            int encoder_right = Read_Encoder(3);

            /* 编码器脉冲换算成当前 RPM */
            float instant_speed_left = encoder_left * PULSE_TO_RPM;
            float instant_speed_right = encoder_right * PULSE_TO_RPM;

            /* 正常情况保持原来的滤波；只有反向过零时才临时提高瞬时速度占比 */
            if (left_reverse_brake_flag != 0U)
            {
                Speed_Left = REVERSE_FILTER_NEW_RATIO * instant_speed_left + (1.0f - REVERSE_FILTER_NEW_RATIO) * Speed_Left;
            }
            else
            {
                Speed_Left = NORMAL_FILTER_NEW_RATIO * instant_speed_left + (1.0f - NORMAL_FILTER_NEW_RATIO) * Speed_Left;
            }

            if (right_reverse_brake_flag != 0U)
            {
                Speed_Right = REVERSE_FILTER_NEW_RATIO * instant_speed_right + (1.0f - REVERSE_FILTER_NEW_RATIO) * Speed_Right;
            }
            else
            {
                Speed_Right = NORMAL_FILTER_NEW_RATIO * instant_speed_right + (1.0f - NORMAL_FILTER_NEW_RATIO) * Speed_Right;
            }

            /* 反向时先给0目标，等轮子基本停住后再放行到新方向 */
            if (left_reverse_brake_flag != 0U)
            {
                Target_Speed_Left = 0.0f;
                if (AbsFloat(Speed_Left) <= REVERSE_RELEASE_RPM)
                {
                    left_reverse_brake_flag = 0U;
                    PID_Reset_Left();
                    Target_Speed_Left = uart2_target_left_rpm;
                }
            }
            else
            {
                Target_Speed_Left = uart2_target_left_rpm;
            }

            if (right_reverse_brake_flag != 0U)
            {
                Target_Speed_Right = 0.0f;
                if (AbsFloat(Speed_Right) <= REVERSE_RELEASE_RPM)
                {
                    right_reverse_brake_flag = 0U;
                    PID_Reset_Right();
                    Target_Speed_Right = uart2_target_right_rpm;
                }
            }
            else
            {
                Target_Speed_Right = uart2_target_right_rpm;
            }

            /* PID 输入单位保持为 RPM */
            PWM_Left = PID_Calc_Left(Target_Speed_Left, Speed_Left);
            PWM_Right = PID_Calc_Right(Target_Speed_Right, Speed_Right);

            /* 输出 PWM */
            Set_Pwm(PWM_Left, PWM_Right);
        }

        /* T2 每 15ms 准备一次串口反馈数据 */
        T2++;
        if (T2 >= UART2_SEND_PERIOD_MS)
        {
            T2 = 0U;
            uart2_feedback_left_mm_s = RpmToMmps(Speed_Left * LEFT_SPEED_SIGN);
            uart2_feedback_right_mm_s = RpmToMmps(Speed_Right * RIGHT_SPEED_SIGN);
            uart2_send_flag = 1U;
        }

        /* T3用于速度变化检测 */
        // T3++;
        // if (T3 == 1000)
        // {
        //     uart2_target_left_rpm = 50.0f;
        //     uart2_target_right_rpm = 50.0f;
        // }
        // else if(T3 == 5000)
        // {
        //     uart2_target_left_rpm = -50.0f;
        //     uart2_target_right_rpm = -50.0f;
        // }
        // else if(T3 == 10000)
        // {
        //     uart2_target_left_rpm = 0.0f;
        //     uart2_target_right_rpm = 0.0f;
        // }
        // else if(T3 == 15000)
        // {
        //     uart2_target_left_rpm = -50.0f;
        //     uart2_target_right_rpm = 50.0f;
        // }
        // else if(T3 == 20000)
        // {
        //     uart2_target_left_rpm = 0.0f;
        //     uart2_target_right_rpm = 0.0f;
        // }
        
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
