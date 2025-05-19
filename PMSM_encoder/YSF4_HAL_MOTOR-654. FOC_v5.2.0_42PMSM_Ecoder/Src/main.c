/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motorcontrol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>   
#include <string.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_RULER_TIM_PERIOD 65535
#define GO_HOME_SPEED 400

#define delay_time 10
#define control_delay 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// 定义存储与该算法相关变量的结构体
typedef struct
{
    unsigned int t_0; /* t0信号发送器开始工作的时刻, 单位s */
    unsigned int t_01; /* 从t0到t1的时间间隔, 单位s */
    float f0; /* 时刻t0对应的频率， 单位hz */
    float f1; /* 时刻t1对应的频率， 单位hz */
    float k; /* 指数函数的底数 */
    float p; /* 系数 p */
    float A; /* 扫频信号的幅值 */
} my_sweep_t;


typedef struct 
{
    uint8_t start_flag;
    uint8_t frame_len;
    uint8_t header_check;
    uint8_t data_buf[12];
    uint8_t frame_check;
} frame_matlab_t;



// 光栅尺溢出记数
int encoder_ruler_overflow_count = 0;

/// @brief  状态变量 0 对应没有启动，1 对应开始启动未到达左端，2 对应到达左端后向右运动且未到达右端， 3对应到达右端后开始回正， 4 对应回到了中间位置并停止
int start_status = 0; 

float now_position = 0;

float left_position = 0, right_position = 0;
float mid_position = 0;

float limit_position = 2;

int find_home_flag = 0; 

// my_sweep_t sweep;

// sweep = {
//     .t_0 = 0,
//     .t_01 = 0,
//     .f0 = 0,
//     .f1 = 0,
//     .k = 0,
//     .p = 0,
//     .A = 0
// };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}
int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart5, &ch, 1, 0xffff);
    return ch;
}


int init_my_sweep(my_sweep_t *sweep, unsigned int t_0, unsigned int t_01, float f0, float f1, float A);
float run_my_sweep(my_sweep_t *sweep, unsigned int t_now);
uint8_t get_uint8_sum_check(uint8_t *data, int len);
void  send_data_2_matlab(float data1, float data2, float data3);
void function_sweep_identification(void);


float Get_Encoder_Ruler_Count(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void my_motor_control(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
    MX_MotorControl_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

//! 初始化
HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
MC_AlignEncoderMotor1();
HAL_Delay(1000);

//使能溢出中断
__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);  
__HAL_TIM_URS_ENABLE(&htim3);      
__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE); 

    // //先回零
    // while(find_home_flag != 1)
    // {
    //     my_motor_control();
    //     HAL_Delay(100);
    // }

    // //扫频
    // HAL_Delay(1000);
    // MC_StartMotor1();
    find_home_flag = 1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    //对电机进行控制


    function_sweep_identification();
    // HAL_Delay(10);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM8_UP_TIM13_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* TIM8_BRK_TIM12_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = M1_PULSE_NBR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim8.Init.RepetitionCounter = (REP_COUNTER);
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin : GoHome_Pin */
  GPIO_InitStruct.Pin = GoHome_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GoHome_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_Stop_Pin */
  GPIO_InitStruct.Pin = Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM_P_Pin LIM_N_Pin */
  GPIO_InitStruct.Pin = LIM_P_Pin|LIM_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */


// TIM中断回调函数中查询溢出状态
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
        {
            encoder_ruler_overflow_count--; 
        }
        else
        {
            encoder_ruler_overflow_count++;
        }			
    }
}

// GPIO中断
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GoHome_Pin)
    {
        printf("handle_gohome_test: start\n");
        if(HAL_GPIO_ReadPin(GoHome_GPIO_Port,GoHome_Pin) == GPIO_PIN_RESET && start_status == 0)
        {
            int16_t  hFinalSpeed = GO_HOME_SPEED/6;
            uint16_t hDurationms = 0;
            MC_ProgramSpeedRampMotor1(hFinalSpeed,hDurationms);
            MC_StartMotor1();

            //
            start_status = 1;	
        }
    }				
}


// 计算光栅尺位置的代码
float Get_Encoder_Ruler_Count(void)
{
    int32_t CaptureNumber = 0;
    float   ruler_pos = 0.0f;  //mm
    
    uint32_t Value = __HAL_TIM_GET_COUNTER(&htim3);
    int32_t Period = ENCODER_RULER_TIM_PERIOD + 1;


    
    CaptureNumber =  Value + encoder_ruler_overflow_count * Period;

    
    
    ruler_pos = (float)(CaptureNumber* 0.005f); //mm
    // printf("%ld,%ld,%ld,%f  end \n",Value,Period,CaptureNumber,ruler_pos);
    return ruler_pos;
}

// 电机回零函数
void my_motor_control(void)
{
    
    now_position = Get_Encoder_Ruler_Count();
    printf("now: %.2f , mid: %f  \n",now_position,mid_position);

    if(start_status == 0)
    {

    }

    else if( start_status == 1)
    {
        //向左运动，设置速度时间，启动

        //读取PG1 的电平
        if(HAL_GPIO_ReadPin(LIM_P_GPIO_Port,LIM_P_Pin) == GPIO_PIN_RESET)
        {
            left_position = now_position;
            start_status = 2;
            int16_t  hFinalSpeed = -GO_HOME_SPEED/6;
            uint16_t hDurationms = 0;
            MC_ProgramSpeedRampMotor1(hFinalSpeed,hDurationms);
            HAL_Delay(delay_time);
        }
    
    }

    else if( start_status == 2)
    {
        //向右运动，设置速度时间，启动
        if(HAL_GPIO_ReadPin(LIM_N_GPIO_Port,LIM_N_Pin) == GPIO_PIN_RESET)
        {
            right_position = now_position;
            mid_position = left_position - (left_position - right_position) /2;
            //!!!!!!!!!!!!!!!!!!!
            start_status = 3;
            int16_t  hFinalSpeed = GO_HOME_SPEED/6;
            uint16_t hDurationms = 0;
            MC_ProgramSpeedRampMotor1(hFinalSpeed,hDurationms);
            HAL_Delay(delay_time);
        }

    }

    else if( start_status == 3)
    {
        //根据现在位置进行回零控制
        if(abs(now_position - mid_position) < limit_position)
        {
            MC_StopMotor1();
            start_status = 4;
            find_home_flag = 1;
        }
        else if( now_position < mid_position)
        {
            int16_t  hFinalSpeed = GO_HOME_SPEED/6;
            uint16_t hDurationms = 0;
            MC_ProgramSpeedRampMotor1(hFinalSpeed,hDurationms);
        }
        else if( now_position > mid_position)
        {
            int16_t  hFinalSpeed = -GO_HOME_SPEED/6;
            uint16_t hDurationms = 0;
            MC_ProgramSpeedRampMotor1(hFinalSpeed,hDurationms);
        }
        HAL_Delay(delay_time);

        // printf("now:%f,mid:%f \n",now_position,mid_position);

        // int16_t  hFinalSpeed = -GO_HOME_SPEED/6;
        // uint16_t hDurationms = 100;
        // MC_ProgramSpeedRampMotor1(hFinalSpeed,hDurationms);

        // HAL_Delay(control_delay);
    }
}

/// @brief 暂时废弃，等待完善
/// @param  
void back_zero_function(void)
{
    if(abs(now_position - mid_position) < limit_position)
    {
        MC_StopMotor1();
    }
    else if( now_position > mid_position)
    {
        int16_t  hFinalSpeed = GO_HOME_SPEED/6;
        uint16_t hDurationms = 0;
        MC_ProgramSpeedRampMotor1(hFinalSpeed,hDurationms);
    }
    else if( now_position < mid_position)
    {
        int16_t  hFinalSpeed = -GO_HOME_SPEED/6;
        uint16_t hDurationms = 0;
        MC_ProgramSpeedRampMotor1(hFinalSpeed,hDurationms);
    }
}


// 初始化函数，用于设置扫频信号的一些配置信息
/*
函数：init_my_sweep
功能：初始化一个频率随着时间指数增加的正弦扫频信号的结构体
输入：unsigned int t_0 t0信号发送器开始工作的时刻, 单位ms
输入：unsigned int t_01 从t0到t1的时间间隔, 单位ms
输入：float f0 时刻t0对应的频率， 单位hz
输入：float f1 时刻t1对应的频率， 单位hz
输入：float A 扫频信号的幅值
输出：int 0 = 成功， 其他表示异常
*/
int init_my_sweep(my_sweep_t *sweep, unsigned int t_0, unsigned int t_01, float f0, float f1, float A)
{
    if ((t_01 == 0) || (f0 <= 0.0f) || (f1 <= 0.0f) || (f0 == f1) || (A == 0) || (!sweep))
    {
        return -1; /*非法入参*/
    }

    sweep->t_0 = t_0;
    sweep->t_01 = t_01;
    sweep->f0 = f0;
    sweep->f1 = f1;
    sweep->A = A;

    /* start add code here */
    // sweep->k = /*计算指数函数的底数k，注意时间的单位要转换为秒*/
    // sweep->p = /*计算系数p, 注意单位转换*/

    sweep->k = exp(1/(0.001*t_01)*log(f1/f0));
    sweep->p = 2* 3.14159265 * f0 /(log(sweep->k));

    /* end add code here */

    return 0;
}

// 获取正弦扫频信号的函数
/*
函数：run_my_sweep
功能：根据当前时间输出频率随着时间指数增加的正弦扫频信号
输入：sweep 信号发生器结构体指针
输入：unsigned int t_now 当前时间 单位ms
输出：float 扫频信号
*/
float run_my_sweep(my_sweep_t *sweep, unsigned int t_now)
{
    float t = 0.0f; //相对时间 t
    float y = 0.0f; //扫频信号

    if (!sweep)
    {
        return 0.0f; /*非法入参*/
    }

    if (t_now < sweep->t_0)
    {
        return 0.0f; /*时间还未得到*/
    }

    t = (t_now - sweep->t_0) % sweep->t_01;
    t = t * 0.001f;
    y = sweep->A * sin(sweep->p*(pow(sweep->k,t)-1));

    // 此处代码还需根据具体公式补充计算 t 和 y 的逻辑
    return y;
}

/* 对 uint8_t 数值进行求和 ,获得这组数据一个uint8_t特征*/
uint8_t get_uint8_sum_check(uint8_t *data, int len)
{
    int i = 0;
    uint8_t sum = 0;
    for (i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

/*
函数：send_data_2_matlab
功能：往matlab发送三个浮点数
输入：三个浮点数
输出：无
*/
void  send_data_2_matlab(float data1, float data2, float data3)
{
    frame_matlab_t  frame = {0};
    int i = 0;

    /*填充帧头*/
    frame.start_flag = 0xAA;
    frame.frame_len  = sizeof(frame);
    frame.header_check = get_uint8_sum_check((uint8_t *)&frame, 2);
    /*填充数据*/
    memcpy((uint8_t *)&frame.data_buf[0], (uint8_t *)&data1, 4);
    memcpy((uint8_t *)&frame.data_buf[4], (uint8_t *)&data2, 4);
    // 原代码未完整展示填充 data3 的部分，补充如下
    memcpy((uint8_t *)&frame.data_buf[8], (uint8_t *)&data3, 4);
    frame.frame_check = get_uint8_sum_check((uint8_t *)&frame, sizeof(frame));
    // 原代码未展示发送数据的部分，假设使用 HAL_UART_Transmit 函数发送
    HAL_UART_Transmit(&huart1, (uint8_t *)&frame, sizeof(frame), HAL_MAX_DELAY); //!!!!! 串口1？
}

//*实现扫频辨识功能，该功能将被放置在main函数的while(1)中运行*/
void function_sweep_identification(void)
{
    static my_sweep_t sweep = {0};
    int16_t sweep_input = 0;
    int16_t sweep_output = 0;
    uint32_t sys_tick = 0;
    static uint32_t init_flag = 0;
    static uint32_t last_sys_tick = 0;
    static uint32_t start_sys_tick = 0;
    // 频率在10s内，从0.5hz变化到10hz，幅度为1500digitcurrent
    uint32_t t_period_ms = 10 * 1000; // 10s
    float f0 = 0.5;
    float f1 = 10;
    float Amp = 1700.0f;
    float time = 0.0f;
    sys_tick = HAL_GetTick(); // 获取当前时刻，单位ms
    time = 0.001f * sys_tick; // 单位s
    /*进入的条件时回零成功，且按了K1运行键,就开始执行扫频辨识过程,
   注意find_home_flag是回零成功的标志位，是一个全局变量,需要在外部实现这个标志位*/
    if ((find_home_flag == 1) && (MC_GetSTMStateMotor1() == RUN))
    {
        if (last_sys_tick != sys_tick) // 如果当前时刻发生了变化，这个条件每ms都会成立一次
        {
            last_sys_tick = sys_tick;
            if (sys_tick % 10 == 0) // 通过%把频率从1000hz降低到100hz，即每10ms发生一次
            {
                // 初始化扫频配置
                if (init_flag == 0)
                {
                    init_my_sweep(&sweep, sys_tick, t_period_ms, f0, f1, Amp);
                    printf("sweep-init:k=%.5f,p=%.5f\r\n", (float)sweep.k, (float)sweep.p);
                    init_flag = 1;
                }
                // 获取正弦扫频信号
                sweep_input = (int16_t)run_my_sweep(&sweep, sys_tick);
                // 将正弦扫频信号输入到STMCSDK的力矩控制API中
                MC_ProgramTorqueRampMotor1(sweep_input, 0);
                // 获取丝杆的转速信息，单位为0.1hz
                sweep_output = MC_GetMecSpeedAverageMotor1();
                // 把时间，input, output 发送到 matlab
                send_data_2_matlab(time, (float)sweep_input, (float)sweep_output);
            }
        }
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
