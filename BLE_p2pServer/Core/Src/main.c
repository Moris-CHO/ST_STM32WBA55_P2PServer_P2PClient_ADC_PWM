/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_VIN 				    adc_result[1]
#define ADC_IIN				      adc_result[2]
#define ADC_ISHUNT 			    adc_result[0]
#define ADC_BUFFER_SIZE			(3UL)
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)
#define Fs_I 2000000.0    // 샘플링 주파수 (Hz)
#define Fc_I 200000.0     // 차단 주파수 (Hz)
#define Fs 800.0          // 샘플링 주파수 (Hz)
#define Fc 80.0           // 차단 주파수 (Hz)

#define TIM1_CLK_HZ     64000000UL            // TIM1 클럭: 64 MHz
#define T_DTS_NS        (1e9f / TIM1_CLK_HZ)  // 15.625ns
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc4;
DMA_NodeTypeDef Node_GPDMA1_Channel2;
DMA_QListTypeDef List_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel2;

CRC_HandleTypeDef hcrc;

RAMCFG_HandleTypeDef hramcfg_SRAM1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */
DMA_HandleTypeDef hdma_adc4;
volatile float V_IN = 0;
volatile float I_IN = 0;
volatile float I_SHUNT = 0;
volatile float V_IN_OLD = 0;
volatile float I_IN_OLD = 0;
volatile float I_SHUNT_OLD = 0;
volatile float V_IN_OLD_Filtered = 0;
volatile float I_IN_OLD_Filtered = 0;
volatile float I_SHUNT_OLD_Filtered = 0;
volatile float V_IN_RMS = 0;
volatile float I_IN_RMS = 0;
volatile float I_SHUNT_RMS = 0;
volatile float DMA_CNT = 0;
volatile int pwm_change_cnt = 0;
volatile int pwm_ref = 250;
volatile uint32_t dead_ref = 10000;         //ns 단위 , 1000 = 1us
volatile uint32_t dead_ref_start = 1000;    //ns 단위 , 1000 = 1us
volatile uint32_t dead_ref_end = 600;       //ns 단위 , 1000 = 1us, 25단위로 작성
volatile int freq_hz_old = 0;
volatile int dead_ref_old = 0;
volatile int main_status = 0;
volatile int ratio = 30;
volatile int it_while_cnt = 0;
volatile float sensing_time = 0.75;

volatile uint16_t adc_result[ADC_BUFFER_SIZE];
volatile float adc_index = 0;
volatile uint8_t ubDmaTransferStatus = 2U;
volatile int pwm_start = 1;//0;
double alpha = (2.0 * M_PI * Fc / Fs) / (2.0 * M_PI * Fc / Fs + 1.0);
double alpha_I = (2.0 * M_PI * Fc_I / Fs_I) / (2.0 * M_PI * Fc_I / Fs_I + 1.0);
// 필터 상태 변수 (이전 출력 저장)
double prev_output = 0.0;
double prev_output_I = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void PWM_SET_FREQ_DUTY_DEADTIME(uint16_t freq_hz, uint8_t dead_ref);
float low_pass_filter(float input);
double low_pass_filter_I(double input);
uint8_t compute_deadtime_dtg(uint32_t deadtime_ns, uint32_t t_dts_ns);
void Sweep_DeadTime_Down(void);
void Read_ADCChannels_Polling(void);
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
	uint32_t tmp_index;

  /* Initialize ADC group regular data buffer values */
  for (tmp_index = 0; tmp_index < ADC_BUFFER_SIZE; tmp_index++)
  {
    adc_result[tmp_index] = VAR_CONVERTED_DATA_INIT_VALUE;
  }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_RAMCFG_Init();
  MX_RTC_Init();
  MX_ICACHE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  MX_ADC4_Init();
  if (HAL_ADCEx_Calibration_Start(&hadc4) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

  HAL_ADC_Start_DMA(&hadc4,(uint32_t*)adc_result,ADC_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init(NULL);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if((pwm_start != 0) && (ubDmaTransferStatus == 1))
    {
      PWM_SET_FREQ_DUTY_DEADTIME(pwm_ref, dead_ref);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  	  if(dead_ref > dead_ref_end)
  	  {
  	    Sweep_DeadTime_Down();
        //HAL_Delay(1000);
  	  }
  	  else
  	  {
    	  if(pwm_ref > 177)
        {
          pwm_ref--;
          HAL_Delay(100);
        }
  	  }

      V_IN_OLD = (ADC_VIN*132.34 *3.3/4095);
      I_SHUNT_OLD = (((ADC_ISHUNT * 3.3/4095)-1.65)*6.25)*0.707;
      I_IN_OLD = (ADC_IIN*3.3/4095)*5.88;

      V_IN += (V_IN_OLD*V_IN_OLD);
      I_SHUNT += (I_SHUNT_OLD*I_SHUNT_OLD);
      I_IN += (I_IN_OLD*I_IN_OLD);
      DMA_CNT++;
      if(DMA_CNT > 50)
      {
      V_IN_RMS = sqrt(fabs(V_IN)/DMA_CNT);
      I_SHUNT_RMS = sqrt(fabs(I_SHUNT)/DMA_CNT);
      I_IN_RMS = sqrt(fabs(I_IN)/DMA_CNT);


      DMA_CNT = 0;
      V_IN = 0;
      I_SHUNT = 0;
      I_IN = 0;
      }
      ubDmaTransferStatus = 0;
    }
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL1.PLLM = 2;
  RCC_OscInitStruct.PLL1.PLLN = 8;
  RCC_OscInitStruct.PLL1.PLLP = 2;
  RCC_OscInitStruct.PLL1.PLLQ = 2;
  RCC_OscInitStruct.PLL1.PLLR = 2;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK7|RCC_CLOCKTYPE_HCLK5;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB7CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHB5_PLL1_CLKDivider = RCC_SYSCLK_PLL1_DIV4;
  RCC_ClkInitStruct.AHB5_HSEHSI_CLKDivider = RCC_SYSCLK_HSEHSI_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

   /* Select SysTick source clock */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

   /* Re-Initialize Tick with new clock source */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RADIOST;
  PeriphClkInit.RadioSlpTimClockSelection = RCC_RADIOSTCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoPowerOff = DISABLE;
  hadc4.Init.LowPowerAutonomousDPD = ADC_LP_AUTONOMOUS_DPD_DISABLE;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 3;
  hadc4.Init.DiscontinuousConvMode = ENABLE;
  hadc4.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc4.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc4.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.GeneratingPolynomial = 7607;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Full retention for ICACHE in stop mode
  */
  LL_PWR_SetICacheRAMStopRetention(LL_PWR_ICACHERAM_STOP_FULL_RETENTION);

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  LL_ICACHE_SetMode(LL_ICACHE_1WAY);
  LL_ICACHE_Enable();
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief RAMCFG Initialization Function
  * @param None
  * @retval None
  */
void MX_RAMCFG_Init(void)
{

  /* USER CODE BEGIN RAMCFG_Init 0 */

  /* USER CODE END RAMCFG_Init 0 */

  /* USER CODE BEGIN RAMCFG_Init 1 */

  /* USER CODE END RAMCFG_Init 1 */

  /** Initialize RAMCFG SRAM1
  */
  hramcfg_SRAM1.Instance = RAMCFG_SRAM1;
  if (HAL_RAMCFG_Init(&hramcfg_SRAM1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RAMCFG_Init 2 */

  /* USER CODE END RAMCFG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = 31;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_ONLY;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  if (HAL_RTCEx_SetSSRU_IT(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.BinaryAutoClr = RTC_ALARMSUBSECONDBIN_AUTOCLR_NO;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDBINMASK_NONE;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 214;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 107;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 64;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 214000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 10700;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
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
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
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
void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void PWM_SET_FREQ_DUTY_DEADTIME(uint16_t freq_khz, uint8_t dead_ref_ns)
{
    if ((freq_hz_old != freq_khz) || (dead_ref_old != dead_ref_ns))
    {
        uint32_t tim_clk = 64000000UL;  // 타이머 클럭 (64MHz)
        uint32_t period = tim_clk / (freq_khz*1000);
        uint32_t pulse = period / 2;  // 듀티비 50%

        uint32_t period2 = period * ratio;
        uint32_t pulse2 = (period2 / 2) + (period / sensing_time);  // 센싱용 타이밍

        // --- DeadTime 계산 및 설정 ---
        uint32_t t_dts_ns = 1000000000UL / tim_clk;  // 15.625ns
        uint8_t dtg_val = compute_deadtime_dtg(dead_ref_ns, t_dts_ns);

//        // --- PWM 출력 중단 없이 레지스터 업데이트 ---
//        __HAL_TIM_DISABLE(&htim1);
//        __HAL_TIM_DISABLE(&htim2);

        htim1.Instance->ARR = period - 1;
        htim1.Instance->CCR1 = pulse;

        // Dead Time 갱신
//        MODIFY_REG(htim1.Instance->BDTR, TIM_BDTR_DTG, dtg_val);
//        SET_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);  // PWM enable

        htim2.Instance->ARR = period2 - 1;
        htim2.Instance->CCR1 = pulse2;

        // UG 이벤트로 레지스터 갱신
        SET_BIT(htim1.Instance->EGR, TIM_EGR_UG);
        SET_BIT(htim2.Instance->EGR, TIM_EGR_UG);

//        __HAL_TIM_ENABLE(&htim1);
//        __HAL_TIM_ENABLE(&htim2);

        freq_hz_old = freq_khz;
        dead_ref_old = dead_ref_ns;
    }
}

// 1차 IIR Low-Pass 필터 함수
float low_pass_filter(float input) {
    float output = alpha * input + (1.0 - alpha) * prev_output;
    prev_output = output;
    return output;
}

// 1차 IIR Low-Pass 필터 함수
double low_pass_filter_I(double input) {
    double output = alpha_I * input + (1.0 - alpha_I) * prev_output_I;
    prev_output = output;
    return output;
}

//void HAL_TIM_OC_DelayElpasedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM2)
//	{
//		if(HAL_ADC_GetState(&hadc4) == HAL_ADC_STATE_READY)
//		{
//		Read_ADCChannels_Polling();
//		ubDmaTransferStatus = 1;
//		}
//		else
//			HAL_ADC_Stop(&hadc4);
//	}
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);  // ADC 시점 확인용 토글
//
//}
//void Read_ADCChannels_Polling(void)
//{
//    ADC_ChannelConfTypeDef sConfig = {0};
//
//    // === 1. 채널 1 (예: IN8) ===
//    sConfig.Channel = ADC_CHANNEL_3;
//    sConfig.Rank = ADC_REGULAR_RANK_1;
//    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;  // 적절한 샘플링 시간
//    HAL_ADC_ConfigChannel(&hadc4, &sConfig);
//
//    HAL_ADC_Start(&hadc4);
//    HAL_ADC_PollForConversion(&hadc4, 10);
//    adc_result[0] = HAL_ADC_GetValue(&hadc4);
//    HAL_ADC_Stop(&hadc4);
//
//    // === 2. 채널 2 (예: IN9) ===
//    sConfig.Channel = ADC_CHANNEL_8;
//    HAL_ADC_ConfigChannel(&hadc4, &sConfig);
//
//    HAL_ADC_Start(&hadc4);
//    HAL_ADC_PollForConversion(&hadc4, 10);
//    adc_result[1] = HAL_ADC_GetValue(&hadc4);
//    HAL_ADC_Stop(&hadc4);
//
//    // === 3. 채널 3 (예: IN10) ===
//    sConfig.Channel = ADC_CHANNEL_10;
//    HAL_ADC_ConfigChannel(&hadc4, &sConfig);
//
//    HAL_ADC_Start(&hadc4);
//    HAL_ADC_PollForConversion(&hadc4, 10);
//    adc_result[2] = HAL_ADC_GetValue(&hadc4);
//    HAL_ADC_Stop(&hadc4);
//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC4)
    {

            ubDmaTransferStatus = 1;  // 데이터 처리 조건 만족
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);  // ADC 시점 확인용 토글
    }
}


uint8_t compute_deadtime_dtg(uint32_t deadtime_ns, uint32_t t_dts_ns)
{
    uint32_t dtg = deadtime_ns / t_dts_ns;

    if (dtg <= 127)
        return (uint8_t)(dtg);  // 0b0xxxxxxx
    else if (dtg <= (64 + 31 * 2))
        return (uint8_t)(0x80 | ((dtg - 64) / 2));  // 0b10xxxxx
    else if (dtg <= (32 + 31) * 8)
        return (uint8_t)(0xC0 | ((dtg / 8) - 32));  // 0b110xxxxx
    else if (dtg <= (32 + 31) * 16)
        return (uint8_t)(0xE0 | ((dtg / 16) - 32));  // 0b111xxxxx
    else
        return 0xFF;  // 최대값
}

/**
 * @brief DeadTime(ns)를 DTG 값으로 변환
 */
uint8_t Compute_DTG_From_ns(uint32_t deadtime_ns)
{
    float clocks = deadtime_ns / T_DTS_NS;

    if (clocks <= 127.0f)
        return (uint8_t)(clocks + 0.5f); // 반올림
    else
        return 127; // 최대값 제한
}

/**
 * @brief TIM1의 DeadTime을 실시간 변경
 */
void Update_TIM1_DeadTime(uint32_t deadtime_ns)
{
    uint8_t dtg = Compute_DTG_From_ns(deadtime_ns);

    // BDTR 레지스터 직접 접근하여 DeadTime 변경
    TIM1->BDTR &= ~(0xFF);    // DTG 클리어
    TIM1->BDTR |= dtg;        // 새 DTG 값 적용
}


// 예: DeadTime을 1us → 550ns까지 줄여가며 테스트
void Sweep_DeadTime_Down(void)
{
    for (dead_ref = dead_ref_start; dead_ref >= dead_ref_end; dead_ref -= 25)  // 단위: ns
    {
        Update_TIM1_DeadTime(dead_ref);
        HAL_Delay(500); // 100ms 대기 (오실로스코프 확인용)
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
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
