/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "DOT_MATRIX.h"
#include <arm_math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MATRIX_DISPLAY_UNIT1 0
#define FFT_LENGTH 64
#define MATRIX_X 32
#define MATRIX_Y 8
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint16_t raw_microphone_input[FFT_LENGTH];
bool status = false;
arm_rfft_fast_instance_f32 fft_instance;
arm_status status_fft;

float32_t input_fft[FFT_LENGTH];
float32_t output_fft[FFT_LENGTH];
float32_t output_fft_mag[FFT_LENGTH / 2];

float32_t window_coff[FFT_LENGTH];

char data_avgs[MATRIX_X];

int yvalue;
int peaks[MATRIX_X] = {0};
int displaycolumn, displayvalue;

int MY_ARRAY[] = {0, 128, 192, 224, 240, 248, 252, 254, 255};
bool adc_dma_flag = false;

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 8000 Hz

* 0 Hz - 1200 Hz
  gain = 0
  desired attenuation = -45 dB
  actual attenuation = -47.35344673133169 dB

* 1300 Hz - 1400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.129857707910803 dB

* 1500 Hz - 4000 Hz
  gain = 0
  desired attenuation = -45 dB
  actual attenuation = -47.35344673133169 dB

*/

#define FILTER_TAP_NUM 109

static float filter_taps[FILTER_TAP_NUM] = {
  0.0009546973999630191,
  0.003168463762582948,
  -0.00014550904213110895,
  -0.0017745426915630291,
  -0.002664106543097514,
  -0.00036921593047873914,
  0.003018132083844269,
  0.003850383032839122,
  0.0003728120301078761,
  -0.004458838069926212,
  -0.005326982598694947,
  -0.0002608863102040281,
  0.006246838898338785,
  0.007030902140359828,
  0.0000012699594340840336,
  -0.008400842352906581,
  -0.008926052876460491,
  0.0004439041844444118,
  0.010910045918458835,
  0.010959999083078232,
  -0.001107982528777111,
  -0.013743212798719732,
  -0.013065879748489363,
  0.00201720111617057,
  0.01684902548044834,
  0.015164746414713071,
  -0.0031873034645785326,
  -0.020153081865200102,
  -0.017170818908960734,
  0.004616256862906045,
  0.023564130118216217,
  0.018999989034372274,
  -0.006289433783701813,
  -0.02697561269817318,
  -0.020566141122448665,
  0.008171691693646257,
  0.03026645273348758,
  0.02179714590281802,
  -0.010211816983530925,
  -0.03331727937348779,
  -0.02263070427420169,
  0.012345268053275658,
  0.03600596240989731,
  0.023026005012736047,
  -0.014490320751035603,
  -0.03822778476732951,
  -0.022961285857759466,
  0.016563922334333316,
  0.03988647733784863,
  0.022439122204093522,
  -0.01847596424486482,
  -0.04091237110489502,
  -0.021484839554491906,
  0.020143026983816276,
  0.04125859582635884,
  0.020143026983816276,
  -0.021484839554491906,
  -0.04091237110489502,
  -0.01847596424486482,
  0.022439122204093522,
  0.03988647733784863,
  0.016563922334333316,
  -0.022961285857759466,
  -0.03822778476732951,
  -0.014490320751035603,
  0.023026005012736047,
  0.03600596240989731,
  0.012345268053275658,
  -0.02263070427420169,
  -0.03331727937348779,
  -0.010211816983530925,
  0.02179714590281802,
  0.03026645273348758,
  0.008171691693646257,
  -0.020566141122448665,
  -0.02697561269817318,
  -0.006289433783701813,
  0.018999989034372274,
  0.023564130118216217,
  0.004616256862906045,
  -0.017170818908960734,
  -0.020153081865200102,
  -0.0031873034645785326,
  0.015164746414713071,
  0.01684902548044834,
  0.00201720111617057,
  -0.013065879748489363,
  -0.013743212798719732,
  -0.001107982528777111,
  0.010959999083078232,
  0.010910045918458835,
  0.0004439041844444118,
  -0.008926052876460491,
  -0.008400842352906581,
  0.0000012699594340840336,
  0.007030902140359828,
  0.006246838898338785,
  -0.0002608863102040281,
  -0.005326982598694947,
  -0.004458838069926212,
  0.0003728120301078761,
  0.003850383032839122,
  0.003018132083844269,
  -0.00036921593047873914,
  -0.002664106543097514,
  -0.0017745426915630291,
  -0.00014550904213110895,
  0.003168463762582948,
  0.0009546973999630191
};


float32_t firState[FILTER_TAP_NUM + FFT_LENGTH - 1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long constrain(long x, long a, long b)
{
  if (x < a)
    return a;
  if (x > b)
    return b;
  return x;
}

uint8_t reverse_bits(uint8_t b)
{
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}
#define SAMPLE_RATE 8000 // Hz
#define FREQ 440         // A4 note

float current_phase = 0.0f; // Global or static to retain between calls

void generate_test_tone()
{
  float phase_increment = 2.0f * PI * FREQ / SAMPLE_RATE;

  for (int i = 0; i < FFT_LENGTH; i++)
  {
    raw_microphone_input[i] = (uint16_t)(2048 + 2047 * sinf(current_phase));
    current_phase += phase_increment;

    // Keep phase in range to avoid float overflow
    if (current_phase >= 2.0f * PI)
      current_phase -= 2.0f * PI;
  }

  adc_dma_flag = true; // simulate DMA completion
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DOT_MATRIX_Init_TMR(&hspi1, &htim2);
  MATRIX_CLEAR(MATRIX_DISPLAY_UNIT1);

  HAL_TIM_Base_Start(&htim3); // Starting TIM3 Trig source
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_DMA_Init(&hdma_adc1);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)raw_microphone_input, FFT_LENGTH) != HAL_OK)
  {
    Error_Handler(); // Could help debug DMA misconfig
  }

  status_fft = arm_rfft_fast_init_f32(&fft_instance, FFT_LENGTH);

  arm_fir_instance_f32 fir;
  arm_fir_init_f32(&fir, FILTER_TAP_NUM, filter_taps, firState, FFT_LENGTH);

  arm_hamming_f32(window_coff, FFT_LENGTH); // Generate Window coefficients
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Sampling at
    if (adc_dma_flag)
    {
      adc_dma_flag = false;

      // Sample microphone input
      for (int i = 0; i < FFT_LENGTH; i++)
      {
        input_fft[i] = (float32_t)raw_microphone_input[i] - 2048.0f; // Center around 0
        // input_fft[i] = arm_sin_f32(2.0f * PI * 8 * i / FFT_LENGTH);
      }

      // Applying High-pass filter
      arm_fir_f32(&fir, input_fft, input_fft, FFT_LENGTH);
      // // // arm_iir_lattice_f32(&fir, input_fft, input_fft, FFT_LENGTH);

      // for (int i = 0; i < FFT_LENGTH; i++)
      // {
      //   input_fft[i] *= window_coff[i]; // Apply Window
      // }

      arm_rfft_fast_f32(&fft_instance, input_fft, output_fft, 0); // Fast FFT

      arm_cmplx_mag_f32(output_fft, output_fft_mag, FFT_LENGTH / 2); // Get mag values

      // for(int i = 0; i < 10; i++)
      // {
      //   output_fft_mag[i] *= 0.03f; // supprse them
      // }

      int step = (FFT_LENGTH / 2) / MATRIX_X;
      int c = 0;
      for (int i = 0; i < (FFT_LENGTH / 2); i += step)
      {
        data_avgs[c] = 0;
        for (int k = 0; k < step; k++)
        {
          data_avgs[c] = data_avgs[c] + output_fft_mag[i + k];
        }
        data_avgs[c] = data_avgs[c] / step;
        c++;
      }

      for (int i = 0; i < MATRIX_X; i++)
      {
        data_avgs[i] = constrain(data_avgs[i], 0, 32);        // set max & min values for buckets
        data_avgs[i] = map(data_avgs[i], 0, 32, 0, MATRIX_Y); // remap averaged values to yres
        yvalue = data_avgs[i];

        peaks[i] = peaks[i] - 1; // decay by one light
        if (yvalue > peaks[i])
          peaks[i] = yvalue;
        yvalue = peaks[i];
        displayvalue = MY_ARRAY[yvalue];
        displaycolumn = 31 - i;
        SET_COLUMN(MATRIX_DISPLAY_UNIT1, displaycolumn, reverse_bits(displayvalue)); // for left to right
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  adc_dma_flag = true;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  adc_dma_flag = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  MATRIX_TMR_OVF_ISR(htim);
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
