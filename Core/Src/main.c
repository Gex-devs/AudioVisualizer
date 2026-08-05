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
#include <arm_math.h>
#include <arduino_utils.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  int bin_low;
  int bin_high;
} LogBinMap;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MATRIX_DISPLAY_UNIT1 0
#define FFT_LENGTH 64
#define BUFFER_SIZE (FFT_LENGTH) // 24bits 32 data frame
#define MATRIX_X 32
#define MATRIX_Y 8

#define SAMPLE_RATE 16000.0f
#define MIN_FREQ 2000.0f
#define MAX_FREQ 7000.0f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart6;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
bool i2s_dma_full_flag = false;
bool i2s_dma_half_flag = false;

arm_status status_fft;
arm_fir_instance_f32 fir;
arm_rfft_fast_instance_f32 fft_instance;

float32_t input_fft[FFT_LENGTH];
float32_t output_fft[FFT_LENGTH];
float32_t output_fft_mag[FFT_LENGTH / 2];
float32_t output_bins[MATRIX_X];
uint32_t i2s_buffer[BUFFER_SIZE];

LogBinMap lut[MATRIX_X];

float32_t window_coff[FFT_LENGTH];

float32_t data_avgs[MATRIX_X];

int yvalue;
int peaks[MATRIX_X] = {0};
int displaycolumn, displayvalue;

int MY_ARRAY[] = {0, 128, 192, 224, 240, 248, 252, 254, 255};

void build_log_bin_lut(LogBinMap *lut, int NUM_BINS)
{
  int fft_bins = FFT_LENGTH / 2;
  float32_t freq_resolution = SAMPLE_RATE / (float32_t)FFT_LENGTH;

  float32_t log_min = log2f(MIN_FREQ);
  float32_t log_max = log2f(MAX_FREQ);
  float32_t log_range = log_max - log_min;

  for (int i = 0; i < NUM_BINS; i++)
  {
    float32_t freq_low = powf(2.0f, log_min + (float32_t)i / NUM_BINS * log_range);
    float32_t freq_high = powf(2.0f, log_min + (float32_t)(i + 1) / NUM_BINS * log_range);

    int bin_low = (int)floorf(freq_low / freq_resolution);
    int bin_high = (int)ceilf(freq_high / freq_resolution);

    bin_low = bin_low < 0 ? 0 : bin_low;
    bin_high = bin_high >= fft_bins ? fft_bins - 1 : bin_high;

    if (bin_low > bin_high)
      bin_high = bin_low;

    lut[i].bin_low = bin_low;
    lut[i].bin_high = bin_high;
  }
}

void rebin_log_audio(float *fft_mag, float *rebin, const LogBinMap *lut, int NUM_BINS)
{
  for (int i = 0; i < NUM_BINS; i++)
  {
    int bin_low = lut[i].bin_low;
    int bin_high = lut[i].bin_high;

    float32_t sum = 0.0f;
    int count = bin_high - bin_low + 1;

    for (int j = bin_low; j <= bin_high; j++)
      sum += fft_mag[j];

    rebin[i] = sum / (float32_t)count;
  }
}
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

* 0 Hz - 3000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.44566068037808 dB

* 4000 Hz - 8000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.5163656958569183 dB

*/

#define FILTER_TAP_NUM 19

static float32_t filter_taps[FILTER_TAP_NUM] = {
    -0.01286737409931033,
    -0.011582917042481518,
    0.0709435597229363,
    -0.05850997474212666,
    -0.040910558942368495,
    0.03894710089289564,
    0.0947634922313574,
    -0.04258608472470373,
    -0.31412123784627477,
    0.543381234624982,
    -0.31412123784627477,
    -0.04258608472470373,
    0.0947634922313574,
    0.03894710089289564,
    -0.040910558942368495,
    -0.05850997474212666,
    0.0709435597229363,
    -0.011582917042481518,
    -0.01286737409931033};

float32_t firState[FILTER_TAP_NUM + FFT_LENGTH - 1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void audio_visualizer_update(float32_t *fft_magnitudes);
void process_audio_data(float32_t *output_fft_mag);
void test_visualizer_sine(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void test_visualizer_sine(void)
{
  static float32_t sweep_freq = 200.0f;
  static float32_t sweep_dir = 1.0f;

  float32_t phase_inc = 2.0f * M_PI * sweep_freq / 16000.0f; // 16kHz here
  float32_t ph = 0.0f;

  for (int i = 0; i < FFT_LENGTH; i++)
  {
    input_fft[i] = 500000.0f * sinf(ph);
    ph += phase_inc;
    if (ph >= 2.0f * M_PI)
      ph -= 2.0f * M_PI;
  }

  float32_t mean = 0.0f;
  for (int i = 0; i < FFT_LENGTH; i++)
    mean += input_fft[i];
  mean /= FFT_LENGTH;
  for (int i = 0; i < FFT_LENGTH; i++)
    input_fft[i] -= mean;
  for (int i = 0; i < FFT_LENGTH; i++)
    input_fft[i] *= window_coff[i];

  arm_rfft_fast_f32(&fft_instance, input_fft, output_fft, 0);
  arm_cmplx_mag_f32(output_fft, output_fft_mag, FFT_LENGTH / 2);
  audio_visualizer_update(output_fft_mag);

  // Sweep 200Hz → 7500Hz (stay safely below 8000Hz Nyquist limit)
  sweep_freq += sweep_dir * 200.0f;
  if (sweep_freq >= 7500.0f)
    sweep_dir = -1.0f;
  if (sweep_freq <= 200.0f)
    sweep_dir = 1.0f;

  HAL_Delay(80);
}

/**
 * @brief input i2s buffer, reconstuct and return magnitude, implemented for inmp411
 * @return float32_t
 */
void process_audio_data(float32_t *output_fft_mag)
{
  for (int i = 0; i < FFT_LENGTH - 1; i++)
  {
    int32_t left = i2s_buffer[i];
    int32_t right = i2s_buffer[i + 1];

    // squre the audio channels
    right = right * right; // R^2
    left = left * left;    // L^2

    // root mean square https://en.wikipedia.org/wiki/Root_mean_square
    double sample = sqrt((left + right) / 2);
    // int32_t sample = (left + right) / 2; // average

    input_fft[i] = (float32_t)sample; // Store in FFT input buffer as float
  }

  // Applying High-pass filter
  // arm_fir_f32(&fir, input_fft, input_fft, FFT_LENGTH);

  float mean = 0;
  for (int i = 0; i < FFT_LENGTH; i++)
  {
    mean += input_fft[i]; // Calculate mean for DC offset removal
  }
  mean /= FFT_LENGTH;

  for (int i = 0; i < FFT_LENGTH; i++)
  {
    input_fft[i] -= mean; // DC offset removal
  }

  for (int i = 0; i < FFT_LENGTH; i++)
  {
    input_fft[i] *= window_coff[i]; // Apply Window
  }

  arm_rfft_fast_f32(&fft_instance, input_fft, output_fft, 0); // Fast FFT
  // arm_fir_f32(&fir, output_fft, output_fft, FFT_LENGTH);         // apply filter
  arm_cmplx_mag_f32(output_fft, output_fft_mag, FFT_LENGTH / 2); // Get mag values
}

// TODO: Optimize this, it's hard to read.
void audio_visualizer_update(float32_t *output_fft_mag)
{
  // Find max magnitude this frame for self-normalization
  float32_t max_mag = 0.0f;
  arm_max_f32(output_fft_mag, FFT_LENGTH / 2, &max_mag, NULL);

  // Noise floor gate — skip update if signal is too quiet
  if (max_mag < 5000.0f)
  {
    for (int i = 0; i < MATRIX_X; i++)
    {
      peaks[i] -= 1;
      if (peaks[i] < 0)
        peaks[i] = 0;
      SET_COLUMN(MATRIX_DISPLAY_UNIT1, 31 - i, reverse_bits(MY_ARRAY[peaks[i]]));
    }
    return;
  }

  for (int i = 0; i < MATRIX_X; i++)
  {
    // Normalize 0.0–1.0 then scale to MATRIX_Y
    float32_t normalized = output_fft_mag[i] / max_mag;
    int yvalue = (int)(normalized * MATRIX_Y);
    if (yvalue > MATRIX_Y)
      yvalue = MATRIX_Y; // clamp

    // Peak decay — clamped to 0
    peaks[i] -= 1;
    if (peaks[i] < 0)
      peaks[i] = 0;
    if (yvalue > peaks[i])
      peaks[i] = yvalue;

    displayvalue = MY_ARRAY[peaks[i]];
    displaycolumn = 31 - i;
    SET_COLUMN(MATRIX_DISPLAY_UNIT1, displaycolumn, reverse_bits(displayvalue));
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
  MX_I2S3_Init();
  MX_SPI2_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  DOT_MATRIX_Init_TMR(&hspi2, &htim2);
  MATRIX_CLEAR(MATRIX_DISPLAY_UNIT1);

  // Create fast fft instance
  status_fft = arm_rfft_fast_init_f32(&fft_instance, FFT_LENGTH);

  arm_fir_init_f32(&fir, FILTER_TAP_NUM, filter_taps, firState, FFT_LENGTH);

  arm_hamming_f32(window_coff, FFT_LENGTH); // Generate Window coefficients

  // build lut for log rebinning
  build_log_bin_lut(lut, MATRIX_X);

  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)i2s_buffer, BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Sampling at
    // if (i2s_dma_half_flag)
    // {
    //   float32_t out_mag[FFT_LENGTH / 2];
    //   float32_t output_bins[FFT_LENGTH / 2];

    //   process_audio_data(&i2s_buffer[0], &out_mag[0]); // Process second half of the buffer
    //   rebin_log_audio(out_mag, output_bins, lut, MATRIX_X); // Optional: re-bin FFT magnitudes into logarithmic frequency bins matching the display columns
    //   audio_visualizer_update(output_bins);
    //   i2s_dma_half_flag = false;
    //   // Sample microphone input
    // }

    if (i2s_dma_full_flag)
    {
      float32_t out_mag[FFT_LENGTH / 2];
      // float32_t output_bins[FFT_LENGTH / 2];

      process_audio_data(out_mag);                      // Process second half of the buffer
      rebin_log_audio(out_mag, out_mag, lut, MATRIX_X); // Optional: re-bin FFT magnitudes into logarithmic frequency bins matching the display columns
      audio_visualizer_update(out_mag);                 // Update display based on FFT results
      i2s_dma_full_flag = false;
      // Sample microphone input
    }
    // test_visualizer_sine(); // Call this instead of process_audio_data() to test with a synthetic sine wave signal. Comment out the above if blocks when using this.
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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
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
  htim2.Init.Period = 4294967295;
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
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Half buffer ready (0..63)
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  i2s_dma_half_flag = true;
}

// Full buffer ready (64..127)
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  i2s_dma_full_flag = true;
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
