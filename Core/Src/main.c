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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_capture.h"
#include "measurement_tasks.h"
#include "tjc_usart_hmi.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_ALIAS_STATE_NORMAL = 0,
  APP_ALIAS_STATE_PROBE_DISCARD,
  APP_ALIAS_STATE_PROBE_CAPTURE,
  APP_ALIAS_STATE_NORMAL_DISCARD
} APP_AliasState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
ADC_CaptureStats g_adc_capture_stats;
MeasurementParameterResult g_measurement_parameters;
MeasurementComponentResult g_measurement_components;
volatile uint32_t g_adc_capture_sequence;
volatile uint32_t g_spectrum_measurement_sequence;
volatile uint32_t g_app_error_code;
#define FREQUENCY_FILTER_LENGTH 5U
#define FREQUENCY_STARTUP_CONFIRM_COUNT 3U
#define FREQUENCY_STEP_CONFIRM_COUNT 4U
#define FREQUENCY_STEP_THRESHOLD_RATIO 0.25F
#define FREQUENCY_CANDIDATE_TOLERANCE_RATIO 0.05F
#define FREQUENCY_CANDIDATE_TOLERANCE_HZ 500.0F
#define LCD_PAGE_INTRO 0U
#define LCD_PAGE_MEASUREMENT 1U
#define LCD_PAGE_SPECTRUM 2U
#define LCD_PAGE_UNKNOWN 0xFFU
#define LCD_TIME_WAVEFORM_COMPONENT_ID 12U
#define LCD_SPECTRUM_COMPONENT_ID 8U
#define LCD_TIME_STATUS_COMPONENT "t_test1"
#define LCD_SPECTRUM_STATUS_COMPONENT "t_test2"
#define LCD_SPECTRUM_CALIBRATION_MODE 0U
#define LCD_PAGE_QUERY_INTERVAL_MS 250U
#define LCD_PAGE_QUERY_TIMEOUT_MS 1000U
#define LCD_PAGE_CONFIRM_MAX_AGE_MS 350U
#define LCD_COMM_OK_MAX_AGE_MS 1500U
#define LCD_COLOR_OK 2016U
#define LCD_COLOR_NG 63488U
#define LCD_TEXT_REFRESH_INTERVAL_MS 500U
#define LCD_WAVEFORM_REFRESH_INTERVAL_MS 600U
#define LCD_SPECTRUM_REFRESH_INTERVAL_MS 600U
#define LCD_TRANSPARENT_TIMEOUT_MS 1200U
#define MEASUREMENT_REFRESH_INTERVAL_MS 100U
#define APP_IWDG_START_KEY 0xCCCCU
#define APP_IWDG_WRITE_ACCESS_KEY 0x5555U
#define APP_IWDG_RELOAD_KEY 0xAAAAU
#define APP_IWDG_PRESCALER_DIV64 4U
#define APP_IWDG_RELOAD_VALUE 1499U
#define APP_IWDG_REGISTER_TIMEOUT_MS 100U
#define ALIAS_PROBE_INTERVAL_MS 500U
#define ALIAS_STATE_TIMEOUT_MS 200U
#define ALIAS_CONFIRM_COUNT 2U
#define ALIAS_CLEAR_COUNT 2U
static float g_frequency_history[FREQUENCY_FILTER_LENGTH];
static uint32_t g_frequency_history_count;
static uint32_t g_frequency_history_index;
static float g_frequency_candidate;
static uint32_t g_frequency_candidate_count;
static MeasurementPeriodCount g_display_period_count =
    MEASUREMENT_SHOW_ONE_PERIOD;
static uint16_t g_analysis_snapshot[ADC_CAPTURE_ANALYSIS_SAMPLE_COUNT];
static MeasurementWaveformResult g_measurement_waveform;
static MeasurementSpectrumResult g_measurement_spectrum;
static uint8_t g_measurement_valid;
static uint8_t g_waveform_valid = 1U;
static uint8_t g_lcd_current_page = LCD_PAGE_INTRO;
static uint8_t g_lcd_page_query_pending;
static uint8_t g_lcd_text_refresh_requested = 1U;
static uint8_t g_lcd_waveform_refresh_requested = 1U;
static uint8_t g_lcd_spectrum_refresh_requested = 1U;
static uint32_t g_lcd_page_query_tick;
static uint32_t g_lcd_page_confirmed_tick;
static uint32_t g_lcd_text_refresh_tick;
static uint32_t g_lcd_waveform_refresh_tick;
static uint32_t g_lcd_spectrum_refresh_tick;
static uint32_t g_measurement_refresh_tick;
MeasurementRawLowBandSignature g_alias_normal_signature;
MeasurementRawLowBandSignature g_alias_probe_signature;
MeasurementAliasComparisonResult g_alias_comparison;
MeasurementParameterResult g_recovered_parameters;
volatile uint32_t g_alias_detected;
volatile uint32_t g_alias_probe_sequence;
volatile uint32_t g_alias_switch_error_count;
volatile uint32_t g_recovery_valid;
volatile uint32_t g_recovery_sequence;
static APP_AliasState g_alias_state = APP_ALIAS_STATE_NORMAL;
static uint8_t g_alias_positive_count;
static uint8_t g_alias_clear_count;
static uint32_t g_alias_probe_tick;
static uint32_t g_alias_state_tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LCD_ADC_Init(void);
static void LCD_ADC_Update(const MeasurementParameterResult *parameters);
static HAL_StatusTypeDef LCD_ADC_UpdateWaveform(void);
static HAL_StatusTypeDef LCD_ADC_UpdateSpectrum(void);
static void LCD_ADC_ReverseWaveformPoints(MeasurementWaveformResult *waveform);
static void LCD_ADC_UpdateStatus(const char *component);
static void LCD_ADC_ShowError(const char *message);
static float APP_FilterFrequency(float frequency_hz);
static void APP_ProcessLcdEvents(void);
static void APP_RequestLcdPage(void);
static void APP_FillMeasurementInput(MeasurementInputFrame *input);
static void APP_FillMeasurementInputForSamples(
    MeasurementInputFrame *input, const uint16_t *samples,
    uint32_t sample_rate_hz);
static void APP_UpdateAliasDecision(uint8_t alias_detected);
static void APP_UpdateRecoveryResult(void);
static void APP_HandleAliasProbeFrame(void);
static void APP_HandleRestoredNormalFrame(void);
static HAL_StatusTypeDef APP_IWDG_Init(void);
static void APP_IWDG_Refresh(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef APP_IWDG_Init(void)
{
  uint32_t tick_start;

  /* Keep breakpoints usable: this bit only stops IWDG while the Cortex-M4 is
   * halted by a debugger and has no effect during normal execution. */
  __HAL_DBGMCU_FREEZE_IWDG();

  /* IWDG is clocked from its independent LSI oscillator. With the nominal
   * 32 kHz LSI, prescaler 64 and reload 1499 give a 3.0 second timeout. */
  IWDG->KR = APP_IWDG_START_KEY;
  IWDG->KR = APP_IWDG_WRITE_ACCESS_KEY;
  IWDG->PR = APP_IWDG_PRESCALER_DIV64;
  IWDG->RLR = APP_IWDG_RELOAD_VALUE;

  tick_start = HAL_GetTick();
  while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U)
  {
    if ((HAL_GetTick() - tick_start) > APP_IWDG_REGISTER_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  IWDG->KR = APP_IWDG_RELOAD_KEY;
  return HAL_OK;
}

static void APP_IWDG_Refresh(void)
{
  IWDG->KR = APP_IWDG_RELOAD_KEY;
}

static float APP_FilterFrequency(float frequency_hz)
{
  float sorted[FREQUENCY_FILTER_LENGTH];
  float reference;
  uint32_t count;
  uint32_t index;

  if (frequency_hz <= 0.0F)
  {
    g_frequency_history_count = 0U;
    g_frequency_history_index = 0U;
    g_frequency_candidate = 0.0F;
    g_frequency_candidate_count = 0U;
    return 0.0F;
  }

  count = g_frequency_history_count;
  for (index = 0U; index < count; ++index)
  {
    uint32_t insert = index;
    sorted[index] = g_frequency_history[index];
    while ((insert > 0U) && (sorted[insert] < sorted[insert - 1U]))
    {
      float temporary = sorted[insert - 1U];
      sorted[insert - 1U] = sorted[insert];
      sorted[insert] = temporary;
      insert--;
    }
  }

  if (count == 0U)
  {
    float difference;
    float scale;
    float tolerance;

    if (g_frequency_candidate_count == 0U)
    {
      g_frequency_candidate = frequency_hz;
      g_frequency_candidate_count = 1U;
    }
    else
    {
      difference = frequency_hz - g_frequency_candidate;
      if (difference < 0.0F)
      {
        difference = -difference;
      }
      scale = (frequency_hz > g_frequency_candidate) ?
          frequency_hz : g_frequency_candidate;
      tolerance = scale * FREQUENCY_CANDIDATE_TOLERANCE_RATIO;
      if (tolerance < FREQUENCY_CANDIDATE_TOLERANCE_HZ)
      {
        tolerance = FREQUENCY_CANDIDATE_TOLERANCE_HZ;
      }
      if (difference <= tolerance)
      {
        g_frequency_candidate_count++;
      }
      else
      {
        g_frequency_candidate = frequency_hz;
        g_frequency_candidate_count = 1U;
      }
    }
    if (g_frequency_candidate_count < FREQUENCY_STARTUP_CONFIRM_COUNT)
    {
      return 0.0F;
    }
    /* The first confirmed frequency must be rendered immediately. Do not
     * make the official response time depend on the periodic LCD cadence. */
    g_lcd_text_refresh_requested = 1U;
    g_lcd_waveform_refresh_requested = 1U;
    g_lcd_spectrum_refresh_requested = 1U;
    g_frequency_candidate = 0.0F;
    g_frequency_candidate_count = 0U;
  }
  else
  {
    float difference = frequency_hz - sorted[count / 2U];
    float scale;
    float tolerance;

    reference = sorted[count / 2U];
    if (difference < 0.0F)
    {
      difference = -difference;
    }
    if (difference > reference * FREQUENCY_STEP_THRESHOLD_RATIO)
    {
      if (g_frequency_candidate_count == 0U)
      {
        g_frequency_candidate = frequency_hz;
        g_frequency_candidate_count = 1U;
      }
      else
      {
        float candidate_difference = frequency_hz - g_frequency_candidate;

        if (candidate_difference < 0.0F)
        {
          candidate_difference = -candidate_difference;
        }
        scale = (frequency_hz > g_frequency_candidate) ?
            frequency_hz : g_frequency_candidate;
        tolerance = scale * FREQUENCY_CANDIDATE_TOLERANCE_RATIO;
        if (tolerance < FREQUENCY_CANDIDATE_TOLERANCE_HZ)
        {
          tolerance = FREQUENCY_CANDIDATE_TOLERANCE_HZ;
        }
        if (candidate_difference <= tolerance)
        {
          g_frequency_candidate_count++;
        }
        else
        {
          g_frequency_candidate = frequency_hz;
          g_frequency_candidate_count = 1U;
        }
      }
      /* A single subharmonic decision (for example 100 kHz -> 10 kHz) must
       * not alter either the displayed frequency or the waveform period. A
       * real generator step is accepted after four matching measurements. */
      if (g_frequency_candidate_count < FREQUENCY_STEP_CONFIRM_COUNT)
      {
        return reference;
      }
      /* A real generator step has now passed the stability check. Request a
       * new frame immediately so an older periodic frame cannot consume the
       * remaining two-second response-time margin. */
      g_lcd_text_refresh_requested = 1U;
      g_lcd_waveform_refresh_requested = 1U;
      g_lcd_spectrum_refresh_requested = 1U;
      g_frequency_history_count = 0U;
      g_frequency_history_index = 0U;
      g_frequency_candidate = 0.0F;
      g_frequency_candidate_count = 0U;
    }
    else
    {
      g_frequency_candidate = 0.0F;
      g_frequency_candidate_count = 0U;
    }
  }

  g_frequency_history[g_frequency_history_index] = frequency_hz;
  g_frequency_history_index =
      (g_frequency_history_index + 1U) % FREQUENCY_FILTER_LENGTH;
  if (g_frequency_history_count < FREQUENCY_FILTER_LENGTH)
  {
    g_frequency_history_count++;
  }

  count = g_frequency_history_count;
  for (index = 0U; index < count; ++index)
  {
    uint32_t insert = index;
    sorted[index] = g_frequency_history[index];
    while ((insert > 0U) && (sorted[insert] < sorted[insert - 1U]))
    {
      float temporary = sorted[insert - 1U];
      sorted[insert - 1U] = sorted[insert];
      sorted[insert] = temporary;
      insert--;
    }
  }
  return sorted[count / 2U];
}

static void APP_ProcessLcdEvents(void)
{
  TJC_LCD_Event event;

  while (TJC_LCD_ReadEvent(&event))
  {
    if (event.type == TJC_LCD_EVENT_USER_COMMAND)
    {
      if (event.value == 0x01U)
      {
        g_display_period_count = MEASUREMENT_SHOW_ONE_PERIOD;
        g_lcd_text_refresh_requested = 1U;
        g_lcd_waveform_refresh_requested = 1U;
      }
      else if (event.value == 0x02U)
      {
        g_display_period_count = MEASUREMENT_SHOW_THREE_PERIODS;
        g_lcd_text_refresh_requested = 1U;
        g_lcd_waveform_refresh_requested = 1U;
      }
      else if (event.value == 0x03U)
      {
        g_lcd_spectrum_refresh_requested = 1U;
      }
      else if (event.value == 0x04U)
      {
        g_lcd_text_refresh_requested = 1U;
        g_lcd_waveform_refresh_requested = 1U;
      }
      else
      {
        continue;
      }
    }
    else if (event.type == TJC_LCD_EVENT_PAGE_ID)
    {
      uint8_t previous_page = g_lcd_current_page;

      g_lcd_current_page = event.value;
      g_lcd_page_query_pending = 0U;
      g_lcd_page_confirmed_tick = HAL_GetTick();
      if ((event.value == LCD_PAGE_MEASUREMENT) &&
          (previous_page != LCD_PAGE_MEASUREMENT))
      {
        g_lcd_text_refresh_requested = 1U;
        g_lcd_waveform_refresh_requested = 1U;
      }
      else if ((event.value == LCD_PAGE_SPECTRUM) &&
               (previous_page != LCD_PAGE_SPECTRUM))
      {
        g_lcd_spectrum_refresh_requested = 1U;
      }
    }
  }
}

static void APP_RequestLcdPage(void)
{
  uint32_t now = HAL_GetTick();

  if ((g_lcd_page_query_pending != 0U) &&
      ((now - g_lcd_page_query_tick) >= LCD_PAGE_QUERY_TIMEOUT_MS))
  {
    g_lcd_page_query_pending = 0U;
    g_lcd_current_page = LCD_PAGE_UNKNOWN;
  }

  if ((g_lcd_page_query_pending == 0U) &&
      ((now - g_lcd_page_query_tick) >= LCD_PAGE_QUERY_INTERVAL_MS) &&
      (TJC_LCD_SendCommand("sendme") == HAL_OK))
  {
    g_lcd_page_query_pending = 1U;
    g_lcd_page_query_tick = now;
  }
}

static void APP_FillMeasurementInput(MeasurementInputFrame *input)
{
  APP_FillMeasurementInputForSamples(
      input, g_analysis_snapshot, ADC_CAPTURE_NORMAL_SAMPLE_RATE_HZ);
}

static void APP_FillMeasurementInputForSamples(
    MeasurementInputFrame *input, const uint16_t *samples,
    uint32_t sample_rate_hz)
{
  input->samples = samples;
  input->sample_count = ADC_CAPTURE_ANALYSIS_SAMPLE_COUNT;
  input->sample_rate_hz = sample_rate_hz;
  input->adc_reference_mv = ADC_CAPTURE_VREF_MV;
  input->adc_full_scale_code = 4095U;
}

static void APP_UpdateAliasDecision(uint8_t alias_detected)
{
  uint32_t previous_decision = g_alias_detected;

  if (alias_detected != 0U)
  {
    g_alias_clear_count = 0U;
    if (g_alias_positive_count < ALIAS_CONFIRM_COUNT)
    {
      ++g_alias_positive_count;
    }
    if (g_alias_positive_count >= ALIAS_CONFIRM_COUNT)
    {
      g_alias_detected = 1U;
    }
  }
  else
  {
    g_alias_positive_count = 0U;
    if (g_alias_clear_count < ALIAS_CLEAR_COUNT)
    {
      ++g_alias_clear_count;
    }
    if (g_alias_clear_count >= ALIAS_CLEAR_COUNT)
    {
      g_alias_detected = 0U;
    }
  }

  if (previous_decision != g_alias_detected)
  {
    g_lcd_text_refresh_requested = 1U;
    g_lcd_waveform_refresh_requested = 1U;
    g_lcd_spectrum_refresh_requested = 1U;
  }
}

static void APP_UpdateRecoveryResult(void)
{
  MeasurementParameterResult recovered_parameters;
  uint32_t previous_valid = g_recovery_valid;
  uint32_t next_valid = 0U;

  if (g_alias_comparison.alias_detected != 0U)
  {
    if (Measurement_RecoverParameters(
            &g_alias_comparison,
            &recovered_parameters) == MEASUREMENT_STATUS_OK)
    {
      g_recovered_parameters = recovered_parameters;
      ++g_recovery_sequence;
      __DMB();
      next_valid = 1U;
    }
  }
  else if (g_alias_detected != 0U)
  {
    /* Keep the last good reconstruction during the clear hysteresis window.
     * A single clean comparison must not overwrite it with an incomplete
     * result before the alias decision is actually cleared. */
    next_valid = g_recovery_valid;
  }

  g_recovery_valid = next_valid;

  if (previous_valid != next_valid)
  {
    g_lcd_text_refresh_requested = 1U;
    g_lcd_waveform_refresh_requested = 1U;
    g_lcd_spectrum_refresh_requested = 1U;
  }
}

static void APP_HandleAliasProbeFrame(void)
{
  MeasurementInputFrame probe_input;
  MeasurementStatus signature_status;
  MeasurementStatus comparison_status = MEASUREMENT_STATUS_INVALID_ARGUMENT;

  APP_FillMeasurementInputForSamples(
      &probe_input, ADC_Capture_GetStableAnalysisSamples(),
      ADC_Capture_GetSampleRateHz());
  signature_status = Measurement_ExtractRawLowBandSignature(
      &probe_input, &g_alias_probe_signature);
  if (signature_status == MEASUREMENT_STATUS_OK)
  {
    comparison_status = Measurement_CompareRawLowBandSignatures(
        &g_alias_normal_signature, &g_alias_probe_signature,
        &g_alias_comparison);
  }

  if ((signature_status == MEASUREMENT_STATUS_OK) &&
      (comparison_status == MEASUREMENT_STATUS_OK))
  {
    ++g_alias_probe_sequence;
    APP_UpdateAliasDecision(g_alias_comparison.alias_detected);
    APP_UpdateRecoveryResult();
  }
  else
  {
    /* A detector failure must not silently validate a possibly aliased value. */
    g_app_error_code = 0x70U;
    APP_UpdateAliasDecision(1U);
    g_recovery_valid = 0U;
    g_lcd_text_refresh_requested = 1U;
    g_lcd_waveform_refresh_requested = 1U;
    g_lcd_spectrum_refresh_requested = 1U;
  }

  g_alias_state = APP_ALIAS_STATE_NORMAL_DISCARD;
  g_alias_state_tick = HAL_GetTick();
  if (ADC_Capture_SetRate(ADC_CAPTURE_RATE_NORMAL) != HAL_OK)
  {
    ++g_alias_switch_error_count;
    g_app_error_code = 0x61U;
    LCD_ADC_ShowError("ADC RATE ERROR");
    Error_Handler();
  }
  g_alias_state_tick = HAL_GetTick();
  g_alias_probe_tick = HAL_GetTick();
  if ((g_alias_detected == 0U) && (g_alias_positive_count != 0U))
  {
    /* Keep two independent positive decisions, but do not leave a moving
     * alias visible for another complete 500 ms interval.  The restored
     * normal frame is still discarded before this immediate confirmation. */
    g_alias_probe_tick -= ALIAS_PROBE_INTERVAL_MS;
  }
}

static void APP_HandleRestoredNormalFrame(void)
{
  ADC_Capture_ClearReady();
  g_alias_state = APP_ALIAS_STATE_NORMAL;
  g_alias_state_tick = HAL_GetTick();
}

static void LCD_ADC_Init(void)
{

  HAL_Delay(800U);
  (void)TJC_LCD_SendCommand("bkcmd=0");
  (void)TJC_LCD_SendCommand("recmod=0");
  (void)TJC_LCD_SendCommand("sleep=0");
  (void)TJC_LCD_SendCommand("thsp=0");
  (void)TJC_LCD_SendCommand("dim=100");
  /* G.HMI Program.s already selects page0. Do not issue another page command
   * here, because that can restart the screen's own startup sequence. */
  g_lcd_current_page = LCD_PAGE_UNKNOWN;
  g_lcd_page_confirmed_tick = 0U;
}

static void LCD_ADC_Update(const MeasurementParameterResult *parameters)
{
  const MeasurementParameterResult *display_parameters = parameters;
  char text[24];
  uint32_t vpp_mv;
  uint32_t rms_mv;
  uint32_t frequency_hz;

  if (g_alias_detected != 0U)
  {
    if (g_recovery_valid == 0U)
    {
      (void)TJC_LCD_SetText("t_vpp", "--");
      (void)TJC_LCD_SetText("t_vrms", "--");
      (void)TJC_LCD_SetText("t_freq", "--");
      LCD_ADC_UpdateStatus(LCD_TIME_STATUS_COMPONENT);
      return;
    }
    display_parameters = &g_recovered_parameters;
  }

  vpp_mv = (uint32_t)(display_parameters->vpp_mv + 0.5F);
  rms_mv = (uint32_t)(display_parameters->rms_mv + 0.5F);
  frequency_hz = (display_parameters->fundamental_hz > 0.0F) ?
      (uint32_t)(display_parameters->fundamental_hz + 0.5F) : 0U;

  (void)snprintf(text, sizeof(text), "%lu", (unsigned long)vpp_mv);
  (void)TJC_LCD_SetText("t_vpp", text);
  (void)snprintf(text, sizeof(text), "%lu", (unsigned long)rms_mv);
  (void)TJC_LCD_SetText("t_vrms", text);
  (void)snprintf(text, sizeof(text), "%lu.%03lu",
                 (unsigned long)(frequency_hz / 1000U),
                 (unsigned long)(frequency_hz % 1000U));
  (void)TJC_LCD_SetText("t_freq", text);

  LCD_ADC_UpdateStatus(LCD_TIME_STATUS_COMPONENT);
}

static void LCD_ADC_UpdateStatus(const char *component)
{
  uint32_t now = HAL_GetTick();

  if (((g_alias_detected == 0U) || (g_recovery_valid != 0U)) &&
      (g_lcd_current_page != LCD_PAGE_UNKNOWN) &&
      ((now - g_lcd_page_confirmed_tick) <= LCD_COMM_OK_MAX_AGE_MS))
  {
    (void)TJC_LCD_SetColor(component, LCD_COLOR_OK);
    (void)TJC_LCD_SetText(component, "OK");
  }
  else
  {
    (void)TJC_LCD_SetColor(component, LCD_COLOR_NG);
    (void)TJC_LCD_SetText(component, "NG");
  }
}

static void LCD_ADC_ReverseWaveformPoints(MeasurementWaveformResult *waveform)
{
  uint32_t point;

  if ((waveform == NULL) || (waveform->point_count < 2U))
  {
    return;
  }

  /* G_2's rotated transparent waveform component consumes x=0 at the
   * physical right edge. Reverse only the time-domain packet so its left
   * edge remains the beginning of the generated period. Spectrum packets
   * have their own horizontal calibration below. */
  for (point = 0U; point < (waveform->point_count / 2U); ++point)
  {
    uint32_t opposite = waveform->point_count - 1U - point;
    uint8_t temporary = waveform->points[point];

    waveform->points[point] = waveform->points[opposite];
    waveform->points[opposite] = temporary;
  }
}

static HAL_StatusTypeDef LCD_ADC_UpdateWaveform(void)
{
  MeasurementInputFrame input;

  if (g_alias_detected != 0U)
  {
    if ((g_recovery_valid == 0U) ||
        (Measurement_RecoverWaveform(
             &g_alias_comparison, g_display_period_count,
             &g_measurement_waveform) != MEASUREMENT_STATUS_OK))
    {
      g_waveform_valid = 0U;
      (void)TJC_LCD_Printf("cle %u,255",
                          (unsigned int)LCD_TIME_WAVEFORM_COMPONENT_ID);
      return HAL_OK;
    }

    g_waveform_valid = 1U;
  }
  else
  {
    APP_FillMeasurementInput(&input);
    if (Measurement_Task1_PrepareWaveform(
            &input, g_measurement_parameters.fundamental_hz,
            g_display_period_count,
            &g_measurement_waveform) != MEASUREMENT_STATUS_OK)
    {
      g_waveform_valid = 0U;
      g_lcd_text_refresh_requested = 1U;
      (void)TJC_LCD_Printf("cle %u,255",
                          (unsigned int)LCD_TIME_WAVEFORM_COMPONENT_ID);
      return HAL_ERROR;
    }

    g_waveform_valid = 1U;
  }

  LCD_ADC_ReverseWaveformPoints(&g_measurement_waveform);
  return TJC_LCD_WaveformAddTransparent(
      LCD_TIME_WAVEFORM_COMPONENT_ID, 0U, g_measurement_waveform.points,
      (uint16_t)g_measurement_waveform.point_count,
      LCD_TRANSPARENT_TIMEOUT_MS);
}

static HAL_StatusTypeDef LCD_ADC_UpdateSpectrum(void)
{
  static const char *const frequency_components[
      MEASUREMENT_MAX_COMPONENT_COUNT] = {"f1", "f2", "f3"};
  static const char *const amplitude_components[
      MEASUREMENT_MAX_COMPONENT_COUNT] = {
          "f1_vpp", "f2_vpp", "f3_vpp"};
  MeasurementInputFrame input;
  MeasurementStatus spectrum_status = MEASUREMENT_STATUS_OK;
  MeasurementStatus component_status = MEASUREMENT_STATUS_OK;
  char text[24];
  uint32_t component;
  uint32_t point;
  uint8_t recovery_mode =
      ((g_alias_detected != 0U) && (g_recovery_valid != 0U)) ? 1U : 0U;

  if ((g_alias_detected != 0U) && (recovery_mode == 0U))
  {
    (void)TJC_LCD_Printf("cle %u,255",
                        (unsigned int)LCD_SPECTRUM_COMPONENT_ID);
    for (component = 0U;
         component < MEASUREMENT_MAX_COMPONENT_COUNT;
         ++component)
    {
      (void)TJC_LCD_SetText(frequency_components[component], "--");
      (void)TJC_LCD_SetText(amplitude_components[component], "--");
    }
    LCD_ADC_UpdateStatus(LCD_SPECTRUM_STATUS_COMPONENT);
    return HAL_OK;
  }

  APP_FillMeasurementInput(&input);
  if (LCD_SPECTRUM_CALIBRATION_MODE != 0U)
  {
    uint32_t boundary;

    (void)memset(g_measurement_spectrum.bars, 0,
                 sizeof(g_measurement_spectrum.bars));
    g_measurement_spectrum.bar_count = MEASUREMENT_WAVEFORM_POINT_COUNT;
    g_measurement_spectrum.frequency_span_hz = 500000.0F;
    /* The 600-pixel plot contains 15 equal cells. Draw all 16 shared cell
     * boundaries, including the left and right edges, at 40-pixel steps. */
    for (boundary = 0U; boundary <= 15U; ++boundary)
    {
      uint32_t centre = (boundary == 15U) ?
          (MEASUREMENT_WAVEFORM_POINT_COUNT - 1U) : (boundary * 40U);
      uint32_t first = (centre == 0U) ? 0U : (centre - 1U);
      uint32_t last = (centre + 1U < MEASUREMENT_WAVEFORM_POINT_COUNT) ?
          (centre + 1U) : centre;
      uint32_t line_x;

      for (line_x = first; line_x <= last; ++line_x)
      {
        g_measurement_spectrum.bars[line_x] = 245U;
      }
    }
    for (component = 0U;
         component < MEASUREMENT_MAX_COMPONENT_COUNT;
         ++component)
    {
      (void)TJC_LCD_SetText(frequency_components[component], "--");
      (void)TJC_LCD_SetText(amplitude_components[component], "--");
    }
  }
  else
  {
    if (recovery_mode != 0U)
    {
      spectrum_status = Measurement_RecoverSpectrum(
          &g_alias_comparison, &g_measurement_spectrum);
      component_status = Measurement_RecoverComponents(
          &g_alias_comparison, &g_measurement_components);
    }
    else
    {
      spectrum_status = Measurement_Task3_PrepareSpectrum(
          &input, &g_measurement_spectrum);
      component_status = Measurement_Task4_MeasureComponents(
          &input, &g_measurement_components);
    }

    if (spectrum_status != MEASUREMENT_STATUS_OK)
    {
      (void)TJC_LCD_Printf("cle %u,255",
                          (unsigned int)LCD_SPECTRUM_COMPONENT_ID);
      return HAL_ERROR;
    }
    if (component_status != MEASUREMENT_STATUS_OK)
    {
      for (component = 0U;
           component < MEASUREMENT_MAX_COMPONENT_COUNT;
           ++component)
      {
        (void)TJC_LCD_SetText(frequency_components[component], "--");
        (void)TJC_LCD_SetText(amplitude_components[component], "--");
      }
      return HAL_ERROR;
    }

    ++g_spectrum_measurement_sequence;
    for (component = 0U;
         component < MEASUREMENT_MAX_COMPONENT_COUNT;
         ++component)
    {
      if (component < g_measurement_components.component_count)
      {
        uint32_t frequency_hz = (uint32_t)(
            g_measurement_components.components[component].frequency_hz +
            0.5F);
        uint32_t amplitude_mv = (uint32_t)(
            g_measurement_components.components[component].amplitude_mv +
            0.5F);

        (void)snprintf(text, sizeof(text), "%lu.%03lu",
                       (unsigned long)(frequency_hz / 1000U),
                       (unsigned long)(frequency_hz % 1000U));
        (void)TJC_LCD_SetText(frequency_components[component], text);
        (void)snprintf(text, sizeof(text), "%lu",
                       (unsigned long)amplitude_mv);
        (void)TJC_LCD_SetText(amplitude_components[component], text);
      }
      else
      {
        (void)TJC_LCD_SetText(frequency_components[component], "--");
        (void)TJC_LCD_SetText(amplitude_components[component], "--");
      }
    }
  }
  LCD_ADC_UpdateStatus(LCD_SPECTRUM_STATUS_COMPONENT);

  /* G_2.HMI displays transparent waveform samples in reverse horizontal
   * order: a logical x appears at (599 - x). Reverse the complete packet so
   * frequency aligns with the HMI's 0..500 kHz scale. */
  for (point = 0U;
       point < (g_measurement_spectrum.bar_count / 2U);
       ++point)
  {
    uint32_t opposite = g_measurement_spectrum.bar_count - 1U - point;
    uint8_t temporary = g_measurement_spectrum.bars[point];

    g_measurement_spectrum.bars[point] =
        g_measurement_spectrum.bars[opposite];
    g_measurement_spectrum.bars[opposite] = temporary;
  }

  return TJC_LCD_WaveformAddTransparent(
      LCD_SPECTRUM_COMPONENT_ID, 0U, g_measurement_spectrum.bars,
      (uint16_t)g_measurement_spectrum.bar_count,
      LCD_TRANSPARENT_TIMEOUT_MS);
}

static void LCD_ADC_ShowError(const char *message)
{
  (void)TJC_LCD_Page("1");
  HAL_Delay(30U);
  (void)TJC_LCD_SetColor(LCD_TIME_STATUS_COMPONENT, LCD_COLOR_NG);
  (void)TJC_LCD_SetText(LCD_TIME_STATUS_COMPONENT, message);
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  TJC_LCD_Init(&huart6);
  (void)TJC_LCD_StartReceiveIT();
  LCD_ADC_Init();
  if (ADC_Capture_Init() != HAL_OK)
  {
    g_app_error_code = 0x10U;
    LCD_ADC_ShowError("ADC INIT ERROR");
    Error_Handler();
  }
  if (ADC_Capture_Start() != HAL_OK)
  {
    g_app_error_code = 0x20U;
    LCD_ADC_ShowError("ADC START ERROR");
    Error_Handler();
  }
  /* Verify the first normal frame immediately.  Otherwise a high-frequency
   * input such as 3.8 MHz can be displayed briefly as its roughly 413 kHz
   * normal-rate alias before the first scheduled probe. */
  g_alias_probe_tick = HAL_GetTick() - ALIAS_PROBE_INTERVAL_MS;
  if (APP_IWDG_Init() != HAL_OK)
  {
    g_app_error_code = 0x50U;
    LCD_ADC_ShowError("IWDG INIT ERROR");
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now;

    APP_ProcessLcdEvents();
    APP_RequestLcdPage();

    if (ADC_Capture_HasError() != 0U)
    {
      g_app_error_code = 0x30U;
      LCD_ADC_ShowError("ADC DMA ERROR");
      Error_Handler();
    }

    now = HAL_GetTick();
    if ((g_alias_state != APP_ALIAS_STATE_NORMAL) &&
        ((now - g_alias_state_tick) >= ALIAS_STATE_TIMEOUT_MS))
    {
      ++g_alias_switch_error_count;
      g_app_error_code = 0x62U;
      LCD_ADC_ShowError("ADC PROBE ERROR");
      Error_Handler();
    }
    if (ADC_Capture_IsReady() != 0U)
    {
      if (g_alias_state == APP_ALIAS_STATE_PROBE_DISCARD)
      {
        /* The first frame after changing the ADC interleave delay may contain
         * samples from the old sequencer phase, so never use it for a verdict. */
        ADC_Capture_ClearReady();
        g_alias_state = APP_ALIAS_STATE_PROBE_CAPTURE;
        g_alias_state_tick = HAL_GetTick();
      }
      else if (g_alias_state == APP_ALIAS_STATE_PROBE_CAPTURE)
      {
        APP_HandleAliasProbeFrame();
      }
      else if (g_alias_state == APP_ALIAS_STATE_NORMAL_DISCARD)
      {
        APP_HandleRestoredNormalFrame();
      }
      else if ((g_measurement_valid == 0U) ||
               ((now - g_measurement_refresh_tick) >=
                MEASUREMENT_REFRESH_INTERVAL_MS))
      {
        MeasurementInputFrame input;
        MeasurementParameterResult measured_parameters;
        uint8_t probe_due =
            ((now - g_alias_probe_tick) >= ALIAS_PROBE_INTERVAL_MS) ?
            1U : 0U;

        /* Copy the complete private DMA snapshot before releasing it. The
         * saved normal-rate frame remains valid while probe frames are taken. */
        (void)memcpy(g_analysis_snapshot,
                     ADC_Capture_GetStableAnalysisSamples(),
                     sizeof(g_analysis_snapshot));
        ADC_Capture_CalculateStats(&g_adc_capture_stats);
        if (probe_due == 0U)
        {
          ADC_Capture_ClearReady();
        }
        APP_FillMeasurementInput(&input);
        if (Measurement_Task2_MeasureParameters(
                &input, &measured_parameters) == MEASUREMENT_STATUS_OK)
        {
          measured_parameters.fundamental_hz = APP_FilterFrequency(
              measured_parameters.fundamental_hz);
          if (measured_parameters.fundamental_hz > 0.0F)
          {
            g_measurement_parameters = measured_parameters;
            ++g_adc_capture_sequence;
            g_measurement_valid = 1U;
            g_app_error_code = 0U;
          }
        }
        else
        {
          /* A missing or temporarily out-of-band frame is not a fatal
           * hardware error. Keep the last valid values and retry. */
          g_app_error_code = 0x40U;
        }
        g_measurement_refresh_tick = HAL_GetTick();

        if (probe_due != 0U)
        {
          MeasurementStatus signature_status =
              Measurement_ExtractRawLowBandSignature(
                  &input, &g_alias_normal_signature);

          g_alias_probe_tick = HAL_GetTick();
          if (signature_status == MEASUREMENT_STATUS_OK)
          {
            g_alias_state = APP_ALIAS_STATE_PROBE_DISCARD;
            g_alias_state_tick = HAL_GetTick();
            if (ADC_Capture_SetRate(
                    ADC_CAPTURE_RATE_ALIAS_PROBE) != HAL_OK)
            {
              ++g_alias_switch_error_count;
              g_app_error_code = 0x60U;
              LCD_ADC_ShowError("ADC RATE ERROR");
              Error_Handler();
            }
          }
          else
          {
            /* Treat a repeated detector failure as NG instead of showing an
             * unchecked low-frequency result as valid. */
            ADC_Capture_ClearReady();
            g_app_error_code = 0x70U;
            APP_UpdateAliasDecision(1U);
            g_recovery_valid = 0U;
            g_lcd_text_refresh_requested = 1U;
            g_lcd_waveform_refresh_requested = 1U;
            g_lcd_spectrum_refresh_requested = 1U;
          }
        }
      }
    }

    now = HAL_GetTick();
    if ((g_alias_state == APP_ALIAS_STATE_NORMAL) &&
        (g_lcd_current_page == LCD_PAGE_MEASUREMENT) &&
        (g_lcd_page_query_pending == 0U) &&
        ((now - g_lcd_page_confirmed_tick) <=
         LCD_PAGE_CONFIRM_MAX_AGE_MS) &&
         ((g_measurement_valid != 0U) || (g_alias_detected != 0U)))
    {
      if ((g_lcd_waveform_refresh_requested != 0U) ||
          ((now - g_lcd_waveform_refresh_tick) >=
           LCD_WAVEFORM_REFRESH_INTERVAL_MS))
      {
        HAL_StatusTypeDef waveform_status;

        /* Measure the cadence from the start of the transparent transfer, as
         * on the spectrum page, so UART time does not stretch 600 ms into a
         * progressively slower visible update. */
        g_lcd_waveform_refresh_tick = now;
        waveform_status = LCD_ADC_UpdateWaveform();
        g_lcd_waveform_refresh_requested = 0U;
        if ((waveform_status != HAL_OK) && (g_waveform_valid != 0U))
        {
          /* The page may have changed after its last sendme reply. Never send
           * raw data after a failed FE handshake; query the page again. */
          g_lcd_current_page = LCD_PAGE_UNKNOWN;
          g_lcd_page_query_pending = 0U;
        }
      }

    }

    now = HAL_GetTick();
    if ((g_alias_state == APP_ALIAS_STATE_NORMAL) &&
        (g_lcd_current_page == LCD_PAGE_SPECTRUM) &&
        (g_lcd_page_query_pending == 0U) &&
        ((now - g_lcd_page_confirmed_tick) <=
         LCD_PAGE_CONFIRM_MAX_AGE_MS) &&
         ((g_measurement_valid != 0U) || (g_alias_detected != 0U)) &&
        ((g_lcd_spectrum_refresh_requested != 0U) ||
         ((now - g_lcd_spectrum_refresh_tick) >=
          LCD_SPECTRUM_REFRESH_INTERVAL_MS)))
    {
      HAL_StatusTypeDef spectrum_status;

      /* Measure the cadence from the start of each transfer. Starting
       * the timer after the 600-byte transparent packet would add the UART
       * transfer time and make the visible refresh noticeably slower. The
       * 600 ms scheduler allowance compensates for probe/page-query work so
       * the observed cadence remains close to three refreshes per two seconds. */
      g_lcd_spectrum_refresh_tick = now;
      spectrum_status = LCD_ADC_UpdateSpectrum();
      g_lcd_spectrum_refresh_requested = 0U;
      if (spectrum_status != HAL_OK)
      {
        g_lcd_current_page = LCD_PAGE_UNKNOWN;
        g_lcd_page_query_pending = 0U;
      }
    }

    /* Text writes are harmless on page0/page2 because bkcmd=0 and those
     * component names do not exist there. Sending them independently lets
     * page1 show values even when the screen TX -> MCU RX wire is missing;
     * the status then reads CHECK RX and only waveform transfer is disabled. */
    now = HAL_GetTick();
    if ((g_alias_state == APP_ALIAS_STATE_NORMAL) &&
        ((g_measurement_valid != 0U) || (g_alias_detected != 0U)) &&
        (g_lcd_current_page != LCD_PAGE_SPECTRUM) &&
        ((g_lcd_text_refresh_requested != 0U) ||
         ((now - g_lcd_text_refresh_tick) >=
          LCD_TEXT_REFRESH_INTERVAL_MS)))
    {
      LCD_ADC_Update(&g_measurement_parameters);
      g_lcd_text_refresh_tick = HAL_GetTick();
      g_lcd_text_refresh_requested = 0U;
    }
    APP_IWDG_Refresh();
    HAL_Delay(1U);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  TJC_LCD_UART_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  TJC_LCD_UART_ErrorCallback(huart);
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
