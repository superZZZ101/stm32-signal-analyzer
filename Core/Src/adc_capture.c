#include "adc_capture.h"
#include "main.h"

#include <string.h>

#define ADC_CAPTURE_DMA_WORD_COUNT (ADC_CAPTURE_SAMPLE_COUNT / 2U)
#define ADC_CAPTURE_HALF_SAMPLE_COUNT (ADC_CAPTURE_SAMPLE_COUNT / 2U)

static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static ADC_HandleTypeDef hadc3;
static DMA_HandleTypeDef hdma_adc1;

/* DMA mode 2 packs two chronological 12-bit samples into each 32-bit word. */
static uint32_t adc_dma_words[ADC_CAPTURE_DMA_WORD_COUNT] __attribute__((aligned(4)));
/* Half- and full-transfer callbacks copy one chronological frame here. DMA
 * never writes this buffer, so FFT processing and a slow LCD update cannot
 * tear the frame. */
static uint16_t adc_analysis_samples[ADC_CAPTURE_ANALYSIS_SAMPLE_COUNT]
    __attribute__((aligned(4)));
static volatile uint8_t adc_capture_ready;
static volatile uint8_t adc_capture_running;
static volatile uint8_t adc_capture_error;
static volatile uint8_t adc_snapshot_building;
static ADC_CaptureRate adc_capture_rate = ADC_CAPTURE_RATE_NORMAL;
volatile uint32_t g_adc_capture_start_stage;

static HAL_StatusTypeDef ADC_Capture_ConfigureMultimode(
    ADC_CaptureRate rate)
{
  ADC_MultiModeTypeDef multimode = {0};

  multimode.Mode = ADC_TRIPLEMODE_INTERL;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  if (rate == ADC_CAPTURE_RATE_NORMAL)
  {
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  }
  else if (rate == ADC_CAPTURE_RATE_ALIAS_PROBE)
  {
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_7CYCLES;
  }
  else
  {
    return HAL_ERROR;
  }

  /* HAL clears MULTI before programming it again. This also resets the
   * interleaved sequencer after a running conversion has been stopped. */
  return HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);
}

static HAL_StatusTypeDef ADC_Capture_InitOne(ADC_HandleTypeDef *hadc,
                                              ADC_TypeDef *instance,
                                              FunctionalState dma_requests)
{
  ADC_ChannelConfTypeDef channel = {0};

  hadc->Instance = instance;
  hadc->Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc->Init.Resolution = ADC_RESOLUTION_12B;
  hadc->Init.ScanConvMode = DISABLE;
  hadc->Init.ContinuousConvMode = ENABLE;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.NbrOfDiscConversion = 0U;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.NbrOfConversion = 1U;
  hadc->Init.DMAContinuousRequests = dma_requests;
  hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;

  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    return HAL_ERROR;
  }

  channel.Channel = ADC_CHANNEL_12;
  channel.Rank = 1U;
  channel.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  channel.Offset = 0U;
  return HAL_ADC_ConfigChannel(hadc, &channel);
}

HAL_StatusTypeDef ADC_Capture_Init(void)
{
  /* The slave ADCs are initialized first, as in ST's triple-interleave example. */
  if (ADC_Capture_InitOne(&hadc3, ADC3, DISABLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ADC_Capture_InitOne(&hadc2, ADC2, DISABLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ADC_Capture_InitOne(&hadc1, ADC1, ENABLE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ADC_Capture_ConfigureMultimode(ADC_CAPTURE_RATE_NORMAL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  adc_capture_rate = ADC_CAPTURE_RATE_NORMAL;
  adc_capture_ready = 0U;
  adc_capture_running = 0U;
  adc_capture_error = 0U;
  adc_snapshot_building = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef ADC_Capture_Start(void)
{
  if (adc_capture_running != 0U)
  {
    g_adc_capture_start_stage = 1U;
    return HAL_BUSY;
  }

  adc_capture_ready = 0U;
  adc_capture_error = 0U;
  adc_snapshot_building = 0U;

  if (HAL_ADC_Start(&hadc3) != HAL_OK)
  {
    g_adc_capture_start_stage = 2U;
    return HAL_ERROR;
  }
  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    g_adc_capture_start_stage = 3U;
    (void)HAL_ADC_Stop(&hadc3);
    return HAL_ERROR;
  }
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_dma_words,
                                   ADC_CAPTURE_DMA_WORD_COUNT) != HAL_OK)
  {
    g_adc_capture_start_stage = 4U;
    (void)HAL_ADC_Stop(&hadc2);
    (void)HAL_ADC_Stop(&hadc3);
    return HAL_ERROR;
  }

  adc_capture_running = 1U;
  g_adc_capture_start_stage = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef ADC_Capture_Stop(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (HAL_ADCEx_MultiModeStop_DMA(&hadc1) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  if (HAL_ADC_Stop(&hadc2) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  if (HAL_ADC_Stop(&hadc3) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  adc_capture_running = 0U;
  adc_snapshot_building = 0U;
  return status;
}

HAL_StatusTypeDef ADC_Capture_SetRate(ADC_CaptureRate rate)
{
  uint8_t restart_capture;

  if ((rate != ADC_CAPTURE_RATE_NORMAL) &&
      (rate != ADC_CAPTURE_RATE_ALIAS_PROBE))
  {
    return HAL_ERROR;
  }

  if (rate == adc_capture_rate)
  {
    return HAL_OK;
  }

  restart_capture = adc_capture_running;
  if ((restart_capture != 0U) && (ADC_Capture_Stop() != HAL_OK))
  {
    return HAL_ERROR;
  }

  if (ADC_Capture_ConfigureMultimode(rate) != HAL_OK)
  {
    return HAL_ERROR;
  }
  adc_capture_rate = rate;

  if ((restart_capture != 0U) && (ADC_Capture_Start() != HAL_OK))
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

ADC_CaptureRate ADC_Capture_GetRate(void)
{
  return adc_capture_rate;
}

uint32_t ADC_Capture_GetSampleRateHz(void)
{
  if (adc_capture_rate == ADC_CAPTURE_RATE_ALIAS_PROBE)
  {
    return ADC_CAPTURE_ALIAS_PROBE_SAMPLE_RATE_HZ;
  }
  return ADC_CAPTURE_NORMAL_SAMPLE_RATE_HZ;
}

uint8_t ADC_Capture_IsReady(void)
{
  return adc_capture_ready;
}

void ADC_Capture_ClearReady(void)
{
  __DMB();
  adc_capture_ready = 0U;
}

uint8_t ADC_Capture_HasError(void)
{
  return adc_capture_error;
}

const uint16_t *ADC_Capture_GetSamples(void)
{
  return adc_analysis_samples;
}

const uint16_t *ADC_Capture_GetStableAnalysisSamples(void)
{
  return adc_analysis_samples;
}

static uint32_t ADC_Capture_SqrtU64(uint64_t value)
{
  uint64_t bit = (uint64_t)1U << 62;
  uint64_t result = 0U;

  while (bit > value)
  {
    bit >>= 2;
  }
  while (bit != 0U)
  {
    if (value >= (result + bit))
    {
      value -= result + bit;
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }
    bit >>= 2;
  }
  return (uint32_t)result;
}

void ADC_Capture_CalculateStats(ADC_CaptureStats *stats)
{
  const uint16_t *samples = ADC_Capture_GetSamples();
  uint64_t sum = 0U;
  uint64_t square_sum = 0U;
  uint32_t mean;
  uint32_t rms_code;
  uint16_t minimum = 4095U;
  uint16_t maximum = 0U;
  uint32_t index;

  if (stats == NULL)
  {
    return;
  }

  for (index = 0U; index < ADC_CAPTURE_SAMPLE_COUNT; ++index)
  {
    uint16_t sample = (uint16_t)(samples[index] & 0x0FFFU);
    if (sample < minimum)
    {
      minimum = sample;
    }
    if (sample > maximum)
    {
      maximum = sample;
    }
    sum += sample;
  }

  mean = (uint32_t)((sum + (ADC_CAPTURE_SAMPLE_COUNT / 2U)) /
                    ADC_CAPTURE_SAMPLE_COUNT);

  for (index = 0U; index < ADC_CAPTURE_SAMPLE_COUNT; ++index)
  {
    int32_t centered = (int32_t)(samples[index] & 0x0FFFU) - (int32_t)mean;
    square_sum += (uint64_t)((int64_t)centered * centered);
  }

  rms_code = ADC_Capture_SqrtU64(square_sum / ADC_CAPTURE_SAMPLE_COUNT);
  stats->sample_count = ADC_CAPTURE_SAMPLE_COUNT;
  stats->minimum_code = minimum;
  stats->maximum_code = maximum;
  stats->mean_code = (uint16_t)mean;
  stats->mean_mv = (mean * ADC_CAPTURE_VREF_MV + 2047U) / 4095U;
  stats->vpp_mv = ((uint32_t)(maximum - minimum) * ADC_CAPTURE_VREF_MV + 2047U) / 4095U;
  stats->rms_ac_mv = (rms_code * ADC_CAPTURE_VREF_MV + 2047U) / 4095U;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();
  __HAL_RCC_ADC3_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpio);

  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    /* Keep DMA busy after transfer-complete so Stop_DMA can abort cleanly
       before the next one-shot capture is armed. */
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_adc1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_adc1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  }
}

void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (adc_snapshot_building != 0U) &&
      (adc_capture_ready == 0U))
  {
    const uint16_t *dma_samples = (const uint16_t *)adc_dma_words;

    (void)memcpy(&adc_analysis_samples[ADC_CAPTURE_HALF_SAMPLE_COUNT],
                 &dma_samples[ADC_CAPTURE_HALF_SAMPLE_COUNT],
                 ADC_CAPTURE_HALF_SAMPLE_COUNT * sizeof(uint16_t));
    __DMB();
    adc_capture_ready = 1U;
    adc_snapshot_building = 0U;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (adc_capture_ready == 0U))
  {
    const uint16_t *dma_samples = (const uint16_t *)adc_dma_words;

    (void)memcpy(adc_analysis_samples, dma_samples,
                 ADC_CAPTURE_HALF_SAMPLE_COUNT * sizeof(uint16_t));
    __DMB();
    adc_snapshot_building = 1U;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_capture_error = 1U;
    adc_capture_running = 0U;
    adc_snapshot_building = 0U;
  }
}
