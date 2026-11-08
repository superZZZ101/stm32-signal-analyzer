#ifndef ADC_CAPTURE_H
#define ADC_CAPTURE_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_CAPTURE_SAMPLE_COUNT  16384U
#define ADC_CAPTURE_ANALYSIS_SAMPLE_COUNT ADC_CAPTURE_SAMPLE_COUNT
/* Calibrated from 10 kHz, 100 kHz and 500 kHz reference measurements.
 * The HSI-derived ADC clock is about 0.29% above its nominal value. */
#define ADC_CAPTURE_NORMAL_SAMPLE_RATE_HZ      4212000U
#define ADC_CAPTURE_ALIAS_PROBE_SAMPLE_RATE_HZ 3008571U
/* Backward-compatible name for the normal measurement rate. */
#define ADC_CAPTURE_SAMPLE_RATE_HZ ADC_CAPTURE_NORMAL_SAMPLE_RATE_HZ
#define ADC_CAPTURE_VREF_MV       3300U

typedef enum
{
  ADC_CAPTURE_RATE_NORMAL = 0,
  ADC_CAPTURE_RATE_ALIAS_PROBE
} ADC_CaptureRate;

typedef struct
{
  uint32_t sample_count;
  uint16_t minimum_code;
  uint16_t maximum_code;
  uint16_t mean_code;
  uint32_t mean_mv;
  uint32_t vpp_mv;
  uint32_t rms_ac_mv;
} ADC_CaptureStats;

HAL_StatusTypeDef ADC_Capture_Init(void);
HAL_StatusTypeDef ADC_Capture_Start(void);
HAL_StatusTypeDef ADC_Capture_Stop(void);
HAL_StatusTypeDef ADC_Capture_SetRate(ADC_CaptureRate rate);
ADC_CaptureRate ADC_Capture_GetRate(void);
uint32_t ADC_Capture_GetSampleRateHz(void);
uint8_t ADC_Capture_IsReady(void);
void ADC_Capture_ClearReady(void);
uint8_t ADC_Capture_HasError(void);
const uint16_t *ADC_Capture_GetSamples(void);
/* Returns a private chronological frame which DMA will not overwrite until
 * ADC_Capture_ClearReady() asks the ISR to assemble the next frame. */
const uint16_t *ADC_Capture_GetStableAnalysisSamples(void);
void ADC_Capture_CalculateStats(ADC_CaptureStats *stats);

#ifdef __cplusplus
}
#endif

#endif
