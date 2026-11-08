#ifndef MEASUREMENT_TASKS_H
#define MEASUREMENT_TASKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MEASUREMENT_WAVEFORM_POINT_COUNT 600U
#define MEASUREMENT_MAX_COMPONENT_COUNT  3U
#define MEASUREMENT_RAW_SAMPLE_COUNT      16384U
#define MEASUREMENT_FFT_POINT_COUNT       4096U
#define MEASUREMENT_DECIMATION_FACTOR     4U
/* Dual-rate alias detector settings.  The signature uses a separate raw,
 * non-decimated FFT; these are input-referred sine peak amplitudes. */
#define MEASUREMENT_ALIAS_MAX_PEAK_COUNT             8U
#define MEASUREMENT_ALIAS_SCAN_MIN_FREQUENCY_HZ      8000.0F
#define MEASUREMENT_ALIAS_SCAN_MAX_FREQUENCY_HZ      1450000.0F
#define MEASUREMENT_ALIAS_VALID_BAND_MAX_HZ           510000.0F
#define MEASUREMENT_ALIAS_SIGNATURE_MIN_PEAK_MV      3.0F
#define MEASUREMENT_ALIAS_STRONG_PEAK_MV             4.0F
#define MEASUREMENT_ALIAS_FREQUENCY_TOLERANCE_HZ     1000.0F
#define MEASUREMENT_ALIAS_AMPLITUDE_TOLERANCE_MV     2.0F
#define MEASUREMENT_ALIAS_AMPLITUDE_TOLERANCE_RATIO  0.20F

/* Analogue front-end voltage gain from the BNC input to PC2.
 *
 * All voltage results exposed by the measurement tasks are input-referred,
 * so the ADC-side voltage is divided by this value.  Start with the nominal
 * gain and replace it with CH2_Vpp / CH1_Vpp after bench calibration. */
#define MEASUREMENT_FRONT_END_GAIN         4.951F

typedef enum
{
  MEASUREMENT_STATUS_OK = 0,
  MEASUREMENT_STATUS_INVALID_ARGUMENT,
  MEASUREMENT_STATUS_NOT_IMPLEMENTED
} MeasurementStatus;

typedef enum
{
  MEASUREMENT_SHOW_ONE_PERIOD = 1,
  MEASUREMENT_SHOW_THREE_PERIODS = 3
} MeasurementPeriodCount;

/* One calibrated ADC frame shared by all four measurement tasks.
 *
 * The DSP path consumes the first MEASUREMENT_RAW_SAMPLE_COUNT samples. The
 * caller must freeze DMA (or copy the frame) while a task is running. After
 * low-pass filtering and 4:1 decimation, the 4096-point FFT bin spacing is
 * sample_rate_hz / 4 / 4096 (about 257 Hz at 4.212 MSPS), satisfying the
 * official <=500 Hz requirement. */
typedef struct
{
  const uint16_t *samples;
  uint32_t sample_count;
  uint32_t sample_rate_hz;
  uint32_t adc_reference_mv;
  uint16_t adc_full_scale_code;
} MeasurementInputFrame;

typedef struct
{
  MeasurementPeriodCount period_count;
  uint8_t points[MEASUREMENT_WAVEFORM_POINT_COUNT];
  uint32_t point_count;
} MeasurementWaveformResult;

typedef struct
{
  float vpp_mv;
  float rms_mv;
  float fundamental_hz;
} MeasurementParameterResult;

typedef struct
{
  uint8_t bars[MEASUREMENT_WAVEFORM_POINT_COUNT];
  uint32_t bar_count;
  float frequency_span_hz;
} MeasurementSpectrumResult;

typedef struct
{
  float frequency_hz;
  float amplitude_mv; /* Sine peak amplitude, not RMS or peak-to-peak. */
} MeasurementSpectrumComponent;

typedef struct
{
  MeasurementSpectrumComponent components[MEASUREMENT_MAX_COMPONENT_COUNT];
  uint32_t component_count;
} MeasurementComponentResult;

/* One peak from a raw, undecimated frame.  The coefficients are already
 * referred back to the BNC input and use the module's synthesis convention:
 *
 *   x(t) = cosine_mv * cos(wt) - sine_mv * sin(wt)
 *
 * Keeping both coefficients from the normal-rate frame lets the stable
 * low-band signal be reconstructed after moving aliases are rejected. */
typedef struct
{
  float frequency_hz;
  float amplitude_mv; /* Sine peak amplitude, not RMS or peak-to-peak. */
  float cosine_mv;
  float sine_mv;
} MeasurementRawSpectrumPeak;

/* Low-band peaks obtained directly from the first 4096 ADC samples.  No FIR
 * or decimation is used, so aliases already present at the ADC input remain
 * visible and can be compared between two deliberately different rates. */
typedef struct
{
  MeasurementRawSpectrumPeak peaks[MEASUREMENT_ALIAS_MAX_PEAK_COUNT];
  uint32_t peak_count;
  uint32_t sample_rate_hz;
} MeasurementRawLowBandSignature;

typedef enum
{
  MEASUREMENT_ALIAS_PEAK_NONE = 0,
  MEASUREMENT_ALIAS_PEAK_NORMAL,
  MEASUREMENT_ALIAS_PEAK_PROBE
} MeasurementAliasPeakSource;

/* A line which stayed at the same frequency and amplitude at both sample
 * rates.  Frequency and amplitude are the two-rate averages.  Phase comes
 * exclusively from the normal-rate frame; its coefficients are rescaled to
 * the averaged amplitude without changing their angle. */
typedef struct
{
  float frequency_hz;
  float amplitude_mv;
  float cosine_mv;
  float sine_mv;
  uint32_t harmonic_order;
} MeasurementRecoveredComponent;

typedef struct
{
  uint8_t alias_detected;
  MeasurementAliasPeakSource unmatched_source;
  float unmatched_frequency_hz;
  float unmatched_amplitude_mv;
  MeasurementRecoveredComponent
      stable_components[MEASUREMENT_MAX_COMPONENT_COUNT];
  uint32_t stable_component_count;
} MeasurementAliasComparisonResult;

/* Requirement 1: prepare exactly one or three complete periods for display.
 * fundamental_hz is the temporally confirmed frequency used as an analysis
 * hint so an isolated subharmonic decision cannot change the time scale. */
MeasurementStatus Measurement_Task1_PrepareWaveform(
    const MeasurementInputFrame *input,
    float fundamental_hz,
    MeasurementPeriodCount period_count,
    MeasurementWaveformResult *result);

/* Requirement 2: measure Vpp, true RMS and the fundamental frequency. */
MeasurementStatus Measurement_Task2_MeasureParameters(
    const MeasurementInputFrame *input,
    MeasurementParameterResult *result);

/* Requirement 3: prepare a qualitative positive-frequency spectrum plot. */
MeasurementStatus Measurement_Task3_PrepareSpectrum(
    const MeasurementInputFrame *input,
    MeasurementSpectrumResult *result);

/* Requirement 4: measure each line's frequency and sine peak amplitude. */
MeasurementStatus Measurement_Task4_MeasureComponents(
    const MeasurementInputFrame *input,
    MeasurementComponentResult *result);

/* Build and compare signatures for dual-rate alias detection.  Calls use the
 * module's existing static CMSIS-DSP buffers and therefore must be serialized
 * with Tasks 1-4.  Capture/DMA must be frozen (or the frame copied), just as
 * for the normal measurement tasks. */
MeasurementStatus Measurement_ExtractRawLowBandSignature(
    const MeasurementInputFrame *input,
    MeasurementRawLowBandSignature *signature);

MeasurementStatus Measurement_CompareRawLowBandSignatures(
    const MeasurementRawLowBandSignature *normal_signature,
    const MeasurementRawLowBandSignature *probe_signature,
    MeasurementAliasComparisonResult *result);

/* Rebuild the valid <=500 kHz signal from the lines which stayed fixed across
 * both sample rates.  These functions remain useful when alias_detected is
 * set: the unmatched moving line is omitted while the stable u_b lines are
 * rendered and measured. */
MeasurementStatus Measurement_RecoverParameters(
    const MeasurementAliasComparisonResult *comparison,
    MeasurementParameterResult *result);

MeasurementStatus Measurement_RecoverWaveform(
    const MeasurementAliasComparisonResult *comparison,
    MeasurementPeriodCount period_count,
    MeasurementWaveformResult *result);

MeasurementStatus Measurement_RecoverSpectrum(
    const MeasurementAliasComparisonResult *comparison,
    MeasurementSpectrumResult *result);

MeasurementStatus Measurement_RecoverComponents(
    const MeasurementAliasComparisonResult *comparison,
    MeasurementComponentResult *result);

#ifdef __cplusplus
}
#endif

#endif
