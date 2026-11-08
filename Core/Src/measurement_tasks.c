#include "measurement_tasks.h"

#include "stm32f4xx.h"
#include "arm_math.h"

#include <float.h>
#include <string.h>

#define MEASUREMENT_FFT_LENGTH             MEASUREMENT_FFT_POINT_COUNT
#define MEASUREMENT_FFT_HALF_LENGTH        (MEASUREMENT_FFT_LENGTH / 2U)
#define MEASUREMENT_TWO_PI                 6.28318530718F
#define MEASUREMENT_MIN_FREQUENCY_HZ       10000.0F
#define MEASUREMENT_MAX_FREQUENCY_HZ       500000.0F
/* Allow the Hann main lobe and small generator/sample-clock tolerances to
 * cross the nominal 500 kHz limit without making the real peak disappear.
 * Displayed spectrum scaling remains fixed at the specified 500 kHz. */
#define MEASUREMENT_SEARCH_MAX_FREQUENCY_HZ 510000.0F
#define MEASUREMENT_FREQUENCY_GRID_HZ      500.0F
#define MEASUREMENT_PEAK_CANDIDATE_COUNT   12U
#define MEASUREMENT_PEAK_EXCLUSION_BINS    6U
#define MEASUREMENT_HARMONIC_TOLERANCE_BIN 1.5F
#define MEASUREMENT_FREQUENCY_HINT_TOLERANCE_BIN 1.5F
#define MEASUREMENT_MIN_COMPONENT_MV       2.5F
#define MEASUREMENT_MAX_HARMONIC_ORDER     100U
#define MEASUREMENT_SPECTRUM_MAX_HEIGHT    245.0F
#define MEASUREMENT_SPECTRUM_DISPLAY_SPAN_HZ 500000.0F
#define MEASUREMENT_SPECTRUM_LINE_HALF_WIDTH 1U
#define MEASUREMENT_ALIAS_PEAK_EXCLUSION_BINS 3U
/* At low frequencies, hundreds of noisy ADC samples occur in each period and
 * bias absolute max/min estimates upward.  Above this point the time-domain
 * estimator is already stable and is more reliable near the 500 kHz FFT
 * search boundary. */
#define MEASUREMENT_SYNTHESIZED_VPP_MAX_FREQUENCY_HZ 25000.0F

#define MEASUREMENT_FILTER_TAP_COUNT       49U
#define MEASUREMENT_FILTER_HALF_LENGTH     \
  ((MEASUREMENT_FILTER_TAP_COUNT - 1U) / 2U)
#define MEASUREMENT_RAW_INDEX_MASK         \
  (MEASUREMENT_RAW_SAMPLE_COUNT - 1U)

typedef struct
{
  float refined_bin;
  float magnitude_squared;
  float amplitude_code;
  float cosine_code;
  float sine_code;
  uint32_t harmonic_order;
} MeasurementTone;

typedef struct
{
  MeasurementTone tones[MEASUREMENT_MAX_COMPONENT_COUNT];
  uint32_t tone_count;
  float fundamental_bin;
  float fundamental_hz;
} MeasurementAnalysis;

typedef struct
{
  uint32_t peak_bin;
  float magnitude_squared;
  MeasurementTone tone;
} MeasurementPeakCandidate;

/* 49-tap, unity-DC-gain Kaiser-windowed low-pass FIR. At the calibrated
 * 4.212 MSPS rate its passband is flat through 500 kHz, attenuation is about
 * 85 dB at 1 MHz, and the >=1 MHz stopband remains above 80 dB. It protects
 * the digital 4:1 decimator from u_J.
 *
 * An analogue anti-alias low-pass is still required ahead of the ADC because
 * no digital algorithm can remove an interference tone which has already
 * aliased below 500 kHz at the raw ADC sampling stage. */
static const float measurement_filter_taps[MEASUREMENT_FILTER_TAP_COUNT] =
{
   3.506192218345e-05F,  5.669782722809e-05F, -1.049708090443e-04F,
  -3.859561937712e-04F, -2.432843921942e-04F,  6.838562201478e-04F,
   1.475357385810e-03F,  3.770471075424e-04F, -2.549102350081e-03F,
  -3.787406448647e-03F,  2.592587294269e-04F,  7.033486214022e-03F,
   7.556184989131e-03F, -3.295506794174e-03F, -1.606810344271e-02F,
  -1.245610874915e-02F,  1.195643108511e-02F,  3.306027566952e-02F,
   1.750924526825e-02F, -3.446746995221e-02F, -6.980966966519e-02F,
  -2.135304935235e-02F,  1.219161710338e-01F,  2.845336443237e-01F,
   3.561358207473e-01F,  2.845336443237e-01F,  1.219161710338e-01F,
  -2.135304935235e-02F, -6.980966966519e-02F, -3.446746995221e-02F,
   1.750924526825e-02F,  3.306027566952e-02F,  1.195643108511e-02F,
  -1.245610874915e-02F, -1.606810344271e-02F, -3.295506794174e-03F,
   7.556184989131e-03F,  7.033486214022e-03F,  2.592587294269e-04F,
  -3.787406448647e-03F, -2.549102350081e-03F,  3.770471075424e-04F,
   1.475357385810e-03F,  6.838562201478e-04F, -2.432843921942e-04F,
  -3.859561937712e-04F, -1.049708090443e-04F,  5.669782722809e-05F,
   3.506192218345e-05F
};

/* The F407 CCM is CPU-only memory and is ideal for the three DSP work
 * buffers. The Hann table stays in ordinary SRAM, leaving 16 KiB CCM spare. */
static float measurement_filtered_signal[MEASUREMENT_FFT_LENGTH]
    __attribute__((section(".ccmram"), aligned(4)));
static float measurement_fft_input[MEASUREMENT_FFT_LENGTH]
    __attribute__((section(".ccmram"), aligned(4)));
static float measurement_fft_output[MEASUREMENT_FFT_LENGTH]
    __attribute__((section(".ccmram"), aligned(4)));
static float measurement_fft_window[MEASUREMENT_FFT_LENGTH];
static arm_rfft_fast_instance_f32 measurement_fft_instance;
static float measurement_fft_window_sum;
static float measurement_raw_mean;
static uint8_t measurement_fft_initialized;
static MeasurementPeakCandidate
    measurement_peak_candidates[MEASUREMENT_PEAK_CANDIDATE_COUNT];
static MeasurementTone
    measurement_best_family[MEASUREMENT_MAX_COMPONENT_COUNT];

static uint16_t Measurement_GetCode(const MeasurementInputFrame *input,
                                    uint32_t index)
{
  return (uint16_t)(input->samples[index] & input->adc_full_scale_code);
}

static uint8_t Measurement_InputIsValid(const MeasurementInputFrame *input)
{
  if ((input == NULL) || (input->samples == NULL) ||
      (input->sample_count < MEASUREMENT_RAW_SAMPLE_COUNT) ||
      (input->sample_rate_hz == 0U) || (input->adc_reference_mv == 0U) ||
      (input->adc_full_scale_code == 0U))
  {
    return 0U;
  }
  return 1U;
}

static float Measurement_Absolute(float value)
{
  return (value < 0.0F) ? -value : value;
}

static float Measurement_Maximum(float first, float second)
{
  return (first > second) ? first : second;
}

static float Measurement_SquareRoot(float value)
{
  float result = 0.0F;

  if (value > 0.0F)
  {
    (void)arm_sqrt_f32(value, &result);
  }
  return result;
}

static arm_status Measurement_FftInit(void)
{
  arm_status status;
  uint32_t index;

  if (measurement_fft_initialized != 0U)
  {
    return ARM_MATH_SUCCESS;
  }

  status = arm_rfft_fast_init_f32(&measurement_fft_instance,
                                  MEASUREMENT_FFT_LENGTH);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }

  measurement_fft_window_sum = 0.0F;
  for (index = 0U; index < MEASUREMENT_FFT_LENGTH; ++index)
  {
    float window = 0.5F - 0.5F * arm_cos_f32(
        (MEASUREMENT_TWO_PI * (float)index) /
        (float)(MEASUREMENT_FFT_LENGTH - 1U));
    measurement_fft_window[index] = window;
    measurement_fft_window_sum += window;
  }
  measurement_fft_initialized = 1U;
  return ARM_MATH_SUCCESS;
}

static void Measurement_FilterAndDecimate(const MeasurementInputFrame *input)
{
  uint64_t raw_sum = 0U;
  uint32_t output_index;
  uint32_t raw_index;

  for (raw_index = 0U; raw_index < MEASUREMENT_RAW_SAMPLE_COUNT; ++raw_index)
  {
    raw_sum += Measurement_GetCode(input, raw_index);
  }
  measurement_raw_mean =
      (float)raw_sum / (float)MEASUREMENT_RAW_SAMPLE_COUNT;

  /* A centred circular convolution avoids a start-up transient. The FFT Hann
   * window suppresses the small discontinuity possible at the frame wrap. */
  for (output_index = 0U;
       output_index < MEASUREMENT_FFT_LENGTH;
       ++output_index)
  {
    uint32_t centre_index = output_index * MEASUREMENT_DECIMATION_FACTOR;
    float filtered = 0.0F;
    uint32_t tap;

    for (tap = 0U; tap < MEASUREMENT_FILTER_TAP_COUNT; ++tap)
    {
      uint32_t sample_index =
          (centre_index + tap + MEASUREMENT_RAW_SAMPLE_COUNT -
           MEASUREMENT_FILTER_HALF_LENGTH) & MEASUREMENT_RAW_INDEX_MASK;
       filtered += measurement_filter_taps[tap] *
                   ((float)Measurement_GetCode(input, sample_index) -
                    measurement_raw_mean);
    }
    measurement_filtered_signal[output_index] = filtered;
  }
}

static arm_status Measurement_RunFft(const MeasurementInputFrame *input)
{
  uint32_t index;

  if (Measurement_FftInit() != ARM_MATH_SUCCESS)
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }

  Measurement_FilterAndDecimate(input);
  for (index = 0U; index < MEASUREMENT_FFT_LENGTH; ++index)
  {
    measurement_fft_input[index] =
        measurement_filtered_signal[index] * measurement_fft_window[index];
  }
  arm_rfft_fast_f32(&measurement_fft_instance, measurement_fft_input,
                    measurement_fft_output, 0U);
  return ARM_MATH_SUCCESS;
}

static float Measurement_GetMagnitudeSquared(uint32_t bin)
{
  float real = measurement_fft_output[2U * bin];
  float imaginary = measurement_fft_output[(2U * bin) + 1U];
  return (real * real) + (imaginary * imaginary);
}

static float Measurement_RefinePeakBin(uint32_t peak_bin)
{
  float left = Measurement_SquareRoot(
      Measurement_GetMagnitudeSquared(peak_bin - 1U));
  float centre = Measurement_SquareRoot(
      Measurement_GetMagnitudeSquared(peak_bin));
  float right = Measurement_SquareRoot(
      Measurement_GetMagnitudeSquared(peak_bin + 1U));
  float denominator = left + (2.0F * centre) + right;
  float refined_bin = (float)peak_bin;

  /* Three-bin Hann estimator. Unlike a generic parabola this compensates the
   * Hann main-lobe shape and remains accurate close to a half-bin offset. */
  if (denominator > 0.000001F)
  {
    float offset = 2.0F * (right - left) / denominator;

    if (offset > 0.5F)
    {
      offset = 0.5F;
    }
    else if (offset < -0.5F)
    {
      offset = -0.5F;
    }
    refined_bin += offset;
  }
  return refined_bin;
}

/* Estimate a raw (unfiltered) sine peak at an interpolated FFT bin.  The raw
 * centred samples are temporarily held in measurement_filtered_signal so the
 * existing three 4096-float work buffers can be reused without increasing
 * CCM consumption. */
static void Measurement_EstimateRawToneCode(float refined_bin,
                                             float *cosine_code,
                                             float *sine_code)
{
  float omega = (MEASUREMENT_TWO_PI * refined_bin) /
                (float)MEASUREMENT_FFT_LENGTH;
  float cosine_step = arm_cos_f32(omega);
  float sine_step = arm_sin_f32(omega);
  float cosine = 1.0F;
  float sine = 0.0F;
  float real_sum = 0.0F;
  float imaginary_sum = 0.0F;
  uint32_t index;

  for (index = 0U; index < MEASUREMENT_FFT_LENGTH; ++index)
  {
    float weighted = measurement_filtered_signal[index] *
                     measurement_fft_window[index];
    float next_cosine;

    real_sum += weighted * cosine;
    imaginary_sum -= weighted * sine;
    next_cosine = (cosine * cosine_step) - (sine * sine_step);
    sine = (sine * cosine_step) + (cosine * sine_step);
    cosine = next_cosine;

    if ((index & 0xFFU) == 0xFFU)
    {
      float norm = Measurement_SquareRoot((cosine * cosine) + (sine * sine));
      if (norm > 0.0F)
      {
        cosine /= norm;
        sine /= norm;
      }
    }
  }

  /* imaginary_sum deliberately has the same sign convention as
   * Measurement_EstimateTone(): synthesis is cosine*cos - sine*sin. */
  *cosine_code = (2.0F / measurement_fft_window_sum) * real_sum;
  *sine_code = (2.0F / measurement_fft_window_sum) * imaginary_sum;
}

static float Measurement_FilterGain(float decimated_bin)
{
  float raw_omega = (MEASUREMENT_TWO_PI * decimated_bin) /
                    ((float)MEASUREMENT_FFT_LENGTH *
                     (float)MEASUREMENT_DECIMATION_FACTOR);
  float gain = measurement_filter_taps[MEASUREMENT_FILTER_HALF_LENGTH];
  uint32_t distance;

  for (distance = 1U;
       distance <= MEASUREMENT_FILTER_HALF_LENGTH;
       ++distance)
  {
    gain += 2.0F *
            measurement_filter_taps[MEASUREMENT_FILTER_HALF_LENGTH + distance] *
            arm_cos_f32(raw_omega * (float)distance);
  }
  return gain;
}

static void Measurement_EstimateTone(float refined_bin,
                                     MeasurementTone *tone)
{
  float omega = (MEASUREMENT_TWO_PI * refined_bin) /
                (float)MEASUREMENT_FFT_LENGTH;
  float cosine_step = arm_cos_f32(omega);
  float sine_step = arm_sin_f32(omega);
  float cosine = 1.0F;
  float sine = 0.0F;
  float real_sum = 0.0F;
  float imaginary_sum = 0.0F;
  float gain = Measurement_FilterGain(refined_bin);
  float scale;
  uint32_t index;

  for (index = 0U; index < MEASUREMENT_FFT_LENGTH; ++index)
  {
    float weighted = measurement_filtered_signal[index] *
                     measurement_fft_window[index];
    float next_cosine;

    real_sum += weighted * cosine;
    imaginary_sum -= weighted * sine;
    next_cosine = (cosine * cosine_step) - (sine * sine_step);
    sine = (sine * cosine_step) + (cosine * sine_step);
    cosine = next_cosine;

    /* Limit accumulated round-off in the oscillator recurrence. */
    if ((index & 0xFFU) == 0xFFU)
    {
      float norm = Measurement_SquareRoot((cosine * cosine) + (sine * sine));
      if (norm > 0.0F)
      {
        cosine /= norm;
        sine /= norm;
      }
    }
  }

  if (Measurement_Absolute(gain) < 0.01F)
  {
    gain = 1.0F;
  }
  scale = 2.0F / (measurement_fft_window_sum * gain);
  tone->refined_bin = refined_bin;
  tone->cosine_code = real_sum * scale;
  tone->sine_code = imaginary_sum * scale;
  tone->amplitude_code = Measurement_SquareRoot(
      (tone->cosine_code * tone->cosine_code) +
      (tone->sine_code * tone->sine_code));
}

static uint8_t Measurement_BinIsExcluded(
    uint32_t bin,
    const MeasurementPeakCandidate *candidates,
    uint32_t candidate_count)
{
  uint32_t candidate;

  for (candidate = 0U; candidate < candidate_count; ++candidate)
  {
    uint32_t selected_bin = candidates[candidate].peak_bin;
    uint32_t distance = (bin > selected_bin) ?
        (bin - selected_bin) : (selected_bin - bin);

    if (distance <= MEASUREMENT_PEAK_EXCLUSION_BINS)
    {
      return 1U;
    }
  }
  return 0U;
}

static uint32_t Measurement_FindPeakCandidates(
    const MeasurementInputFrame *input,
    MeasurementPeakCandidate *candidates)
{
  float bins_per_hz =
      ((float)MEASUREMENT_FFT_LENGTH *
       (float)MEASUREMENT_DECIMATION_FACTOR) /
      (float)input->sample_rate_hz;
  uint32_t minimum_bin =
      (uint32_t)(MEASUREMENT_MIN_FREQUENCY_HZ * bins_per_hz);
  uint32_t maximum_bin =
      (uint32_t)(MEASUREMENT_SEARCH_MAX_FREQUENCY_HZ * bins_per_hz) + 1U;
  uint32_t candidate_count = 0U;
  uint32_t selection;

  if (minimum_bin < 2U)
  {
    minimum_bin = 2U;
  }
  if (maximum_bin >= (MEASUREMENT_FFT_HALF_LENGTH - 1U))
  {
    maximum_bin = MEASUREMENT_FFT_HALF_LENGTH - 2U;
  }

  for (selection = 0U;
       selection < MEASUREMENT_PEAK_CANDIDATE_COUNT;
       ++selection)
  {
    uint32_t best_bin = 0U;
    float best_magnitude_squared = 0.0F;
    uint32_t bin;

    for (bin = minimum_bin; bin <= maximum_bin; ++bin)
    {
      float magnitude_squared;

      if (Measurement_BinIsExcluded(bin, candidates, candidate_count) != 0U)
      {
        continue;
      }
      magnitude_squared = Measurement_GetMagnitudeSquared(bin);
      if ((magnitude_squared < Measurement_GetMagnitudeSquared(bin - 1U)) ||
          (magnitude_squared < Measurement_GetMagnitudeSquared(bin + 1U)))
      {
        continue;
      }
      if (magnitude_squared > best_magnitude_squared)
      {
        best_magnitude_squared = magnitude_squared;
        best_bin = bin;
      }
    }

    if (best_bin == 0U)
    {
      break;
    }
    candidates[candidate_count].peak_bin = best_bin;
    candidates[candidate_count].magnitude_squared = best_magnitude_squared;
    candidates[candidate_count].tone.magnitude_squared = best_magnitude_squared;
    Measurement_EstimateTone(Measurement_RefinePeakBin(best_bin),
                             &candidates[candidate_count].tone);
    candidate_count++;
  }
  return candidate_count;
}

static void Measurement_SortTonesByOrder(MeasurementTone *tones,
                                         uint32_t tone_count)
{
  uint32_t index;

  for (index = 1U; index < tone_count; ++index)
  {
    MeasurementTone value = tones[index];
    uint32_t position = index;

    while ((position > 0U) &&
           (tones[position - 1U].harmonic_order > value.harmonic_order))
    {
      tones[position] = tones[position - 1U];
      position--;
    }
    tones[position] = value;
  }
}

static uint32_t Measurement_SelectFamilyForBase(
    const MeasurementPeakCandidate *candidates,
    uint32_t candidate_count,
    uint32_t base_candidate,
    float minimum_amplitude_code,
    MeasurementTone *family,
    float *energy)
{
  uint32_t selected_candidate[MEASUREMENT_MAX_COMPONENT_COUNT - 1U] = {0U};
  float selected_energy[MEASUREMENT_MAX_COMPONENT_COUNT - 1U] = {0.0F};
  uint32_t selected_order[MEASUREMENT_MAX_COMPONENT_COUNT - 1U] = {0U};
  float base_bin = candidates[base_candidate].tone.refined_bin;
  uint32_t selected_count = 0U;
  uint32_t candidate;

  *energy = candidates[base_candidate].tone.amplitude_code *
            candidates[base_candidate].tone.amplitude_code;
  family[0] = candidates[base_candidate].tone;
  family[0].harmonic_order = 1U;

  for (candidate = 0U; candidate < candidate_count; ++candidate)
  {
    float ratio;
    float candidate_energy;
    uint32_t order;
    uint32_t slot;

    if ((candidate == base_candidate) ||
        (candidates[candidate].tone.amplitude_code < minimum_amplitude_code) ||
        (candidates[candidate].tone.refined_bin <= base_bin))
    {
      continue;
    }

    ratio = candidates[candidate].tone.refined_bin / base_bin;
    order = (uint32_t)(ratio + 0.5F);
    if ((order < 2U) || (order > MEASUREMENT_MAX_HARMONIC_ORDER) ||
        (Measurement_Absolute(candidates[candidate].tone.refined_bin -
                              ((float)order * base_bin)) >
         MEASUREMENT_HARMONIC_TOLERANCE_BIN))
    {
      continue;
    }

    candidate_energy = candidates[candidate].tone.amplitude_code *
                       candidates[candidate].tone.amplitude_code;
    for (slot = 0U; slot < selected_count; ++slot)
    {
      if (selected_order[slot] == order)
      {
        if (candidate_energy > selected_energy[slot])
        {
          selected_candidate[slot] = candidate;
          selected_energy[slot] = candidate_energy;
        }
        break;
      }
    }
    if (slot < selected_count)
    {
      continue;
    }

    if (selected_count < (MEASUREMENT_MAX_COMPONENT_COUNT - 1U))
    {
      selected_candidate[selected_count] = candidate;
      selected_energy[selected_count] = candidate_energy;
      selected_order[selected_count] = order;
      selected_count++;
    }
    else
    {
      uint32_t weakest = 0U;
      for (slot = 1U; slot < selected_count; ++slot)
      {
        if (selected_energy[slot] < selected_energy[weakest])
        {
          weakest = slot;
        }
      }
      if (candidate_energy > selected_energy[weakest])
      {
        selected_candidate[weakest] = candidate;
        selected_energy[weakest] = candidate_energy;
        selected_order[weakest] = order;
      }
    }
  }

  for (candidate = 0U; candidate < selected_count; ++candidate)
  {
    family[candidate + 1U] = candidates[selected_candidate[candidate]].tone;
    family[candidate + 1U].harmonic_order = selected_order[candidate];
    *energy += selected_energy[candidate];
  }
  Measurement_SortTonesByOrder(family, selected_count + 1U);
  return selected_count + 1U;
}

static MeasurementStatus Measurement_Analyze(
    const MeasurementInputFrame *input,
    float preferred_fundamental_hz,
    MeasurementAnalysis *analysis)
{
  float minimum_amplitude_code;
  float best_energy = 0.0F;
  uint32_t best_count = 0U;
  uint32_t candidate_count;
  uint32_t base_candidate;
  uint32_t tone_index;

  if ((Measurement_InputIsValid(input) == 0U) || (analysis == NULL))
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(analysis, 0, sizeof(*analysis));
  (void)memset(measurement_peak_candidates, 0,
               sizeof(measurement_peak_candidates));

  if (Measurement_RunFft(input) != ARM_MATH_SUCCESS)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  candidate_count = Measurement_FindPeakCandidates(
      input, measurement_peak_candidates);
  minimum_amplitude_code =
      (MEASUREMENT_MIN_COMPONENT_MV * MEASUREMENT_FRONT_END_GAIN *
       (float)input->adc_full_scale_code) /
      (float)input->adc_reference_mv;

  /* Waveform rendering supplies the temporally confirmed frequency. Prefer
   * the matching spectral peak so a short-lived subharmonic candidate cannot
   * turn one displayed period into many periods. Other tasks pass zero and
   * retain the unrestricted harmonic-family search. */
  if (preferred_fundamental_hz > 0.0F)
  {
    float preferred_bin =
        (preferred_fundamental_hz * (float)MEASUREMENT_DECIMATION_FACTOR *
         (float)MEASUREMENT_FFT_LENGTH) / (float)input->sample_rate_hz;
    float closest_distance = FLT_MAX;
    uint32_t preferred_candidate = candidate_count;

    for (base_candidate = 0U;
         base_candidate < candidate_count;
         ++base_candidate)
    {
      float distance = Measurement_Absolute(
          measurement_peak_candidates[base_candidate].tone.refined_bin -
          preferred_bin);

      if ((measurement_peak_candidates[base_candidate].tone.amplitude_code >=
           minimum_amplitude_code) &&
          (distance <= MEASUREMENT_FREQUENCY_HINT_TOLERANCE_BIN) &&
          (distance < closest_distance))
      {
        closest_distance = distance;
        preferred_candidate = base_candidate;
      }
    }
    if (preferred_candidate < candidate_count)
    {
      MeasurementTone family[MEASUREMENT_MAX_COMPONENT_COUNT];
      float energy;

      best_count = Measurement_SelectFamilyForBase(
          measurement_peak_candidates, candidate_count, preferred_candidate,
          minimum_amplitude_code, family, &energy);
      best_energy = energy;
      (void)memcpy(measurement_best_family, family,
                   best_count * sizeof(measurement_best_family[0]));
    }
  }

  if (best_count == 0U)
  {
    for (base_candidate = 0U;
         base_candidate < candidate_count;
         ++base_candidate)
    {
      MeasurementTone family[MEASUREMENT_MAX_COMPONENT_COUNT];
      float energy;
      uint32_t family_count;

      if (measurement_peak_candidates[base_candidate].tone.amplitude_code <
          minimum_amplitude_code)
      {
        continue;
      }
      family_count = Measurement_SelectFamilyForBase(
          measurement_peak_candidates, candidate_count, base_candidate,
          minimum_amplitude_code, family, &energy);
      if ((family_count >= 2U) &&
          ((family_count > best_count) ||
           ((family_count == best_count) && (energy > best_energy))))
      {
        best_energy = energy;
        best_count = family_count;
        (void)memcpy(measurement_best_family, family,
                     family_count * sizeof(measurement_best_family[0]));
      }
    }
  }

  /* The official waveform contains two or three lines, but accepting one
   * valid tone makes generator/ADC bring-up possible without weakening the
   * harmonic-family choice used for the scored composite signals. */
  if (best_count == 0U)
  {
    for (base_candidate = 0U;
         base_candidate < candidate_count;
         ++base_candidate)
    {
      float amplitude =
          measurement_peak_candidates[base_candidate].tone.amplitude_code;
      float energy = amplitude * amplitude;

      if ((amplitude >= minimum_amplitude_code) && (energy > best_energy))
      {
        best_energy = energy;
        best_count = 1U;
        measurement_best_family[0] =
            measurement_peak_candidates[base_candidate].tone;
        measurement_best_family[0].harmonic_order = 1U;
      }
    }
  }
  if (best_count == 0U)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  /* All spectral lines are integer harmonics. A weighted common-base fit uses
   * a strong high-order harmonic to improve the frequency of a weak base. */
  {
    float numerator = 0.0F;
    float denominator = 0.0F;

    for (tone_index = 0U; tone_index < best_count; ++tone_index)
    {
      float weight = measurement_best_family[tone_index].amplitude_code *
                     measurement_best_family[tone_index].amplitude_code;
      float order =
          (float)measurement_best_family[tone_index].harmonic_order;
      numerator += weight * order *
                   measurement_best_family[tone_index].refined_bin;
      denominator += weight * order * order;
    }
    if (denominator <= 0.0F)
    {
      return MEASUREMENT_STATUS_INVALID_ARGUMENT;
    }
    analysis->fundamental_bin = numerator / denominator;
  }

  analysis->tone_count = best_count;
  for (tone_index = 0U; tone_index < best_count; ++tone_index)
  {
    uint32_t order = measurement_best_family[tone_index].harmonic_order;
    Measurement_EstimateTone(analysis->fundamental_bin * (float)order,
                             &analysis->tones[tone_index]);
    analysis->tones[tone_index].harmonic_order = order;
  }
  Measurement_SortTonesByOrder(analysis->tones, analysis->tone_count);

  {
    float measured_hz =
        (analysis->fundamental_bin * (float)input->sample_rate_hz) /
        ((float)MEASUREMENT_DECIMATION_FACTOR *
         (float)MEASUREMENT_FFT_LENGTH);
    analysis->fundamental_hz =
        (float)((uint32_t)((measured_hz / MEASUREMENT_FREQUENCY_GRID_HZ) +
                          0.5F)) *
        MEASUREMENT_FREQUENCY_GRID_HZ;
  }
  return MEASUREMENT_STATUS_OK;
}

static float Measurement_Synthesize(const MeasurementAnalysis *analysis,
                                    float fundamental_phase)
{
  float value = 0.0F;
  uint32_t tone;

  for (tone = 0U; tone < analysis->tone_count; ++tone)
  {
    float angle = fundamental_phase *
                  (float)analysis->tones[tone].harmonic_order;
    value += analysis->tones[tone].cosine_code * arm_cos_f32(angle) -
             analysis->tones[tone].sine_code * arm_sin_f32(angle);
  }
  return value;
}

static float Measurement_CalculateSynthesizedVpp(
    const MeasurementAnalysis *analysis)
{
  float minimum = FLT_MAX;
  float maximum = -FLT_MAX;
  uint32_t point;

  if ((analysis == NULL) || (analysis->tone_count == 0U))
  {
    return 0.0F;
  }

  /* The specified input contains no more than three sinusoidal components.
   * Reconstructing those detected lines rejects isolated ADC spikes while
   * preserving their relative phases, so it also gives the true Vpp of a
   * composite waveform.  A dense phase grid keeps the extremum error below
   * 1 mV even for the highest supported harmonic order. */
  for (point = 0U; point < MEASUREMENT_FFT_LENGTH; ++point)
  {
    float phase = (MEASUREMENT_TWO_PI * (float)point) /
                  (float)MEASUREMENT_FFT_LENGTH;
    float value = Measurement_Synthesize(analysis, phase);

    if (value < minimum)
    {
      minimum = value;
    }
    if (value > maximum)
    {
      maximum = value;
    }
  }
  return maximum - minimum;
}

static float Measurement_QuadraticExtremum(float left,
                                            float centre,
                                            float right,
                                            uint8_t maximum)
{
  float curvature = left - (2.0F * centre) + right;
  float offset;

  if (((maximum != 0U) &&
       ((centre < left) || (centre < right) || (curvature >= -0.000001F))) ||
      ((maximum == 0U) &&
       ((centre > left) || (centre > right) || (curvature <= 0.000001F))))
  {
    return centre;
  }
  offset = 0.5F * (left - right) / curvature;
  if (Measurement_Absolute(offset) > 0.5F)
  {
    return centre;
  }
  return centre + (0.25F * (right - left) * offset);
}

static float Measurement_SelectMedian(float *values, uint32_t count)
{
  int32_t left = 0;
  int32_t right = (int32_t)count - 1;
  int32_t target = (int32_t)(count / 2U);

  while (left < right)
  {
    float pivot = values[(left + right) / 2];
    int32_t lower = left;
    int32_t upper = right;

    while (lower <= upper)
    {
      while (values[lower] < pivot)
      {
        lower++;
      }
      while (values[upper] > pivot)
      {
        upper--;
      }
      if (lower <= upper)
      {
        float temporary = values[lower];
        values[lower] = values[upper];
        values[upper] = temporary;
        lower++;
        upper--;
      }
    }

    if (target <= upper)
    {
      right = upper;
    }
    else if (target >= lower)
    {
      left = lower;
    }
    else
    {
      break;
    }
  }
  return values[target];
}

static MeasurementStatus Measurement_CalculateTimeDomainMetrics(
    const MeasurementInputFrame *input,
    float fundamental_bin,
    float *vpp_code,
    float *rms_code)
{
  float samples_per_period;
  float filtered_sum = 0.0F;
  float square_sum = 0.0F;
  float previous = 0.0F;
  float period_maximum = -FLT_MAX;
  float period_maximum_left = 0.0F;
  float period_maximum_right = 0.0F;
  float period_minimum = FLT_MAX;
  float period_minimum_left = 0.0F;
  float period_minimum_right = 0.0F;
  uint32_t filtered_count = 0U;
  uint32_t previous_index = 0U;
  uint32_t period_maximum_index = 0U;
  uint32_t period_minimum_index = 0U;
  uint32_t period_index = 0U;
  uint32_t period_vpp_count = 0U;
  uint8_t previous_valid = 0U;
  uint8_t period_has_maximum = 0U;
  uint8_t period_has_minimum = 0U;
  uint32_t centre_index;

  if ((input == NULL) || (vpp_code == NULL) || (rms_code == NULL) ||
      (fundamental_bin <= 0.0F))
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  samples_per_period =
      ((float)MEASUREMENT_FFT_LENGTH *
       (float)MEASUREMENT_DECIMATION_FACTOR) / fundamental_bin;
  if (samples_per_period < 4.0F)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  /* Re-run the anti-interference FIR at the full ADC rate. RMS is then a
   * direct true-RMS measurement rather than the magnitude of one coherent FFT
   * line. One interpolated peak-to-peak value is collected per fundamental
   * period; their median rejects ADC spikes without losing composite-waveform
   * peaks. The edge samples are omitted to avoid a circular-frame seam. */
  for (centre_index = MEASUREMENT_FILTER_HALF_LENGTH;
       centre_index <
           (MEASUREMENT_RAW_SAMPLE_COUNT - MEASUREMENT_FILTER_HALF_LENGTH);
       ++centre_index)
  {
    float filtered = 0.0F;
    uint32_t tap;

    for (tap = 0U; tap < MEASUREMENT_FILTER_TAP_COUNT; ++tap)
    {
      uint32_t sample_index =
          centre_index + tap - MEASUREMENT_FILTER_HALF_LENGTH;
      filtered += measurement_filter_taps[tap] *
                  ((float)Measurement_GetCode(input, sample_index) -
                   measurement_raw_mean);
    }
    filtered_sum += filtered;
    square_sum += filtered * filtered;
    filtered_count++;

    /* The current sample supplies the right neighbour when the preceding
     * sample was a period's discrete maximum or minimum. */
    if (previous_valid != 0U)
    {
      if (period_maximum_index == previous_index)
      {
        period_maximum_right = filtered;
      }
      if (period_minimum_index == previous_index)
      {
        period_minimum_right = filtered;
      }
    }

    /* Recompute each boundary from its period number so float round-off does
     * not accumulate over roughly two thousand 500 kHz periods. */
    while ((float)centre_index >=
           ((float)MEASUREMENT_FILTER_HALF_LENGTH +
            ((float)(period_index + 1U) * samples_per_period)))
    {
      if ((period_index > 0U) &&
          (period_has_maximum != 0U) &&
          (period_has_minimum != 0U) &&
          (period_vpp_count < MEASUREMENT_FFT_LENGTH))
      {
        float maximum = Measurement_QuadraticExtremum(
            period_maximum_left, period_maximum, period_maximum_right, 1U);
        float minimum = Measurement_QuadraticExtremum(
            period_minimum_left, period_minimum, period_minimum_right, 0U);
        measurement_fft_input[period_vpp_count] = maximum - minimum;
        period_vpp_count++;
      }
      period_index++;
      period_maximum = -FLT_MAX;
      period_minimum = FLT_MAX;
      period_has_maximum = 0U;
      period_has_minimum = 0U;
    }

    if ((period_has_maximum == 0U) || (filtered > period_maximum))
    {
      period_maximum = filtered;
      period_maximum_left =
          (previous_valid != 0U) ? previous : filtered;
      period_maximum_right = filtered;
      period_maximum_index = centre_index;
      period_has_maximum = 1U;
    }
    if ((period_has_minimum == 0U) || (filtered < period_minimum))
    {
      period_minimum = filtered;
      period_minimum_left =
          (previous_valid != 0U) ? previous : filtered;
      period_minimum_right = filtered;
      period_minimum_index = centre_index;
      period_has_minimum = 1U;
    }

    previous = filtered;
    previous_index = centre_index;
    previous_valid = 1U;
  }

  if ((filtered_count == 0U) || (period_vpp_count < 3U))
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  {
    float filtered_mean = filtered_sum / (float)filtered_count;
    float variance = (square_sum / (float)filtered_count) -
                     (filtered_mean * filtered_mean);
    *rms_code = Measurement_SquareRoot(variance);
  }
  *vpp_code = Measurement_SelectMedian(measurement_fft_input,
                                       period_vpp_count);
  return MEASUREMENT_STATUS_OK;
}

MeasurementStatus Measurement_Task1_PrepareWaveform(
    const MeasurementInputFrame *input,
    float fundamental_hz,
    MeasurementPeriodCount period_count,
    MeasurementWaveformResult *result)
{
  MeasurementAnalysis analysis;
  MeasurementStatus status;
  float minimum = FLT_MAX;
  float maximum = -FLT_MAX;
  uint32_t point;

  if ((result == NULL) || (fundamental_hz <= 0.0F) ||
      ((period_count != MEASUREMENT_SHOW_ONE_PERIOD) &&
       (period_count != MEASUREMENT_SHOW_THREE_PERIODS)))
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result, 0, sizeof(*result));

  status = Measurement_Analyze(input, fundamental_hz, &analysis);
  if (status != MEASUREMENT_STATUS_OK)
  {
    return status;
  }

  /* Synthesis from the detected low-frequency components removes u_J and
   * gives an exact one- or three-period boundary even near 500 kHz. */
  for (point = 0U; point < MEASUREMENT_WAVEFORM_POINT_COUNT; ++point)
  {
    float phase =
        (MEASUREMENT_TWO_PI * (float)period_count * (float)point) /
        (float)(MEASUREMENT_WAVEFORM_POINT_COUNT - 1U);
    float value = Measurement_Synthesize(&analysis, phase);

    measurement_fft_input[point] = value;
    if (value < minimum)
    {
      minimum = value;
    }
    if (value > maximum)
    {
      maximum = value;
    }
  }

  result->period_count = period_count;
  result->point_count = MEASUREMENT_WAVEFORM_POINT_COUNT;
  if ((maximum - minimum) <= 0.000001F)
  {
    (void)memset(result->points, 128, sizeof(result->points));
    return MEASUREMENT_STATUS_OK;
  }

  for (point = 0U; point < MEASUREMENT_WAVEFORM_POINT_COUNT; ++point)
  {
    float scaled = ((measurement_fft_input[point] - minimum) * 255.0F) /
                   (maximum - minimum);
    if (scaled <= 0.0F)
    {
      result->points[point] = 0U;
    }
    else if (scaled >= 255.0F)
    {
      result->points[point] = 255U;
    }
    else
    {
      result->points[point] = (uint8_t)(scaled + 0.5F);
    }
  }
  return MEASUREMENT_STATUS_OK;
}

MeasurementStatus Measurement_Task2_MeasureParameters(
    const MeasurementInputFrame *input,
    MeasurementParameterResult *result)
{
  MeasurementAnalysis analysis;
  MeasurementStatus status;
  float vpp_code;
  float rms_code;
  float code_to_mv;

  if (result == NULL)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result, 0, sizeof(*result));
  status = Measurement_Analyze(input, 0.0F, &analysis);
  if (status != MEASUREMENT_STATUS_OK)
  {
    return status;
  }

  status = Measurement_CalculateTimeDomainMetrics(
      input, analysis.fundamental_bin, &vpp_code, &rms_code);
  if (status != MEASUREMENT_STATUS_OK)
  {
    return status;
  }
  /* Absolute per-period extrema become increasingly noise-biased at low
   * frequencies because each period contains more ADC samples.  For a
   * composite input, reconstruct the detected lines at every frequency so a
   * rejected u_J component cannot move the VPP extrema.  A single tone near
   * the 500 kHz search boundary keeps the time-domain estimator, which is more
   * stable there than its phase reconstruction.  RMS is always the filtered
   * true-RMS. */
  if ((analysis.tone_count >= 2U) ||
      (analysis.fundamental_hz <=
       MEASUREMENT_SYNTHESIZED_VPP_MAX_FREQUENCY_HZ))
  {
    float synthesized_vpp_code =
        Measurement_CalculateSynthesizedVpp(&analysis);
    if (synthesized_vpp_code > 0.0F)
    {
      vpp_code = synthesized_vpp_code;
    }
  }
  code_to_mv = (float)input->adc_reference_mv /
               ((float)input->adc_full_scale_code *
                MEASUREMENT_FRONT_END_GAIN);
  result->vpp_mv = vpp_code * code_to_mv;
  result->rms_mv = rms_code * code_to_mv;
  result->fundamental_hz = analysis.fundamental_hz;
  return MEASUREMENT_STATUS_OK;
}

MeasurementStatus Measurement_Task3_PrepareSpectrum(
    const MeasurementInputFrame *input,
    MeasurementSpectrumResult *result)
{
  MeasurementAnalysis analysis;
  MeasurementStatus status;
  float maximum_amplitude = 0.0F;
  float span_hz;
  uint32_t tone;

  if ((result == NULL) || (Measurement_InputIsValid(input) == 0U))
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result->bars, 0, sizeof(result->bars));
  result->bar_count = MEASUREMENT_WAVEFORM_POINT_COUNT;
  /* The HMI scale is marked from 0 to 500 kHz across 15 equal cells. */
  span_hz = MEASUREMENT_SPECTRUM_DISPLAY_SPAN_HZ;
  result->frequency_span_hz = span_hz;

  status = Measurement_Analyze(input, 0.0F, &analysis);
  if (status != MEASUREMENT_STATUS_OK)
  {
    return status;
  }
  for (tone = 0U; tone < analysis.tone_count; ++tone)
  {
    if (analysis.tones[tone].amplitude_code > maximum_amplitude)
    {
      maximum_amplitude = analysis.tones[tone].amplitude_code;
    }
  }
  if (maximum_amplitude <= 0.0F)
  {
    return MEASUREMENT_STATUS_OK;
  }

  /* bars[] contains isolated line heights, not a continuous spectrum curve. */
  for (tone = 0U; tone < analysis.tone_count; ++tone)
  {
    float frequency_hz = analysis.fundamental_hz *
                         (float)analysis.tones[tone].harmonic_order;
    uint32_t x = (uint32_t)(
        ((frequency_hz / span_hz) *
         (float)(MEASUREMENT_WAVEFORM_POINT_COUNT - 1U)) + 0.5F);
    uint32_t height = (uint32_t)(
        ((analysis.tones[tone].amplitude_code / maximum_amplitude) *
         MEASUREMENT_SPECTRUM_MAX_HEIGHT) + 0.5F);

    if (x >= MEASUREMENT_WAVEFORM_POINT_COUNT)
    {
      x = MEASUREMENT_WAVEFORM_POINT_COUNT - 1U;
    }
    if (height > 255U)
    {
      height = 255U;
    }
    {
      uint32_t first = (x > MEASUREMENT_SPECTRUM_LINE_HALF_WIDTH) ?
          (x - MEASUREMENT_SPECTRUM_LINE_HALF_WIDTH) : 0U;
      uint32_t last = x + MEASUREMENT_SPECTRUM_LINE_HALF_WIDTH;
      uint32_t line_x;

      if (last >= MEASUREMENT_WAVEFORM_POINT_COUNT)
      {
        last = MEASUREMENT_WAVEFORM_POINT_COUNT - 1U;
      }
      /* A one-sample impulse is drawn as a triangle by the HMI waveform
       * component. Three equal samples look like a narrow discrete bar. */
      for (line_x = first; line_x <= last; ++line_x)
      {
        if (result->bars[line_x] < (uint8_t)height)
        {
          result->bars[line_x] = (uint8_t)height;
        }
      }
    }
  }
  return MEASUREMENT_STATUS_OK;
}

MeasurementStatus Measurement_Task4_MeasureComponents(
    const MeasurementInputFrame *input,
    MeasurementComponentResult *result)
{
  MeasurementAnalysis analysis;
  MeasurementStatus status;
  float code_to_mv;
  uint32_t tone;

  if (result == NULL)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result, 0, sizeof(*result));
  status = Measurement_Analyze(input, 0.0F, &analysis);
  if (status != MEASUREMENT_STATUS_OK)
  {
    return status;
  }

  code_to_mv = (float)input->adc_reference_mv /
               ((float)input->adc_full_scale_code *
                MEASUREMENT_FRONT_END_GAIN);
  result->component_count = analysis.tone_count;
  for (tone = 0U; tone < analysis.tone_count; ++tone)
  {
    result->components[tone].frequency_hz =
        analysis.fundamental_hz *
        (float)analysis.tones[tone].harmonic_order;
    /* Frequency reporting uses the common harmonic-family fit above, but
     * amplitude must be evaluated at each line's own refined FFT peak. A tiny
     * base-fit error is multiplied by the harmonic order and otherwise makes
     * the 450 kHz line fluctuate even when the source amplitude is steady. */
    result->components[tone].amplitude_mv =
        measurement_best_family[tone].amplitude_code * code_to_mv;
  }
  return MEASUREMENT_STATUS_OK;
}

static uint8_t Measurement_RawPeakBinIsExcluded(
    uint32_t bin,
    const uint32_t *selected_bins,
    uint32_t selected_count)
{
  uint32_t selected;

  for (selected = 0U; selected < selected_count; ++selected)
  {
    uint32_t selected_bin = selected_bins[selected];
    uint32_t distance = (bin > selected_bin) ?
        (bin - selected_bin) : (selected_bin - bin);

    if (distance <= MEASUREMENT_ALIAS_PEAK_EXCLUSION_BINS)
    {
      return 1U;
    }
  }
  return 0U;
}

static void Measurement_SortSignatureByFrequency(
    MeasurementRawLowBandSignature *signature)
{
  uint32_t index;

  for (index = 1U; index < signature->peak_count; ++index)
  {
    MeasurementRawSpectrumPeak value = signature->peaks[index];
    uint32_t position = index;

    while ((position > 0U) &&
           (signature->peaks[position - 1U].frequency_hz >
            value.frequency_hz))
    {
      signature->peaks[position] = signature->peaks[position - 1U];
      position--;
    }
    signature->peaks[position] = value;
  }
}

MeasurementStatus Measurement_ExtractRawLowBandSignature(
    const MeasurementInputFrame *input,
    MeasurementRawLowBandSignature *signature)
{
  uint32_t selected_bins[MEASUREMENT_ALIAS_MAX_PEAK_COUNT] = {0U};
  uint64_t raw_sum = 0U;
  float raw_mean;
  float bins_per_hz;
  float code_to_mv;
  uint32_t minimum_bin;
  uint32_t maximum_bin;
  uint32_t index;
  uint32_t selection;

  if (signature == NULL)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(signature, 0, sizeof(*signature));
  if ((input == NULL) || (input->samples == NULL) ||
      (input->sample_count < MEASUREMENT_FFT_LENGTH) ||
      (input->sample_rate_hz == 0U) || (input->adc_reference_mv == 0U) ||
      (input->adc_full_scale_code == 0U))
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  if (Measurement_FftInit() != ARM_MATH_SUCCESS)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  /* A contiguous raw block avoids any 4:1 decimation alias.  Removing only
   * its DC mean prevents the bias point from leaking into the 10 kHz band. */
  for (index = 0U; index < MEASUREMENT_FFT_LENGTH; ++index)
  {
    raw_sum += Measurement_GetCode(input, index);
  }
  raw_mean = (float)raw_sum / (float)MEASUREMENT_FFT_LENGTH;
  for (index = 0U; index < MEASUREMENT_FFT_LENGTH; ++index)
  {
    float centred = (float)Measurement_GetCode(input, index) - raw_mean;

    measurement_filtered_signal[index] = centred;
    measurement_fft_input[index] = centred * measurement_fft_window[index];
  }
  arm_rfft_fast_f32(&measurement_fft_instance, measurement_fft_input,
                    measurement_fft_output, 0U);

  bins_per_hz = (float)MEASUREMENT_FFT_LENGTH /
                (float)input->sample_rate_hz;
  minimum_bin = (uint32_t)(
      MEASUREMENT_ALIAS_SCAN_MIN_FREQUENCY_HZ * bins_per_hz);
  maximum_bin =
      (uint32_t)(MEASUREMENT_ALIAS_SCAN_MAX_FREQUENCY_HZ * bins_per_hz) + 1U;
  if (minimum_bin < 2U)
  {
    minimum_bin = 2U;
  }
  if (maximum_bin >= (MEASUREMENT_FFT_HALF_LENGTH - 1U))
  {
    maximum_bin = MEASUREMENT_FFT_HALF_LENGTH - 2U;
  }
  if (minimum_bin > maximum_bin)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  signature->sample_rate_hz = input->sample_rate_hz;
  code_to_mv = (float)input->adc_reference_mv /
               ((float)input->adc_full_scale_code *
                MEASUREMENT_FRONT_END_GAIN);

  /* Select strongest separated local maxima.  A Hann main lobe spans about
   * four bins, so excluding three bins avoids reporting its skirt twice. */
  for (selection = 0U;
       selection < MEASUREMENT_ALIAS_MAX_PEAK_COUNT;
       ++selection)
  {
    uint32_t best_bin = 0U;
    float best_magnitude_squared = 0.0F;
    uint32_t bin;

    for (bin = minimum_bin; bin <= maximum_bin; ++bin)
    {
      float magnitude_squared;

      if (Measurement_RawPeakBinIsExcluded(
              bin, selected_bins, selection) != 0U)
      {
        continue;
      }
      magnitude_squared = Measurement_GetMagnitudeSquared(bin);
      if ((magnitude_squared < Measurement_GetMagnitudeSquared(bin - 1U)) ||
          (magnitude_squared < Measurement_GetMagnitudeSquared(bin + 1U)))
      {
        continue;
      }
      if (magnitude_squared > best_magnitude_squared)
      {
        best_magnitude_squared = magnitude_squared;
        best_bin = bin;
      }
    }

    if (best_bin == 0U)
    {
      break;
    }
    selected_bins[selection] = best_bin;
    {
      float refined_bin = Measurement_RefinePeakBin(best_bin);
      float frequency_hz =
          (refined_bin * (float)input->sample_rate_hz) /
          (float)MEASUREMENT_FFT_LENGTH;
      float cosine_code;
      float sine_code;
      float cosine_mv;
      float sine_mv;
      float amplitude_mv;

      Measurement_EstimateRawToneCode(refined_bin, &cosine_code, &sine_code);
      cosine_mv = cosine_code * code_to_mv;
      sine_mv = sine_code * code_to_mv;
      amplitude_mv = Measurement_SquareRoot(
          (cosine_mv * cosine_mv) + (sine_mv * sine_mv));

      /* Candidates are visited in FFT-magnitude order.  Hann scalloping is
       * small enough that once the best remaining line is below this gate,
       * later candidates cannot be a materially stronger low-band line. */
      if (amplitude_mv < MEASUREMENT_ALIAS_SIGNATURE_MIN_PEAK_MV)
      {
        break;
      }
      if ((frequency_hz < MEASUREMENT_ALIAS_SCAN_MIN_FREQUENCY_HZ) ||
          (frequency_hz > MEASUREMENT_ALIAS_SCAN_MAX_FREQUENCY_HZ))
      {
        continue;
      }

      signature->peaks[signature->peak_count].frequency_hz = frequency_hz;
      signature->peaks[signature->peak_count].amplitude_mv = amplitude_mv;
      signature->peaks[signature->peak_count].cosine_mv = cosine_mv;
      signature->peaks[signature->peak_count].sine_mv = sine_mv;
      signature->peak_count++;
    }
  }

  Measurement_SortSignatureByFrequency(signature);
  return MEASUREMENT_STATUS_OK;
}

MeasurementStatus Measurement_CompareRawLowBandSignatures(
    const MeasurementRawLowBandSignature *normal_signature,
    const MeasurementRawLowBandSignature *probe_signature,
    MeasurementAliasComparisonResult *result)
{
  uint8_t normal_matched[MEASUREMENT_ALIAS_MAX_PEAK_COUNT] = {0U};
  uint8_t probe_matched[MEASUREMENT_ALIAS_MAX_PEAK_COUNT] = {0U};
  uint8_t normal_match_index[MEASUREMENT_ALIAS_MAX_PEAK_COUNT];
  uint32_t normal;
  uint32_t probe;

  if (result == NULL)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result, 0, sizeof(*result));
  (void)memset(normal_match_index, 0xFF, sizeof(normal_match_index));
  if ((normal_signature == NULL) || (probe_signature == NULL) ||
      (normal_signature->sample_rate_hz == 0U) ||
      (probe_signature->sample_rate_hz == 0U) ||
      (normal_signature->peak_count > MEASUREMENT_ALIAS_MAX_PEAK_COUNT) ||
      (probe_signature->peak_count > MEASUREMENT_ALIAS_MAX_PEAK_COUNT))
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  /* The normal measurement path decimates by four. Energy just above the
   * specified 500 kHz band can fold back below 500 kHz before the FIR reaches
   * full stop-band attenuation. A material raw line above the guard band is
   * therefore NG even if it happens to remain at the same frequency after the
   * rate change. */
  for (normal = 0U; normal < normal_signature->peak_count; ++normal)
  {
    float amplitude_mv = normal_signature->peaks[normal].amplitude_mv;

    if ((normal_signature->peaks[normal].frequency_hz >
         MEASUREMENT_ALIAS_VALID_BAND_MAX_HZ) &&
        (amplitude_mv >= MEASUREMENT_ALIAS_STRONG_PEAK_MV) &&
        (amplitude_mv > result->unmatched_amplitude_mv))
    {
      result->alias_detected = 1U;
      result->unmatched_source = MEASUREMENT_ALIAS_PEAK_NORMAL;
      result->unmatched_frequency_hz =
          normal_signature->peaks[normal].frequency_hz;
      result->unmatched_amplitude_mv = amplitude_mv;
    }
  }
  for (probe = 0U; probe < probe_signature->peak_count; ++probe)
  {
    float amplitude_mv = probe_signature->peaks[probe].amplitude_mv;

    if ((probe_signature->peaks[probe].frequency_hz >
         MEASUREMENT_ALIAS_VALID_BAND_MAX_HZ) &&
        (amplitude_mv >= MEASUREMENT_ALIAS_STRONG_PEAK_MV) &&
        (amplitude_mv > result->unmatched_amplitude_mv))
    {
      result->alias_detected = 1U;
      result->unmatched_source = MEASUREMENT_ALIAS_PEAK_PROBE;
      result->unmatched_frequency_hz =
          probe_signature->peaks[probe].frequency_hz;
      result->unmatched_amplitude_mv = amplitude_mv;
    }
  }

  /* Repeatedly take the best compatible one-to-one pair.  Frequency must be
   * stable under the rate change and amplitude must agree within the configured
   * absolute/relative tolerance. */
  for (;;)
  {
    float best_score = FLT_MAX;
    uint32_t best_normal = MEASUREMENT_ALIAS_MAX_PEAK_COUNT;
    uint32_t best_probe = MEASUREMENT_ALIAS_MAX_PEAK_COUNT;

    for (normal = 0U; normal < normal_signature->peak_count; ++normal)
    {
      if (normal_matched[normal] != 0U)
      {
        continue;
      }
      for (probe = 0U; probe < probe_signature->peak_count; ++probe)
      {
        float frequency_difference;
        float amplitude_difference;
        float amplitude_tolerance;
        float larger_amplitude;
        float score;

        if (probe_matched[probe] != 0U)
        {
          continue;
        }
        frequency_difference = Measurement_Absolute(
            normal_signature->peaks[normal].frequency_hz -
            probe_signature->peaks[probe].frequency_hz);
        if (frequency_difference >
            MEASUREMENT_ALIAS_FREQUENCY_TOLERANCE_HZ)
        {
          continue;
        }

        amplitude_difference = Measurement_Absolute(
            normal_signature->peaks[normal].amplitude_mv -
            probe_signature->peaks[probe].amplitude_mv);
        larger_amplitude = Measurement_Maximum(
            normal_signature->peaks[normal].amplitude_mv,
            probe_signature->peaks[probe].amplitude_mv);
        amplitude_tolerance = Measurement_Maximum(
            MEASUREMENT_ALIAS_AMPLITUDE_TOLERANCE_MV,
            MEASUREMENT_ALIAS_AMPLITUDE_TOLERANCE_RATIO * larger_amplitude);
        if (amplitude_difference > amplitude_tolerance)
        {
          continue;
        }

        score =
            (frequency_difference /
             MEASUREMENT_ALIAS_FREQUENCY_TOLERANCE_HZ) +
            (amplitude_difference / amplitude_tolerance);
        if (score < best_score)
        {
          best_score = score;
          best_normal = normal;
          best_probe = probe;
        }
      }
    }

    if (best_normal >= MEASUREMENT_ALIAS_MAX_PEAK_COUNT)
    {
      break;
    }
    normal_matched[best_normal] = 1U;
    probe_matched[best_probe] = 1U;
    normal_match_index[best_normal] = (uint8_t)best_probe;
  }

  /* The normal signature is frequency-sorted, hence walking it here also
   * makes the lowest retained stable line the fundamental.  A 510 kHz guard
   * is kept solely so a nominal 500 kHz line is not lost to clock tolerance.
   * Valid u_b contains at most three lines. */
  for (normal = 0U;
       (normal < normal_signature->peak_count) &&
       (result->stable_component_count < MEASUREMENT_MAX_COMPONENT_COUNT);
       ++normal)
  {
    uint32_t matched_probe = normal_match_index[normal];

    if ((matched_probe >= probe_signature->peak_count) ||
        (normal_signature->peaks[normal].frequency_hz >
         MEASUREMENT_ALIAS_VALID_BAND_MAX_HZ) ||
        (probe_signature->peaks[matched_probe].frequency_hz >
         MEASUREMENT_ALIAS_VALID_BAND_MAX_HZ))
    {
      continue;
    }
    {
      const MeasurementRawSpectrumPeak *normal_peak =
          &normal_signature->peaks[normal];
      const MeasurementRawSpectrumPeak *probe_peak =
          &probe_signature->peaks[matched_probe];
      MeasurementRecoveredComponent *stable =
          &result->stable_components[result->stable_component_count];
      float normal_vector_amplitude = Measurement_SquareRoot(
          (normal_peak->cosine_mv * normal_peak->cosine_mv) +
          (normal_peak->sine_mv * normal_peak->sine_mv));

      stable->frequency_hz =
          0.5F * (normal_peak->frequency_hz + probe_peak->frequency_hz);
      stable->amplitude_mv =
          0.5F * (normal_peak->amplitude_mv + probe_peak->amplitude_mv);
      if (normal_vector_amplitude > 0.000001F)
      {
        float amplitude_scale =
            stable->amplitude_mv / normal_vector_amplitude;

        stable->cosine_mv = normal_peak->cosine_mv * amplitude_scale;
        stable->sine_mv = normal_peak->sine_mv * amplitude_scale;
      }
      else
      {
        stable->cosine_mv = stable->amplitude_mv;
        stable->sine_mv = 0.0F;
      }
      result->stable_component_count++;
    }
  }
  if (result->stable_component_count > 0U)
  {
    float fundamental_hz = result->stable_components[0].frequency_hz;
    uint32_t component;

    for (component = 0U;
         component < result->stable_component_count;
         ++component)
    {
      uint32_t order = (uint32_t)(
          (result->stable_components[component].frequency_hz /
           fundamental_hz) + 0.5F);

      result->stable_components[component].harmonic_order =
          (order > 0U) ? order : 1U;
    }
    result->stable_components[0].harmonic_order = 1U;
  }

  /* Check both directions: a moving alias may enter or leave the 10-500 kHz
   * band at the probe rate.  Report the strongest unmatched material peak. */
  for (normal = 0U; normal < normal_signature->peak_count; ++normal)
  {
    float amplitude_mv = normal_signature->peaks[normal].amplitude_mv;

    if ((normal_matched[normal] == 0U) &&
        (amplitude_mv >= MEASUREMENT_ALIAS_STRONG_PEAK_MV) &&
        (amplitude_mv > result->unmatched_amplitude_mv))
    {
      result->alias_detected = 1U;
      result->unmatched_source = MEASUREMENT_ALIAS_PEAK_NORMAL;
      result->unmatched_frequency_hz =
          normal_signature->peaks[normal].frequency_hz;
      result->unmatched_amplitude_mv = amplitude_mv;
    }
  }
  for (probe = 0U; probe < probe_signature->peak_count; ++probe)
  {
    float amplitude_mv = probe_signature->peaks[probe].amplitude_mv;

    if ((probe_matched[probe] == 0U) &&
        (amplitude_mv >= MEASUREMENT_ALIAS_STRONG_PEAK_MV) &&
        (amplitude_mv > result->unmatched_amplitude_mv))
    {
      result->alias_detected = 1U;
      result->unmatched_source = MEASUREMENT_ALIAS_PEAK_PROBE;
      result->unmatched_frequency_hz =
          probe_signature->peaks[probe].frequency_hz;
      result->unmatched_amplitude_mv = amplitude_mv;
    }
  }
  return MEASUREMENT_STATUS_OK;
}

static uint8_t Measurement_RecoveryIsValid(
    const MeasurementAliasComparisonResult *comparison)
{
  uint32_t component;

  if ((comparison == NULL) || (comparison->stable_component_count == 0U) ||
      (comparison->stable_component_count > MEASUREMENT_MAX_COMPONENT_COUNT))
  {
    return 0U;
  }
  for (component = 0U;
       component < comparison->stable_component_count;
       ++component)
  {
    const MeasurementRecoveredComponent *stable =
        &comparison->stable_components[component];

    if ((stable->frequency_hz <= 0.0F) || (stable->amplitude_mv < 0.0F) ||
        (stable->harmonic_order == 0U))
    {
      return 0U;
    }
  }
  return 1U;
}

static float Measurement_SynthesizeRecovered(
    const MeasurementAliasComparisonResult *comparison,
    float fundamental_phase)
{
  float value_mv = 0.0F;
  uint32_t component;

  for (component = 0U;
       component < comparison->stable_component_count;
       ++component)
  {
    const MeasurementRecoveredComponent *stable =
        &comparison->stable_components[component];
    float angle = fundamental_phase * (float)stable->harmonic_order;

    value_mv += stable->cosine_mv * arm_cos_f32(angle) -
                stable->sine_mv * arm_sin_f32(angle);
  }
  return value_mv;
}

MeasurementStatus Measurement_RecoverParameters(
    const MeasurementAliasComparisonResult *comparison,
    MeasurementParameterResult *result)
{
  float minimum_mv = FLT_MAX;
  float maximum_mv = -FLT_MAX;
  float rms_squared_mv = 0.0F;
  uint32_t component;
  uint32_t point;

  if (result == NULL)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result, 0, sizeof(*result));
  if (Measurement_RecoveryIsValid(comparison) == 0U)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  for (component = 0U;
       component < comparison->stable_component_count;
       ++component)
  {
    float amplitude_mv =
        comparison->stable_components[component].amplitude_mv;

    rms_squared_mv += 0.5F * amplitude_mv * amplitude_mv;
  }
  for (point = 0U; point < MEASUREMENT_FFT_LENGTH; ++point)
  {
    float phase = (MEASUREMENT_TWO_PI * (float)point) /
                  (float)MEASUREMENT_FFT_LENGTH;
    float value_mv = Measurement_SynthesizeRecovered(comparison, phase);

    if (value_mv < minimum_mv)
    {
      minimum_mv = value_mv;
    }
    if (value_mv > maximum_mv)
    {
      maximum_mv = value_mv;
    }
  }

  result->vpp_mv = maximum_mv - minimum_mv;
  result->rms_mv = Measurement_SquareRoot(rms_squared_mv);
  result->fundamental_hz = comparison->stable_components[0].frequency_hz;
  return MEASUREMENT_STATUS_OK;
}

MeasurementStatus Measurement_RecoverWaveform(
    const MeasurementAliasComparisonResult *comparison,
    MeasurementPeriodCount period_count,
    MeasurementWaveformResult *result)
{
  float minimum_mv = FLT_MAX;
  float maximum_mv = -FLT_MAX;
  uint32_t point;

  if ((result == NULL) ||
      ((period_count != MEASUREMENT_SHOW_ONE_PERIOD) &&
       (period_count != MEASUREMENT_SHOW_THREE_PERIODS)))
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result, 0, sizeof(*result));
  if (Measurement_RecoveryIsValid(comparison) == 0U)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  for (point = 0U; point < MEASUREMENT_WAVEFORM_POINT_COUNT; ++point)
  {
    float phase =
        (MEASUREMENT_TWO_PI * (float)period_count * (float)point) /
        (float)(MEASUREMENT_WAVEFORM_POINT_COUNT - 1U);
    float value_mv = Measurement_SynthesizeRecovered(comparison, phase);

    measurement_fft_input[point] = value_mv;
    if (value_mv < minimum_mv)
    {
      minimum_mv = value_mv;
    }
    if (value_mv > maximum_mv)
    {
      maximum_mv = value_mv;
    }
  }

  result->period_count = period_count;
  result->point_count = MEASUREMENT_WAVEFORM_POINT_COUNT;
  if ((maximum_mv - minimum_mv) <= 0.000001F)
  {
    (void)memset(result->points, 128, sizeof(result->points));
    return MEASUREMENT_STATUS_OK;
  }
  for (point = 0U; point < MEASUREMENT_WAVEFORM_POINT_COUNT; ++point)
  {
    float scaled = ((measurement_fft_input[point] - minimum_mv) * 255.0F) /
                   (maximum_mv - minimum_mv);

    if (scaled <= 0.0F)
    {
      result->points[point] = 0U;
    }
    else if (scaled >= 255.0F)
    {
      result->points[point] = 255U;
    }
    else
    {
      result->points[point] = (uint8_t)(scaled + 0.5F);
    }
  }
  return MEASUREMENT_STATUS_OK;
}

MeasurementStatus Measurement_RecoverSpectrum(
    const MeasurementAliasComparisonResult *comparison,
    MeasurementSpectrumResult *result)
{
  float maximum_amplitude_mv = 0.0F;
  uint32_t component;

  if (result == NULL)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result, 0, sizeof(*result));
  if (Measurement_RecoveryIsValid(comparison) == 0U)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  result->bar_count = MEASUREMENT_WAVEFORM_POINT_COUNT;
  result->frequency_span_hz = MEASUREMENT_SPECTRUM_DISPLAY_SPAN_HZ;
  for (component = 0U;
       component < comparison->stable_component_count;
       ++component)
  {
    if (comparison->stable_components[component].amplitude_mv >
        maximum_amplitude_mv)
    {
      maximum_amplitude_mv =
          comparison->stable_components[component].amplitude_mv;
    }
  }
  if (maximum_amplitude_mv <= 0.0F)
  {
    return MEASUREMENT_STATUS_OK;
  }

  for (component = 0U;
       component < comparison->stable_component_count;
       ++component)
  {
    const MeasurementRecoveredComponent *stable =
        &comparison->stable_components[component];
    uint32_t x = (uint32_t)(
        ((stable->frequency_hz / MEASUREMENT_SPECTRUM_DISPLAY_SPAN_HZ) *
         (float)(MEASUREMENT_WAVEFORM_POINT_COUNT - 1U)) + 0.5F);
    uint32_t height = (uint32_t)(
        ((stable->amplitude_mv / maximum_amplitude_mv) *
         MEASUREMENT_SPECTRUM_MAX_HEIGHT) + 0.5F);
    uint32_t first;
    uint32_t last;
    uint32_t line_x;

    if (x >= MEASUREMENT_WAVEFORM_POINT_COUNT)
    {
      x = MEASUREMENT_WAVEFORM_POINT_COUNT - 1U;
    }
    if (height > 255U)
    {
      height = 255U;
    }
    first = (x > MEASUREMENT_SPECTRUM_LINE_HALF_WIDTH) ?
        (x - MEASUREMENT_SPECTRUM_LINE_HALF_WIDTH) : 0U;
    last = x + MEASUREMENT_SPECTRUM_LINE_HALF_WIDTH;
    if (last >= MEASUREMENT_WAVEFORM_POINT_COUNT)
    {
      last = MEASUREMENT_WAVEFORM_POINT_COUNT - 1U;
    }
    for (line_x = first; line_x <= last; ++line_x)
    {
      if (result->bars[line_x] < (uint8_t)height)
      {
        result->bars[line_x] = (uint8_t)height;
      }
    }
  }
  return MEASUREMENT_STATUS_OK;
}

MeasurementStatus Measurement_RecoverComponents(
    const MeasurementAliasComparisonResult *comparison,
    MeasurementComponentResult *result)
{
  uint32_t component;

  if (result == NULL)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }
  (void)memset(result, 0, sizeof(*result));
  if (Measurement_RecoveryIsValid(comparison) == 0U)
  {
    return MEASUREMENT_STATUS_INVALID_ARGUMENT;
  }

  result->component_count = comparison->stable_component_count;
  for (component = 0U;
       component < comparison->stable_component_count;
       ++component)
  {
    result->components[component].frequency_hz =
        comparison->stable_components[component].frequency_hz;
    result->components[component].amplitude_mv =
        comparison->stable_components[component].amplitude_mv;
  }
  return MEASUREMENT_STATUS_OK;
}
