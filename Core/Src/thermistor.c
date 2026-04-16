/*
 * thermistor.c
 *
 *  Created on: Apr 16, 2026
 */

#include "thermistor.h"

#include <string.h>

#include "adc.h"

#define THERMISTOR_ADC_CHANNEL ADC_CHANNEL_9
#define ADC_TIMEOUT_MS 10U

#define MEDIAN_WINDOW 5U

// 20 counts ~= 0.6deg/sample about 25 deg
#define SLEW_MAX 20U

typedef struct {
  uint16_t buf[MEDIAN_WINDOW];
  uint8_t  head;
  bool     primed;
} MedianState;

static MedianState s_median   = {0};
static uint16_t    s_slew_out = 0U;
static bool        s_slew_primed = false;

static uint16_t median_filter(uint16_t sample) {
  s_median.buf[s_median.head] = sample;
  s_median.head = (uint8_t)((s_median.head + 1U) % MEDIAN_WINDOW);

  uint16_t tmp[MEDIAN_WINDOW];
  (void)memcpy(tmp, s_median.buf, sizeof(tmp));
  for (uint8_t i = 1U; i < MEDIAN_WINDOW; i++) {
    uint16_t key = tmp[i];
    uint8_t  j   = i;
    while (j > 0U && tmp[j - 1U] > key) {
      tmp[j] = tmp[j - 1U];
      j--;
    }
    tmp[j] = key;
  }
  return tmp[MEDIAN_WINDOW / 2U];
}

static uint16_t slew_clamp(uint16_t sample) {
  if (!s_slew_primed) {
    s_slew_out    = sample;
    s_slew_primed = true;
    return sample;
  }

  if (sample > s_slew_out + SLEW_MAX) {
    s_slew_out = s_slew_out + SLEW_MAX;
  } else if (sample + SLEW_MAX < s_slew_out) {
    s_slew_out = s_slew_out - SLEW_MAX;
  } else {
    s_slew_out = sample;
  }
  return s_slew_out;
}

HAL_StatusTypeDef thermistor_init(void) {
  ADC_ChannelConfTypeDef channel_config = {0};

  channel_config.Channel      = THERMISTOR_ADC_CHANNEL;
  channel_config.Rank         = ADC_REGULAR_RANK_1;
  channel_config.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &channel_config) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

bool thermistor_read(uint16_t *adc_raw) {
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return false;
  }

  if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc1);
    return false;
  }

  uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  if (!s_median.primed) {
    for (uint8_t i = 0U; i < MEDIAN_WINDOW; i++) {
      s_median.buf[i] = raw;
    }
    s_median.primed = true;
  }

  *adc_raw = slew_clamp(median_filter(raw));
  return true;
}
