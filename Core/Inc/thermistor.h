/*
 * thermistor.h
 *
 *  Created on: Apr 16, 2026
 */

#ifndef INC_THERMISTOR_H_
#define INC_THERMISTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

HAL_StatusTypeDef thermistor_init(void);

bool thermistor_read(uint16_t *adc_raw);

#endif /* INC_THERMISTOR_H_ */
