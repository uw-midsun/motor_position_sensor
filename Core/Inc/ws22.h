/*
 * ws22.h
 *
 *  Created on: Apr 15, 2026
 *      Author: daiya
 */

#ifndef INC_WS22_H_
#define INC_WS22_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

#define WS22_PACKET_SIZE 4U
#define WS22_SENSOR_DEVICE_TYPE 7U

HAL_StatusTypeDef ws22_init(void);

HAL_StatusTypeDef ws22_send_measurement(uint16_t thermistor_raw, uint16_t angle_raw);

HAL_StatusTypeDef ws22_handle_uart_tx_complete(UART_HandleTypeDef *huart);
void ws22_handle_uart_error(UART_HandleTypeDef *huart);

#endif /* INC_WS22_H_ */
