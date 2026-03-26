#pragma once

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>

/* Inter-component Headers */
#include "spi.h"
#include "status.h"
#include "uart.h"

/* Intra-component Headers */

struct MotorSensorStorage;
struct MotorSensorConfig;

typedef struct {
  UartPort uart_port;
  UartSettings uart_settings;
  SpiPort spi_port;
  SpiSettings spi_settings;
} MotorSensorConfig;

typedef struct {
  MotorSensorConfig *config;
  uint16_t mlx903_reading;
  uint16_t thermistor_reading;
} MotorSensorStorage;

StatusCode motor_sensor_init(MotorSensorStorage *storage, MotorSensorConfig *config);