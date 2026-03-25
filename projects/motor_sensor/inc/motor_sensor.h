#pragma once

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>

/* Inter-component Headers */
#include "status.h"

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
  uint32_t readings;
} MotorSensorStorage;

StatusCode motor_sensor_init(MotorSensorStorage *storage, MotorSensorConfig *config);