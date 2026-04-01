#pragma once

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */
#include "adc.h"
#include "gpio.h"
#include "status.h"

/* Intra-component Headers */
#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"

StatusCode thermistor_init(MotorSensorStorage *motor_sensor_storage);

StatusCode thermistor_run();
