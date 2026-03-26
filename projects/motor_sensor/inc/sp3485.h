#pragma once

#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"
#include "uart.h"

#define MASK_6B 0x3F
#define MASK_7B 0x7F

StatusCode sp3485_init(MotorSensorStorage *storage);

StatusCode sp3485_run();