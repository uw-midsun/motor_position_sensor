#pragma once

#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"
#include "uart.h"

#define MASK_6B 0x3F
#define MASK_7B 0x7F

#define WS22_SENSOR_DEVICE_TYPE 7U

StatusCode sp3485_init(MotorSensorStorage *storage);

StatusCode sp3485_run();

StatusCode sp3485_report_device_type();