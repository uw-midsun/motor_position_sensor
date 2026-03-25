#pragma once

#include "uart.h"

StatusCode sp3485_init(MotorSensorStorage *storage);

StatusCode sp3485_run();