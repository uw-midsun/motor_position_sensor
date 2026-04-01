#include <stdio.h>
/* Standard library Headers */

/* Inter-component Headers */
#include "delay.h"
#include "gpio.h"
#include "log.h"
#include "master_task.h"
#include "spi.h"
#include "tasks.h"

/* Intra-component Headers */
#include "mlx90382.h"
#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"
#include "sp3485.h"
#include "thermistor.h"

MotorSensorStorage motor_sensor_storage = { 0 };

MotorSensorConfig motor_sensor_config = {
  .uart_port = MOTOR_SENSOR_UART_PORT,
  .uart_settings = { .tx = GPIO_MOTOR_SENSOR_UART_TX,
                     .rx = GPIO_MOTOR_SENSOR_UART_RX,
                     .baudrate = MOTOR_SENSOR_UART_BAUDRATE },
  .spi_port = MOTOR_SENSOR_SPI_PORT,
  .spi_settings = { .baudrate = MOTOR_SENSOR_SPI_BAUDRATE,
                    .mode = MOTOR_SENSOR_SPI_MODE,
                    .mosi = GPIO_MOTOR_SENSOR_SPI_MOSI,
                    .miso = GPIO_MOTOR_SENSOR_SPI_MISO,
                    .sclk = GPIO_MOTOR_SENSOR_SPI_SCLK,
                    .cs = GPIO_MOTOR_SENSOR_SPI_CS },
};

static uint8_t cycle_count = 0;

void pre_loop_init() {}

TASK(motor_sensor_task, TASK_STACK_512) {
  motor_sensor_init(&motor_sensor_storage, &motor_sensor_config);
  // Add a delay here?
  while (true) {
    mlx90382_run();
    thermistor_run();
    if (cycle_count == 100) {
      cycle_count = 0;
      sp3485_report_device_type();
    } else {
      sp3485_run();
    }
    cycle_count++;
  }
}

void run_medium_cycle() {}

void run_slow_cycle() {}

int main() {
  tasks_init();
  
#if (MOTOR_DEBUG == 1U)
  log_init();
#endif

  gpio_init();

  tasks_init_task(motor_sensor_task, TASK_PRIORITY(2), NULL);

  tasks_start();

  LOG_DEBUG("exiting main?");
  return 0;
}
