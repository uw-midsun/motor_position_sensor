#include <stdio.h>
/* Standard library Headers */

/* Inter-component Headers */
#include "gpio.h"
#include "log.h"
#include "master_task.h"
#include "spi.h"
#include "tasks.h"

/* Intra-component Headers */
#include "mlx90382.h"
#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"

MotorSensorStorage motor_sensor_storage = { 0 };

MotorSensorConfig motor_sensor_config = {
  .uart_port = MOTOR_SENSOR_UART_PORT,
  .uart_settings = { .tx = GPIO_MOTOR_SENSOR_UART_TX,
                     .rx = GPIO_MOTOR_SENSOR_UART_RX,
                     .baudrate = MOTOR_SENSOR_UART_BAUDRATE },
  .spi_port = MOTOR_SENSOR_SPI_PORT,
  .spi_settings = { .baudrate = MOTOR_SENSOR_SPI_BAUDRATE,
                    .mode = MOTOR_SENSOR_SPI_MODE,
                    .mosi = GPIO_MOTOR_SENSOR_SPI_CS,
                    .miso = GPIO_MOTOR_SENSOR_SPI_MISO,
                    .sclk = GPIO_MOTOR_SENSOR_SPI_SCLK,
                    .cs = GPIO_MOTOR_SENSOR_SPI_CS },
};

void pre_loop_init() {}

void run_fast_cycle() {
  mlx90382_run();
}

void run_medium_cycle() {}

void run_slow_cycle() {}

int main() {
  tasks_init();
  log_init();
  gpio_init();

  init_master_task();

  motor_sensor_init(&motor_sensor_storage, &motor_sensor_config);

  tasks_start();

  LOG_DEBUG("exiting main?");
  return 0;
}
