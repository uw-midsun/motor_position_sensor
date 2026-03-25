/* Standard library Headers */

/* Inter-component Headers */
#include "adc.h"
#include "gpio.h"
#include "spi.h"
#include "uart.h"

/* Intra-component Headers */
#include "mlx90382.h"
#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"

static MotorSensorStorage *s_motor_sensor_storage;

static GpioAddress s_motor_sensor_uart_de = GPIO_MOTOR_SENSOR_UART_DE;
static GpioAddress s_motor_sensor_uart_nre = GPIO_MOTOR_SENSOR_UART_NRE;

StatusCode motor_sensor_init(MotorSensorStorage *storage, MotorSensorConfig *config) {
  if (storage == NULL || config == NULL) {
    return STATUS_CODE_INVALID_ARGS;
  }

  s_motor_sensor_storage = storage;
  s_motor_sensor_storage->config = config;

  // Initalize other components...

  uart_init(s_motor_sensor_storage->config->uart_port,
            &s_motor_sensor_storage->config->uart_settings);
  spi_init(s_motor_sensor_storage->config->spi_port, &s_motor_sensor_storage->config->spi_settings);

  mlx90382_init(s_motor_sensor_storage);

  gpio_init_pin(&s_motor_sensor_uart_de, GPIO_OUTPUT_PUSH_PULL, GPIO_STATE_LOW);
  gpio_init_pin(&s_motor_sensor_uart_nre, GPIO_OUTPUT_PUSH_PULL, GPIO_STATE_LOW);

  /* ADC initialization must happen at the very end, so all channels are registered */
  adc_init();

  return STATUS_CODE_OK;
}