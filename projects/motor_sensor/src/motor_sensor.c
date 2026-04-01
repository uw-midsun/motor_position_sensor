/* Standard library Headers */

/* Inter-component Headers */
#include "adc.h"
#include "gpio.h"
#include "log.h"
#include "spi.h"
#include "uart.h"

/* Intra-component Headers */
#include "mlx90382.h"
#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"
#include "sp3485.h"
#include "thermistor.h"

static MotorSensorStorage *s_motor_sensor_storage;

static GpioAddress s_motor_sensor_uart_de = GPIO_MOTOR_SENSOR_UART_DE;
static GpioAddress s_motor_sensor_uart_nre = GPIO_MOTOR_SENSOR_UART_NRE;

static StatusCode status = STATUS_CODE_OK;

StatusCode motor_sensor_init(MotorSensorStorage *storage, MotorSensorConfig *config) {
  if (storage == NULL || config == NULL) {
    LOG_DEBUG("invalid args\r\n");
    return STATUS_CODE_INVALID_ARGS;
  }

  s_motor_sensor_storage = storage;
  s_motor_sensor_storage->config = config;

  status = uart_init(s_motor_sensor_storage->config->uart_port,
                     &s_motor_sensor_storage->config->uart_settings);

  if (status != STATUS_CODE_OK) {
    LOG_DEBUG("uart_init failed with exit code %d\r\n", status);
  }

  status = spi_init(s_motor_sensor_storage->config->spi_port,
                    &s_motor_sensor_storage->config->spi_settings);
  if (status != STATUS_CODE_OK) {
    LOG_DEBUG("spi_init failed with exit code %d\r\n", status);
  }

  status = mlx90382_init(s_motor_sensor_storage);
  if (status != STATUS_CODE_OK) {
    LOG_DEBUG("mlx90382_init failed with exit code %d\r\n", status);
  }

  status = sp3485_init(s_motor_sensor_storage);
  if (status != STATUS_CODE_OK) {
    LOG_DEBUG("sp3485_init failed with exit code %d\r\n", status);
  }

  status = thermistor_init(s_motor_sensor_storage);
  if (status != STATUS_CODE_OK) {
    LOG_DEBUG("thermistor_init failed with exit code %d\r\n", status);
  }

  gpio_init_pin(&s_motor_sensor_uart_de, GPIO_OUTPUT_PUSH_PULL, GPIO_STATE_HIGH);
  gpio_init_pin(&s_motor_sensor_uart_nre, GPIO_OUTPUT_PUSH_PULL, GPIO_STATE_LOW);

  /* ADC initialization must happen at the very end, so all channels are registered */
  status = adc_init();
  if (status != STATUS_CODE_OK) {
    LOG_DEBUG("adc_init failed with exit code %d\r\n", status);
  }

  return STATUS_CODE_OK;
}