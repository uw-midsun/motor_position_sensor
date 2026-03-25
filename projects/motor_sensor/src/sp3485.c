/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */
#include "sp3485.h"

#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"

static GpioAddress sp3485_uart_de = GPIO_MOTOR_SENSOR_UART_DE;
static GpioAddress sp3485_uart_nre = GPIO_MOTOR_SENSOR_UART_NRE;

static MotorSensorStorage *s_motor_sensor_storage;

StatusCode sp3485_init(MotorSensorStorage *storage) {
  if (storage == NULL) {
    return STATUS_CODE_INVALID_ARGS;
  }

  s_motor_sensor_storage = storage;

  status_ok_or_return(gpio_set_state(&sp3485_uart_de, GPIO_STATE_HIGH));
  status_ok_or_return(gpio_set_state(&sp3485_uart_nre, GPIO_STATE_LOW));

  return STATUS_CODE_OK;
}

StatusCode sp3485_run() {
  uint8_t tx_buf[2U];

  tx_buf[1U] = (s_motor_sensor_storage->reading >> 8) & 0xFFU;
  tx_buf[0U] = s_motor_sensor_storage->reading & 0xFFU;

  s_motor_sensor_storage->reading;
  uart_tx(MOTOR_SENSOR_UART_PORT, tx_buf, 2U);

  return STATUS_CODE_OK;
}