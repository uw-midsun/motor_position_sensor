/* Standard library Headers */

/* Inter-component Headers */
#include "log.h"
#include "delay.h"

/* Intra-component Headers */
#include "sp3485.h"
#include "motor_sensor_hw_defs.h"

static GpioAddress gpio_sp3485_uart_de = GPIO_MOTOR_SENSOR_UART_DE;
static GpioAddress gpio_sp3485_uart_nre = GPIO_MOTOR_SENSOR_UART_NRE;

static MotorSensorStorage *s_motor_sensor_storage;

static size_t uart_tx_length = 4U;

StatusCode sp3485_init(MotorSensorStorage *storage) {
  if (storage == NULL) {
    return STATUS_CODE_INVALID_ARGS;
  }

  s_motor_sensor_storage = storage;

  status_ok_or_return(gpio_set_state(&gpio_sp3485_uart_de, GPIO_STATE_HIGH));
  status_ok_or_return(gpio_set_state(&gpio_sp3485_uart_nre, GPIO_STATE_LOW));

  return STATUS_CODE_OK;
}

StatusCode sp3485_report_device_type() {
  uint8_t tx_buf[4U];

  uint16_t device_type = WS22_SENSOR_DEVICE_TYPE;

  tx_buf[0U] = (0b10 << 6) | (WS22_SENSOR_DEVICE_TYPE & MASK_6B);
  tx_buf[1U] = 0;
  tx_buf[2U] = (uint8_t) (((s_motor_sensor_storage->mlx903_reading >> 2) >> 7) & MASK_7B);
  tx_buf[3U] = (uint8_t) ((s_motor_sensor_storage->mlx903_reading >> 2) & MASK_7B);

#if (MOTOR_DEBUG == 1)
  LOG_DEBUG("tx bytes: %u | %u | %u | %u |\r\n", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
    delay_ms(10);
#endif
  uart_tx(MOTOR_SENSOR_UART_PORT, tx_buf, &uart_tx_length);
  uart_send_break(MOTOR_SENSOR_UART_PORT);

  return STATUS_CODE_OK;
}

StatusCode sp3485_run() {
  uint8_t tx_buf[4U];

  tx_buf[0U] = (0b11 << 6) | ((s_motor_sensor_storage->thermistor_reading >> 10) & MASK_6B);
  tx_buf[1U] = ((s_motor_sensor_storage->thermistor_reading >> 3) & MASK_7B);
  tx_buf[2U] = ((s_motor_sensor_storage->mlx903_reading >> 2) >> 7) & MASK_7B;
  tx_buf[3U] = (s_motor_sensor_storage->mlx903_reading >> 2) & MASK_7B;

#if (MOTOR_DEBUG == 1)
  LOG_DEBUG("tx bytes: %u | %u | %u | %u |\r\n", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
  delay_ms(10);
#endif
  uart_tx(MOTOR_SENSOR_UART_PORT, tx_buf, &uart_tx_length);
  uart_send_break(MOTOR_SENSOR_UART_PORT);

  return STATUS_CODE_OK;
}