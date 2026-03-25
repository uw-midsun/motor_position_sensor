/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */
#include "mlx90382.h"

#include "motor_sensor.h"
#include "motor_sensor_hw_defs.h"

static MotorSensorStorage *s_motor_sensor_storage;

static StatusCode register_read(Mlx90382Registers addr, uint16_t *out) {
  if (addr > MLX90382_MAX_REG_ADDR) {
    return STATUS_CODE_INVALID_ARGS;
  }

  uint8_t tx_buffer[5U];
  uint8_t rx_buffer[5U];
  tx_buffer[0U] = MLX90382_RR_BASE | (SPI_ADDR(addr) >> 8);
  tx_buffer[1U] = SPI_ADDR(addr) & 0xFFU;
  tx_buffer[2U] = DUMMY_BYTE;
  tx_buffer[3U] = DUMMY_BYTE;
  tx_buffer[4U] = DUMMY_BYTE;

  status_ok_or_return(spi_exchange(MOTOR_SENSOR_SPI_PORT, tx_buffer, 5U, rx_buffer, 5));

  *out = rx_buffer[3U] << 8 | rx_buffer[4U];

  return STATUS_CODE_OK;
}

static StatusCode register_write(Mlx90382Registers addr, uint16_t data) {
  if (addr > MLX90382_MAX_REG_ADDR) {
    return STATUS_CODE_INVALID_ARGS;
  }

  uint8_t tx_buffer[4U];
  uint8_t rx_buffer[4U];

  tx_buffer[0U] = MLX90382_RW_BASE | (SPI_ADDR(addr) >> 8);
  tx_buffer[1U] = SPI_ADDR(addr) & 0xFFU;
  tx_buffer[2U] = (data >> 8) & 0xFFU;
  tx_buffer[3U] = data & 0xFFU;

  // Try with rx_data = NULL and rx_len = 0
  status_ok_or_return(spi_exchange(MOTOR_SENSOR_SPI_PORT, tx_buffer, 4U, rx_buffer, 4U));

  return STATUS_CODE_OK;
}

StatusCode mlx90382_init(MotorSensorStorage *storage) {
  // 0x200 SENSING_MODE[2:0] = 2 indicates Y-Z mode
  // 0x200 GPIO_IF[4:3] = 2 indicates SPI bus mode
  // SPI_FADDR0 = 0x00 to read angle
  // SPI_FADDR1 = 0x00
  // SPI_FADDR2 = 0x00
  // SPI_FADDR3 = 0x00

  if (storage == NULL) {
    return STATUS_CODE_INVALID_ARGS;
  }

  s_motor_sensor_storage = storage;

  uint16_t mode;

  status_ok_or_return(register_read(MLX90382_REG_CONFIG, &mode));

  mode &= ~(MLX90382_CONFIG_SENSING_MODE);
  mode |= (0x2U);  // Y-Z sensing mode

  mode &= ~(MLX90382_CONFIG_GPIO_IF);
  mode |= (0x2U << 3U);  // SPI bus mode

  status_ok_or_return(register_write(MLX90382_REG_CONFIG, mode));

  status_ok_or_return(register_write(MLX90382_REG_FADDR0, 0U));
  status_ok_or_return(register_write(MLX90382_REG_FADDR2, 0U));

  return STATUS_CODE_OK;
}

StatusCode mlx90382_run() {
  uint8_t tx_buffer[4U] = { 0x00, 0x00, 0x00, 0x00 };
  uint8_t rx_buffer[4U];

  status_ok_or_return(spi_exchange(MOTOR_SENSOR_SPI_PORT, tx_buffer, 4U, rx_buffer, 4U));

  s_motor_sensor_storage->reading = (uint16_t)((rx_buffer[2U] << 8) | rx_buffer[3U]);

  return STATUS_CODE_OK;
}
