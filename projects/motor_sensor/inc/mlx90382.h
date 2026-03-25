#pragma once

/* Standard library Headers */

/* Inter-component Headers */
#include "spi.h"

/* Intra-component Headers */

#define SPI_ADDR(x) x >> 1

#define DUMMY_BYTE 0x00U

#define MLX90382_RR_BASE 0xCC  // {7'b1100110, addr[9]}
#define MLX90382_RW_BASE 0x78  // {7'b0111100, addr[9]}

#define MLX90382_CONFIG_SENSING_MODE 0b111
#define MLX90382_CONFIG_GPIO_IF 0b11000

#define MLX90382_MAX_REG_ADDR 0x2FFU

typedef enum {
  MLX90382_REG_CONFIG = 0x200,
  MLX90382_REG_FADDR0 = 0x230,
  MLX90382_REG_FADDR2 = 0x232,
  MLX90382_REG_FR_OPTIONS = 0x234,
} Mlx90382Registers;

StatusCode mlx90382_init(MotorSensorStorage *storage);

StatusCode mlx90382_run();
