#pragma once

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */


#define MOTOR_SENSOR_GPIO_DEF(PORT, PIN) \
  { .port = GPIO_PORT_##PORT, .pin = PIN }

/************************************************************************************************
 * Motor Sensor UART definitions - for RS485 bridge
 ************************************************************************************************/

#define GPIO_MOTOR_SENSOR_UART_TX MOTOR_SENSOR_GPIO_DEF(A, 2)

#define GPIO_MOTOR_SENSOR_UART_RX MOTOR_SENSOR_GPIO_DEF(A, 3)

#define GPIO_MOTOR_SENSOR_UART_DE MOTOR_SENSOR_GPIO_DEF(A, 1)

#define GPIO_MOTOR_SENSOR_UART_NRE MOTOR_SENSOR_GPIO_DEF(A, 4)

#define MOTOR_SENSOR_UART_PORT UART_PORT_2

#define MOTOR_SENSOR_UART_BAUDRATE 115200

/************************************************************************************************
 * Motor Sensor SPI definitions - for MLX magnetic sensor
 ************************************************************************************************/

#define GPIO_MOTOR_SENSOR_SPI_MOSI MOTOR_SENSOR_GPIO_DEF(B, 15)

#define GPIO_MOTOR_SENSOR_SPI_MISO MOTOR_SENSOR_GPIO_DEF(B, 14)

#define GPIO_MOTOR_SENSOR_SPI_SCLK MOTOR_SENSOR_GPIO_DEF(B, 13)

#define GPIO_MOTOR_SENSOR_SPI_CS MOTOR_SENSOR_GPIO_DEF(B, 12)

#define MOTOR_SENSOR_SPI_BAUDRATE 5000

#define MOTOR_SENSOR_SPI_MODE SPI_MODE_3

#define MOTOR_SENSOR_SPI_PORT SPI_PORT_2

/************************************************************************************************
 * Motor Sensor misc. definitions
 ************************************************************************************************/

#define GPIO_MOTOR_SENSOR_THERM MOTOR_SENSOR_GPIO_DEF(B, 1)

#define GPIO_MOTOR_SENSOR_HALL MOTOR_SENSOR_GPIO_DEF(A, 6)
 