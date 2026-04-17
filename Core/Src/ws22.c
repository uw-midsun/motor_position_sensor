/*
 * ws22.c
 *
 *  Created on: Apr 15, 2026
 */

#include "ws22.h"

#include "usart.h"

#define WS22_MASK_6B 0x3FU
#define WS22_MASK_7B 0x7FU

static uint8_t s_uart_packet_a[WS22_PACKET_SIZE];
static uint8_t s_uart_packet_b[WS22_PACKET_SIZE];
static uint8_t *volatile s_uart_tx_buffer = s_uart_packet_a;
static uint8_t *volatile s_uart_fill_buffer = s_uart_packet_b;

static volatile bool s_uart_tx_busy = false;
static volatile bool s_uart_packet_pending = false;
static uint32_t s_last_device_type_tick = 0U;
static bool s_device_type_sent = false;

static void ws22_build_device_type_packet(uint8_t *packet, uint16_t angle_raw) {
  uint16_t angle_14b = (uint16_t)(angle_raw >> 2U);

  packet[0U] = (uint8_t)((0x02U << 6U) | (WS22_SENSOR_DEVICE_TYPE & WS22_MASK_6B));
  packet[1U] = 0x00U;
  packet[2U] = (uint8_t)((angle_14b >> 7U) & WS22_MASK_7B);
  packet[3U] = (uint8_t)(angle_14b & WS22_MASK_7B);
}

static void ws22_build_data_packet(uint8_t *packet, uint16_t thermistor_raw,
                                   uint16_t angle_raw) {
  uint16_t angle_14b = (uint16_t)(angle_raw >> 2U);

  packet[0U] =
      (uint8_t)((0x03U << 6U) | ((thermistor_raw >> 6U) & WS22_MASK_6B));
  packet[1U] = (uint8_t)((thermistor_raw << 1U) & WS22_MASK_7B);
  packet[2U] = (uint8_t)((angle_14b >> 7U) & WS22_MASK_7B);
  packet[3U] = (uint8_t)(angle_14b & WS22_MASK_7B);
}

static void ws22_queue_packet(const uint8_t *packet) {
  uint32_t index;

  __disable_irq();
  for (index = 0U; index < WS22_PACKET_SIZE; ++index) {
    s_uart_fill_buffer[index] = packet[index];
  }
  s_uart_packet_pending = true;
  __enable_irq();
}

static HAL_StatusTypeDef ws22_kick_tx(void) {
  HAL_StatusTypeDef hal_status = HAL_OK;
  uint8_t *next_buffer = NULL;

  __disable_irq();
  if (!s_uart_tx_busy && s_uart_packet_pending) {
    uint8_t *previous_tx_buffer = (uint8_t *)s_uart_tx_buffer;

    s_uart_tx_buffer = s_uart_fill_buffer;
    s_uart_fill_buffer = previous_tx_buffer;
    s_uart_packet_pending = false;
    s_uart_tx_busy = true;
    next_buffer = (uint8_t *)s_uart_tx_buffer;
  }
  __enable_irq();

  if (next_buffer != NULL) {
    hal_status = HAL_UART_Transmit_IT(&huart2, next_buffer, WS22_PACKET_SIZE);
    if (hal_status != HAL_OK) {
      __disable_irq();
      s_uart_tx_busy = false;
      __enable_irq();
    }
  }

  return hal_status;
}

HAL_StatusTypeDef ws22_init(void) {
  HAL_StatusTypeDef hal_status = HAL_OK;
  uint32_t index;

  for (index = 0U; index < WS22_PACKET_SIZE; ++index) {
    s_uart_packet_a[index] = 0U;
    s_uart_packet_b[index] = 0U;
  }

  s_uart_tx_buffer = s_uart_packet_a;
  s_uart_fill_buffer = s_uart_packet_b;
  s_uart_tx_busy = false;
  s_uart_packet_pending = false;
  s_last_device_type_tick = 0U;
  s_device_type_sent = false;

  return hal_status;
}

HAL_StatusTypeDef ws22_send_measurement(uint16_t thermistor_raw,
                                        uint16_t angle_raw) {
  uint8_t local_packet[WS22_PACKET_SIZE];

  uint32_t now = HAL_GetTick();
  if (!s_device_type_sent || (now - s_last_device_type_tick) >= 100U) {
    ws22_build_device_type_packet(local_packet, angle_raw);
    s_last_device_type_tick = now;
    s_device_type_sent = true;
  } else {
    ws22_build_data_packet(local_packet, thermistor_raw, angle_raw);
  }

  ws22_queue_packet(local_packet);
  return ws22_kick_tx();
}

HAL_StatusTypeDef ws22_handle_uart_tx_complete(UART_HandleTypeDef *huart) {
  if ((huart == NULL) || (huart->Instance != USART2)) {
    return HAL_ERROR;
  }

  s_uart_tx_busy = false;
  return ws22_kick_tx();
}

void ws22_handle_uart_error(UART_HandleTypeDef *huart) {
  if ((huart == NULL) || (huart->Instance != USART2)) {
    return;
  }

  s_uart_tx_busy = false;
}
