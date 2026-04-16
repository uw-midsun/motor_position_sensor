/*
 * mlx90382.c
 *
 *  Created on: Apr 15, 2026
 */

#include "mlx90382.h"
#include "spi.h"

#define MLX90382_CS_Pin GPIO_PIN_12
#define MLX90382_CS_GPIO_Port GPIOB
#define MLX90382_SPI_TIMEOUT_MS 10U

static volatile uint16_t s_mlx90382_reading = 0U;
static volatile bool s_spi_dma_busy = false;
static volatile bool s_spi_dma_done = false;

static uint8_t s_spi_frame_tx[MLX90382_FRAME_TRANSFER_SIZE] = {
    MLX90382_FR_COMMAND, 0x00U, 0x00U, 0x00U, 0x00U};
static uint8_t s_spi_frame_rx[MLX90382_FRAME_TRANSFER_SIZE];

static void mlx90382_chip_select(bool selected) {
  HAL_GPIO_WritePin(MLX90382_CS_GPIO_Port, MLX90382_CS_Pin,
                    selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint16_t mlx90382_to_spi_addr(uint16_t reg_addr) {
  return (uint16_t)(reg_addr >> 1U);
}

static HAL_StatusTypeDef mlx90382_register_read(uint16_t reg_addr,
                                                uint16_t *data) {
  HAL_StatusTypeDef hal_status;
  uint16_t spi_addr;
  uint8_t tx_buffer[5U];
  uint8_t rx_buffer[5U];

  if ((data == NULL) || (reg_addr > MLX90382_MAX_REG_ADDR)) {
    return HAL_ERROR;
  }

  spi_addr = mlx90382_to_spi_addr(reg_addr);

  tx_buffer[0U] = (uint8_t)(MLX90382_RR_BASE | ((spi_addr >> 8U) & 0x01U));
  tx_buffer[1U] = (uint8_t)(spi_addr & 0xFFU);
  tx_buffer[2U] = 0x00U;
  tx_buffer[3U] = 0x00U;
  tx_buffer[4U] = 0x00U;

  mlx90382_chip_select(true);
  hal_status = HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer,
                                       sizeof(tx_buffer),
                                       MLX90382_SPI_TIMEOUT_MS);
  mlx90382_chip_select(false);

  if (hal_status != HAL_OK) {
    return hal_status;
  }

  *data = (uint16_t)(((uint16_t)rx_buffer[3U] << 8U) | rx_buffer[4U]);
  return HAL_OK;
}

static HAL_StatusTypeDef mlx90382_register_write(uint16_t reg_addr,
                                                 uint16_t data) {
  HAL_StatusTypeDef hal_status;
  uint16_t spi_addr;
  uint8_t tx_buffer[4U];
  uint8_t rx_buffer[4U];

  if (reg_addr > MLX90382_MAX_REG_ADDR) {
    return HAL_ERROR;
  }

  spi_addr = mlx90382_to_spi_addr(reg_addr);

  tx_buffer[0U] = (uint8_t)(MLX90382_RW_BASE | ((spi_addr >> 8U) & 0x01U));
  tx_buffer[1U] = (uint8_t)(spi_addr & 0xFFU);
  tx_buffer[2U] = (uint8_t)((data >> 8U) & 0xFFU);
  tx_buffer[3U] = (uint8_t)(data & 0xFFU);

  mlx90382_chip_select(true);
  hal_status = HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer,
                                       sizeof(tx_buffer),
                                       MLX90382_SPI_TIMEOUT_MS);
  mlx90382_chip_select(false);

  return hal_status;
}

static HAL_StatusTypeDef mlx90382_nvram_write_verify(uint16_t reg_addr,
                                                     uint16_t data) {
  HAL_StatusTypeDef hal_status;
  uint16_t verify = 0U;

  hal_status = mlx90382_register_write(reg_addr, data);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_register_read(reg_addr, &verify);
  if (hal_status != HAL_OK) return hal_status;

  return (verify == data) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef mlx90382_nvram_write_if_changed(uint16_t reg_addr,
                                                         uint16_t data,
                                                         bool *wrote) {
  HAL_StatusTypeDef hal_status;
  uint16_t current = 0U;

  hal_status = mlx90382_register_read(reg_addr, &current);
  if (hal_status != HAL_OK) return hal_status;

  if (current == data) return HAL_OK;

  *wrote = true;
  return mlx90382_nvram_write_verify(reg_addr, data);
}

static uint16_t mlx90382_parse_frame_word(const uint8_t *frame) {
  return (uint16_t)(((uint16_t)frame[3U] << 8U) | frame[4U]);
}

HAL_StatusTypeDef mlx90382_init(void) {
  HAL_StatusTypeDef hal_status;
  uint16_t config_reg = 0U;
  uint16_t crc_status = 0U;
  uint16_t crc = 0U;
  bool nvram_changed = false;

  s_mlx90382_reading = 0U;
  s_spi_dma_busy = false;
  s_spi_dma_done = false;

  hal_status =
      mlx90382_nvram_write_verify(MLX90382_REG_DE_SR, MLX90382_DE_SR_DISABLE);
  if (hal_status != HAL_OK) return hal_status;

  hal_status =
      mlx90382_nvram_write_verify(MLX90382_REG_IN_APPLICATION, 0x0006U);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_register_read(MLX90382_REG_CONFIG, &config_reg);
  if (hal_status != HAL_OK) return hal_status;

  config_reg &= (uint16_t)~MLX90382_CONFIG_SENSING_MODE_MASK;
  config_reg |= MLX90382_SENSING_MODE_YZ;
  config_reg &= (uint16_t)~MLX90382_CONFIG_GPIO_IF_MASK;
  config_reg |= MLX90382_GPIO_IF_SPI_BUS;
  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_CONFIG, config_reg,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_FADDR01, 0x0000U,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_FADDR23, 0x0000U,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_FR_CONFIG,
                                               MLX90382_FR_CONFIG_5BYTE_FRAME,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_SC_Y1,
                                               MLX90382_SC_Y1_FULL_RANGE,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_SC_Y2,
                                               MLX90382_SC_Y2_FULL_RANGE,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_SC_YE,
                                               MLX90382_SC_YE_FULL_RANGE,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  {
    uint16_t de_cfg = 0U;

    hal_status = mlx90382_register_read(MLX90382_REG_DISABLE_CFG, &de_cfg);
    if (hal_status != HAL_OK) return hal_status;

    de_cfg &= (uint16_t)~MLX90382_DE_DSP_RMM_MASK;
    hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_DISABLE_CFG,
                                                 de_cfg, &nvram_changed);
    if (hal_status != HAL_OK) return hal_status;
  }

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_RMM_CFG,
                                               MLX90382_RMM_CFG_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ_GAIN,
                                               MLX90382_PEQ_GAIN_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ00_01,
                                               MLX90382_PEQ00_01_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ02_03,
                                               MLX90382_PEQ02_03_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ04_05,
                                               MLX90382_PEQ04_05_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ06_07,
                                               MLX90382_PEQ06_07_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ08_09,
                                               MLX90382_PEQ08_09_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ10_11,
                                               MLX90382_PEQ10_11_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ12_13,
                                               MLX90382_PEQ12_13_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_PEQ14_15,
                                               MLX90382_PEQ14_15_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_S_IQ,
                                               MLX90382_S_IQ_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;
  hal_status = mlx90382_nvram_write_if_changed(MLX90382_REG_S_QQ,
                                               MLX90382_S_QQ_VAL,
                                               &nvram_changed);
  if (hal_status != HAL_OK) return hal_status;

  if (nvram_changed) {
    hal_status = mlx90382_register_write(MLX90382_REG_CRC_CTRL, 0x0001U);
    if (hal_status != HAL_OK) return hal_status;

    HAL_Delay(1U);  // >= 10 us

    hal_status = mlx90382_register_read(MLX90382_REG_CRC_CTRL, &crc_status);
    if (hal_status != HAL_OK) return hal_status;
    if (!(crc_status & 0x0002U)) return HAL_ERROR;

    hal_status = mlx90382_register_read(MLX90382_REG_CRC, &crc);
    if (hal_status != HAL_OK) return hal_status;

    hal_status = mlx90382_nvram_write_verify(MLX90382_REG_CUS_CRC, crc);
    if (hal_status != HAL_OK) return hal_status;

    hal_status = mlx90382_nvram_write_verify(MLX90382_REG_NVOP_KEY,
                                             MLX90382_NVOP_KEY_LO);
    if (hal_status != HAL_OK) return hal_status;

    hal_status = mlx90382_nvram_write_verify(MLX90382_REG_NVOP_KEY,
                                             MLX90382_NVOP_KEY_HI);
    if (hal_status != HAL_OK) return hal_status;

    hal_status =
        mlx90382_register_write(MLX90382_REG_NVM_CTRL, MLX90382_STORE_REQ);
    if (hal_status != HAL_OK) return hal_status;

    HAL_Delay(50U);  // >= 15 ms for NVM write to complete
  }

  hal_status = mlx90382_register_write(MLX90382_REG_IN_APPLICATION, 0x0000U);
  if (hal_status != HAL_OK) return hal_status;

  return HAL_OK;
}

HAL_StatusTypeDef mlx90382_start_frame_dma(void) {
  HAL_StatusTypeDef hal_status;

  if (s_spi_dma_busy) return HAL_BUSY;

  s_spi_dma_busy = true;
  s_spi_dma_done = false;

  mlx90382_chip_select(true);
  hal_status = HAL_SPI_TransmitReceive_DMA(
      &hspi2, s_spi_frame_tx, s_spi_frame_rx, MLX90382_FRAME_TRANSFER_SIZE);
  if (hal_status != HAL_OK) {
    mlx90382_chip_select(false);
    s_spi_dma_busy = false;
  }

  return hal_status;
}

bool mlx90382_is_busy(void) {
  return s_spi_dma_busy;
}

bool mlx90382_take_reading(uint16_t *reading) {
  if ((reading == NULL) || !s_spi_dma_done) {
    return false;
  }

  *reading = s_mlx90382_reading;
  s_spi_dma_done = false;
  return true;
}

void mlx90382_handle_spi_txrx_complete(SPI_HandleTypeDef *hspi) {
  if ((hspi == NULL) || (hspi->Instance != SPI2)) {
    return;
  }

  mlx90382_chip_select(false);
  s_mlx90382_reading = mlx90382_parse_frame_word(s_spi_frame_rx);
  s_spi_dma_busy = false;
  s_spi_dma_done = true;
}

void mlx90382_handle_spi_error(SPI_HandleTypeDef *hspi) {
  if ((hspi == NULL) || (hspi->Instance != SPI2)) {
    return;
  }

  mlx90382_chip_select(false);
  s_spi_dma_busy = false;
  s_spi_dma_done = false;
}
