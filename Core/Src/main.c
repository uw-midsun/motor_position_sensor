/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  APP_STATUS_OK = 0,
  APP_STATUS_SPI_ERROR,
  APP_STATUS_UART_ERROR,
  APP_STATUS_ADC_ERROR
} AppStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WS22_PACKET_SIZE 4U
#define WS22_SENSOR_DEVICE_TYPE 5U

#define MASK_6B 0x3FU
#define MASK_7B 0x7FU

#define MLX90382_CS_Pin GPIO_PIN_12
#define MLX90382_CS_GPIO_Port GPIOB

#define MOTOR_SENSOR_UART_DE_Pin GPIO_PIN_1
#define MOTOR_SENSOR_UART_DE_GPIO_Port GPIOA
#define MOTOR_SENSOR_UART_NRE_Pin GPIO_PIN_4
#define MOTOR_SENSOR_UART_NRE_GPIO_Port GPIOA

#define THERMISTOR_ADC_CHANNEL ADC_CHANNEL_9

#define SPI_TIMEOUT_MS 10U
#define UART_TIMEOUT_MS 10U
#define ADC_TIMEOUT_MS 10U

#define MLX90382_RR_BASE 0xCCU
#define MLX90382_RW_BASE 0x78U
#define MLX90382_FR_COMMAND 0x00U

#define MLX90382_MAX_REG_ADDR 0x025EU
#define MLX90382_REG_CONFIG 0x0200U
#define MLX90382_REG_FADDR01 0x0230U
#define MLX90382_REG_FADDR23 0x0232U
#define MLX90382_REG_FR_CONFIG 0x0234U

#define MLX90382_CONFIG_SENSING_MODE_MASK 0x0007U
#define MLX90382_CONFIG_GPIO_IF_MASK 0x0018U
#define MLX90382_SENSING_MODE_YZ 0x0002U
#define MLX90382_GPIO_IF_SPI_BUS 0x0010U

#define MLX90382_FR_PATTERN 0x000AU
#define MLX90382_FR_FS_ENABLE 0x0010U
#define MLX90382_FR_CRC_DISABLE 0x0000U
#define MLX90382_FR_CONFIG_5BYTE_FRAME (MLX90382_FR_PATTERN | MLX90382_FR_FS_ENABLE | MLX90382_FR_CRC_DISABLE)

#define MLX90382_FRAME_TRANSFER_SIZE 5U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t s_spi_frame_tx[MLX90382_FRAME_TRANSFER_SIZE] = {
  MLX90382_FR_COMMAND, 0x00U, 0x00U, 0x00U, 0x00U
};
static uint8_t s_spi_frame_rx[MLX90382_FRAME_TRANSFER_SIZE];
static uint8_t s_uart_packet_a[WS22_PACKET_SIZE];
static uint8_t s_uart_packet_b[WS22_PACKET_SIZE];
static uint8_t *volatile s_uart_tx_buffer = s_uart_packet_a;
static uint8_t *volatile s_uart_fill_buffer = s_uart_packet_b;

static volatile bool s_spi_dma_busy = false;
static volatile bool s_spi_dma_done = false;
static volatile bool s_uart_tx_busy = false;
static volatile bool s_uart_packet_pending = false;
static volatile uint16_t s_mlx90382_reading = 0U;
static volatile uint32_t s_spi_error_count = 0U;
static volatile uint32_t s_uart_error_count = 0U;
static volatile uint32_t s_adc_error_count = 0U;
static volatile AppStatus s_app_status = APP_STATUS_OK;

static bool s_device_type_sent = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void mlx90382_chip_select(bool selected);
static uint16_t mlx90382_to_spi_addr(uint16_t reg_addr);
static HAL_StatusTypeDef mlx90382_register_read(uint16_t reg_addr, uint16_t *data);
static HAL_StatusTypeDef mlx90382_register_write(uint16_t reg_addr, uint16_t data);
static HAL_StatusTypeDef mlx90382_init(void);
static uint16_t mlx90382_parse_frame_word(const uint8_t *frame);
static HAL_StatusTypeDef mlx90382_start_frame_dma(void);

static HAL_StatusTypeDef thermistor_init(void);
static uint16_t thermistor_read(void);

static void ws22_build_device_type_packet(uint8_t *packet, uint16_t angle_raw);
static void ws22_build_data_packet(uint8_t *packet, uint16_t thermistor_raw, uint16_t angle_raw);
static void ws22_queue_packet(const uint8_t *packet);
static HAL_StatusTypeDef ws22_kick_tx(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void mlx90382_chip_select(bool selected) {
  HAL_GPIO_WritePin(MLX90382_CS_GPIO_Port, MLX90382_CS_Pin,
                    selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint16_t mlx90382_to_spi_addr(uint16_t reg_addr) {
  return (uint16_t)(reg_addr >> 1U);
}

static HAL_StatusTypeDef mlx90382_register_read(uint16_t reg_addr, uint16_t *data) {
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
  hal_status = HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, sizeof(tx_buffer), SPI_TIMEOUT_MS);
  mlx90382_chip_select(false);

  if (hal_status != HAL_OK) {
    return hal_status;
  }

  *data = (uint16_t)(((uint16_t)rx_buffer[3U] << 8U) | rx_buffer[4U]);
  return HAL_OK;
}

static HAL_StatusTypeDef mlx90382_register_write(uint16_t reg_addr, uint16_t data) {
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
  hal_status = HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, sizeof(tx_buffer), SPI_TIMEOUT_MS);
  mlx90382_chip_select(false);

  return hal_status;
}

static HAL_StatusTypeDef mlx90382_init(void) {
  HAL_StatusTypeDef hal_status;
  uint16_t config_reg = 0U;

  hal_status = mlx90382_register_read(MLX90382_REG_CONFIG, &config_reg);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  config_reg &= (uint16_t)~MLX90382_CONFIG_SENSING_MODE_MASK;
  config_reg |= MLX90382_SENSING_MODE_YZ;

  config_reg &= (uint16_t)~MLX90382_CONFIG_GPIO_IF_MASK;
  config_reg |= MLX90382_GPIO_IF_SPI_BUS;

  hal_status = mlx90382_register_write(MLX90382_REG_CONFIG, config_reg);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  hal_status = mlx90382_register_write(MLX90382_REG_FADDR01, 0x0000U);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  hal_status = mlx90382_register_write(MLX90382_REG_FADDR23, 0x0000U);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  hal_status = mlx90382_register_write(MLX90382_REG_FR_CONFIG, MLX90382_FR_CONFIG_5BYTE_FRAME);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  return HAL_OK;
}

static uint16_t mlx90382_parse_frame_word(const uint8_t *frame) {
  return (uint16_t)(((uint16_t)frame[3U] << 8U) | frame[4U]);
}

static HAL_StatusTypeDef mlx90382_start_frame_dma(void) {
  HAL_StatusTypeDef hal_status;

  if (s_spi_dma_busy) {
    return HAL_BUSY;
  }

  s_spi_dma_busy = true;
  s_spi_dma_done = false;

  mlx90382_chip_select(true);
  hal_status = HAL_SPI_TransmitReceive_DMA(&hspi2, s_spi_frame_tx, s_spi_frame_rx, MLX90382_FRAME_TRANSFER_SIZE);
  if (hal_status != HAL_OK) {
    mlx90382_chip_select(false);
    s_spi_dma_busy = false;
  }

  return hal_status;
}

static HAL_StatusTypeDef thermistor_init(void) {
  ADC_ChannelConfTypeDef channel_config = { 0 };

  channel_config.Channel = THERMISTOR_ADC_CHANNEL;
  channel_config.Rank = ADC_REGULAR_RANK_1;
  channel_config.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &channel_config) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static uint16_t thermistor_read(void) {
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    s_adc_error_count++;
    s_app_status = APP_STATUS_ADC_ERROR;
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc1);
    s_adc_error_count++;
    s_app_status = APP_STATUS_ADC_ERROR;
    return 0U;
  }

  {
    uint16_t adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    return adc_value;
  }
}

static void ws22_build_device_type_packet(uint8_t *packet, uint16_t angle_raw) {
  uint16_t angle_14b = (uint16_t)(angle_raw >> 2U);

  packet[0U] = (uint8_t)((0x02U << 6U) | (WS22_SENSOR_DEVICE_TYPE & MASK_6B));
  packet[1U] = 0x00U;
  packet[2U] = (uint8_t)((angle_14b >> 7U) & MASK_7B);
  packet[3U] = (uint8_t)(angle_14b & MASK_7B);
}

static void ws22_build_data_packet(uint8_t *packet, uint16_t thermistor_raw, uint16_t angle_raw) {
  uint16_t angle_14b = (uint16_t)(angle_raw >> 2U);

  packet[0U] = (uint8_t)((0x03U << 6U) | ((thermistor_raw >> 6U) & MASK_6B));
  packet[1U] = (uint8_t)((thermistor_raw << 1U) & MASK_7B);
  packet[2U] = (uint8_t)((angle_14b >> 7U) & MASK_7B);
  packet[3U] = (uint8_t)(angle_14b & MASK_7B);
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    HAL_StatusTypeDef hal_status;

    s_uart_tx_busy = false;
    hal_status = ws22_kick_tx();
    if (hal_status != HAL_OK) {
      s_uart_error_count++;
      s_app_status = APP_STATUS_UART_ERROR;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    s_uart_tx_busy = false;
    s_uart_error_count++;
    s_app_status = APP_STATUS_UART_ERROR;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    mlx90382_chip_select(false);
    s_mlx90382_reading = mlx90382_parse_frame_word(s_spi_frame_rx);
    s_spi_dma_busy = false;
    s_spi_dma_done = true;
    s_app_status = APP_STATUS_OK;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    mlx90382_chip_select(false);
    s_spi_dma_busy = false;
    s_spi_dma_done = false;
    s_spi_error_count++;
    s_app_status = APP_STATUS_SPI_ERROR;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  if (thermistor_init() != HAL_OK) {
    Error_Handler();
  }

  if (mlx90382_init() != HAL_OK) {
    Error_Handler();
  }

  if (mlx90382_start_frame_dma() != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (s_spi_dma_done) {
      uint8_t local_packet[WS22_PACKET_SIZE];
      uint16_t thermistor_raw;

      s_spi_dma_done = false;
      thermistor_raw = thermistor_read();

      if (!s_device_type_sent) {
        ws22_build_device_type_packet(local_packet, (uint16_t)s_mlx90382_reading);
        s_device_type_sent = true;
      } else {
        ws22_build_data_packet(local_packet, thermistor_raw, (uint16_t)s_mlx90382_reading);
      }

      ws22_queue_packet(local_packet);
      if (ws22_kick_tx() != HAL_OK) {
        s_uart_error_count++;
        s_app_status = APP_STATUS_UART_ERROR;
      }
    }

    if (!s_spi_dma_busy && !s_spi_dma_done) {
      if (mlx90382_start_frame_dma() != HAL_OK) {
        s_spi_error_count++;
        s_app_status = APP_STATUS_SPI_ERROR;
        HAL_Delay(1U);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
