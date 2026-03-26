#include "thermistor.h"

#include "status.h"

static GpioAddress gpio_motor_sensor_therm = GPIO_MOTOR_SENSOR_THERM;

static MotorSensorStorage *s_motor_sensor_storage;

static volatile uint16_t adc_reading;

StatusCode thermistor_init(MotorSensorStorage *motor_sensor_storage) {
  s_motor_sensor_storage = motor_sensor_storage;
  gpio_init_pin(&gpio_motor_sensor_therm, GPIO_ANALOG, GPIO_STATE_LOW);
  adc_add_channel(gpio_motor_sensor_therm);

  return STATUS_CODE_OK;
}

StatusCode thermistor_run() {
  adc_read_raw(gpio_motor_sensor_therm, &adc_reading);

  s_motor_sensor_storage->thermistor_reading = adc_reading;

  return STATUS_CODE_OK;
}
