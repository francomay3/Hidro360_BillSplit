#pragma once

#include "driver/gpio.h"
#include "ds18b20.h"
#include "onewire_bus.h"

class Ds18b20TemperatureSensor
{
private:
  static constexpr const char *TAG = "temp. sensor";
  gpio_num_t data_pin;
  onewire_bus_handle_t bus;
  ds18b20_device_handle_t ds18b20;
  bool sensor_found;

public:
  Ds18b20TemperatureSensor(gpio_num_t pin);
  float readTemperatureC();
  ~Ds18b20TemperatureSensor();
};