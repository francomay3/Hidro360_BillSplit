#include "ds18b20_temperature_sensor.h"

/**
 * @brief Class to interface with DS18B20 digital temperature sensor
 *
 * This class provides a simple interface to read temperature values from a DS18B20 sensor
 * connected via 1-Wire bus. The sensor communicates over a single data pin and provides
 * temperature readings in Celsius.
 *
 * The DS18B20 features:
 * - Temperature measurements from -55°C to +125°C
 * - ±0.5°C accuracy from -10°C to +85°C
 * - 9-12 bit configurable resolution
 * - 1-Wire interface requiring only one digital pin
 *
 * Wiring connections:
 * - VDD (red wire): Connect to 3.3V or 5V
 * - GND (black wire): Connect to ground
 * - DATA (yellow/white wire): Connect to GPIO pin through a 4.7kΩ pullup resistor
 * - diagram: https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2019/06/ds18b20_esp32_single_normal.png?quality=100&strip=all&ssl=1
 *
 * The 4.7kΩ pullup resistor must be connected between the DATA line and VDD.
 *
 * Example usage:
 * @code
 * Ds18b20TemperatureSensor sensor(GPIO_NUM_4);  // Initialize with GPIO pin 4
 * float temp = sensor.readTemperatureC();       // Read temperature in Celsius
 * @endcode
 *
 */
Ds18b20TemperatureSensor::Ds18b20TemperatureSensor(gpio_num_t pin) : data_pin(pin), bus(nullptr), ds18b20(nullptr), sensor_found(false)
{
  // Configure one-wire bus
  onewire_bus_config_t bus_config = {
      .bus_gpio_num = data_pin,
  };
  onewire_bus_rmt_config_t rmt_config = {
      .max_rx_bytes = 10,
  };
  ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

  // Search for device
  onewire_device_iter_handle_t iter = nullptr;
  ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
  printf("Searching for DS18B20 sensor...\n");

  onewire_device_t onewire_device;
  if (onewire_device_iter_get_next(iter, &onewire_device) == ESP_OK)
  {
    ds18b20_config_t ds_cfg = {};
    if (ds18b20_new_device(&onewire_device, &ds_cfg, &ds18b20) == ESP_OK)
    {
      printf("Found DS18B20, address: %016llX\n", onewire_device.address);
      sensor_found = true;
    }
  }

  ESP_ERROR_CHECK(onewire_del_device_iter(iter));
  printf("DS18B20 %s\n", sensor_found ? "found" : "not found");
}

float Ds18b20TemperatureSensor::readTemperatureC()
{
  if (!sensor_found)
  {
    return 0.0f;
  }

  ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20));
  float temperature;
  ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20, &temperature));
  return temperature;
}

Ds18b20TemperatureSensor::~Ds18b20TemperatureSensor()
{
  if (ds18b20)
  {
    ds18b20_del_device(ds18b20);
  }
  if (bus)
  {
    onewire_bus_del(bus);
  }
}