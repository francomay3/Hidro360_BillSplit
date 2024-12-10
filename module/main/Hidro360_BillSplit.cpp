// Standard libraries
#include <RadioLib.h>

// Local modules
#include "ds18b20_temperature_sensor.h"
#include "misc.h"
#include "LoRa.h"

// GPIO definitions

// Temperature sensor
constexpr gpio_num_t sensor_temperature_pin = GPIO_NUM_27;

// LoRa
constexpr gpio_num_t lora_rst_pin = GPIO_NUM_14;
constexpr gpio_num_t lora_nss_pin = GPIO_NUM_5;
constexpr gpio_num_t lora_sck_pin = GPIO_NUM_18;
constexpr gpio_num_t lora_mosi_pin = GPIO_NUM_23;
constexpr gpio_num_t lora_miso_pin = GPIO_NUM_19;
constexpr gpio_num_t lora_dio0_pin = GPIO_NUM_26;

void main_task(void *pvParameters)
{
    // Initialize the temperature sensor
    auto *temp_sensor = new Ds18b20TemperatureSensor(sensor_temperature_pin);

    // Initialize LoRa handler
    auto *LoRa = new LoRaHandler(lora_rst_pin, lora_nss_pin, lora_sck_pin, lora_mosi_pin, lora_miso_pin, lora_dio0_pin);

    while (true)
    {
        // Read the temperature
        float temperature = temp_sensor->readTemperatureC();
        printf("Current temperature: %.2f°C\n", temperature);

        // Transmit the temperature over LoRa
        LoRa->send(std::to_string(temperature).c_str());

        delay(2000);
    }
}

extern "C" void app_main(void)
{
    xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);
}
