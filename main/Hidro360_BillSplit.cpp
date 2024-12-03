#include <stdio.h>
#include "ds18b20_temperature_sensor.h"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

// GPIO definitions
constexpr gpio_num_t ONEWIRE_BUS_GPIO = GPIO_NUM_26;

static void temp_sensor_task(void *_param)
{
    auto *temp_sensor = new Ds18b20TemperatureSensor(ONEWIRE_BUS_GPIO);
    while (true)
    {
        float temperature = temp_sensor->readTemperatureC();
        printf("Current temperature: %.2f°C\n", temperature);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

extern "C" void app_main(void)
{
    xTaskCreate(temp_sensor_task, "temp_sensor", 4096, nullptr, 5, nullptr);
}
