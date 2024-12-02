// Local modules
#include "utils/ds18b20_temperature_sensor.h"
#include "utils/misc.h"

// GPIO definitions
constexpr gpio_num_t temp_sensor_gpio = GPIO_NUM_26;

extern "C" void app_main(void)
{
    auto *temp_sensor = new Ds18b20TemperatureSensor(temp_sensor_gpio);
    while (true)
    {
        float temperature = temp_sensor->readTemperatureC();
        printf("Current temperature: %.2f°C\n", temperature);
        delay(2000);
    }
}
