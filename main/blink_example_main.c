/*
to check the port run:
ls /dev/cu.usbserial*
and you should see something like:
/dev/cu.usbserial-0001
thats your port.

current port:
port /dev/cu.usbserial-0001

// to build, flash, and monitor run:
// idf.py build
// idf.py -p /dev/cu.usbserial-0001 flash
// idf.py monitor -p /dev/cu.usbserial-0001

before building, source the esp-idf environment variables runnning:
get_idf

to build, flash, and monitor run:
idf.py -p /dev/cu.usbserial-0001 flash monitor

to quit monitor:
ctrl-t ctrl-x
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ds18b20.h"
#include "onewire_bus.h"

static const char *TAG = "blinker says";

#define BUTTON_GPIO 32
#define BUTTON_LED_GPIO 33
#define BLINK_GPIO 25
#define EXAMPLE_ONEWIRE_BUS_GPIO 26
#define EXAMPLE_ONEWIRE_MAX_DS18B20 1

void read_temperature_task(void *param)
{
    // install 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    int ds18b20_device_num = 0;
    ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    do
    {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK)
        { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK)
            {
                ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", ds18b20_device_num, next_onewire_device.address);
                ds18b20_device_num++;
            }
            else
            {
                ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);

    // Infinite loop to continuously read temperature
    while (1)
    {
        // Read temperature from each found device
        for (int i = 0; i < ds18b20_device_num; i++)
        {
            // Trigger a new temperature conversion
            ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20s[i]));

            // Wait for conversion to complete (typical conversion time is 750ms)
            vTaskDelay(pdMS_TO_TICKS(750));

            // Read the temperature
            float temperature;
            ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20s[i], &temperature));
            ESP_LOGI(TAG, "DS18B20[%d] temperature: %.2f°C", i, temperature);
        }

        // Wait before next reading cycle
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void read_button(void *arg)
{
    printf("Button reader configured on GPIO %d!\n", BUTTON_GPIO);

    // Configure button GPIO
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    // Configure LED GPIO
    gpio_reset_pin(BUTTON_LED_GPIO);
    gpio_set_direction(BUTTON_LED_GPIO, GPIO_MODE_OUTPUT);

    while (1)
    {
        int button_state = gpio_get_level(BUTTON_GPIO);
        // When button is pressed (state = 0), turn LED on
        // When button is released (state = 1), turn LED off
        gpio_set_level(BUTTON_LED_GPIO, !button_state); // Invert the state

        static int prev_button_state = -1;
        if (button_state != prev_button_state)
        {
            printf("Button %s!\n", button_state == 0 ? "pressed" : "released");
            prev_button_state = button_state;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void blink_led(void *arg)
{
    printf("Blinker configured to blink GPIO %d!\n", BLINK_GPIO);
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(BLINK_GPIO, 1); // turn on
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(BLINK_GPIO, 0); // turn off
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    xTaskCreate(blink_led, "blink_led", 2048, NULL, 5, NULL);
    xTaskCreate(read_button, "read_button", 2048, NULL, 5, NULL);
    xTaskCreate(read_temperature_task, "temp_sensor", 4096, NULL, 5, NULL);
}
