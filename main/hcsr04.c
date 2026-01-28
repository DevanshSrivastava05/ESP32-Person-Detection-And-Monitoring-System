/*
 * @file hcsr04.c
 * @brief HC-SR04 ultrasonic sensor driver implementation.
 * Handles initialization, trigger pulse generation, and distance measurement.
 */

#include "hcsr04.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HCSR04";

#define TRIG_GPIO 12       /* GPIO for HC-SR04 trigger pin */
#define ECHO_GPIO 13       /* GPIO for HC-SR04 echo pin */
#define TIMEOUT_US 30000   /* 30 ms timeout for pulse measurement */

/*
 * @brief Initialize HC-SR04 sensor pins.
 * Configures trigger as output and echo as input.
 * @return ESP_OK on successful initialization.
 */
esp_err_t hcsr04_init(void)
{
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&trig);
    gpio_config(&echo);

    gpio_set_level(TRIG_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

/*
 * @brief Measure distance using HC-SR04.
 * Sends a trigger pulse and measures echo duration.
 * @return Distance in centimeters, or -1.0 on timeout/error.
 */
float hcsr04_read_distance(void)
{
    /* Trigger pulse */
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    /* Wait for rising edge */
    int64_t startWait = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 0)
    {
        if (esp_timer_get_time() - startWait > TIMEOUT_US)
            return -1.0f;
    }

    int64_t pulse_start = esp_timer_get_time();

    /* Wait for falling edge */
    while (gpio_get_level(ECHO_GPIO) == 1)
    {
        if (esp_timer_get_time() - pulse_start > TIMEOUT_US)
            return -1.0f;
    }

    int64_t pulse_len = esp_timer_get_time() - pulse_start;

    /* Convert microseconds to centimeters */
    float distance_cm = (pulse_len * 0.0343f) / 2.0f;
    return distance_cm;
}
