/*
 * @file led_control.c
 * @brief Minimal LED control module.
 * Handles initialization and control of person, far, and near indicator LEDs.
 */

#include "led_control.h"
#include "driver/gpio.h"

#define LED_FAR_GPIO 14    /* GPIO for far-distance indicator LED */
#define LED_NEAR_GPIO 4    /* GPIO for near-distance indicator LED */

/*
 * @brief Initialize all LED GPIO pins as outputs.
 * Sets all LEDs OFF by default.
 * @return ESP_OK on successful initialization.
 */
esp_err_t led_init(void)
{
    gpio_config_t out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_FAR_GPIO) | (1ULL << LED_NEAR_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out);

    /* default off */
    gpio_set_level(LED_FAR_GPIO, 0);
    gpio_set_level(LED_NEAR_GPIO, 0);
    return ESP_OK;
}

/*
 * @brief Control the "far distance" LED.
 * @param on Set true to turn LED ON, false to turn OFF.
 */
void led_set_far(bool on) { gpio_set_level(LED_FAR_GPIO, on ? 1 : 0); }

/*
 * @brief Control the "near distance" LED.
 * @param on Set true to turn LED ON, false to turn OFF.
 */
void led_set_near(bool on) { gpio_set_level(LED_NEAR_GPIO, on ? 1 : 0); }
