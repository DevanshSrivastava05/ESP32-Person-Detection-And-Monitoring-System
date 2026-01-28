/*
 * @file led_control.h
 * @brief LED control interface for person, far, and near indicators.
 * Provides initialization and simple ON/OFF control APIs.
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief Initialize LED GPIO pins.
 * Configures all indicator LEDs as outputs and turns them OFF by default.
 * @return ESP_OK on success.
 */
esp_err_t led_init(void);

/*
 * @brief Control the far-distance LED (GPIO14).
 * @param on True = LED ON, False = LED OFF.
 */
void led_set_far(bool on);

/*
 * @brief Control the near-distance LED (GPIO4).
 * @param on True = LED ON, False = LED OFF.
 */
void led_set_near(bool on);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROL_H */
