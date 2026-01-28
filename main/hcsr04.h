/*
 * @file hcsr04.h
 * @brief Interface for HC-SR04 ultrasonic distance sensor driver.
 * Provides initialization and distance measurement API.
 */

#ifndef HCSR04_H
#define HCSR04_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief Initialize the HC-SR04 ultrasonic sensor.
 * Configures trigger and echo GPIO pins.
 * @return ESP_OK on successful initialization, error code otherwise.
 */
esp_err_t hcsr04_init(void);

/*
 * @brief Read distance from the HC-SR04 sensor.
 * Sends trigger pulse and measures echo response time.
 * @return Distance in centimeters, or -1.0 on error/timeout.
 */
float hcsr04_read_distance(void);

#ifdef __cplusplus
}
#endif

#endif /* HCSR04_H */
