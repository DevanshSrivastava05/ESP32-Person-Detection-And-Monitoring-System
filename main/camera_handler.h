#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_camera.h"
#include "esp_err.h"
#include <stdint.h>

/* Image dimensions for person detection model */ 
#define MODEL_WIDTH 96
#define MODEL_HEIGHT 96

/**
 * @brief Initialize ESP32-CAM with OV2640
 * @return ESP_OK on success
 */
esp_err_t camera_init(void);

/**
 * @brief Capture image from camera
 * @return Pointer to frame buffer, or NULL on error
 */
camera_fb_t *camera_capture(void);
/**
 * @brief Return frame buffer (must call after use!)
 * @param fb Frame buffer to return
 */

void camera_return_fb(camera_fb_t *fb);
/**
 * @brief Preprocess image for TensorFlow model
 * @param fb Frame buffer from camera
 * @param output_buffer Output buffer (96x96 grayscale)
 * @return ESP_OK on success
 */

esp_err_t preprocess_image(camera_fb_t *fb, uint8_t *output_buffer);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_HANDLER_H */ 
