/*
 * @file camera_handler.c
 * @brief ESP32-CAM Camera Implementation with PSRAM support.
 * Provides initialization, capture, preprocessing, and cleanup routines.
 */

#include "esp_camera.h"
#include "camera_handler.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <string.h>

/*
 * @file camera_handler.c
 * @brief ESP32-CAM Camera Implementation with PSRAM support.
 * Provides initialization, capture, preprocessing, and cleanup routines.
 */

#include "esp_camera.h"
#include "camera_handler.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <string.h>

#define MODEL_WIDTH 96
#define MODEL_HEIGHT 96

static const char *TAG = "CAMERA";

/* ESP32-CAM AI-Thinker pin definitions */
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

/*
 * @brief Initialize ESP32-CAM camera with PSRAM configuration.
 * Validates PSRAM presence, configures sensor settings, and sets grayscale mode.
 * @return ESP_OK on success, ESP_FAIL or error code otherwise.
 */
esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "Initializing ESP32-CAM with PSRAM...");

    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    if (psram_size == 0)
    {
        ESP_LOGE(TAG, "PSRAM NOT DETECTED! Configuration steps required.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✓ PSRAM detected");
    ESP_LOGI(TAG, "  Total: %d bytes (%.1f MB)", psram_size, psram_size / 1024.0 / 1024.0);
    ESP_LOGI(TAG, "  Free: %d bytes (%.1f MB)", psram_free, psram_free / 1024.0 / 1024.0);
    ESP_LOGI(TAG, "  Free DRAM: %lu bytes", esp_get_free_heap_size());

    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sscb_sda = CAM_PIN_SIOD,
        .pin_sscb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = FRAMESIZE_96X96,
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL)
    {
        ESP_LOGE(TAG, "Failed to get sensor handle");
        return ESP_FAIL;
    }

    /* Sensor tuning for person detection */
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 0);
    sensor->set_saturation(sensor, 0);
    sensor->set_whitebal(sensor, 1);
    sensor->set_awb_gain(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_aec2(sensor, 0);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_agc_gain(sensor, 0);
    sensor->set_vflip(sensor, 0);
    sensor->set_hmirror(sensor, 0);

    ESP_LOGI(TAG, "✓ Camera initialized successfully!");
    return ESP_OK;
}

/*
 * @brief Capture image frame from camera.
 * @param None
 * @return Pointer to camera frame buffer, or NULL on failure.
 */
camera_fb_t *camera_capture(void)
{
    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == NULL)
    {
        ESP_LOGE(TAG, "Camera capture failed!");
        return NULL;
    }

    if (fb->buf == NULL || fb->len == 0)
    {
        ESP_LOGE(TAG, "Invalid frame buffer");
        esp_camera_fb_return(fb);
        return NULL;
    }

    return fb;
}

/*
 * @brief Return frame buffer to driver.
 * @param fb Pointer to frame buffer to return.
 */
void camera_return_fb(camera_fb_t *fb)
{
    if (fb != NULL)
    {
        esp_camera_fb_return(fb);
    }
}

/*
 * @brief Preprocess 96x96 grayscale image for TensorFlow Lite.
 * @param fb Pointer to captured frame buffer.
 * @param output_buffer Destination buffer for processed image.
 * @return ESP_OK on success, ESP_FAIL on invalid input.
 */
esp_err_t preprocess_image(camera_fb_t *fb, uint8_t *output_buffer)
{
    if (fb == NULL || fb->buf == NULL)
    {
        ESP_LOGE(TAG, "Invalid frame buffer!");
        return ESP_FAIL;
    }

    if (output_buffer == NULL)
    {
        ESP_LOGE(TAG, "Output buffer is NULL!");
        return ESP_FAIL;
    }

    size_t expected_size = MODEL_WIDTH * MODEL_HEIGHT;
    if (fb->len >= expected_size)
    {
        memcpy(output_buffer, fb->buf, expected_size);
    }
    else
    {
        memcpy(output_buffer, fb->buf, fb->len);
        memset(output_buffer + fb->len, 0, expected_size - fb->len);
    }

    return ESP_OK;
}

/*
 * @brief Deinitialize and free camera resources.
 * @return ESP_OK on successful deinitialization.
 */
esp_err_t camera_deinit(void)
{
    esp_err_t err = esp_camera_deinit();
    return err;
}
