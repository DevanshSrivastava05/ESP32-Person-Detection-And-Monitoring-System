extern "C"
{
#include "camera_handler.h"
#include "led_control.h"
#include "hcsr04.h"
    void app_main(void);
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

/* TensorFlow Lite */ 
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char *TAG = "SYSTEM";

/* Model data */ 
/* model (generated .cc) */ 
extern const unsigned char g_person_detect_model_data[];
extern const int g_person_detect_model_data_len;

/* thresholds */ 
#define FAR_THRESHOLD_CM 50.0f
#define NEAR_THRESHOLD_CM 20.0f
#define PERSON_SCORE_THRESHOLD 60

/* Queue items */ 
typedef struct
{
    bool person;
    int8_t score;
} detect_msg_t;

typedef struct
{
    float distance_cm;
} sensor_msg_t;

/* Queues (single-slot latest value) */ 
static QueueHandle_t qDetect = NULL;
static QueueHandle_t qSensor = NULL;
static SemaphoreHandle_t tflite_mutex = NULL;

/* TFLM objects */ 
static tflite::MicroErrorReporter micro_error_reporter;
static tflite::ErrorReporter *error_reporter = nullptr;
static const tflite::Model *model = nullptr;
static TfLiteTensor *input_tensor = nullptr;
static TfLiteTensor *output_tensor = nullptr;
static tflite::MicroInterpreter *interpreter = nullptr;
static uint8_t *tensor_arena = nullptr;
static const size_t kTensorArenaSize = 140 * 1024; /* increase if you have space */ 

static void log_memory_info()
{
    /* DRAM info */
    size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t total_dram = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t used_dram = total_dram - free_dram;

    /* PSRAM info (external SPI RAM) */
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t used_psram = total_psram - free_psram;

    ESP_LOGI("MEMORY", "===== MEMORY INFORMATION =====");

    /* DRAM summary */
    ESP_LOGI("MEMORY",
             "DRAM: total=%u bytes (%.2f MB), used=%u bytes (%.2f MB), free=%u bytes (%.2f MB)",
             total_dram, total_dram / 1024.0 / 1024.0,
             used_dram, used_dram / 1024.0 / 1024.0,
             free_dram, free_dram / 1024.0 / 1024.0);

    /* PSRAM summary */
    ESP_LOGI("MEMORY",
             "PSRAM: total=%u bytes (%.2f MB), used=%u bytes (%.2f MB), free=%u bytes (%.2f MB)",
             total_psram, total_psram / 1024.0 / 1024.0,
             used_psram, used_psram / 1024.0 / 1024.0,
             free_psram, free_psram / 1024.0 / 1024.0);

    /* Model size + location */
    ESP_LOGI("MEMORY", "Model weight size = %d bytes (%.3f KB)", 
             g_person_detect_model_data_len,
             g_person_detect_model_data_len / 1024.0);

    ESP_LOGI("MEMORY", "Model memory pointer = %p", g_person_detect_model_data);
    if (esp_ptr_external_ram((void *)g_person_detect_model_data))
        ESP_LOGW("MEMORY", "Model is located in PSRAM (!)");
    else if (esp_ptr_internal((void *)g_person_detect_model_data))
        ESP_LOGI("MEMORY", "Model is located in DRAM");
    else
        ESP_LOGI("MEMORY", "Model is located in FLASH (expected)");

    /* Tensor arena */
    ESP_LOGI("MEMORY", "Tensor arena pointer = %p", tensor_arena);
    ESP_LOGI("MEMORY", "Tensor arena size = %u bytes (%.2f KB)", 
             kTensorArenaSize, kTensorArenaSize / 1024.0);

    if (esp_ptr_external_ram(tensor_arena))
        ESP_LOGI("MEMORY", "Tensor arena is located in PSRAM ✓");
    else if (esp_ptr_internal(tensor_arena))
        ESP_LOGI("MEMORY", "Tensor arena is located in DRAM");
    else
        ESP_LOGW("MEMORY", "Tensor arena is in UNKNOWN location!?");

    ESP_LOGI("MEMORY", "================================");
}

/*
* @brief Initializes TensorFlow Lite Micro interpreter and tensor arena.
* @return true if initialization succeeds, false otherwise.
*/
static bool tflite_init()
{
    error_reporter = &micro_error_reporter;

    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    if (free_psram >= kTensorArenaSize)
        tensor_arena = (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM);
    else
        tensor_arena = (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_8BIT);

    if (!tensor_arena)
    {
        ESP_LOGE(TAG, "tensor arena alloc failed");
        return false;
    }

    model = tflite::GetModel(g_person_detect_model_data);
    if (!model || model->version() != TFLITE_SCHEMA_VERSION)
    {
        ESP_LOGE(TAG, "model version mismatch");
        return false;
    }

    static tflite::MicroMutableOpResolver<6> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddAveragePool2D();
    resolver.AddReshape();
    resolver.AddSoftmax();
    resolver.AddFullyConnected();

    static tflite::MicroInterpreter static_interpreter(
        model,
        resolver,
        tensor_arena,
        kTensorArenaSize,
        nullptr, /* resource vars */ 
        nullptr, /* profiler */ 
        false    /* preserve_all_tensors */ 
    );

    interpreter = &static_interpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk)
    {
        ESP_LOGE(TAG, "AllocateTensors failed");
        return false;
    }

    input_tensor = interpreter->input(0);
    output_tensor = interpreter->output(0);

    ESP_LOGI(TAG, "TFLM OK (arena used=%d bytes)", interpreter->arena_used_bytes());
    return true;
}

/*
* @brief Task: Person detection using camera and TFLM inference.
* Captures frames, preprocesses image to 96x96 grayscale, runs inference,
* and sends detection results via queue.
*/
static void task_person_detection(void *arg)
{
    /* small local buffers */ 
    static uint8_t image_buf[96 * 96];

    /* warm up camera */ 
    for (int i = 0; i < 3; i++)
    {
        camera_fb_t *fb = camera_capture();
        if (fb)
            camera_return_fb(fb);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    detect_msg_t msg;
    while (1)
    {
        camera_fb_t *fb = camera_capture();
        if (!fb)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (preprocess_image(fb, image_buf) != ESP_OK)
        {
            camera_return_fb(fb);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* inference protected by mutex */ 
        if (xSemaphoreTake(tflite_mutex, pdMS_TO_TICKS(5000)) == pdTRUE)
        {
            /* copy into input tensor (int8 quantized) */ 
            for (int i = 0; i < 96 * 96; ++i)
            {
                input_tensor->data.int8[i] = (int8_t)(image_buf[i] - 128);
            }
            bool ok = (interpreter->Invoke() == kTfLiteOk);
            if (ok)
            {
                int8_t score_no = output_tensor->data.int8[0];
                int8_t score_person = output_tensor->data.int8[1];

                msg.person = (score_person > PERSON_SCORE_THRESHOLD);
                msg.score = score_person;

                /* publish latest (overwrite) */ 
                xQueueOverwrite(qDetect, &msg);

                /* concise log */ 
                if (msg.person)
                {
                    ESP_LOGI(TAG, "PERSON DETECTED (score=%d)", msg.score);
                }
                else
                {
                    ESP_LOGI(TAG, "No person (score=%d)", msg.score);
                }
            }
            else
            {
                ESP_LOGW(TAG, "Inference failed");
            }
            xSemaphoreGive(tflite_mutex);
        }

        camera_return_fb(fb);
        vTaskDelay(pdMS_TO_TICKS(800)); /* 0.8s between inferences */ 
    }
}

/*
* @brief Task: Reads HC-SR04 distance sensor and publishes distance (cm).
*/
static void task_sensor_read(void *arg)
{
    sensor_msg_t msg;
    while (1)
    {
        msg.distance_cm = hcsr04_read_distance(); /* -1 on error */ 
        /* concise log */ 
        if (msg.distance_cm < 0)
        {
            ESP_LOGI(TAG, "Distance: ERROR");
        }
        else
        {
            ESP_LOGI(TAG, "Distance: %.1f cm", msg.distance_cm);
        }
        xQueueOverwrite(qSensor, &msg);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/*
* @brief Task: Controls LEDs based on person detection and distance rules.
* Person detection overrides obstacle LEDs.
*/
static void task_actuator(void *arg)
{
    detect_msg_t dmsg = {false, 0};
    sensor_msg_t smsg = {-1.0f};

    while (1)
    {
        /* peek latest detection (if any) */ 
        xQueuePeek(qDetect, &dmsg, 0);
        xQueuePeek(qSensor, &smsg, 0);

        if (dmsg.person)
        {
            led_set_far(false);
            led_set_near(false);
        }
        else if (!dmsg.person)
        {
            if (smsg.distance_cm > 0 && smsg.distance_cm <= NEAR_THRESHOLD_CM)
            {
                /* near */ 
                led_set_near(true);
                led_set_far(false);
            }
            else if (smsg.distance_cm > NEAR_THRESHOLD_CM && smsg.distance_cm <= FAR_THRESHOLD_CM)
            {
                /* far */ 
                led_set_near(false);
                led_set_far(true);
            }
            else
            {
                /* clear */ 
                led_set_near(false);
                led_set_far(false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*
* @brief Main application entry point.
* Initializes camera, LEDs, HC-SR04, TFLM, queues, and starts tasks.
*/
extern "C" void app_main(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    /* init hardware */ 
    ESP_LOGI(TAG, "Init camera");
    if (camera_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "camera_init failed");
        return;
    }

    ESP_LOGI(TAG, "Init LEDs");
    if (led_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "led_init failed");
        return;
    }

    ESP_LOGI(TAG, "Init HC-SR04");
    if (hcsr04_init() != ESP_OK)
    {
        ESP_LOGW(TAG, "hcsr04_init warned (proceeding)");
    }

    /* init TFLM */ 
    tflite_mutex = xSemaphoreCreateMutex();
    if (!tflite_mutex)
    {
        ESP_LOGE(TAG, "mutex create failed");
        return;
    }

    if (!tflite_init())
    {
        ESP_LOGE(TAG, "tflite_init failed");
        return;
    }
    log_memory_info();

    /* create queues (single-element latest) */ 
    qDetect = xQueueCreate(1, sizeof(detect_msg_t));
    qSensor = xQueueCreate(1, sizeof(sensor_msg_t));
    if (!qDetect || !qSensor)
    {
        ESP_LOGE(TAG, "queue create failed");
        return;
    }

    /* put sensible default values */ 
    detect_msg_t d0 = {false, 0};
    sensor_msg_t s0 = {-1.0f};
    xQueueOverwrite(qDetect, &d0);
    xQueueOverwrite(qSensor, &s0);

    /* create tasks */ 
    xTaskCreate(task_person_detection, "person_det", 8192, NULL, 6, NULL);
    xTaskCreate(task_sensor_read, "sensor_read", 2048, NULL, 5, NULL);
    xTaskCreate(task_actuator, "actuator", 2048, NULL, 4, NULL);

    ESP_LOGI(TAG, "System started (minimal logs).");
}