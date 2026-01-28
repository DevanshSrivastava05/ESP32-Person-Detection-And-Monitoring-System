idf_component_register(
    SRCS
        "main.cpp"
        "camera_handler.c"
        "led_control.c"
        "hcsr04.c"
        "person_detect_model_data.cc"

    INCLUDE_DIRS
        "."

    REQUIRES 
        esp32-camera
        esp-tflite-micro
        driver
        esp_timer
        nvs_flash
        esp_system
        esp_psram 
        
)
