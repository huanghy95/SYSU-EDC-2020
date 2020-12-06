/* ESP32-CAM XRZ00D1-V240 HTTP Server example with latest ESP-IDF

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"


#include "servo_mpu6050.h"
/* CONFIG Stuff */

// WIFI config
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

static const char *TAG = "ESP32-CAM HTTP AP";

// ESP32-CAM with XRZ00D1-V240 pin map 
// (https://github.com/Electronlibre2012/ESP-CAM-Ai-Thinker-Remote-Control-Tilt-and-Yaw/issues/2#issuecomment-526942984)
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22






/* MAIN Stuff  ------------------------------------------------------------- */

void app_main(void)
{
    mcpwm_example_gpio_initialize();
    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
    xTaskCreate(show_data, "show_data", 2048, NULL, 10, NULL);
    xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
}
