// #include <stdio.h>
// #include <stddef.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// void task1(void *pvParameters) {
//     while(1)
//     {
//         printf("Task running\n");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

// void app_main() {
//     xTaskCreate(task1, "task1", 4096, NULL, 5, NULL);
// }


#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_http_server.h"

#include "esp_camera.h"

#define CAMERA_MODEL_XIAO_ESP32S3
#include "../inc/camera_pins.h"

#define WIFI_SSID "ESP32-CAM-EWS"
#define WIFI_PASS "12345678"

static const char *TAG = "camera_stream";

static httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;

    char part_buf[64];

    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
    static const char* _STREAM_BOUNDARY = "\r\n--frame\r\n";
    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    while(true)
    {
        fb = esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            return ESP_FAIL;
        }

        httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

        size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);
        httpd_resp_send_chunk(req, part_buf, hlen);

        httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

        esp_camera_fb_return(fb);

        vTaskDelay(pdMS_TO_TICKS(30));
    }

    return res;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    const char* html =
        "<html>"
        "<body>"
        "<h1>ESP32 Camera</h1>"
        "<img src=\"/stream\">"
        "</body>"
        "</html>";

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void start_camera_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_start(&stream_httpd, &config);

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler
    };

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler
    };

    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
}

static void wifi_init_softap()
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .password = WIFI_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "WiFi AP started. Connect to %s", WIFI_SSID);
}

static void camera_init()
{

    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,

        .pin_d0 = Y2_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,

        .pin_xclk = XCLK_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,

        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,

        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        
        .xclk_freq_hz = 20000000,
        .pixel_format = PIXFORMAT_JPEG,

        .frame_size = FRAMESIZE_QCIF,
        .jpeg_quality = 30,
        .fb_count = 1
    };

    esp_camera_init(&config);
}

void app_main()
{
    nvs_flash_init();

    camera_init();

    wifi_init_softap();

    start_camera_server();

    ESP_LOGI(TAG, "Camera Stream Ready");
}