#include "ble_receiver.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "BLE_CLIENT";

// Must match controller exactly
#define CONTROLLER_NAME  "ESP32-Controller"
#define REMOTE_SERVICE_UUID    0x4FAF
#define REMOTE_CHAR_UUID       0xBEB5
#define PROFILE_NUM            1
#define PROFILE_APP_IDX        0
#define INVALID_HANDLE         0

// Exported state
char     ble_cmd_dir[8]  = "IDLE";
uint32_t ble_cmd_duty    = 0;
bool     ble_cmd_updated = false;

static bool              is_connected     = false;
static bool              get_server       = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

// Profile connection info
struct gattc_profile_inst {
    esp_gattc_cb_t       gattc_cb;
    uint16_t             gattc_if;
    uint16_t             app_id;
    uint16_t             conn_id;
    uint16_t             service_start_handle;
    uint16_t             service_end_handle;
    uint16_t             char_handle;
    esp_bd_addr_t        remote_bda;
};

static struct gattc_profile_inst gl_profile = {
    .gattc_cb    = NULL,
    .gattc_if    = ESP_GATT_IF_NONE,
};

bool ble_is_connected(void) {
    return is_connected;
}

// Parse "FWD :766" into dir and duty
static void parse_command(const uint8_t *data, uint16_t len) {
    char buf[32] = {0};
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memcpy(buf, data, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "RX: %s", buf);

    char *sep = strchr(buf, ':');
    if (!sep) return;

    *sep = '\0';
    strncpy(ble_cmd_dir, buf,  sizeof(ble_cmd_dir) - 1);
    ble_cmd_duty    = (uint32_t)atoi(sep + 1);
    ble_cmd_updated = true;

    ESP_LOGI(TAG, "DIR: %s  DUTY: %lu", ble_cmd_dir, ble_cmd_duty);
}

// GAP scan callback — finds controller by name
static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(30);
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Scan start failed");
        } else {
            ESP_LOGI(TAG, "Scanning for %s...", CONTROLLER_NAME);
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan = param;
        if (scan->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            uint8_t *adv_name = NULL;
            uint8_t  adv_name_len = 0;
            adv_name = esp_ble_resolve_adv_data(
                scan->scan_rst.ble_adv,
                ESP_BLE_AD_TYPE_NAME_CMPL,
                &adv_name_len
            );

            if (adv_name && adv_name_len > 0) {
                char name[32] = {0};
                memcpy(name, adv_name, adv_name_len < 31 ? adv_name_len : 31);
                ESP_LOGI(TAG, "Found: %s", name);

                if (strcmp(name, CONTROLLER_NAME) == 0) {
                    ESP_LOGI(TAG, "Controller found — connecting");
                    esp_ble_gap_stop_scanning();
                    esp_ble_gattc_open(
                        gl_profile.gattc_if,
                        scan->scan_rst.bda,
                        scan->scan_rst.ble_addr_type,
                        true
                    );
                }
            }
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan stopped");
        break;

    default:
        break;
    }
}

// GATTC callback — handles connection and notifications
static void esp_gattc_cb(esp_gattc_cb_event_t event,
                         esp_gatt_if_t gattc_if,
                         esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        gl_profile.gattc_if = gattc_if;
        ESP_LOGI(TAG, "GATTC registered");

        // Start scanning
        esp_ble_scan_params_t scan_params = {
            .scan_type          = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval      = 0x50,
            .scan_window        = 0x30,
            .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE
        };
        esp_ble_gap_set_scan_params(&scan_params);
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Open failed, status: %d", param->open.status);
            break;
        }
        ESP_LOGI(TAG, "Connected to controller");
        gl_profile.conn_id = param->open.conn_id;
        memcpy(gl_profile.remote_bda, param->open.remote_bda,
               sizeof(esp_bd_addr_t));
        is_connected = true;
        esp_ble_gattc_search_service(gattc_if, param->open.conn_id, NULL);
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        is_connected = false;
        get_server   = false;
        ESP_LOGI(TAG, "Disconnected — rescanning");
        // Restart scan
        esp_ble_scan_params_t scan_params2 = {
            .scan_type          = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval      = 0x50,
            .scan_window        = 0x30,
            .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE
        };
        esp_ble_gap_set_scan_params(&scan_params2);
        break;

    case ESP_GATTC_SEARCH