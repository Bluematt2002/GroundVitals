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

#define CONTROLLER_NAME  "ESP32-Controller"

// Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b (little-endian)
static const uint8_t uuid128[16] = {
    0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
    0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f
};

// Characteristic UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8 (little-endian)
static const uint8_t char_uuid128[16] = {
    0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
};

// Exported state
char     ble_cmd_dir[8]  = "IDLE";
uint32_t ble_cmd_duty    = 0;
bool     ble_cmd_updated = false;

static bool is_connected  = false;
static bool get_server    = false;
static esp_gattc_char_elem_t  *char_elem_result  = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t       gattc_if;
    uint16_t       app_id;
    uint16_t       conn_id;
    uint16_t       service_start_handle;
    uint16_t       service_end_handle;
    uint16_t       char_handle;
    esp_bd_addr_t  remote_bda;
};

static struct gattc_profile_inst gl_profile = {
    .gattc_cb = NULL,
    .gattc_if = ESP_GATT_IF_NONE,
};

bool ble_is_connected(void) {
    return is_connected;
}

static void parse_command(const uint8_t *data, uint16_t len) {
    char buf[32] = {0};
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memcpy(buf, data, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "RX: %s", buf);

    char *sep = strchr(buf, ':');
    if (!sep) return;

    *sep = '\0';
    strncpy(ble_cmd_dir, buf, sizeof(ble_cmd_dir) - 1);
    ble_cmd_duty    = (uint32_t)atoi(sep + 1);
    ble_cmd_updated = true;

    ESP_LOGI(TAG, "DIR: %s  DUTY: %lu", ble_cmd_dir, ble_cmd_duty);
}

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
            uint8_t *adv_name     = NULL;
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

static void esp_gattc_cb(esp_gattc_cb_event_t event,
                         esp_gatt_if_t gattc_if,
                         esp_ble_gattc_cb_param_t *param)
{
    switch (event) {

    case ESP_GATTC_REG_EVT:
        gl_profile.gattc_if = gattc_if;
        ESP_LOGI(TAG, "GATTC registered");
        {
            esp_ble_scan_params_t scan_params = {
                .scan_type          = BLE_SCAN_TYPE_ACTIVE,
                .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
                .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
                .scan_interval      = 0x50,
                .scan_window        = 0x30,
                .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE
            };
            esp_ble_gap_set_scan_params(&scan_params);
        }
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
        {
            esp_ble_scan_params_t scan_params2 = {
                .scan_type          = BLE_SCAN_TYPE_ACTIVE,
                .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
                .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
                .scan_interval      = 0x50,
                .scan_window        = 0x30,
                .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE
            };
            esp_ble_gap_set_scan_params(&scan_params2);
        }
        break;

    case ESP_GATTC_SEARCH_RES_EVT: {
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 &&
            memcmp(param->search_res.srvc_id.uuid.uuid.uuid128,
                   uuid128, 16) == 0) {
            ESP_LOGI(TAG, "Service found");
            gl_profile.service_start_handle = param->search_res.start_handle;
            gl_profile.service_end_handle   = param->search_res.end_handle;
            get_server = true;
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (param->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Service search failed");
            break;
        }
        if (!get_server) {
            ESP_LOGE(TAG, "Service not found");
            break;
        }
        {
            uint16_t count = 1;
            esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_128};
            memcpy(char_uuid.uuid.uuid128, char_uuid128, 16);

            char_elem_result = malloc(sizeof(esp_gattc_char_elem_t));
            if (!char_elem_result) break;

            esp_ble_gattc_get_char_by_uuid(
                gattc_if,
                param->search_cmpl.conn_id,
                gl_profile.service_start_handle,
                gl_profile.service_end_handle,
                char_uuid,
                char_elem_result,
                &count
            );

            if (count > 0 &&
                (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
                gl_profile.char_handle = char_elem_result[0].char_handle;
                esp_ble_gattc_register_for_notify(
                    gattc_if,
                    gl_profile.remote_bda,
                    char_elem_result[0].char_handle
                );
                ESP_LOGI(TAG, "Registered for notifications");
            } else {
                ESP_LOGE(TAG, "Characteristic not found or no notify property");
            }
            free(char_elem_result);
            char_elem_result = NULL;
        }
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Notify register failed");
            break;
        }
        uint16_t count2    = 1;
        uint16_t notify_en = 1;
        esp_bt_uuid_t descr_uuid = {
            .len         = ESP_UUID_LEN_16,
            .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG
        };
        descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t));
        if (!descr_elem_result) break;

        esp_ble_gattc_get_descr_by_char_handle(
            gattc_if,
            gl_profile.conn_id,
            gl_profile.char_handle,
            descr_uuid,
            descr_elem_result,
            &count2
        );

        if (count2 > 0) {
            esp_ble_gattc_write_char_descr(
                gattc_if,
                gl_profile.conn_id,
                descr_elem_result[0].handle,
                sizeof(notify_en),
                (uint8_t *)&notify_en,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE
            );
            ESP_LOGI(TAG, "CCCD written — notifications enabled");
        } else {
            ESP_LOGE(TAG, "CCCD descriptor not found");
        }
        free(descr_elem_result);
        descr_elem_result = NULL;
        break;
    }

    case ESP_GATTC_NOTIFY_EVT:
        parse_command(param->notify.value, param->notify.value_len);
        break;

    default:
        break;
    }
}

// void ble_receiver_init(void) {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
//         ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ESP_ERROR_CHECK(nvs_flash_init());
//     }

//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
//     ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
//     ESP_ERROR_CHECK(esp_bluedroid_init());
//     ESP_ERROR_CHECK(esp_bluedroid_enable());

//     ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
//     ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_gattc_cb));
//     ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));
//     ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));

//     ESP_LOGI(TAG, "BLE client ready — scanning for controller");
// }

void ble_receiver_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE)); 
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(185)); // Changed

    // Maximize transmit power
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);

    // Scan parameters (continuous scan for better sensitivity)
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x100,  // longer interval
        .scan_window = 0x100,    // = interval → continuous scan
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };

    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));

    ESP_LOGI(TAG, "BLE client ready — optimized for long range");
}