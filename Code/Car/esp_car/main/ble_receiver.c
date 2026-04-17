#include "ble_receiver.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "BLE_RECEIVER";

#define DEVICE_NAME      "ESP32-Vehicle"
#define ESP_APP_ID       0x55
#define GATTS_NUM_HANDLE 4

// Must match controller UUIDs exactly
#define SERVICE_UUID  0x4FAF
#define CHAR_UUID     0xBEB5

// Exported command state
char     ble_cmd_dir[8]  = "IDLE";
uint32_t ble_cmd_duty    = 0;
bool     ble_cmd_updated = false;

static bool     is_connected = false;
static uint16_t service_handle;
static uint16_t char_handle;

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
    strncpy(ble_cmd_dir, buf,   sizeof(ble_cmd_dir) - 1);
    ble_cmd_duty    = (uint32_t)atoi(sep + 1);
    ble_cmd_updated = true;

    ESP_LOGI(TAG, "DIR: %s  DUTY: %lu", ble_cmd_dir, ble_cmd_duty);
}

// BLE advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gatts_event_handler(
    esp_gatts_cb_event_t       event,
    esp_gatt_if_t              gatts_if,
    esp_ble_gatts_cb_param_t  *param)
{
    switch (event) {

    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATTS registered");
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_start_advertising(&adv_params);

        esp_gatt_srvc_id_t service_id = {
            .is_primary       = true,
            .id.inst_id       = 0,
            .id.uuid.len      = ESP_UUID_LEN_16,
            .id.uuid.uuid.uuid16 = SERVICE_UUID,
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(service_handle);
        ESP_LOGI(TAG, "Service created and started");

        esp_bt_uuid_t char_uuid = {
            .len            = ESP_UUID_LEN_16,
            .uuid.uuid16    = CHAR_UUID,
        };
        esp_ble_gatts_add_char(
            service_handle,
            &char_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_READ  |
            ESP_GATT_CHAR_PROP_BIT_WRITE |
            ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            NULL, NULL
        );
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        char_handle = param->add_char.attr_handle;
        ESP_LOGI(TAG, "Characteristic added, handle: %d", char_handle);
        break;

    case ESP_GATTS_CONNECT_EVT:
        is_connected = true;
        ESP_LOGI(TAG, "Controller connected");
        // Request faster connection interval for lower latency
        esp_ble_conn_update_params_t conn_params = {
            .latency  = 0,
            .max_int  = 0x10,
            .min_int  = 0x06,
            .timeout  = 400,
        };
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        ESP_LOGI(TAG, "Controller disconnected — restarting advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        parse_command(param->write.value, param->write.len);

        // Send write response
        if (!param->write.is_prep) {
            esp_ble_gatts_send_response(
                gatts_if,
                param->write.conn_id,
                param->write.trans_id,
                ESP_GATT_OK, NULL
            );
        }
        break;

    default:
        break;
    }
}

void ble_receiver_init(void) {
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Init BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Init Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register GATTS callback
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(ESP_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));

    ESP_LOGI(TAG, "BLE receiver ready — advertising as '%s'", DEVICE_NAME);
}