#include "ble.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BLE";

#define DEVICE_NAME "ESP32-S3-SpO2"

#define PROFILE_NUM         1
#define PROFILE_APP_IDX     0

#define SVC_INST_ID         0
#define CHAR_INST_ID        0

#define GATTS_SERVICE_UUID  0xFFF0
#define GATTS_CHAR_UUID     0xFFF1
#define GATTS_DESCR_UUID    0x2902

static uint16_t gatt_if = 0;
static uint16_t conn_id = 0;
static uint16_t char_handle = 0;
static bool connected = false;
static bool service_started = false;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
};

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM];

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                         esp_gatt_if_t gatts_if,
                                         esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gatt_if = gatts_if;
            gl_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
            ESP_LOGI(TAG, "GATTS_REG_EVT, gatt_if=%d", gatt_if);
        } else {
            ESP_LOGE(TAG, "Reg app failed, app_id=%04x, status=%d",
                     param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE ||
                gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                         esp_gatt_if_t gatts_if,
                                         esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(TAG, "REGISTER_APP_EVT, status=%d, app_id=%d",
                     param->reg.status, param->reg.app_id);
            gl_profile_tab[PROFILE_APP_IDX].service_id.is_primary = true;
            gl_profile_tab[PROFILE_APP_IDX].service_id.id.inst_id = SVC_INST_ID;
            gl_profile_tab[PROFILE_APP_IDX].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_IDX].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;

            esp_err_t ret = esp_ble_gatts_create_service(gatts_if,
                                          &gl_profile_tab[PROFILE_APP_IDX].service_id, 3);
            ESP_LOGI(TAG, "create_service ret=%d", ret);
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status=%d, service_handle=%d",
                     param->create.status, param->create.service_handle);
            gl_profile_tab[PROFILE_APP_IDX].service_handle = param->create.service_handle;
            gl_profile_tab[PROFILE_APP_IDX].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_IDX].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID;

            esp_err_t ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_IDX].service_handle,
                                   &gl_profile_tab[PROFILE_APP_IDX].char_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ |
                                   ESP_GATT_CHAR_PROP_BIT_WRITE |
                                   ESP_GATT_CHAR_PROP_BIT_NOTIFY |
                                   ESP_GATT_CHAR_PROP_BIT_INDICATE,
                                   NULL, NULL);
            ESP_LOGI(TAG, "add_char ret=%d", ret);
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT: {
            ESP_LOGI(TAG, "ADD_CHAR_EVT, status=%d, attr_handle=%d",
                     param->add_char.status, param->add_char.attr_handle);
            char_handle = param->add_char.attr_handle;
            gl_profile_tab[PROFILE_APP_IDX].char_handle = char_handle;

            /* Add CCCD descriptor (0x2902) for Notify/Indicate */
            esp_attr_control_t attr_control = {
                .auto_rsp = ESP_GATT_AUTO_RSP
            };
            esp_bt_uuid_t descr_uuid;
            descr_uuid.len = ESP_UUID_LEN_16;
            descr_uuid.uuid.uuid16 = GATTS_DESCR_UUID;
            esp_attr_value_t cccd_value = {
                .attr_max_len = 2,
                .attr_len = 2,
                .attr_value = (uint8_t[2]){0x00, 0x00}
            };
            esp_err_t ret = esp_ble_gatts_add_char_descr(
                gl_profile_tab[PROFILE_APP_IDX].service_handle,
                &descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                &cccd_value, &attr_control);
            ESP_LOGI(TAG, "add_char_descr ret=%d", ret);
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
            ESP_LOGI(TAG, "CCCD descriptor added, handle=%d, status=%d",
                     param->add_char_descr.attr_handle, param->add_char_descr.status);

            /* Start service after all chars and descriptors are added */
            esp_err_t ret = esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_IDX].service_handle);
            ESP_LOGI(TAG, "start_service ret=%d", ret);
            service_started = true;

            /* Set advertising data */
            esp_ble_adv_data_t adv_data = {
                .set_scan_rsp = false,
                .include_name = true,
                .include_txpower = true,
                .min_interval = 0x0006,
                .max_interval = 0x0010,
                .appearance = 0x00,
                .manufacturer_len = 0,
                .p_manufacturer_data = NULL,
                .service_data_len = 0,
                .p_service_data = NULL,
                .service_uuid_len = 0,
                .p_service_uuid = NULL,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };

            ret = esp_ble_gap_config_adv_data(&adv_data);
            ESP_LOGI(TAG, "config_adv_data ret=%d", ret);
            break;
        }
        case ESP_GATTS_CONNECT_EVT: {
            ESP_LOGI(TAG, "CONNECT_EVT, conn_id=%d", param->connect.conn_id);
            conn_id = param->connect.conn_id;
            connected = true;
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;
            conn_params.min_int = 0x10;
            conn_params.timeout = 400;
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(TAG, "DISCONNECT_EVT, reason=0x%x", param->disconnect.reason);
            connected = false;
            /* Restart advertising */
            esp_ble_gap_start_advertising(NULL);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(TAG, "WRITE_EVT, handle=%d, len=%d", param->write.handle, param->write.len);
            break;
        }
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(TAG, "READ_EVT, handle=%d", param->read.handle);
            break;
        }
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "ADV_DATA_SET_COMPLETE, status=%d", param->adv_data_cmpl.status);
            if (param->adv_data_cmpl.status == ESP_BT_STATUS_SUCCESS && service_started) {
                esp_ble_adv_params_t adv_params = {
                    .adv_int_min = 0x20,
                    .adv_int_max = 0x40,
                    .adv_type = ADV_TYPE_IND,
                    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                    .channel_map = ADV_CHNL_ALL,
                    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
                };
                esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
                ESP_LOGI(TAG, "start_advertising ret=%d", ret);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            ESP_LOGI(TAG, "ADV_START_COMPLETE, status=%d", param->adv_start_cmpl.status);
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started! Device name: %s", DEVICE_NAME);
            } else {
                ESP_LOGE(TAG, "Advertising start failed, status=%d", param->adv_start_cmpl.status);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "SCAN_RSP_DATA_SET_COMPLETE, status=%d", param->scan_rsp_data_cmpl.status);
            break;
        default:
            break;
    }
}

void ble_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE...");

    gl_profile_tab[PROFILE_APP_IDX].gatts_cb = gatts_profile_event_handler;
    gl_profile_tab[PROFILE_APP_IDX].gatts_if = ESP_GATT_IF_NONE;

    esp_err_t ret;

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    ESP_LOGI(TAG, "mem_release ret=%d", ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_LOGI(TAG, "bt_controller_init ret=%d", ret);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ESP_LOGI(TAG, "bt_controller_enable ret=%d", ret);

    ret = esp_bluedroid_init();
    ESP_LOGI(TAG, "bluedroid_init ret=%d", ret);

    ret = esp_bluedroid_enable();
    ESP_LOGI(TAG, "bluedroid_enable ret=%d", ret);

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    ESP_LOGI(TAG, "gatts_register_callback ret=%d", ret);

    ret = esp_ble_gap_register_callback(gap_event_handler);
    ESP_LOGI(TAG, "gap_register_callback ret=%d", ret);

    ret = esp_ble_gatts_app_register(PROFILE_APP_IDX);
    ESP_LOGI(TAG, "gatts_app_register ret=%d", ret);

    ret = esp_ble_gap_set_device_name(DEVICE_NAME);
    ESP_LOGI(TAG, "set_device_name ret=%d", ret);

    ESP_LOGI(TAG, "BLE init done, waiting for events...");
}

void ble_send_health_data(uint16_t heart_rate, uint16_t spo2)
{
    if (!connected || char_handle == 0) {
        return;
    }

    uint8_t buffer[7];

    buffer[0] = 0x50;

    uint32_t hrRaw = (uint32_t)heart_rate * 1000 + 1000000;
    buffer[1] = (hrRaw >> 16) & 0xFF;
    buffer[2] = (hrRaw >> 8) & 0xFF;
    buffer[3] = hrRaw & 0xFF;

    uint32_t spo2Raw = (uint32_t)spo2 * 1000 + 1000000;
    buffer[4] = (spo2Raw >> 16) & 0xFF;
    buffer[5] = (spo2Raw >> 8) & 0xFF;
    buffer[6] = spo2Raw & 0xFF;

    esp_ble_gatts_send_indicate(gatt_if, conn_id, char_handle, 7, buffer, false);
}
