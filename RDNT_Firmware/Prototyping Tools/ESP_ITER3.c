/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include <float.h>

#include <stdlib.h>
#include <inttypes.h>
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include <math.h>

#include "esp_dsp.h"

#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"


#define MAX(a, b) ((a) > (b) ? (a) : (b))
// U NEED TO SET THE BLUETOOTH CORE TO 1 IN YOUR CONFIGURATION SETTINGS OR THE LEDS WILL FLICKER

#define GPIO_INPUT_27 GPIO_NUM_27
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_27)

#define ESP_INTR_FLAG_DEFAULT 0
#define DEBOUNCE_TIME_MS 100

#define GPIO_OUTPUT_18 GPIO_NUM_18
#define GPIO_OUTPUT_19 GPIO_NUM_19
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_18 | 1ULL << GPIO_OUTPUT_19)

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 4

#define LED_QTY 96

#define READ_LEN 512 // 256 true values

#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1

#define EXAMPLE_ADC_USE_OUTPUT_TYPE1 1
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1

#define WINDOW 9 // size of gaussian filter. Valid values = 3,5,7,9

double gaussianFilter[WINDOW];
uint8_t result[READ_LEN] = {0};
double true_result[READ_LEN / 2] = {0};  // this is result that has been converted to use 12 bits for voltage
double smoothedReal[READ_LEN / 2] = {0}; // zeroes values as we will add in noiseReduce()

#define FFT_SIZE 256
__attribute__((aligned(16)))
float x1[FFT_SIZE];
// Window coefficients
__attribute__((aligned(16)))
float wind[FFT_SIZE];
// working complex array
__attribute__((aligned(16)))
float y_cf[FFT_SIZE*2];
// Pointers to result arrays
float* y1_cf = &y_cf[0];
float* y2_cf = &y_cf[FFT_SIZE];



static uint32_t last_interrupt_time = 0;

volatile int state;

int led_status = 1;        // start with on mode
int colors[3] = {1, 0, 0}; // grb

static const char *TAG = "EXAMPLE";
#define GATTS_TAG "BLUEETOOTH"

static uint8_t led_strip_pixels[LED_QTY * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;

static adc_channel_t channel[1] = {ADC_CHANNEL_6}; // PIN number 6 IO34
static TaskHandle_t s_task_handle;

rmt_transmit_config_t tx_config = {
    .loop_count = 0, // no transfer loop
};

static void
gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
#define GATTS_SERVICE_UUID 0x00FF
#define GATTS_CHAR_UUID 0xFF01
#define GATTS_NUM_HANDLE 4
#define TEST_DEVICE_NAME "ESP32"
#define TEST_MANUFACTURER_DATA_LEN 17
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define PREPARE_BUF_MAX_SIZE 1024
static esp_gatt_char_prop_t a_property = 0;
static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)
static uint8_t adv_service_uuid128[32] = {
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xEE,
    0x00,
    0x00,
    0x00,
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
struct gatts_profile_inst
{
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
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};
typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;
static prepare_type_env_t a_prepare_write_env;
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;

    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;

                if (prepare_write_env->prepare_buf == NULL)
                {
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            if (status == ESP_GATT_OK)
            {
                if (param->write.offset <= PREPARE_BUF_MAX_SIZE && (param->write.offset + param->write.len) <= PREPARE_BUF_MAX_SIZE)
                {
                    memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value, param->write.len);
                    prepare_write_env->prepare_len += param->write.len;
                }
                else
                {
                    status = (param->write.offset > PREPARE_BUF_MAX_SIZE) ? ESP_GATT_INVALID_OFFSET : ESP_GATT_INVALID_ATTR_LEN;
                }
            }
        }

        esp_gatt_rsp_t gatt_rsp = {
            .attr_value.len = param->write.len,
            .attr_value.handle = param->write.handle,
            .attr_value.offset = param->write.offset,
            .attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE};

        memcpy(gatt_rsp.attr_value.value, param->write.value, param->write.len);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, &gatt_rsp);
    }
}
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE);
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep)
        {
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
            // printf("%.*s\n", param->write.len, (char *)param->write.value);

            // The following piece converts the input message from hex to int but currently indicates the ascii.
            int value = 0;
            for (int i = 0; i < param->write.len; i++)
            {
                value = (value << 8) | param->write.value[i];
            }

            switch (value)
            {
            case 48:
                led_status = 0;
                break;

            case 49:
                led_status = 1;
                break;

            case 50: // G
                colors[0] = 1;
                colors[1] = 0;
                colors[2] = 0;
                break;

            case 51: // R
                colors[0] = 0;
                colors[1] = 1;
                colors[2] = 0;
                break;

            case 52: // B
                colors[0] = 0;
                colors[1] = 0;
                colors[2] = 1;
                break;
            }
            printf("Led_status : %d ", led_status);
            printf("Red : %d ", colors[0]);
            printf("Blue : %d ", colors[1]);
            printf("Green : %d ", colors[2]);

            ESP_LOGI(GATTS_TAG, "Value of Message %d", value);
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                    {
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(notify_data), notify_data, false);
                    }
                }
                else if (descr_value == 0x0002)
                {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                    {
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }
                else
                {
                    ESP_LOGE(GATTS_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                }
            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

        uint16_t char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                                        &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        char_property, NULL, NULL);

        if (add_char_ret)
        {
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
    {
        uint16_t length = 0;
        const uint8_t *prf_char;
        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
        if (get_attr_ret == ESP_FAIL)
        {
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }
        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
        for (int i = 0; i < length; i++)
        {
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n", i, prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret)
        {
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
    {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20;
        conn_params.min_int = 0x10;
        conn_params.timeout = 400;
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK)
        {
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    default:
        break;
    }
}
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    if (gatts_if == ESP_GATT_IF_NONE ||
        gatts_if == gl_profile_tab[0].gatts_if)
    {
        if (gl_profile_tab[0].gatts_cb)
        {
            gl_profile_tab[0].gatts_cb(event, gatts_if, param);
        }
    }
}

void ble_init()
{
    esp_err_t ret;

    // Initialize NVS and Bluetooth
    ret = nvs_flash_init();
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_init();
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_enable();
    ESP_ERROR_CHECK(ret);

    // Register GATTS and GAP callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    ESP_ERROR_CHECK(ret);

    ret = esp_ble_gap_register_callback(gap_event_handler);
    ESP_ERROR_CHECK(ret);

    // Register GATTS application
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    ESP_ERROR_CHECK(ret);
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t interrupt_time = xTaskGetTickCountFromISR();
    if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME_MS)
    {
        state++;
        if (state == 3)
        {
            state = 0;
        }
        switch (state)
        {
        case 0:
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 0);
            break;
        case 1:
            gpio_set_level(GPIO_NUM_18, 1);
            gpio_set_level(GPIO_NUM_19, 0);
            break;
        case 2:
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 1);
            break;
        }

        last_interrupt_time = interrupt_time;
    }
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 32 * 1000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        uint8_t unit = ADC_UNIT_1;
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static bool check_valid_data(const adc_digi_output_data_t *data)
{
    if (data->type1.channel >= SOC_ADC_CHANNEL_NUM(ADC_UNIT_1))
    {
        return false;
    }
    return true;
}

void LightTask(void *arg)
{
    for (;;)
    {
        memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
        for (int led = 0; led < LED_QTY; led++)
        {
            led_strip_pixels[led * 3 + 0] = 255 * led_status * colors[0]; // green
            led_strip_pixels[led * 3 + 1] = 255 * led_status * colors[1]; // red
            led_strip_pixels[led * 3 + 2] = 255 * led_status * colors[2]; // blue
        }
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void buildGaussianFilter()
{
    double left, right;
    double sum = 0;
    double sigma = ((float)(WINDOW - 1)) / 6.0; // customizable sigma

    for (int i = 0; i < WINDOW; i++)
    {
        int t = -(WINDOW / 2) + i;

        left = 1.0 / (sigma * sqrt(2.0 * M_PI));
        right = exp(-(pow(t, 2) / (2 * pow(sigma, 2))));

        gaussianFilter[i] = left * right;
        sum += gaussianFilter[i];
        // Serial.println(gaussianFilter[i], DEC);
    }

    if (sum != 1) // normalize discretized Gaussian filter to continous where integral = 1
    {
        for (int j = 0; j < WINDOW; j++)
        {
            gaussianFilter[j] = gaussianFilter[j] / sum;
            // Serial.println(gaussianFilter[j], DEC);
        }
    }
}

void process_and_show()
{
    dsps_wind_hann_f32(wind, FFT_SIZE);
    float FFTresults[FFT_SIZE] = {0};
    // Convert two input vectors to one complex vector
    for (int i=0 ; i < FFT_SIZE ; i++)
    {
        y_cf[i*2 + 0] = smoothedReal[i] * wind[i]; // Real part is your signal multiply with window
        y_cf[i*2 + 1] = 0 * wind[i];
        FFTresults[i] = y_cf[i*2];
    }
    //printf("---------------------\n");
    dsps_fft2r_fc32(y_cf, FFT_SIZE);
    //Bit reverse 
    dsps_bit_rev_fc32(y_cf, FFT_SIZE);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(y_cf, FFT_SIZE);
    
    //y1_cf - is your result in log scale
    //y2_cf - magnitude of your signal in linear scale

    // for (int i = 0 ; i < FFT_SIZE ; i++) {
    //    printf("%f\n",y2_cf[i]);
    // }
    // printf("---------------\n");

    int histogram[8] = {0}; // 6 bins for the equal intervals, 1 underflow bin, 1 overflow bin
    int max_frequency = 0;
    int max_frequency_index = 0;

    // Find the minimum and maximum values in the FFT array
    float min_value = FLT_MAX;
    float max_value = FLT_MIN;
    for (int i = 0; i < 256; i++) {
        if (FFTresults[i] < min_value) {
            min_value = FFTresults[i];
        }
        if (FFTresults[i] > max_value) {
            max_value = FFTresults[i];
        }
    }

    // Calculate bin width based on the range
    float bin_width = (max_value - min_value) / 6;

    // Count the frequency of values in each bin, and add values outside the range to the underflow and overflow bins
    for (int i = 0; i < 256; i++) {
        int bin_index = (int)((FFTresults[i] - min_value) / bin_width);
        if (bin_index < 0) {
            histogram[0]++; // Underflow bin
        } else if (bin_index >= 6) {
            histogram[7]++; // Overflow bin
        } else {
            histogram[bin_index + 1]++;
        }
    }

    // Find the bin with the maximum frequency
    for (int i = 0; i < 8; i++) {
        if (histogram[i] > max_frequency) {
            max_frequency = histogram[i];
            max_frequency_index = i;
        }
    }

    float postNorm[6] = {0};

    for (int i = 1; i < 7; i ++) //normalize to values from 0 - 15
    {
        //printf("histo pre norm: %i\n",histogram[i]);
        postNorm[i-1] = ((float) histogram[i]/ histogram[max_frequency_index]) * 15;
        //printf("histo post norm: %f\n",postNorm[i-1]);
    }
    
    // Pattern 1 - PURE COLORS
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    for (int block = 0; block < 6; block++) {
        int fraction = postNorm[block];

        for (int led = 0; led < fraction; led++) {
            int led_position = block % 2 == 0 ? led : 15 - led;

            led_strip_pixels[block * 3 * 16 + led_position * 3 + 0] = 255 * led_status * colors[0];
            led_strip_pixels[block * 3 * 16 + led_position * 3 + 1] = 255 * led_status * colors[1];
            led_strip_pixels[block * 3 * 16 + led_position * 3 + 2] = 255 * led_status * colors[2];
        }
    }

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    
}


void lightBt()
{
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    for (int block = 0; block < 6; block++) {
        int fraction = 15;

        for (int led = 0; led < fraction; led++) {
            int led_position = block % 2 == 0 ? led : 15 - led;

            led_strip_pixels[block * 3 * 16 + led_position * 3 + 0] = 255 * led_status * colors[0];
            led_strip_pixels[block * 3 * 16 + led_position * 3 + 1] = 255 * led_status * colors[1];
            led_strip_pixels[block * 3 * 16 + led_position * 3 + 2] = 255 * led_status * colors[2];
        }
    }

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    
}

// algorithms that we can look into.
void noiseReduce()
{
    for (int i = 0; i < WINDOW / 2; i++) // left tail retains value, hence not true convolution
    {
        smoothedReal[i] = true_result[i];
    }

    for (int i = WINDOW / 2; i < (READ_LEN / 2) - WINDOW / 2; i++)
    {
        smoothedReal[i] = 0;
        for (int j = 0; j < WINDOW; j++)
        {
            int index = i + -(WINDOW / 2) + j;
            smoothedReal[i] += gaussianFilter[j] * true_result[index];
        }
    }

    for (int i = (READ_LEN / 2) - WINDOW / 2; i < (READ_LEN / 2); i++) // right tail retains value, hence not true convolution
    {
        smoothedReal[i] = true_result[i];
    }
}

void app_main(void)
{
    buildGaussianFilter();
    // printf("GaussianFilter:\n");
    // for (int i = 0; i < WINDOW; i++)
    // {
    //     printf("%lf\n", gaussianFilter[i]);
    // }
    esp_err_t ret;
    ESP_LOGI(TAG, "Start Example.");
    ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }

    ESP_LOGI(TAG, "Create RMT TX channel");

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");

    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    for (int j = 0; j < LED_QTY; j++)
    {
        led_strip_pixels[j * 3 + 0] = 0; // green
        led_strip_pixels[j * 3 + 1] = 0; // red
        led_strip_pixels[j * 3 + 2] = 0; // blue
    }

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));

    ble_init();

    //xTaskCreatePinnedToCore(LightTask, "LED", 4096, NULL, 10, NULL, 0);

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set, GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(GPIO_OUTPUT_19, 0); // sel2 = A1
    gpio_set_level(GPIO_OUTPUT_18, 0); // sel1 = A0

    // bit mask of the pins, use GPIO27 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    // interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_27, gpio_isr_handler, (void *)GPIO_INPUT_27);

    uint32_t ret_num = 0;

    memset(result, 0xcc, READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1)
    {
        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1)
        {
            ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 1);
            if (ret == ESP_OK)
            {
                // ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32, ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (void *)&result[i];

                    if (check_valid_data(p))
                    {
                        // ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %u", 1, p->type1.channel, result[i]);
                        true_result[i / 2] = (p->type1.data); //* 3.7 / 4096;
                        // printf("%lf\n", (p->type1.data) * 3.7 / 4096);
                    }
                    else // TODO what should happen on invalid data?
                    {
                        ESP_LOGI(TAG, "Invalid data");
                    }
                }

                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */

                
                if(state == 2)
                {
                    lightBt();
                }
                else
                {
                    noiseReduce();
                    process_and_show();
                }
                // PSSC 1 - Gaussian Smooth could be displayed via Excel

                // printf("Post Smoothed:\n");
                //  for (int i = 0; i < (READ_LEN / 2); i++)
                //  {
                //      printf("%lf\n", smoothedReal[i]);
                //  }
                vTaskDelay(100/portTICK_PERIOD_MS);
                //not sure if this needs a vTaskDelay here even when not printing
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                // We try to read `READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}