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

#include <stdlib.h>

#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include <math.h>

#include "esp_dsp.h"

#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#define GPIO_OUTPUT_18 GPIO_NUM_18
#define GPIO_OUTPUT_19 GPIO_NUM_19
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_18 | 1ULL << GPIO_OUTPUT_19)

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 4

#define LED_QTY 36

#define READ_LEN 512
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1

#define EXAMPLE_ADC_USE_OUTPUT_TYPE1 1
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1

int state = 1;

float amplitudes[LED_QTY] = {0};

static const char *TAG = "EXAMPLE";

uint8_t result[READ_LEN] = {0};

static uint8_t led_strip_pixels[LED_QTY * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;

static adc_channel_t channel[1] = {ADC_CHANNEL_6}; // PIN number 6 IO34
static TaskHandle_t s_task_handle;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
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

void LightShow() // 1-dimensional LightShow
{

    // for (int i = 0; i < LED_QTY; i++)
    // {
    //     amplitudes[i] = 0;
    // }

    // // turns fft result into array of LED_QTY length
    // int divider = READ_LEN / LED_QTY;
    // for (int i = 0; i < READ_LEN; i++)
    // {

    //     amplitudes[i / divider] += result[i];
    // }

    // float sum = amplitudes[0];
    // float max_amplitude = amplitudes[0];
    // for (int i = 1; i < LED_QTY; i++)
    // {
    //     if (amplitudes[i] > max_amplitude)
    //     {
    //         max_amplitude = amplitudes[i];
    //     }
    //     sum += amplitudes[i];
    // }
    // int mean = sum / LED_QTY;

    // for (int led = 0; led < LED_QTY; led++)
    // {
    //     if (amplitudes[led] > mean)
    //         led_strip_pixels[led * 3 + 1] = 255;
    //     else
    //         led_strip_pixels[led * 3 + 1] = 0;
    // }
    state = (state * 2) % 15;
    for (int led = 0; led < LED_QTY; led++)
    {
        if (led < 5)
        {
            led_strip_pixels[led * 3 + 0] = (state & 0b001) * 255;        // green
            led_strip_pixels[led * 3 + 1] = ((state & 0b010) >> 1) * 255; // red
            led_strip_pixels[led * 3 + 2] = ((state & 0b100) >> 2) * 255; // blue
        }
        else
        {
            led_strip_pixels[led * 3 + 1] = 0;
        }
    }
}

void app_main(void)
{
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
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));

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

    esp_err_t ret;
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
                // for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                // {
                //     adc_digi_output_data_t *p = (void *)&result[i];
                //     if (check_valid_data(p))
                //     {
                //         ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %u", 1, p->type1.channel, result[i]);
                //     }
                //     else
                //     {
                //         ESP_LOGI(TAG, "Invalid data");
                //     }
                // }
                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
                LightShow();
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                vTaskDelay(333 / portTICK_PERIOD_MS);
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