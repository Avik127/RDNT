#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "esp_log.h"

#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include <math.h>

#include "esp_dsp.h"

// Dependencies: led_strip_encoder.h and led_strip_encoder.c from https://github.com/espressif/esp-idf/blob/v5.0.1/examples/peripherals/rmt/led_strip/main/
// esp_dsp.h component library for espressif-idf

// This program works to confirm PSSC 5

#define GPIO_OUTPUT_18 GPIO_NUM_18
#define GPIO_OUTPUT_19 GPIO_NUM_19
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_18 | 1ULL << GPIO_OUTPUT_19)

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 22

#define LED_QTY 4
#define NUM_BLOCKS 1

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_11
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)

#define READ_LEN 512
float amplitudes[LED_QTY];

static const char *TAG = "example";

static uint8_t led_strip_pixels[LED_QTY * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;

static adc_channel_t channel[1] = {ADC_CHANNEL_6}; // PIN number 6 IO34
static TaskHandle_t s_task_handle;

uint8_t result[READ_LEN] = {0};

bool FFTtime = false;

int N = READ_LEN;
// Input test array
__attribute__((aligned(16))) float x1[READ_LEN];
__attribute__((aligned(16))) float x2[READ_LEN] = {0};
// Window coefficients
__attribute__((aligned(16))) float wind[READ_LEN];
// working complex array
__attribute__((aligned(16))) float y_cf[READ_LEN * 2];
// Pointers to result arrays
float *y1_cf = &y_cf[0];
float *y2_cf = &y_cf[READ_LEN];

float sincostab[CONFIG_DSP_MAX_FFT_SIZE];
// Sum of y1 and y2
__attribute__((aligned(16))) float sum_y[READ_LEN / 2];

void FFTOps()
{
    for (int i = 0; i < N; i++)
    {
        y_cf[i * 2 + 0] = result[i] * wind[i];
        y_cf[i * 2 + 1] = x2[i] * wind[i];
    }
    // FFT

    // unsigned int start_b = dsp_get_cpu_cycle_count();
    // dsps_fft2r_fc32(y_cf, N);
    dsps_fft2r_fc32_ae32_(y_cf, N, sincostab); // optimized for ESP32 chip, experimental results show not useful
    // unsigned int end_b = dsp_get_cpu_cycle_count();

    // Bit reverse
    dsps_bit_rev_fc32(y_cf, N);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(y_cf, N);

    for (int i = 0; i < N / 2; i++)
    {
        y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / N);
        y2_cf[i] = 10 * log10f((y2_cf[i * 2 + 0] * y2_cf[i * 2 + 0] + y2_cf[i * 2 + 1] * y2_cf[i * 2 + 1]) / N);
        // Simple way to show two power spectrums as one plot
        sum_y[i] = fmax(y1_cf[i], y2_cf[i]);
    }

    // Show power spectrum in 64x10 window from -100 to 0 dB from 0..N/4 samples

    // ESP_LOGW(TAG, "Signal x1");
    // dsps_view(y1_cf, N / 2, 64, 10, -60, 40, '|');

    // ESP_LOGW(TAG, "Signals x1 and x2 on one plot");
    // dsps_view(sum_y, N / 2, 128, 15, 0, 100, '|');

    FFTtime = false;
}

void LightShow() // 1-dimensional LightShow
{
    for (int i = 0; i < LED_QTY; i++)
    {
        amplitudes[i] = 0;
    }
    // turns fft result into array of LED_QTY length

    int slicer = READ_LEN / LED_QTY;
    for (int i = 0; i < READ_LEN; i++)
    {

        amplitudes[i / slicer] += y1_cf[i];
    }

    // Normalize amplitudes to the range [0, 255]
    float max_amplitude = amplitudes[0];
    for (int i = 1; i < LED_QTY; i++)
    {
        if (amplitudes[i] > max_amplitude)
        {
            max_amplitude = amplitudes[i];
        }
    }
    for (int i = 0; i < LED_QTY; i++)
    {
        amplitudes[i] /= max_amplitude;
        amplitudes[i] *= 255;
    }

    for (int led = 0; led < LED_QTY; led++)
    {
        uint8_t r = 1;
        uint8_t g = 0;
        uint8_t b = 0;

        r *= amplitudes[led];
        // g *= amplitudes[led];
        // b *= amplitudes[index];

        // led_strip_pixels[led * 3 + 0] = g;
        led_strip_pixels[led * 3 + 1] = r;
        // led_strip_pixels[index * 3 + 2] = b;
    }
    //}
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
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    FFTtime = true;

    return (mustYield == pdTRUE);
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
    ESP_LOGI(TAG, "Starting!");

    ret = dsps_fft2r_init_fc32(sincostab, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }

    // Generate hann window
    dsps_wind_hann_f32(wind, N);

    uint32_t ret_num = 0;
    // uint8_t result[READ_LEN] = {0};
    memset(result, 0xcc, READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb, // runs when 512 is filled
        //.on_pool_ovf = ... runs when all 1024 are filled
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    ESP_LOGI(TAG, "Loop begin!");

    while (1)
    {

        if (FFTtime == true)
        {
            FFTOps();
            LightShow();
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            // vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);
        if (ret == ESP_OK)
        {
            // ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32, ret, ret_num);
            // for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
            // {
            //     adc_digi_output_data_t *p = (void *)&result[i];
            //     uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
            //     uint32_t data = ADC_GET_DATA(p);
            //     /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            //     if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT))
            //     {
            //         ESP_LOGI(TAG, "Unit: %s, Channel: %" PRIu32 ", Value: %lu" PRIx32, unit, chan_num, data);
            //     }
            //     else
            //     {
            //         ESP_LOGW(TAG, "Invalid data [%s_%" PRIu32 "_%" PRIx32 "]", unit, chan_num, data);
            //     }
            // }
            /**
             * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
             * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
             * usually you don't need this delay (as this task will block for a while).
             */
            vTaskDelay(1);
        }
        else if (ret == ESP_ERR_TIMEOUT)
        {
            // We try to read `READ_LEN` until API returns timeout, which means there's no available data
            break;
        }
    }
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}