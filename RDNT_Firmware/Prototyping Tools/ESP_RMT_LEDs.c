#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

// Dependencies: led_strip_encoder.h and led_strip_encoder.c
// Modified from: https://github.com/espressif/esp-idf/blob/v5.0.1/examples/peripherals/rmt/led_strip/main/led_strip_example_main.c

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 27

#define LED_NUMBERS 4
#define CHASE_SPEED_MS 100

static const char *TAG = "example";

static uint8_t led_strip_pixels[LED_NUMBERS * 3];

volatile int LEDs_ENABLED = 0;

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;

static void LightUp(void *arg)
{
    for (;;)
    {
        for (int j = 0; j < LEDs_ENABLED; j++)
        {
            led_strip_pixels[j * 3 + 0] = 0;   // green
            led_strip_pixels[j * 3 + 1] = 255; // red
            led_strip_pixels[j * 3 + 2] = 0;   // blue
        }
        vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
    }
}

static void Incrementer(void *arg)
{
    for (;;)
    {
        LEDs_ENABLED = (LEDs_ENABLED + 1) % 5;
        memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
        vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
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

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    xTaskCreate(LightUp, "LightUp", 2048, NULL, 10, NULL);
    xTaskCreate(Incrementer, "Incrementer", 2048, NULL, 15, NULL);

    while (1)
    {
        // Flush RGB values to LEDs
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
    }
}