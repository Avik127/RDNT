#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

// Dependencies: led_strip_encoder.h and led_strip_encoder.c from https://github.com/espressif/esp-idf/blob/v5.0.1/examples/peripherals/rmt/led_strip/main/

// This program confirms the mode switch button is operating by changing the color of the case LED
// Might be slightly buggy, didnt get to test it completely

#define GPIO_MODE_INPUT 27
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_MODE_INPUT))
#define ESP_INTR_FLAG_DEFAULT 0

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 22

#define LED_NUMBERS 4
#define CHASE_SPEED_MS 100

volatile int state = 1; // 0 = Microphone, 1 = Aux, 2 = Bluetooth
volatile int green = 255;
volatile int red = 0;
volatile int blue = 0;

static const char *TAG = "example";

static uint8_t led_strip_pixels[LED_NUMBERS * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void mode_switch(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
            state = (state * 2) % 7;
            green = (state & 0b001) * 255;
            red = ((state & 0b010) * 255) >> 1;
            blue = ((state & 0b100) * 255) >> 2;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void CaseLED(void *arg)
{
    for (;;)
    {
        for (int j = 0; j < LED_NUMBERS * 3; j++)
        {
            led_strip_pixels[j * 3 + 0] = green; // green
            led_strip_pixels[j * 3 + 1] = red;   // red
            led_strip_pixels[j * 3 + 2] = blue;  // blue
        }
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

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};

    // interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // TODO: experiment with queue size
    // start gpio task
    xTaskCreate(mode_switch, "mode_switch", 2048, NULL, 10, NULL);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_MODE_INPUT, gpio_isr_handler, (void *)GPIO_MODE_INPUT);

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    xTaskCreate(CaseLED, "CaseLED", 2048, NULL, 10, NULL);

    while (1)
    {
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
    }
}