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
// It also changes the gpio outputs for the multiplexxer, corresponding to current mode
// Fixed most debouncing problems (very long presses = 2 presses is only issue left)

#define GPIO_INPUT_27 GPIO_NUM_27
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_27)

#define GPIO_OUTPUT_18 GPIO_NUM_18
#define GPIO_OUTPUT_19 GPIO_NUM_19
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_18 | 1ULL << GPIO_OUTPUT_19)

#define ESP_INTR_FLAG_DEFAULT 0
#define DEBOUNCE_TIME_MS 250

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 22

#define LED_NUMBERS 4

volatile int state; // 001 = Microphone, 010 = Aux, 100 = Bluetooth
volatile int green;
volatile int red;
volatile int blue;
volatile bool transition = false;

static const char *TAG = "example";

static uint8_t led_strip_pixels[LED_NUMBERS * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;

static uint32_t last_interrupt_time = 0;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t interrupt_time = xTaskGetTickCountFromISR();
    if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME_MS)
    {
        transition = true;
        last_interrupt_time = interrupt_time;
    }
}

static void mode_switch(void *arg)
{
    for (;;)
    {
        printf("transition: %i, state: %i \n", transition, state);

        if (transition)
        {
            transition = false;
            state = (state * 2) % 7;
            ESP_ERROR_CHECK(gpio_set_level(19, (state & 0b100) >> 2)); // sel2 = A1
            // printf("a1= %i\n", (state & 0b100) >> 2);
            ESP_ERROR_CHECK(gpio_set_level(18, (state & 0b010) >> 1)); // sel1 = A0
            // printf("a0= %i\n", (state & 0b010) >> 1);

            green = (state & 0b001) * 255;
            red = ((state & 0b010) >> 1) * 255;
            blue = ((state & 0b100) >> 2) * 255;
            for (int j = 0; j < LED_NUMBERS; j++)
            {
                led_strip_pixels[j * 3 + 0] = green; // green
                led_strip_pixels[j * 3 + 1] = red;   // red
                led_strip_pixels[j * 3 + 2] = blue;  // blue
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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

    state = 1; // 001 = Microphone, 010 = Aux, 100 = Bluetooth
    green = 255;
    red = 0;
    blue = 0;

    for (int j = 0; j < LED_NUMBERS; j++)
    {
        led_strip_pixels[j * 3 + 0] = green; // green
        led_strip_pixels[j * 3 + 1] = red;   // red
        led_strip_pixels[j * 3 + 2] = blue;  // blue
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

    printf("starting sels:%i%i \n", gpio_get_level(GPIO_OUTPUT_19), gpio_get_level(GPIO_OUTPUT_18));

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

    // start gpio task
    xTaskCreate(mode_switch, "mode_switch", 2048, NULL, 10, NULL);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_27, gpio_isr_handler, (void *)GPIO_INPUT_27);

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    while (1)
    {
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}