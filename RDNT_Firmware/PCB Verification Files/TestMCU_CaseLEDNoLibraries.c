#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (27) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_6_BIT // Set duty resolution to 6 bits
#define LEDC_ONDUTY (42)               // Set duty to on. ((2 ** 6) - 1) * (.85/1.25) = 43
#define LEDC_OFFDUTY (20)              // Set duty to off. ((2 ** 6) - 1) * (.4/1.25) = 20
#define LEDC_FREQUENCY (800000)        // Frequency in Hertz
#define NUM_LEDS (4)

volatile int pwm_count;
int state;
int divider;

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 800 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE, // check this
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
void PWMTask() // void *arg)
{
    if (state == 0) // reset signal
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        state++;
    }
    else if (state == 1) // white
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_ONDUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        state++;
    }
    else if (state == 2) // reset
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        state++;
    }
    else if (state == 3) // black
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_OFFDUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        state = 0;
    }
}

void timer_callback(void *arg)
{
    pwm_count++;
    if (pwm_count % divider == 0)
    {
        PWMTask();
    }
}

void app_main(void)
{
    state = 0;
    pwm_count = 0;
    divider = (24 * NUM_LEDS) - 1;
    // Set the LEDC peripheral configuration
    ledc_init();

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_OFFDUTY));

    esp_timer_handle_t timer;
    const esp_timer_create_args_t timer_args = {
        .callback = timer_callback,
        .arg = NULL,
        .name = "timer"};
    esp_timer_create(&timer_args, &timer);

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_timer_start_periodic(timer, 1250);
}