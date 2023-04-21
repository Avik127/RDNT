

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include <math.h>

#include "esp_dsp.h"

static const char *TAG = "main";

// This example shows how to use FFT from esp-dsp library

#define N_SAMPLES 256
int N = N_SAMPLES;
// Input test array
__attribute__((aligned(16))) float x1[N_SAMPLES] = {1.744312,
                                                    1.672949,
                                                    1.748828,
                                                    1.750635,
                                                    1.758188,
                                                    1.763464,
                                                    1.767123,
                                                    1.769387,
                                                    1.768844,
                                                    1.766880,
                                                    1.765696,
                                                    1.765317,
                                                    1.765151,
                                                    1.765171,
                                                    1.765388,
                                                    1.765816,
                                                    1.766550,
                                                    1.767674,
                                                    1.769198,
                                                    1.770587,
                                                    1.771420,
                                                    1.772227,
                                                    1.773194,
                                                    1.773146,
                                                    1.769025,
                                                    1.757724,
                                                    1.742451,
                                                    1.730560,
                                                    1.723931,
                                                    1.720308,
                                                    1.718297,
                                                    1.717930,
                                                    1.720725,
                                                    1.728099,
                                                    1.738347,
                                                    1.748722,
                                                    1.758423,
                                                    1.765702,
                                                    1.768779,
                                                    1.768987,
                                                    1.768079,
                                                    1.766692,
                                                    1.765324,
                                                    1.764220,
                                                    1.763469,
                                                    1.763567,
                                                    1.765587,
                                                    1.770076,
                                                    1.775562,
                                                    1.780522,
                                                    1.785178,
                                                    1.789227,
                                                    1.792223,
                                                    1.794258,
                                                    1.795380,
                                                    1.795737,
                                                    1.795649,
                                                    1.795423,
                                                    1.795321,
                                                    1.795288,
                                                    1.795289,
                                                    1.795405,
                                                    1.795631,
                                                    1.795864,
                                                    1.795993,
                                                    1.796010,
                                                    1.795851,
                                                    1.795502,
                                                    1.795339,
                                                    1.795523,
                                                    1.795471,
                                                    1.794796,
                                                    1.793913,
                                                    1.793424,
                                                    1.793485,
                                                    1.793679,
                                                    1.793485,
                                                    1.792887,
                                                    1.792009,
                                                    1.790839,
                                                    1.789451,
                                                    1.787685,
                                                    1.785261,
                                                    1.782647,
                                                    1.780395,
                                                    1.778347,
                                                    1.775923,
                                                    1.772843,
                                                    1.769115,
                                                    1.764541,
                                                    1.759294,
                                                    1.754765,
                                                    1.751907,
                                                    1.750323,
                                                    1.749070,
                                                    1.747577,
                                                    1.745828,
                                                    1.744100,
                                                    1.742789,
                                                    1.742150,
                                                    1.742108,
                                                    1.742798,
                                                    1.744333,
                                                    1.746563,
                                                    1.749589,
                                                    1.753702,
                                                    1.759185,
                                                    1.766134,
                                                    1.773987,
                                                    1.781301,
                                                    1.787124,
                                                    1.791244,
                                                    1.793300,
                                                    1.793140,
                                                    1.791957,
                                                    1.790511,
                                                    1.788780,
                                                    1.787989,
                                                    1.789258,
                                                    1.791188,
                                                    1.792210,
                                                    1.792439,
                                                    1.792591,
                                                    1.792995,
                                                    1.793307,
                                                    1.793142,
                                                    1.793048,
                                                    1.793440,
                                                    1.793677,
                                                    1.793585,
                                                    1.793613,
                                                    1.793759,
                                                    1.793813,
                                                    1.793829,
                                                    1.793813,
                                                    1.793662,
                                                    1.793426,
                                                    1.793297,
                                                    1.793366,
                                                    1.793564,
                                                    1.793813,
                                                    1.794008,
                                                    1.793974,
                                                    1.793659,
                                                    1.793243,
                                                    1.793027,
                                                    1.793140,
                                                    1.793386,
                                                    1.793565,
                                                    1.793650,
                                                    1.793584,
                                                    1.793364,
                                                    1.793112,
                                                    1.792908,
                                                    1.792802,
                                                    1.792799,
                                                    1.792802,
                                                    1.792817,
                                                    1.792950,
                                                    1.793116,
                                                    1.793114,
                                                    1.792937,
                                                    1.792795,
                                                    1.792863,
                                                    1.793157,
                                                    1.793453,
                                                    1.793462,
                                                    1.793216,
                                                    1.793087,
                                                    1.793250,
                                                    1.793406,
                                                    1.793299,
                                                    1.793016,
                                                    1.792691,
                                                    1.792503,
                                                    1.792613,
                                                    1.792897,
                                                    1.793045,
                                                    1.792997,
                                                    1.792950,
                                                    1.792934,
                                                    1.792844,
                                                    1.792705,
                                                    1.792570,
                                                    1.792410,
                                                    1.792232,
                                                    1.791997,
                                                    1.791575,
                                                    1.790887,
                                                    1.789913,
                                                    1.788658,
                                                    1.787331,
                                                    1.786119,
                                                    1.784686,
                                                    1.782585,
                                                    1.780106,
                                                    1.777267,
                                                    1.773455,
                                                    1.768869,
                                                    1.764520,
                                                    1.760668,
                                                    1.756918,
                                                    1.753208,
                                                    1.750000,
                                                    1.747136,
                                                    1.743849,
                                                    1.740320,
                                                    1.737605,
                                                    1.735783,
                                                    1.734423,
                                                    1.733607,
                                                    1.733329,
                                                    1.733258,
                                                    1.733582,
                                                    1.735163,
                                                    1.738608,
                                                    1.743423,
                                                    1.748634,
                                                    1.754063,
                                                    1.759834,
                                                    1.765592,
                                                    1.770816,
                                                    1.775827,
                                                    1.780853,
                                                    1.785221,
                                                    1.788300,
                                                    1.790400,
                                                    1.791872,
                                                    1.792843,
                                                    1.793500,
                                                    1.793651,
                                                    1.792736,
                                                    1.790680,
                                                    1.787781,
                                                    1.784777,
                                                    1.783211,
                                                    1.783968,
                                                    1.786715,
                                                    1.790235,
                                                    1.793122,
                                                    1.795250,
                                                    1.796240,
                                                    1.795227,
                                                    1.793353,
                                                    1.792296,
                                                    1.792188,
                                                    1.792439,
                                                    1.792571,
                                                    1.792458,
                                                    1.792294,
                                                    1.792212,
                                                    1.792191,
                                                    1.792188,
                                                    1.792188,
                                                    1.792188,
                                                    1.792188};
// Window coefficients
__attribute__((aligned(16))) float wind[N_SAMPLES];
// working complex array
__attribute__((aligned(16))) float y_cf[N_SAMPLES * 2];
// Pointers to result arrays
float *y1_cf = &y_cf[0];

// Sum of y1 and y2
__attribute__((aligned(16))) float sum_y[N_SAMPLES / 2];

void app_main()
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Start Example.");
    ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }

    // Generate hann window
    dsps_wind_hann_f32(wind, N);
    // Generate input signal for x1 A=1 , F=0.1
    // dsps_tone_gen_f32(x1, N, 1.0, 0.16, 0);
    // for (int i = 0; i < N; i++)
    // {
    //     printf("%f\n", x1[i]);
    // }

    // Convert two input vectors to one complex vector
    for (int i = 0; i < N; i++)
    {
        y_cf[i * 2 + 0] = x1[i] * wind[i];
        y_cf[i * 2 + 1] = 0 * wind[i];
    }
    // FFT
    unsigned int start_b = dsp_get_cpu_cycle_count();
    dsps_fft2r_fc32(y_cf, N);
    unsigned int end_b = dsp_get_cpu_cycle_count();
    // Bit reverse
    dsps_bit_rev_fc32(y_cf, N);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(y_cf, N);

    for (int i = 0; i < N / 2; i++)
    {
        y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / N);
    }

    for (int i = 0; i < N / 2; i++)
    {
        printf("%f\n", y1_cf[i]);
    }

    // Show power spectrum in 64x10 window from -100 to 0 dB from 0..N/4 samples
    ESP_LOGW(TAG, "Signal x1");
    dsps_view(y1_cf, N / 2, 64, 10, -60, 40, '|');
    // ESP_LOGW(TAG, "Signal x2");
    // dsps_view(y2_cf, N / 2, 64, 10, -60, 40, '|');
    // ESP_LOGW(TAG, "Signals x1 and x2 on one plot");
    // dsps_view(sum_y, N / 2, 64, 10, -60, 40, '|');
    ESP_LOGI(TAG, "FFT for %i complex points take %i cycles", N, end_b - start_b);

    ESP_LOGI(TAG, "End Example.");
}
