

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
__attribute__((aligned(16))) float x1[N_SAMPLES] = {1.826514,
                                                    1.825610,
                                                    1.792188,
                                                    1.794897,
                                                    1.763515,
                                                    1.739594,
                                                    1.711385,
                                                    1.683736,
                                                    1.659273,
                                                    1.638171,
                                                    1.621557,
                                                    1.609940,
                                                    1.600132,
                                                    1.591181,
                                                    1.584774,
                                                    1.579577,
                                                    1.570907,
                                                    1.556519,
                                                    1.540313,
                                                    1.528084,
                                                    1.520652,
                                                    1.514693,
                                                    1.504429,
                                                    1.486382,
                                                    1.463650,
                                                    1.439511,
                                                    1.414848,
                                                    1.391892,
                                                    1.371594,
                                                    1.353437,
                                                    1.332913,
                                                    1.304003,
                                                    1.270026,
                                                    1.238130,
                                                    1.206437,
                                                    1.174754,
                                                    1.146739,
                                                    1.124371,
                                                    1.109086,
                                                    1.101391,
                                                    1.101799,
                                                    1.112828,
                                                    1.131124,
                                                    1.149125,
                                                    1.166510,
                                                    1.186729,
                                                    1.210519,
                                                    1.237251,
                                                    1.268269,
                                                    1.303079,
                                                    1.335516,
                                                    1.359044,
                                                    1.375533,
                                                    1.389089,
                                                    1.400433,
                                                    1.407724,
                                                    1.410320,
                                                    1.408518,
                                                    1.401518,
                                                    1.388801,
                                                    1.371195,
                                                    1.352184,
                                                    1.336200,
                                                    1.324110,
                                                    1.312494,
                                                    1.295854,
                                                    1.267757,
                                                    1.227970,
                                                    1.185924,
                                                    1.148876,
                                                    1.113643,
                                                    1.080824,
                                                    1.054598,
                                                    1.035429,
                                                    1.022460,
                                                    1.013436,
                                                    1.003855,
                                                    0.994550,
                                                    0.988348,
                                                    0.986070,
                                                    0.990578,
                                                    1.002922,
                                                    1.019628,
                                                    1.037241,
                                                    1.054164,
                                                    1.072074,
                                                    1.094245,
                                                    1.122956,
                                                    1.157453,
                                                    1.190767,
                                                    1.220466,
                                                    1.245481,
                                                    1.260451,
                                                    1.265793,
                                                    1.270628,
                                                    1.283656,
                                                    1.303910,
                                                    1.319673,
                                                    1.325137,
                                                    1.322771,
                                                    1.314629,
                                                    1.304477,
                                                    1.290559,
                                                    1.266321,
                                                    1.236364,
                                                    1.208389,
                                                    1.181250,
                                                    1.157276,
                                                    1.140451,
                                                    1.130170,
                                                    1.125518,
                                                    1.123205,
                                                    1.118898,
                                                    1.116562,
                                                    1.123690,
                                                    1.139067,
                                                    1.155014,
                                                    1.168643,
                                                    1.180332,
                                                    1.188281,
                                                    1.191474,
                                                    1.190092,
                                                    1.187784,
                                                    1.190455,
                                                    1.198439,
                                                    1.204453,
                                                    1.202102,
                                                    1.192550,
                                                    1.183182,
                                                    1.180299,
                                                    1.179316,
                                                    1.171315,
                                                    1.157382,
                                                    1.141561,
                                                    1.125386,
                                                    1.112058,
                                                    1.100936,
                                                    1.089387,
                                                    1.079167,
                                                    1.073667,
                                                    1.076526,
                                                    1.088092,
                                                    1.102902,
                                                    1.117710,
                                                    1.132857,
                                                    1.146041,
                                                    1.157585,
                                                    1.173205,
                                                    1.202013,
                                                    1.249568,
                                                    1.306623,
                                                    1.354741,
                                                    1.388385,
                                                    1.413494,
                                                    1.436699,
                                                    1.462432,
                                                    1.492843,
                                                    1.527091,
                                                    1.560375,
                                                    1.586408,
                                                    1.605377,
                                                    1.618596,
                                                    1.626707,
                                                    1.631998,
                                                    1.634332,
                                                    1.629713,
                                                    1.615570,
                                                    1.590880,
                                                    1.558738,
                                                    1.528395,
                                                    1.504801,
                                                    1.487145,
                                                    1.473901,
                                                    1.462034,
                                                    1.449819,
                                                    1.440867,
                                                    1.438837,
                                                    1.443308,
                                                    1.449732,
                                                    1.453249,
                                                    1.454772,
                                                    1.461171,
                                                    1.479749,
                                                    1.510407,
                                                    1.548087,
                                                    1.589423,
                                                    1.631667,
                                                    1.670294,
                                                    1.706871,
                                                    1.747607,
                                                    1.794658,
                                                    1.844202,
                                                    1.891709,
                                                    1.933835,
                                                    1.973617,
                                                    2.017694,
                                                    2.066818,
                                                    2.111918,
                                                    2.145583,
                                                    2.164209,
                                                    2.165911,
                                                    2.154307,
                                                    2.141534,
                                                    2.138338,
                                                    2.141882,
                                                    2.143166,
                                                    2.137792,
                                                    2.123695,
                                                    2.101588,
                                                    2.078580,
                                                    2.061539,
                                                    2.051069,
                                                    2.044419,
                                                    2.035026,
                                                    2.018763,
                                                    2.005443,
                                                    2.005766,
                                                    2.017512,
                                                    2.031140,
                                                    2.037963,
                                                    2.035905,
                                                    2.025401,
                                                    2.009662,
                                                    1.997564,
                                                    1.991344,
                                                    1.985000,
                                                    1.974371,
                                                    1.958314,
                                                    1.936377,
                                                    1.910749,
                                                    1.887298,
                                                    1.872212,
                                                    1.861577,
                                                    1.847317,
                                                    1.828144,
                                                    1.806928,
                                                    1.786436,
                                                    1.770114,
                                                    1.758054,
                                                    1.747410,
                                                    1.735683,
                                                    1.720979,
                                                    1.705098,
                                                    1.697312,
                                                    1.704214,
                                                    1.721410,
                                                    1.740338,
                                                    1.756591,
                                                    1.771949,
                                                    1.786770,
                                                    1.798850,
                                                    1.812158,
                                                    1.862646,
                                                    1.835547,
                                                    1.905103,
                                                    1.897876};
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
