#include <Arduino.h>

#define SAMPLES 1024        // Must be a power of 2, determines the size of vReal and vImag arrays
#define SAMPLING_FREQ 36864 // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.

double vReal[SAMPLES];
double vGaussian[SAMPLES];
// double blurred[SAMPLES];
unsigned long newTime;
unsigned int sampling_period_us;
int AUDIO_IN_PIN = 11;
int DAC_OUT_PIN = 14;

void PrintVector(double *vData)
{
  for (uint16_t i = 0; i < SAMPLES; i++)
  {
    Serial.print(i);
    Serial.print(" ");
    Serial.println(vData[i], 15);
  }
  Serial.println();
}

// double gOp(double val, double sd)
// {
//   return (1.0 / (sqrt(2 * M_PI) * sd)) * exp(-(pow(val, 2) / (2 * pow(sd, 2))));
// }

void Gaussian(double arr[])
{
  int sum = 0;

  for (int i = 0; i < SAMPLES; i++)
  {
    sum = sum + arr[i];
  }
  double mean = sum / SAMPLES;
  Serial.print("mean :");
  Serial.println(mean);

  double variance = 0;
  for (int j = 0; j < SAMPLES; j++)
  {
    variance += pow(arr[j] - mean, 2);
  }
  variance = variance / (SAMPLES - 1); // n-1 because we dont have a full population, just a sample
  Serial.print("variance :");
  Serial.println(variance);

  for (int i = 0; i < SAMPLES; i++)
  {
    vGaussian[i] = (1.0 / (sqrt(2 * M_PI * variance))) * exp(-(pow((i - (SAMPLES / 2) + mean) - mean, 2) / (2 * variance)));
  }

  PrintVector(vGaussian);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));
}

void loop()
{
  for (int i = 0; i < SAMPLES; i++)
  {
    newTime = micros();
    vReal[i] = analogRead(AUDIO_IN_PIN);

    while ((micros() - newTime) < sampling_period_us)
    { /* chill */
    }
  }

  PrintVector(vReal);
  delay(20000);
  Gaussian(vReal);
  delay(200000);
}