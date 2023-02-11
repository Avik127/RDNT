#include <string.h>
#include <Arduino.h>
#include <arduinoFFT.h>
#include "FastLED.h"
#include <vector>

using namespace std;



#define SAMPLES 512         // Must be a power of 2, determines the size of vReal and vImag arrays
#define SAMPLING_FREQ 10000 // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define WINDOW 3            // size of gaussian filter. Valid values = 3,5,7,9

#define AUDIO_IN_PIN 36 // Signal in on this pin
#define DATA_PIN 4

#define NUM_LEDS 4    // must be greater than 0
#define LED_DIVIDER 6 // 6 bins of freqs. listed below

CRGB leds[NUM_LEDS];
double gaussianFilter[WINDOW];

std::tuple<int, int, int> sub; // min freq, max freq, num
std::tuple<int, int, int> bass;
std::tuple<int, int, int> lowmid;
std::tuple<int, int, int> mid;
std::tuple<int, int, int> highmid;
std::tuple<int, int, int> high;

// For PrintVector
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

// Sampling and FFT stuff
unsigned int sampling_period_us;

double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime;
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);

double smoothedReal[SAMPLES] = {0}; // zeroes values as we will add in noiseReduce()

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

// this function computes a convolution of the input sample array vReal with the gaussian filter
//  this is not a true convolution as the edge values are not accounted for. There are faster and more complex
// algorithms that we can look into.
void noiseReduce()
{
  for (int i = 0; i < WINDOW / 2; i++) // left tail retains value, hence not true convolution
  {
    smoothedReal[i] = vReal[i];
  }

  for (int i = WINDOW / 2; i < SAMPLES - WINDOW / 2; i++)
  {
    smoothedReal[i] = 0;
    for (int j = 0; j < WINDOW; j++)
    {
      int index = i + -(WINDOW / 2) + j;
      smoothedReal[i] += gaussianFilter[j] * vReal[index];
    }
  }

  for (int i = SAMPLES - WINDOW / 2; i < SAMPLES; i++) // right tail retains value, hence not true convolution
  {
    smoothedReal[i] = vReal[i];
  }
}

void seriesClearAll()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void BeatDetectionDemo(float freq)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Red;
  }
  FastLED.show();
}
void seriesLightShow(float freq)
{
  if (freq < 20.00)
  {
    Serial.println("too low");
  }

  else if (freq >= std::get<0>(sub) && freq < std::get<1>(sub))
  {
    for (int i = 0; i < std::get<2>(sub); i++)
    {
      leds[i] = CRGB::Red;
    }
    Serial.println("sub");
  }
  else if (freq >= std::get<0>(bass) && freq < std::get<1>(bass))
  {
    for (int i = 0; i < std::get<2>(bass); i++)
    {
      leds[i + std::get<2>(sub)] = CRGB::Orange;
    }
    Serial.println("bass");
  }
  else if (freq >= std::get<0>(lowmid) && freq < std::get<1>(lowmid))
  {
    for (int i = 0; i < std::get<2>(mid); i++)
    {
      leds[i + std::get<2>(sub) + std::get<2>(bass)] = CRGB::Yellow;
    }
    Serial.println("lowmid");
  }
  else if (freq >= std::get<0>(mid) && freq < std::get<1>(mid))
  {
    for (int i = 0; i < std::get<2>(mid); i++)
    {
      leds[i + std::get<2>(sub) + std::get<2>(bass) + std::get<2>(lowmid)] = CRGB::Green;
    }
    Serial.println("mid");
  }
  else if (freq >= std::get<0>(highmid) && freq < std::get<1>(highmid))
  {
    for (int i = 0; i < std::get<2>(high); i++)
    {
      leds[i + std::get<2>(sub) + std::get<2>(bass) + std::get<2>(lowmid) + std::get<2>(mid)] = CRGB::Blue;
    }
    Serial.println("highmid");
  }
  else if (freq >= std::get<0>(high) && freq < std::get<1>(high))
  {
    for (int i = 0; i < std::get<2>(high); i++)
    {
      leds[i + std::get<2>(sub) + std::get<2>(bass) + std::get<2>(lowmid) + std::get<2>(mid) + std::get<2>(highmid)] = CRGB::Indigo;
    }
    Serial.println("high");
  }

  FastLED.show();
}

void lightUp(float freq)
{
  seriesClearAll();
  seriesLightShow(freq);
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
    case SCL_INDEX:
      abscissa = (i * 1.0);
      break;
    case SCL_TIME:
      abscissa = ((i * 1.0) / SAMPLING_FREQ);
      break;
    case SCL_FREQUENCY:
      abscissa = ((i * 1.0 * SAMPLING_FREQ / SAMPLES));
      break;
    }
    Serial.print(abscissa, 6);
    if (scaleType == SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));

  pinMode(AUDIO_IN_PIN, INPUT);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // GRB ordering is assumed

  sub = std::make_tuple(20, 100, (NUM_LEDS % LED_DIVIDER > 0) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  bass = std::make_tuple(100, 200, (NUM_LEDS % LED_DIVIDER > 1) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  lowmid = std::make_tuple(200, 400, (NUM_LEDS % LED_DIVIDER > 2) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  mid = std::make_tuple(400, 2000, (NUM_LEDS % LED_DIVIDER > 3) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  highmid = std::make_tuple(2000, 5000, (NUM_LEDS % LED_DIVIDER > 4) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  high = std::make_tuple(5000, 16000, NUM_LEDS / LED_DIVIDER);

  buildGaussianFilter();
}

void loop()
{

  // Serial.println("starting!");
  for (int i = 0; i < SAMPLES; i++)
  {
    newTime = micros();
    vReal[i] = ((analogRead(AUDIO_IN_PIN)) * 3.3) / 4095; // note: AD2 plugged in outputs 1V without a program running. our program reads it as 0.87 V. 10% error
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) // this calculation is probably the problem ...
    {                                                 /* chill */
    }
  }

  PrintVector(vReal, (16), SCL_INDEX);
  delay(3000);
  noiseReduce();
  PrintVector(smoothedReal, (16), SCL_INDEX);
  delay(3000);
  // FFT.DCRemoval(); // removes voltage offset to set analog wave centered to 0V
  // FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  // FFT.Compute(FFT_FORWARD);
  // FFT.ComplexToMagnitude();

  // double prefix = FFT.MajorPeak();
  // Serial.print("prefix = ");
  // Serial.println(prefix);

  // lightUp(prefix);
}