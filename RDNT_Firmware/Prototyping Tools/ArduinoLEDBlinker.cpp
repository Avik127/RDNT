#include <Arduino.h>
#include "FastLED.h"
#include <vector>

#define DATA_PIN 4

#define NUM_LEDS 4    // must be greater than 0
#define LED_DIVIDER 5 // 5 for 5 bins of freqs. listed below

CRGB leds[NUM_LEDS];

std::tuple<int, int, int> sub; // min freq, max freq, num
std::tuple<int, int, int> low;
std::tuple<int, int, int> mid;
std::tuple<int, int, int> highmid;
std::tuple<int, int, int> high;

void seriesClearAll()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }

  FastLED.show();
  delay(100);
}

void seriesLightShow(int freq)
{
  seriesClearAll();

  if (freq >= std::get<0>(sub) && freq < std::get<1>(sub))
  {
    for (int i = 0; i < std::get<2>(sub); i++)
    {
      leds[i] = CRGB::Red;
    }
  }
  else if (freq >= std::get<0>(low) && freq < std::get<1>(low))
  {
    for (int i = 0; i < std::get<2>(low); i++)
    {
      leds[i + std::get<2>(sub)] = CRGB::Yellow;
    }
  }
  else if (freq >= std::get<0>(mid) && freq < std::get<1>(mid))
  {
    for (int i = 0; i < std::get<2>(mid); i++)
    {
      leds[i + std::get<2>(sub) + std::get<2>(low)] = CRGB::Blue;
    }
  }
  else if (freq >= std::get<0>(highmid) && freq < std::get<1>(highmid))
  {
    for (int i = 0; i < std::get<2>(highmid); i++)
    {
      leds[i + std::get<2>(sub) + std::get<2>(low) + std::get<2>(highmid)] = CRGB::Green;
    }
  }
  else if (freq >= std::get<0>(high) && freq < std::get<1>(high))
  {
    for (int i = 0; i < std::get<2>(high); i++)
    {
      leds[i + std::get<2>(sub) + std::get<2>(low) + std::get<2>(highmid) + std::get<2>(high)] = CRGB::Green;
    }
  }
  FastLED.show();
  delay(100);
}

void setup()
{

  // put your setup code here, to run once:
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // GRB ordering is assumed

  sub = std::make_tuple(20, 60, (NUM_LEDS % LED_DIVIDER > 0) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  low = std::make_tuple(60, 300, (NUM_LEDS % LED_DIVIDER > 1) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  mid = std::make_tuple(300, 2000, (NUM_LEDS % LED_DIVIDER > 2) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  highmid = std::make_tuple(2000, 6000, (NUM_LEDS % LED_DIVIDER > 3) ? (NUM_LEDS / LED_DIVIDER) + 1 : (NUM_LEDS / LED_DIVIDER));
  high = std::make_tuple(6000, 20000, NUM_LEDS / LED_DIVIDER);

  // srand(time(NULL));
}

void loop()
{
  // seriesLightShow(rand() % 6000);
  seriesLightShow(30);
  seriesLightShow(200);
  seriesLightShow(1500);
  seriesLightShow(5000);
  seriesLightShow(16000);
}
