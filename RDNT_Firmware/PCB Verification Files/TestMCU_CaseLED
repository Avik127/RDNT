#include <Arduino.h>
#include "FastLED.h"
#include <vector>

// This program is the fastest way to confirm the MCU is working.
// This flashes the Case LED white if it is being powered correctly

// Dependancies: FastLED Arduino Library (Available through PlatformIO)

//Components: MCU Powered by external power supply (3.3V)
//            Case LED powered by external power supply (3.3V)

#define DATA_PIN 22

#define NUM_LEDS 4 // must be greater than 0

int flag;

CRGB leds[NUM_LEDS];

void seriesLightShow(int flag)
{
  if (flag == 1)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::White;
    }
    flag--;
  }
  else
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
    }
    flag++;
  }

  FastLED.show();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // GRB ordering is assumed
  flag = 0;
}

void loop()
{
  seriesLightShow(flag);
}