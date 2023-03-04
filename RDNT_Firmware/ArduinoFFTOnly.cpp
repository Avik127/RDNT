#include <string.h>
#include <Arduino.h>
#include <arduinoFFT.h>

using namespace std;

// (Heavily) adapted from https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display/blob/master/ESP32_Spectrum_Display_02.ino
// Adjusted to allow brightness changes on press+hold, Auto-cycle for 3 button presses within 2 seconds
// Edited to add Neomatrix support for easier compatibility with different layouts.

#define SAMPLES         2048          // Must be a power of 2, determines the size of vReal and vImag arrays
#define SAMPLING_FREQ   36864         // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define AMPLITUDE       1000          // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.
#define AUDIO_IN_PIN    35            // Signal in on this pin

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


void detectBeat (float freq)
{
  if (freq < 20.00) { Serial.println("too low"); }
  if (freq >= 20.00 && freq < 150.00) { Serial.println("sub bass"); }
  if (freq >= 150.00 && freq < 500.00) { Serial.println("bass"); } 
  if (freq >= 500.00 && freq < 2000.00) { Serial.println("mid"); }
  if (freq >= 2000.00 && freq < 4400.00) { Serial.println("high-mid"); }
  if (freq >= 4400.00) { Serial.println("too high"); }
  delay(200);
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
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));
  pinMode(AUDIO_IN_PIN, INPUT);
}

void loop() {
  // Sample the audio pin
  // ========================================================================================= //
  // ESP32: 4096 bits over a range of 3.3 V
  // Arduino: 1024 bits over a range of 5 V
  // when we are reading analogRead, we can get up to 4095 that corresponds to 3.3 V
  // the FFT code expects a max value of 1023 that corresponds to a voltage of 5 V
  // IDEA 1: translate the analogRead value back into its raw voltage, and then rescale it over the 1024-bit/5 V
  //
  // double E32_scale = 3.30 / 4095.0;
  // double ard_scale = 1023.0 / 5.0;
  // double full_scale = E32_scale * ard_scale;
  // Inputs: 1kHz sine wave @ 1.55V Amplitude and 1.65V Offset
  // Results: FFT.MajorPeak() = 3278.06 (hz????) LMAO go fuck yourself idiot
  // ========================================================================================= //
  // IDEA 2: 

  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    vReal[i] = analogRead(AUDIO_IN_PIN);
    // vReal[i] = analogRead(AUDIO_IN_PIN) * 2450 / 4095; // A conversion takes about 9.7uS on an ESP32
    // vReal[i] = map(analogRead(AUDIO_IN_PIN),0,4096,0,3.3); // A conversion takes about 9.7uS on an ESP32
    // Serial.println(analogRead(AUDIO_IN_PIN));
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) { /* chill */ }
  }

  // PrintVector(vReal, (SAMPLES >> 1), SCL_INDEX);

  // Compute FFT
  // Serial.println("Computing FFT...");
  // delay(1000);
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  // Serial.println("Done Computing!");
  // delay(1000);

  // Serial.println(FFT.MajorPeak());
  float fftFreq = (FFT.MajorPeak() / 4.12);
  // Serial.println(fftFreq);
  detectBeat(fftFreq);
  // PrintVector(vReal, (SAMPLES >> 1), SCL_FREQUENCY);
  // delay(200);
  
  
  // while(1) {}

}