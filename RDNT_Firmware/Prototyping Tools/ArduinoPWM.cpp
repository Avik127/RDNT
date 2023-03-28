#include <Arduino.h>
// int LEDPin = 16;

int LEDPin = 26;
int PWMChannel = 0;

int PWMFreq = 800000;
int PWMRes = 5;
int dON = 21;
int dOFF = 10;
int dRESET = 0;

hw_timer_t *timer = NULL;

void onTimer()
{
  if ((int)ledcRead(PWMChannel) != dON)
  {
    // Serial.println("PWM ON");
    ledcWrite(PWMChannel, dON);
  }
  // else
  // {
  //   // Serial.println("PWM OFF");
  //   ledcWrite(PWMChannel, dOFF);
  // }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (ledcSetup(PWMChannel, PWMFreq, PWMRes) != PWMFreq) // channel, freq, res
  {
    Serial.println("Problem channel 0!");
    while (1)
    {
      delay(1);
    }
  }

  timer = timerBegin(0, 100, true); // 80mil / 100 = 800,000 aka every 1.25 microseconds
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10, true);

  ledcAttachPin(LEDPin, PWMChannel);
  Serial.println("Starting!");
  ledcWrite(PWMChannel, 0); // set data pin to reset

  timerAlarmEnable(timer);
}

void loop()
{
  ;
}