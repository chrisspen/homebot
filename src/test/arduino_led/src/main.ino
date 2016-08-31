#include <SoftPWM.h>

#define LED1 8
#define LED2 9
#define LED3 10

void setup()
{
  SoftPWMBegin();
  
  SoftPWMSet(LED1, 0);
  SoftPWMSet(LED2, 0);
  SoftPWMSet(LED3, 0);

  SoftPWMSetFadeTime(LED1, 1000, 1000);
  SoftPWMSetFadeTime(LED2, 1000, 1000);
  SoftPWMSetFadeTime(LED3, 1000, 1000);
}

void loop()
{
  SoftPWMSet(LED1, 255);
  SoftPWMSet(LED2, 255);
  SoftPWMSet(LED3, 255);
  delay(1000);
  SoftPWMSet(LED1, 0);
  SoftPWMSet(LED2, 0);
  SoftPWMSet(LED3, 0);
  delay(1000);
  delay(1000);
}
