#include "DualMC33926MotorShield.h"


//MOTORS AND ENCODERS PINS AND OBJECTS
//-------SHIELD 1-------
#define nD2m1 10
#define nSFm1 11
//--------Motor 1-----
#define M1DIR 46
#define M1PWM 12
#define M1FB A6

//-------SHIELD 2-------
#define nD2m23 48
#define nSFm23 52
//--------Motor 2-----
#define M2DIR 50
#define M2PWM 4 //17
#define M2FB A7
//--------Motor 3-----
#define M3DIR 40
#define M3PWM 13 //16
#define M3FB A5

//-------ENCODER 1-------
#define EncA1 2
#define EncB1 3
//-------ENCODER 2-------
#define EncA2 18
#define EncB2 19
//-------ENCODER 3-------
#define EncA3 20
#define EncB3 21

DualMC33926MotorShield md(M3DIR, M3PWM, M3FB, nD2m23, nSFm23);
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual MC33926 Motor Shield");
  md.init1M();
}

void loop()
{
  for (int i = 0; i <= 200; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = 200; i >= -200; i--)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = -200; i <= 0; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }

//  for (int i = 0; i <= 200; i++)
//  {
//    md.setM2Speed(i);
//    stopIfFault();
//    if (abs(i)%200 == 100)
//    {
//      Serial.print("M2 current: ");
//      Serial.println(md.getM2CurrentMilliamps());
//    }
//    delay(2);
//  }
//  
//  for (int i = 200; i >= -200; i--)
//  {
//    md.setM2Speed(i);
//    stopIfFault();
//    if (abs(i)%200 == 100)
//    {
//      Serial.print("M2 current: ");
//      Serial.println(md.getM2CurrentMilliamps());
//    }
//    delay(2);
//  }
//  
//  for (int i = -200; i <= 0; i++)
//  {
//    md.setM2Speed(i);
//    stopIfFault();
//    if (abs(i)%200 == 100)
//    {
//      Serial.print("M2 current: ");
//      Serial.println(md.getM2CurrentMilliamps());
//    }
//    delay(2);
//  }
}
