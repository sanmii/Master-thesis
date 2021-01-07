/*
  Example code for the MC33926 Library.
  Created by Martino Migliavacca, August 2, 2013.
  Released into the public domain.
*/

#include <Triskar.h>
#include <DualMC33926MotorShield.h>

DualMC33926MotorShield m1(50,3,48,20);//M1DIR, M1PWM, nD2, nSF
DualMC33926MotorShield m2m3(46,2,42,16,19,21);// M2DIR, M2PWM, M3DIR, M3PWM, nD2, nSF
Triskar triskar(m1, m2m3);

int pwm = 0;
int diff = 10;

void setup() {
  triskar.stop();
}

void loop()  {
  triskar.run(0, ((float)(pwm)) / 255, 0);

  pwm = pwm + diff;

  if (pwm <= -255 || pwm >= 255) {
    diff = -diff ; 
  }
  
  delay(10);
}
