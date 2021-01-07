#include "DualMC33926MotorShield.h"

// Constructors ////////////////////////////////////////////////////////////////

DualMC33926MotorShield::DualMC33926MotorShield()
{
  //Pin map
  _nD2 = 4;
  _M1DIR = 50;
  _M2DIR = 8;
  _nSF = 12;
  _M1FB = A0; 
  _M2FB = A1;
}

DualMC33926MotorShield::DualMC33926MotorShield(unsigned char M1DIR, unsigned char M1PWM, unsigned char M1FB,
                                               unsigned char M2DIR, unsigned char M2PWM, unsigned char M2FB,
                                               unsigned char nD2, unsigned char nSF)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _nD2 = nD2;
  _M1DIR = M1DIR;
  _M1PWM = M1PWM;
  _M2DIR = M2DIR;
  _M2PWM = M2PWM;
  _nSF = nSF;
  _M1FB = M1FB; 
  _M2FB = M2FB;
  
}

DualMC33926MotorShield::DualMC33926MotorShield(unsigned char M1DIR, unsigned char M1PWM, unsigned char M1FB,
                                               unsigned char nD2, unsigned char nSF)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _nD2 = nD2;
  _M1DIR = M1DIR;
  _M1PWM = M1PWM;
  _nSF = nSF;
  _M1FB = M1FB; 
}

DualMC33926MotorShield::DualMC33926MotorShield(unsigned char M1DIR, unsigned char M1PWM, unsigned char nD2,unsigned char nSF)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _nSF = nSF;
  _nD2 = nD2;
  _M1DIR = M1DIR;
  _M1PWM = M1PWM;
  
}

DualMC33926MotorShield::DualMC33926MotorShield(unsigned char M2DIR, unsigned char M2PWM, 
												unsigned char M3DIR, unsigned char M3PWM,
												unsigned char nD2,unsigned char nSF)
{
  _nSF = nSF;
  _nD2 = nD2;
  _M1DIR = M2DIR;
  _M1PWM = M2PWM;
  _M2DIR = M3DIR;
  _M2PWM = M3PWM;
  

}

// Public Methods //////////////////////////////////////////////////////////////
void DualMC33926MotorShield::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M1FB,INPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);
  pinMode(_M2FB,INPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on
  pinMode(_nSF,INPUT);

}

// Public Methods //////////////////////////////////////////////////////////////
void DualMC33926MotorShield::init1M()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on
  pinMode(_nSF,INPUT);

}

void DualMC33926MotorShield::init2M()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on
  pinMode(_nSF,INPUT);
}

void DualMC33926MotorShield::setM1Dir(boolean dir){
	if(dir) digitalWrite(_M1DIR,HIGH);
	else digitalWrite(_M1DIR,LOW);
}

void DualMC33926MotorShield::setM2Dir(boolean dir){
	if(dir) digitalWrite(_M2DIR,HIGH);
	else digitalWrite(_M2DIR,LOW);
}

// Set speed for motor 1, speed is a number betwenn -255 and 255
void DualMC33926MotorShield::setM1Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
  if (reverse)
    digitalWrite(_M1DIR,LOW);
  else
    digitalWrite(_M1DIR,HIGH);
  analogWrite(_M1PWM,speed); // default to using analogWrite,
}

 



// Set speed for motor 2, speed is a number betwenn -200 and 200
void DualMC33926MotorShield::setM2Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
  if (reverse)
    digitalWrite(_M2DIR,LOW);
  else
    digitalWrite(_M2DIR,HIGH);
  analogWrite(_M2PWM,speed); // default to using analogWrite
 
}

// Set speed for motor 1 and 2
void DualMC33926MotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

// Return motor 1 current value in milliamps.
unsigned int DualMC33926MotorShield::getM1CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
  return analogRead(_M1FB) * 9;
}

// Return motor 2 current value in milliamps.
unsigned int DualMC33926MotorShield::getM2CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
  return analogRead(_M2FB) * 9;
}

// Return error status
unsigned char DualMC33926MotorShield::getFault()
{
  return !digitalRead(_nSF);
}
