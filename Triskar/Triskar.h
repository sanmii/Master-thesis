/*
  Triskar.h - Library for controlling the Triskar robot.
  Created by Martino Migliavacca, August 2, 2013.
  Released into the public domain.
  Modification made by Matteo Lucchelli including
    - encoders reader
    - PID speed control
	- Odometry
  In order to use the library, put triskar.pidLoop() function in the loop of Arduino sketch, use run(forward,angular) 
  or run2(strafe,forward,angular) to move the robot using inverse kinematic. use runM(motor1, motor2, motor3) if you 
  want to define each motor's speed. stop() function will stop the robot resetting all speed values. stop2() also stops 
  the robot, but it doesn't reset Iterm PID value.
  Have a look to the example .ino file for further clarification.
*/

#ifndef Triskar_h
#define Triskar_h

#include "Arduino.h"
#include "DualMC33926MotorShield.h" // library of the old driver
#include "CytronMotorDriver.h"
#include "Encoder.h"
#define NMOTOR 3
#define wheel_radius  3.5f //cm
#define robot_radius  16.0f  //cm

class Triskar
{
private:

	#define KP  0.35f; //old was 0.5
	#define KI  0.8f;  //old was 0.9
	#define m1_R     (-1.0f / wheel_radius)
	#define mL_R     (-robot_radius / wheel_radius)
	#define C60_R    (0.500000000f / wheel_radius)   // cos(60°) / R
	#define C30_R    (0.866025404f / wheel_radius)   // cos(30°) / R
	#define LOOPTIME        25                     // PID loop time
	double speed_req[3];         //SETPOINT
	double speed_act[3];                              // speed (actual value)
	int PWM_val[3];                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255) this one  is the same for the new library
	double last_error[3];
	double Iterm[3];
	float Kp;
	float Ki;
	float Kd;
	double posX;
	double posY;
	double posTh;
	double speedX;
	double speedY;
	double speedTh;
	int direction;

	unsigned long lastMilliLoop;
	unsigned long lastMillis[3];
	unsigned long deltaPid;

	//DualMC33926MotorShield & _m1;
	//DualMC33926MotorShield & _m2m3;
    
    CytronMD & _m1;
    CytronMD & _m2;
    CytronMD & _m3;
    
	Encoder & _e1;
	Encoder & _e2;
	Encoder & _e3;
	
	void getMotorRPM(long deltaT,int pos,int i);
	void getMotorRadS(long deltaT,int pos,int i);
	void getMotorCmS(long deltaT,int pos,int i);
	
	int updatePid(double targetValue, double currentValue, int i);
	void direct_kinematics(void);
	void makeOdometry(unsigned long int deltaT);

public:
	Triskar(CytronMD & _m1, CytronMD & _m2, CytronMD & _m3,
			Encoder & e1, Encoder & e2, Encoder & e3);
	
	void PIDLoop(); //put in loop
	
	void run(float forward, float angular);
	void run2(float strafe, float forward, float angular);
	void runM(float m1, float m2, float m3);
	 
	void setM1Speed(float m1);
	void setM2Speed(float m2);
	void setM3Speed(float m3);

	void stop(void);  //stop the robot, reset odometry and Iterm
	void stop2(void); //stop the robot, reset odometry(not Iterm)
	void stop3(void); //stop the robot, reset Iterm(not odometry)
	
	//void PIDLoop2();

	double getPosX();
	double getPosY();
	double getPosTh();
	double getSpeedX();
	double getSpeedY();
	double getSpeedTh();
	int getPosWheel(int i);

	void setPosX(double _posX);
	void setPosY(double _posY);
	void setPosTh(double _posTh);

	void setIterm(int i, double val);
	void resetIterm();
	
	float getKi();
	void setKp(float val);
	void resetKp();
	
	float getKp();
	void setKi(float val);
	void resetKi();

	boolean isStopped();
	boolean isRotating();
	boolean isTraslating();
	int getDirection(); //0 - stopped, 1-forward, 2-backward


};

#endif /* Triskar_h */
