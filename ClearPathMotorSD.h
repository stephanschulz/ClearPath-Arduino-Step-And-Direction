/*
 ClearPathMotorSD.h - Library for interfacing with Clearpath motors using an Arduino- Version 1
 Teknic 2017 Brendan Flosenzier
 
 Copyright (c) 2017 Teknic Inc. This work is free to use, copy and distribute under the terms of the standard
 MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

/* 
 A ClearPathMotorSD is activated by creating an instance of the ClearPathMotorSD class.
 
 There can several instances of ClearPathMotorSD however each must be attached to different pins.
 
 This class is used in conjuntion with the ClearPathStepGen class which manages and sends step pulses to each motor.
 
 Note: Each attached motor must have its direction/B pin connected to one of pins 8-13 on an arduino UNO (PORTB) 
 in order to work with the ClearPathStepGen object.  Other devices can be connected to pins 8-13 as well.
 If you are using another Arduino besides the UNO, the ClearPathStepGen must be modifyed to use a different port.
 
 The functions for a ClearPathMotorSD are:
 
 ClearPathMotorSD - default constructor for initializing the motor
 
 attach() - Attachs pins to this motor, and declares them as input/outputs
 
 stopMove()  - Interupts the current move, the motor may abruptly stop
 
 move() - sets the maximum veloctiy
 
 disable() - disables the motor
 
 enable() - enables the motor
 
 getCommandedPosition() - Returns the absolute cmomanded position where the position on enable=0
 
 readHLFB() - Returns the value of the motor's HLFB Pin
 
 setMaxVel() - sets the maximum veloctiy
 
 setMaxAccel() - sets the acceleration
 
 commandDone() - returns wheter or not there is a valid current command
 
 */
#ifndef ClearPathMotorSD_h
#define ClearPathMotorSD_h
#include "Arduino.h"
class ClearPathMotorSD
{
public:
    ClearPathMotorSD();
    void attach(int);
    void attach(int, int);
    void attach(int, int, int);
    void attach(int, int, int, int);
    boolean move(long);
    boolean moveFast(long);
    void enable();
    long getCommandedPosition();
    boolean readHLFB();
    void stopMove();
    int calcSteps();
    void setMaxVel(long); 
    void setMaxAccel(long);
    boolean commandDone();
    void disable();
    
    
    uint8_t PinA;
    uint8_t PinB;
    uint8_t PinE;
    uint8_t PinH;
    boolean Enabled; 
    int moveStateX;
    volatile long AbsPosition;
    
    void setPositionLimits(long _min, long _max);
    void decelerateStopOverDistance(long _stopDist);
    void decelerateStopWithAccel(long _accel);
    void printInfo();
    boolean getDirection();
    
private:
    volatile long CommandX;
    boolean _direction;
    uint8_t _BurstX;
    
    // All of the position, velocity and acceleration parameters are signed and in Q24.8,
    // with all arithmetic performed in fixed point.
    
    int32_t VelLimitQx;					// Velocity limit
    int32_t AccLimitQx;					// Acceleration limit
   
    uint32_t StepsSent;				// Accumulated integer position
    int32_t VelRefQx;					// Current velocity
    int32_t AccelRefQx;					// Current acceleration
    
    long _TX;				// Current time
    long _TX1;				// End of ramp up time
    long _TX2;				// Beginning of phase 2 time
    long _TX3;				// Beginning of ramp down time
    long _TAUX;				// Integer burst value. double of _TX2, of half time; i.e. full moevment duration
    
    boolean _flag;          // only when all scheduled steps are done can new movement data be processsed
    int32_t AccelRefQxS;
  
    uint32_t MovePosnQx;        // Current position
    long TargetPosnQx;			// total move length in Q24.8
    long TriangleMovePeakQx;	//half move length
//    long distanceToEnd;
    
//    long minPosnQx; //lower position limit
//    long maxPosnQx; //upper position limit
    long minAbsPosition; //lower position limit
    long maxAbsPosition; //upper position limit
    uint8_t fractionalBits; //position amount removed every cycle
    
    long decelDistanceQx;
    long decelAbsDistance;
};
#endif
