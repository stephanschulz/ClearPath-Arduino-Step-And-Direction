/*
  ClearPathMotorSD.h - Library for interfacing with Clearpath-SD motors using an Arduino- Version 1
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
#include "Arduino.h"
#include "ClearPathMotorSD.h"


/*		
	This is an internal Function used by ClearPathStepGen to calculate how many pulses to send to each motor.
	It tracks the current command, as well as how many steps have been sent, and calculates how many steps
	to send in the next ISR.
*/
int ClearPathMotorSD::calcSteps()
{  
	_TX++;//increment time
    
	if(!Enabled)
		return 0;

	// Process current move state.
	switch(moveStateX){
            
            //cases 3,1,2
            //in case 1 we ramp up at increasing speed
            //no matter how long the total requested movement is accel is the same
            //after speed reaches max we continue to move at max speed if we not have reach half point
            //at half point we move in to case 2
            //either we continue to move at max speed for a while, if TX3 ramp down time has not been reached
            //or for shorter movementes we right away stat ramping down with decreasing speed
            
            //Start 0-------TX1--------TX2--------TX3-------TAUX End
            //Start |rampUp----maxVel--Mid--maxVel----rampDown|         
		case 3: //MARK: IdleState state, executed only once.

			if(CommandX == 0) //If no/finished command/, do nothing set everything to 0
			{
				MovePosnQx=0;
				VelRefQx=0;
				StepsSent=0;
				_TX=0;  // Current time
				_TX1=0; // End of ramp up time
				_TX2=0; // Beginning of phase 2 time
				_TX3=0; // Beginning of ramp down time
                //ramp up and ramp down should take same amount of time
				_BurstX=0;
			}
			else
			{
				// Compute Move parameters
				TargetPosnQx = CommandX<<fractionalBits; // multiplies CommandX by 1024
				TriangleMovePeakQx = abs(TargetPosnQx>>1); // div by 2; defines half move length
				if(TargetPosnQx > 0)
					AccelRefQxS = AccLimitQx;
				else
					AccelRefQxS = -AccLimitQx;
				AccelRefQx = AccelRefQxS; //set Current acceleration
                
				// Do immediate move if half move length <= maximum acceleration.
                // since acceleration is just a number of steps executed in quick succession
				if(TriangleMovePeakQx <= AccLimitQx) {
					AccelRefQx = 0;
					VelRefQx = 0;
					MovePosnQx = TargetPosnQx;
					moveStateX = 3;	//Set to Move Idle
					CommandX=0;		//Zero command
					break;
				}
				// Otherwise, execute move and go to Phase1
				MovePosnQx = MovePosnQx + VelRefQx;// + (AccelRefQx>>1);
				VelRefQx = VelRefQx + AccelRefQx;
				moveStateX = 1;
			}
			break;

		case 1:		//MARK:Phase 1, first half of move
			
			// Execute move
			MovePosnQx = MovePosnQx + VelRefQx + (AccelRefQx>>1); //position+speed+half accel
			VelRefQx = VelRefQx + AccelRefQx;//increase speed by accel value

			// Check position.
			if(abs(MovePosnQx) >= TriangleMovePeakQx) {
				// If half move reached, compute time parameters and go to Phase2
                // this point can be reached with and without having reached max velocity; i.e. VelLimitQx
                
				if(_flag)		//This makes sure you go one step past half in order to make sure Phase 2 goes well
				{
					if(_TX1 == 0) {
                        //in case we never hit max velocity. because _TX1 = _TX is set 20 lines down
						_TX1 = _TX; //store End-of-ramp-up-time. Time that has past up until now
					}
					if(_TX2 == 0) {
						_TX2 = _TX; // Beginning of phase 2 time
					}
                    //TODO:?why check _TX2 == 0 when this doesn't get set anywhere else ?
                    
					AccelRefQx = -AccelRefQx;	//Set deceleration
					_TX3 = (_TX2<<1) - _TX1;	//compute time params. if _TX2 == _TX1 then also == _TX3
                    //TX3 means Beginning of ramp down time and should be same duration as ramp up
                    //_TX2<<1 is the total length of the movement
                    //since ramp up and down have same duration we take (TX2<<1) - TX1 and get TX3
					_TAUX = _TX2<<1; //double half point; i.e. full duration
					moveStateX = 2;			//Start Phase 2
				}
				_flag=true;
				
			}
			else {
				// Otherwise, check velocity.
				if(labs(VelRefQx) >= VelLimitQx) {
					// If maximum velocity is reached, compute TX1 and set AX = 0, and VelRefQx=VelLimitQx.
                    // then continue to move at maximum (constant) speed, without acceleration
					if(_TX1 == 0) {
						AccelRefQx = 0;
						_TX1 = _TX;     //store End-of-ramp-up-time. Time that has past up until now
						if(VelRefQx > 0)
							VelRefQx=VelLimitQx;
						else
							VelRefQx=-VelLimitQx;
					}
				}
			}
			break;

		case 2:		//MARK:Phase 2, 2nd half of move
			// Execute move
			MovePosnQx = MovePosnQx + VelRefQx + (AccelRefQx>>1); //position+speed+half accel
			VelRefQx = VelRefQx + AccelRefQx; //in Phase 2 accel should be 0 since we pasted first half of move

			// Check time.
			if(_TX >= _TX3) {
				// If beyond TX3, wait for done condition.
                // now we need to start ramping down
				AccelRefQx = -AccelRefQxS;
                //--for positions that never go in to negative (like linear actuator)
                //we decrease speed and it eventually will be negative
                //and accel is already negative
                //so (VelRefQx*AccelRefQx > 0) means if both are negative; -1 * -1 = +1
                //--for motors that can go in neg and pos direction
                //we reach end of need for speed when vel and accel both have same sign
				if((_TX > _TAUX) || (labs(MovePosnQx) > labs(TargetPosnQx)) || (VelRefQx*AccelRefQx > 0)) {
            	// If done, enforce final position.
                    
					AccelRefQx = 0;
					VelRefQx = 0;
					MovePosnQx = TargetPosnQx;
					moveStateX = 3;
					CommandX=0;
				}
			}
			break;
            
		case 4:		//MARK: case 4, Fast move case
			// Execute move
			TargetPosnQx = CommandX<<fractionalBits;
				MovePosnQx = TargetPosnQx;
			if(TargetPosnQx-MovePosnQx>50<<fractionalBits){
				MovePosnQx=MovePosnQx+50<<fractionalBits;
			}
			else{
				MovePosnQx = TargetPosnQx;
				CommandX=0;
				moveStateX = 3;
			}

			break;
            
        case 6: //MARK: case 6, custom decelerate over set distance
            //decelerate
            
//            MovePosnQx = MovePosnQx + VelRefQx + (AccelRefQx>>1); //position+speed+half accel
//            VelRefQx = VelRefQx + AccelRefQx; //in Phase 2 accel should be 0 since we pasted first half of move

            
            if(_flag)        //wait for flag means wait for all already scheduled step PIN changes to be done
            {
                //https://www.omnicalculator.com/physics/acceleration
                //a = 2 * (Δd - v_i * Δt) / Δt²
                
//                _decelDistance = 5500000; //_VX * 2000; //5500000;
//                if(_decelDistance > (TargetPosnQx-MovePosnQx) ){ 
//                    _decelDistance = TargetPosnQx - MovePosnQx;
//                    
////                    Serial.print(" adjst dec ");
////                    Serial.print(_decelDistance);
//                }
                
                _TX = 0;
                _TAUX = 550000;
//                _TX3 = _TX; //reached ramp down time; start ramp down right away
//                _TAUX = _TX<<1; //sets total time to a long time, so it will not be the cause for ramp down to finish
                
//                AccelRefQx = -AccLimitQx; // 10 
//                if(moveStateX == 1) AccelRefQx = -15;    //Set deceleration if currently still in pahse 1
                //otherwise AccelRefQx sign is already correct
                
               // _AX = 10; //2 * (_decelDistance - (_VX * 2)) / (2*2); //_AXMX;
                
                if(_direction){
                    if(AbsPosition + decelAbsDistance > maxAbsPosition){
            //            TargetPosnQx = labs(maxAbsPosition - AbsPosition)<<fractionalBits;
                        decelDistanceQx = labs(maxAbsPosition - AbsPosition)<<fractionalBits;
            //            Serial.print(">>> maxPosnQx ");
            //            Serial.print(decelDistanceQx);
            //            Serial.println(); 
                    } else {
            //            TargetPosnQx = MovePosnQx + decelDistanceQx;
                        decelDistanceQx = decelAbsDistance<<fractionalBits;
                    }
                }
                else{
                    if(AbsPosition - decelAbsDistance < minAbsPosition){
            //            TargetPosnQx = labs(AbsPosition - minAbsPosition)<<fractionalBits;
                        decelDistanceQx = labs(AbsPosition - minAbsPosition)<<fractionalBits;
            //            Serial.print("<<< minPosnQ");
            //            Serial.print(decelDistanceQx);
            //            Serial.println(); 
                    } else {
            //            TargetPosnQx = MovePosnQx + decelDistanceQx;
                        decelDistanceQx = decelAbsDistance<<fractionalBits;
                    }
                }
                
                Serial.print("decelDistanceQx ");
                Serial.print(decelDistanceQx);
                Serial.println();  
               
//                Serial.print(" TargetPosnQx ");
//                Serial.print(TargetPosnQx);
//                Serial.print(" decelDistanceQx ");
//                Serial.print(decelDistanceQx);
////                Serial.print(" minPosnQx ");
////                Serial.print(minPosnQx);
//                Serial.print(" maxPosnQx ");
//                Serial.print(maxPosnQx);
//                Serial.println();
                
//                TargetPosnQx = MovePosnQx + decelDistanceQx;
                
//                if(_direction){
//                    if(AbsPosition + (decelDistanceQx>>fractionalBits) > maxAbsPosition){
//                        TargetPosnQx = labs(maxAbsPosition - AbsPosition)<<fractionalBits;
//                        decelDistanceQx = labs(TargetPosnQx - MovePosnQx);
//                        Serial.print(">>> maxPosnQx ");
//                        Serial.print(decelDistanceQx);
//                        Serial.println(); 
//                    } else {
//                        TargetPosnQx = MovePosnQx + decelDistanceQx;
//                    }
//                }
//                else{
//                    if(AbsPosition - (decelDistanceQx>>fractionalBits) < minAbsPosition){
//                        TargetPosnQx = labs(AbsPosition - minAbsPosition)<<fractionalBits;
//                        decelDistanceQx = labs(MovePosnQx - TargetPosnQx);
//                        Serial.print("<<< minPosnQ");
//                        Serial.print(decelDistanceQx);
//                        Serial.println(); 
//                    } else {
//                        TargetPosnQx = MovePosnQx + decelDistanceQx;
//                    }
//                }
                
//              if(labs(TargetPosnQx) > maxPosnQx){
//                    TargetPosnQx = maxPosnQx;
//                    decelDistanceQx = labs(TargetPosnQx - MovePosnQx);
//                    Serial.print(">>> maxPosnQx ");
//                    Serial.print(decelDistanceQx);
//                    Serial.println(); 
//                } else if(labs(TargetPosnQx) < minPosnQx){
//                    TargetPosnQx = minPosnQx;
//                    decelDistanceQx = labs(MovePosnQx - TargetPosnQx);
//                    Serial.print("<<< minPosnQ");
//                    Serial.print(decelDistanceQx);
//                    Serial.println(); 
//                }
                
                //                distanceToEnd = TargetPosnQx - MovePosnQx;
                AccelRefQx =  -(VelRefQx*VelRefQx) / (decelDistanceQx<<1) ;
                AccelRefQx = min(-1,AccelRefQx);
//                Serial.print("TargetPosnQx ");
//                Serial.print(TargetPosnQx);
//                Serial.print(" MovePosnQx ");
//                Serial.print(MovePosnQx);
//                Serial.print(" decelDistanceQx ");
//                Serial.print(decelDistanceQx);
//                Serial.println(); 
                
//                TriangleMovePeakQx = MovePosnQx + (_decelDistance>>1); // don't really need to set this, since it's not used for ramp down
                
                moveStateX = 7;            //Start Phase 2; decelerate 
            }
            break;
        case 7:        //MARK:Phase 7, custom decel 2nd half of move
            // Execute move
            MovePosnQx = MovePosnQx + VelRefQx + (AccelRefQx>>1); //position+speed+half accel
            VelRefQx = VelRefQx + AccelRefQx; //in Phase 2 accel should be 0 since we pasted first half of move

            // Check time.
//            if(_TX >= _TX3) {
                // If beyond TX3, wait for done condition.
                // now we need to start ramping down
//                AccelRefQx = -15;
                //--for positions that never go in to negative (like linear actuator)
                //we decrease speed and it eventually will be negative
                //and accel is already negative
                //so (VelRefQx*AccelRefQx > 0) means if both are negative; -1 * -1 = +1
                //--for motors that can go in neg and pos direction
                //we reach end of need for speed when vel and accel both have same sign
            
            //constantly re-calculate distance to target and get updated AccelRefQx
            decelDistanceQx = TargetPosnQx - MovePosnQx;
            
            if((_TX > _TAUX) || (labs(MovePosnQx) > labs(TargetPosnQx)) || labs(decelDistanceQx) <= 0 || (VelRefQx*AccelRefQx > 0)) {
                // If done, enforce final position.
                    
//                    Serial.print("_TX ");
//                    Serial.print(TargetPosnQx);
//                    Serial.print(" _TAUX ");
//                    Serial.print(_TAUX);
//                    Serial.print(" ? ");
//                    Serial.print( (_TX > _TAUX) );
//                    Serial.println(); 
//                    
//                    Serial.print("labs(MovePosnQx) ");
//                    Serial.print(labs(MovePosnQx));
//                    Serial.print(" labs(TargetPosnQx) ");
//                    Serial.print(labs(TargetPosnQx));
//                    Serial.print(" ? ");
//                    Serial.print( (labs(MovePosnQx) > labs(TargetPosnQx)) );
//                    Serial.println(); 
//                Serial.print(" updated decelDistanceQx == ");
//                Serial.print(decelDistanceQx);
//                Serial.println();
//                
//                    Serial.print("VelRefQx ");
//                    Serial.print(VelRefQx);
//                    Serial.print(" AccelRefQx ");
//                    Serial.print(AccelRefQx);
//                    Serial.print(" ? ");
//                    Serial.print( (VelRefQx*AccelRefQx > 0) );
//                    Serial.println(); 
//                    
                    AccelRefQx = 0;
                    VelRefQx = 0;
//                    MovePosnQx = TargetPosnQx;
                TargetPosnQx = MovePosnQx;
                    moveStateX = 3;
                    CommandX=0;
            }else{
                AccelRefQx =  -(VelRefQx*VelRefQx) / (decelDistanceQx<<1) ;
                AccelRefQx = min(-1,AccelRefQx);
            }
//            }
            break;
	}
	// Compute burst value
	_BurstX = (MovePosnQx - StepsSent)>>fractionalBits;
	// Update accumulated integer position
	StepsSent += (long)(_BurstX)<<fractionalBits;

	//check which direction, and incement absPosition
	if(_direction)
		AbsPosition+=_BurstX;
	else
		AbsPosition-=_BurstX;
	return _BurstX;

}

void ClearPathMotorSD::decelerateStopWithAccel(long _accel)
{
    
}
void ClearPathMotorSD::decelerateStopOverDistance(long _stopDist)
{
    //TODO:make sure stopDist does not go beyond limits
    //TODO: set maybe a deceleration AccLimitQx, but need to set back for normal move
//    _decelDistance = stopDist;
//    Serial.print("TargetPosnQx ");
//    Serial.print(TargetPosnQx);
//    Serial.print(", MovePosnQx ");
//    Serial.print(MovePosnQx);
//    Serial.print(", AccelRefQx ");
//    Serial.print(AccelRefQx);
//    Serial.println();  
//    
//    Serial.print("moveStateX ");
//    Serial.print(moveStateX);
//    Serial.println();  
    
    Serial.print("AbsPosition ");
    Serial.print(AbsPosition);
    Serial.println();  
    
//    if((MovePosnQx + _decelDistance) > (TargetPosnQx-MovePosnQx) ){ 
//        _decelDistance = TargetPosnQx - MovePosnQx;
//    }
    
//    TargetPosnQx
    //make move dist 800L * 68L
//    stopDist = 800L*15;
//    decelDistanceQx = _stopDist<<fractionalBits;
    
    decelAbsDistance = _stopDist;
    
   
//    _decelDistance = 5500000;
//    AccLimitQx=(accelMax*(1<<fractionalBits))/4000000;
    moveStateX = 6;
}
/*		
	This is the default constructor.  This intializes the variables.
*/
ClearPathMotorSD::ClearPathMotorSD()
{
	moveStateX=3;
	PinA=0;
	PinB=0;
	PinE=0;
	PinH=0;
	Enabled=false;
	VelLimitQx=0;					
	AccLimitQx=0;
	MovePosnQx=0;				
	StepsSent=0;				
	VelRefQx=0;				
	AccelRefQx=0;					
	_TX=0;					
	_TX1=0;				
	_TX2=0;				
	_TX3=0;			
	_TAUX=0;					
	_flag=0;
	AccelRefQxS=0;					
	TargetPosnQx=0;				
	TriangleMovePeakQx=0;					
	CommandX=0;
	fractionalBits=10;
	_BurstX=0;
	AbsPosition=0;
    
//    minPosnQx = 0;
//    maxPosnQx = 55705600; //(800L * 68) << 10
    
    minAbsPosition = 0;
    maxAbsPosition = 54400; //800L*68
}

/*		
	This is the one pin attach function.  It asociates the passed number, as this motors Step Pin
*/
void ClearPathMotorSD::attach(int BPin)
{
  PinA=0;
  PinB=BPin;
  PinE=0;
  PinH=0;
  pinMode(PinB,OUTPUT);
}

/*		
	This is the two pin attach function.  
	It asociates the 1st number, as this motors Direction Pin
	and the 2nd number with the Step Pin
*/
void ClearPathMotorSD::attach(int APin, int BPin)
{
  PinA=APin;
  PinB=BPin;
  PinE=0;
  PinH=0;
  pinMode(PinA,OUTPUT);
  pinMode(PinB,OUTPUT);
}

/*		
	This is the three pin attach function.  
	It asociates the 1st number, as this motors Direction Pin,
	the 2nd number with the Step Pin,
	and the 3rd number with the Enable Pin
*/
void ClearPathMotorSD::attach(int APin, int BPin, int EPin)
{
  PinA=APin;
  PinB=BPin;
  PinE=EPin;
  PinH=0;
  pinMode(PinA,OUTPUT);
  pinMode(PinB,OUTPUT);
  pinMode(PinE,OUTPUT);
}

/*		
	This is the four pin attach function.  
	It asociates the 1st number, as this motors Direction Pin,
	the 2nd number with the Step Pin,
	the 3rd number with the Enable Pin,
	and the 4th number as the HLFB Pin
*/
void ClearPathMotorSD::attach(int APin, int BPin, int EPin, int HPin)
{
  PinA=APin;
  PinB=BPin;
  PinE=EPin;
  PinH=HPin;
  pinMode(PinA,OUTPUT);
  pinMode(PinB,OUTPUT);
  pinMode(PinE,OUTPUT);
  pinMode(PinH,INPUT_PULLUP);
}

/*		
	This function clears the current move, and puts the motor in a
	move idle state, without disabling it, or clearing the position.

	This may cause an abrupt stop.
*/
void ClearPathMotorSD::stopMove()
{
	cli();
	MovePosnQx=0;
	VelRefQx=0;
	StepsSent=0;
	_TX=0;
	_TX1=0;
	_TX2=0;
	_TX3=0;
	_BurstX=0;
	moveStateX = 3;
	CommandX=0;
	sei();
}

/*		
	This function commands a directional move
	The move cannot be longer than 2,000,000 counts
	If there is a current move, it will NOT be overwritten

	The function will return true if the move was accepted
*/
boolean ClearPathMotorSD::move(long dist)
{
  if(CommandX==0)
  {
	  if(dist<0)
	  {
		  if(PinA!=0)
		  {
			  digitalWrite(PinA,HIGH);
			  delay(1);
			  _direction=true;
		  }
		  CommandX=-dist;
	  }
	  else
	  {
		  if(PinA!=0)
		  {
			  digitalWrite(PinA,LOW);
			  delay(1);
			  _direction=false;
		  }
			CommandX=dist;
	  }
      
//      Serial.print(", time: ");
//      Serial.print(F(__TIME__));
//      Serial.println();
      
      return true;
  }
  else
	  return false;

}

/*		
	This function commands a directional move which will burst out steps as fast as possible with no acceleration or velocity limits
*/
boolean ClearPathMotorSD::moveFast(long dist)
{
  if(CommandX==0)
  {
	  if(dist<0)
	  {
		  if(PinA!=0)
		  {
			  digitalWrite(PinA,HIGH);
			  _direction=true;
		  }
		  cli();
		  moveStateX = 4;
		  CommandX=-dist;
		  sei();
		  
	  }
	  else
	  {
		  if(PinA!=0)
		  {
			  digitalWrite(PinA,LOW);
			  _direction=false;
		  }
			cli();
			moveStateX = 4;
			CommandX=dist;
			sei();
	  }
	  return true;
  }
  else
	  return false;

}
/*		
	This function sets the velocity in Counts/sec assuming the ISR frequency is 2kHz.
	The maximum value for velMax is 100,000, the minimum is 2
*/
void ClearPathMotorSD::setMaxVel(long velMax)
{
	int n = velMax/2000;
	if(n<51)
		VelLimitQx=(velMax*(1<<fractionalBits))/2000;
	else
		VelLimitQx=50*(1<<fractionalBits);

}
/*		
	This function sets the acceleration in Counts/sec/sec assuming the ISR frequency is 2kHz.
	The maximum value for accelMax is 2,000,000, the minimum is 4,000
*/
void ClearPathMotorSD::setMaxAccel(long accelMax)
{
  AccLimitQx=(accelMax*(1<<fractionalBits))/4000000;
    //for example (20000 × (1<<10))/4000000 = 5.12
    Serial.print("AccLimitQx ");
    Serial.println(AccLimitQx);
}

void ClearPathMotorSD::setPositionLimits(long _min, long _max){
    
//    minPosnQx = _min<<fractionalBits;
//    maxPosnQx = _max<<fractionalBits;
    minAbsPosition = _min;
    maxAbsPosition = _max;
}

/*		
	This function returns the absolute commanded position
*/
long ClearPathMotorSD::getCommandedPosition()
{
	return AbsPosition;
}

/*        
    This function returns the current direction
*/
boolean ClearPathMotorSD::getDirection()
{
    return _direction;
}

/*		
	This function returns true if there is no current command
	It returns false if there is a current command
*/
boolean ClearPathMotorSD::commandDone()
{
	if(CommandX==0)
		return true;
	else
		return false;
}


/*		
	This function returns the value of the HLFB Pin
*/
boolean ClearPathMotorSD::readHLFB()
{
	if(PinH!=0)
		return !digitalRead(PinH);
	else
		return false;
}

/*		
	This function enables the motor
*/
void ClearPathMotorSD::enable()
{

	if(PinE!=0)
		digitalWrite(PinE,HIGH);
	AbsPosition=0;
	Enabled=true;
}

/*		
	This function returns zeros out the current command, and digitally writes the enable pin LOW
	If the motor was not attached with an enable pin, then it just zeros the command
*/
void ClearPathMotorSD::disable()
{
	stopMove();
	if(PinE!=0)
		digitalWrite(PinE,LOW);
	Enabled=false;
	
}

void ClearPathMotorSD::printInfo(){
    
//    Serial.print("moveStateX ");
//    Serial.println(moveStateX); 
//    
//    Serial.print("CommandX ");
//    Serial.println(CommandX);
//    Serial.print("_direction ");
//    Serial.println(_direction);   
//    
//    Serial.print("TargetPosnQx  ");
//    Serial.println(TargetPosnQx);
//    Serial.print("TriangleMovePeakQx ");
//    Serial.println(TriangleMovePeakQx);
//    Serial.print("MovePosnQx  ");
//    Serial.println(MovePosnQx);
//    
//    Serial.print("StepsSent ");
//    Serial.println(StepsSent);
//    Serial.print("absP ");
//    Serial.println(AbsPosition);
//    
//    Serial.print("_TX cur time ");
//    Serial.println(_TX);
//    Serial.print("_TX1 ramp up end time ");
//    Serial.println(_TX1);
//    Serial.print("_TX2 peak time ");
//    Serial.println(_TX2);
//    Serial.print("_TX3 ramp down start time ");
//    Serial.println(_TX3);
//    Serial.print("total time ");
//    Serial.println(_TAUX);
//    
//    Serial.print("VelLimitQx ");
//    Serial.println(VelLimitQx);
//    Serial.print("VelRefQx  ");
//    Serial.println(VelRefQx);
//    
//    Serial.print("AccLimitQx ");
//    Serial.println(AccLimitQx);
//    Serial.print("AccelRefQx   ");
//    Serial.println(AccelRefQx);
//    Serial.print("_AXS  ");
//    Serial.println(_AXS);
    

    
    
    Serial.println();
    
}
