/* LineFollower Class
 * 
 * @author: Navid Fattahi 	<navid.fattahi (at) alumni.ubc.ca>
 *			Alireza Afshar
 * 
 * August 2013
 * this code is public domain, enjoy!
 *
 */

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #if defined(__AVR__)
    #include <avr/io.h>
  #endif
  #include "WProgram.h"
#endif

#include <LineFollower.h>

#define sensorFR  53
#define sensorFL  51
#define sensorBR  49
#define sensorBL  47

LineFollower::LineFollower(int pin_senFR, int pin_senFL, int pin_senBR, int pin_senBL) : _motor1(1, MOTOR12_64KHZ), _motor2(2, MOTOR12_64KHZ)
{
	_pinFR = pin_senFR;
	_pinFL = pin_senFL;
	_pinBR = pin_senBR;
	_pinBL = pin_senBL;
	gridCounter = 0;
	
    //_motor1 = new AF_DCMotor(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
    //_motor2 = new AF_DCMotor(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
}

void LineFollower::setMotorSpeed(int m1speed, int m2speed)
{
	_motor1.setSpeed(m1speed);
	_motor2.setSpeed(m2speed);
}

void LineFollower::moveForward()
{
  
  _motor2.setSpeed(180);
  _motor1.setSpeed(180);
  int i=1;
  int MIDflag=0;
  int MIDcounter=0;
  
  while(i==1)
  {
    
  //Rover Reacts Based on the Reading of Each Sensor
  //These Conditions Were Placed Like Below Based On Their Priority
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH && digitalRead(sensorFR)==HIGH && digitalRead(sensorFL)==HIGH)
    forward();
  else if(digitalRead(sensorBL)==LOW && digitalRead(sensorBR)==LOW)  
    forward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW && MIDflag==1)
  {
    backward();
    pause();
    //Count The Number of Grids & Print on Serial Monitor
    gridCounter++; 
    Serial.print("GRID#: ");
    Serial.println(gridCounter);
    break;
  }
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW && MIDflag==0)
    forward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorBR)==LOW)
  {
   _motor1.run(RELEASE);      // turn it on go right
   _motor2.run(FORWARD);
  }
  else if(digitalRead(sensorFL)==LOW && digitalRead(sensorBL)==LOW)
  {
   _motor1.run(FORWARD);      // turn it on go left
   _motor2.run(RELEASE);
  }
  else if(digitalRead(sensorBL)==LOW)
    right();
  else if(digitalRead(sensorBR)==LOW)
    left();
  else if(digitalRead(sensorFR)==LOW)
    right();
  else if(digitalRead(sensorFL)==LOW)
    left();
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH)
    forward();
  if(digitalRead(sensorFL)==HIGH && digitalRead(sensorFR)==HIGH)
    forward();
  if(digitalRead(sensorBR)==LOW && digitalRead(sensorBL)==LOW && MIDflag==0)
    MIDflag=1;
  }
  delay(500);
}

void LineFollower::moveBackward()
{
  _motor2.setSpeed(200);
  _motor1.setSpeed(200);
  int i=1;
  int MIDflag=0;
  int MIDcounter=0;
  
  while(i==1)
  {
    
  //Rover Reacts Based on the Reading of Each Sensor
  //These Conditions Were Placed Like Below Based On Their Priority
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH && digitalRead(sensorFR)==HIGH && digitalRead(sensorFL)==HIGH)
    backward();
  else if(digitalRead(sensorBL)==LOW && digitalRead(sensorBR)==LOW)  
    backward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW && MIDflag==0)
    backward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorBR)==LOW)
  {
   _motor1.run(RELEASE);      // turn it on go right
   _motor2.run(BACKWARD);
  }
  else if(digitalRead(sensorFL)==LOW && digitalRead(sensorBL)==LOW)
  {
   _motor1.run(BACKWARD);      // turn it on go left
   _motor2.run(RELEASE);
  }
  else if(digitalRead(sensorBL)==LOW)
    right();
  else if(digitalRead(sensorBR)==LOW)
    left();
  else if(digitalRead(sensorFR)==LOW)
    right();
  else if(digitalRead(sensorFL)==LOW)
    left();
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH)
    backward();
  if(digitalRead(sensorFL)==HIGH && digitalRead(sensorFR)==HIGH)
     backward();
  if(digitalRead(sensorBR)==LOW && digitalRead(sensorBL)==LOW && MIDflag==0)
  {
    backward();
    MIDflag=1;
  }
  if((digitalRead(sensorFR)==HIGH || digitalRead(sensorFL)==HIGH) && MIDflag==2)
  {
    MIDcounter++;
  }  
  
  if(MIDcounter==150 && MIDflag==2)
  {
    forward();
    pause();
    //Count The Number of Grids & Print on Serial Monitor
    MIDflag=3;
    MIDcounter=0;
    break;
  }
  
  
  if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW && MIDflag==1)
  {
    backward();
    MIDcounter++;
  }
  if(MIDcounter==20 && MIDflag==1)
  {
    MIDflag=2;
    MIDcounter=0;
  }
  
  }
  
  while(MIDflag==3)
  {
   if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH && digitalRead(sensorFR)==HIGH && digitalRead(sensorFL)==HIGH)
    forward();
  else if(digitalRead(sensorBL)==LOW && digitalRead(sensorBR)==LOW)  
    forward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW)
    MIDcounter++;
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorBR)==LOW)
  {
   _motor1.run(RELEASE);      // turn it on go right
   _motor2.run(FORWARD);
  }
  else if(digitalRead(sensorFL)==LOW && digitalRead(sensorBL)==LOW)
  {
   _motor1.run(FORWARD);      // turn it on go left
   _motor2.run(RELEASE);
  }
  else if(digitalRead(sensorBL)==LOW)
    right();
  else if(digitalRead(sensorBR)==LOW)
    left();
  else if(digitalRead(sensorFR)==LOW)
    right();
  else if(digitalRead(sensorFL)==LOW)
    left();
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH)
    forward();
  //if(digitalRead(sensorFL)==HIGH && digitalRead(sensorFR)==HIGH)
  //  forward();
  if(MIDcounter==10)
  {
    backward();
    pause();
    //Count The Number of Grids & Print on Serial Monitor
    gridCounter++; 
    Serial.print("GRID#: ");
    Serial.println(gridCounter);
    break;
  }
  } 
  delay(1000);
}

void LineFollower::turnRight()
{
  int turnCounter=0;
  _motor2.setSpeed(200);
  _motor1.setSpeed(200);
  int i=1;
  int flag=0;
  
  while(digitalRead(sensorFR)==LOW)
  {
    _motor1.run(BACKWARD);
  }
  
  pause();
  _motor2.run(FORWARD);
  
  while(i==1)
  {
  if(digitalRead(sensorFL)==LOW && flag==0)
    flag=1;
  if(digitalRead(sensorFL)==HIGH && flag==1)
  {
    turnCounter++;
    Serial.print(turnCounter);
  }
  if(digitalRead(sensorFL)==LOW && flag==2)
  {
    pause();
    break;
  }
  
  if(turnCounter==50)
    flag=2;
    
  }
  
  i=1;
  int MIDflag=0;
  int MIDcounter=0;
  
  while(i==1)
  {
    
  //Rover Reacts Based on the Reading of Each Sensor
  //These Conditions Were Placed Like Below Based On Their Priority
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH && digitalRead(sensorFR)==HIGH && digitalRead(sensorFL)==HIGH)
    forward();
  else if(digitalRead(sensorBL)==LOW && digitalRead(sensorBR)==LOW)  
    forward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW)
  {
    backward();
    pause();
    //Count The Number of Grids & Print on Serial Monitor
    gridCounter++; 
    Serial.print("GRID#: ");
    Serial.println(gridCounter);
    break;
  }
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorBR)==LOW)
  {
   _motor1.run(RELEASE);      // turn it on go right
   _motor2.run(FORWARD);
  }
  else if(digitalRead(sensorFL)==LOW && digitalRead(sensorBL)==LOW)
  {
   _motor1.run(FORWARD);      // turn it on go left
   _motor2.run(RELEASE);
  }
  else if(digitalRead(sensorBL)==LOW)
    right();
  else if(digitalRead(sensorBR)==LOW)
    left();
  else if(digitalRead(sensorFR)==LOW)
    right();
  else if(digitalRead(sensorFL)==LOW)
    left();
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH)
    forward();
  if(digitalRead(sensorFL)==HIGH && digitalRead(sensorFR)==HIGH)
    forward();
  }
  delay(500);
}

void LineFollower::turnLeft()
{
  int turnCounter=0;
  _motor2.setSpeed(200);
  _motor1.setSpeed(200);
  int i=1;
  int flag=0;
  
  while(digitalRead(sensorFL)==LOW)
    {
      _motor2.run(BACKWARD);
    }
  pause();
  _motor1.run(FORWARD);
  while(i==1)
  {
  if(digitalRead(sensorFR)==LOW && flag==0)
    flag=1;
  if(digitalRead(sensorFR)==HIGH && flag==1)
  {
    turnCounter++;
    Serial.println(turnCounter);
  }
  if(digitalRead(sensorFR)==LOW && flag==2)
  {
    pause();
    break;
  }
    
  if(turnCounter==50)
    flag=2;  
  }
  
  i=1;
  int MIDflag=0;
  int MIDcounter=0;
  
  while(i==1)
  {
    
  //Rover Reacts Based on the Reading of Each Sensor
  //These Conditions Were Placed Like Below Based On Their Priority
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH && digitalRead(sensorFR)==HIGH && digitalRead(sensorFL)==HIGH)
    forward();
  else if(digitalRead(sensorBL)==LOW && digitalRead(sensorBR)==LOW)  
    forward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW)
  {
    backward();
    pause();
    //Count The Number of Grids & Print on Serial Monitor
    gridCounter++; 
    Serial.print("GRID#: ");
    Serial.println(gridCounter);
    break;
  }
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorBR)==LOW)
  {
   _motor1.run(RELEASE);      // turn it on go right
   _motor2.run(FORWARD);
  }
  else if(digitalRead(sensorFL)==LOW && digitalRead(sensorBL)==LOW)
  {
   _motor1.run(FORWARD);      // turn it on go left
   _motor2.run(RELEASE);
  }
  else if(digitalRead(sensorBL)==LOW)
    right();
  else if(digitalRead(sensorBR)==LOW)
    left();
  else if(digitalRead(sensorFR)==LOW)
    right();
  else if(digitalRead(sensorFL)==LOW)
    left();
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH)
    forward();
  if(digitalRead(sensorFL)==HIGH && digitalRead(sensorFR)==HIGH)
    forward();
  }
  delay(500);
}

void LineFollower::turnAround()
{
  int turning=1;
  _motor1.setSpeed(200);
  _motor2.setSpeed(200);
  int i=1;
  int flag=0;
  while(i==1)
  {
    left();
    if(digitalRead(sensorBR)==HIGH && flag==0)
      flag=1;
    if(digitalRead(sensorBR)==LOW && flag==1)
      flag=2;
    if(digitalRead(sensorBL)==LOW && flag==2)
      flag=3;
    if(((digitalRead(sensorBR)==HIGH && digitalRead(sensorBL)==HIGH) || (digitalRead(sensorFR)==HIGH && digitalRead(sensorFL)==HIGH))  && flag==3)
    {
      pause();
      turning=0;
      break;
     }
  }
  
  delay(300);
  int MIDflag=0;
  int MIDcounter=0;
  
  
  while(turning==0)
  {  
  //Rover Reacts Based on the Reading of Each Sensor
  //These Conditions Were Placed Like Below Based On Their Priority
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH && digitalRead(sensorFR)==HIGH && digitalRead(sensorFL)==HIGH)
    backward();
  else if(digitalRead(sensorBL)==LOW && digitalRead(sensorBR)==LOW)  
    backward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW && MIDflag==0)
    backward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorBR)==LOW)
  {
   _motor1.run(RELEASE);      // turn it on go right
   _motor2.run(BACKWARD);
  }
  else if(digitalRead(sensorFL)==LOW && digitalRead(sensorBL)==LOW)
  {
   _motor1.run(BACKWARD);      // turn it on go left
   _motor2.run(RELEASE);
  }
  else if(digitalRead(sensorBL)==LOW)
    right();
  else if(digitalRead(sensorBR)==LOW)
    left();
  else if(digitalRead(sensorFR)==LOW)
    right();
  else if(digitalRead(sensorFL)==LOW)
    left();
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH)
    backward();
  if(digitalRead(sensorFL)==HIGH && digitalRead(sensorFR)==HIGH)
     backward();
  if(digitalRead(sensorBR)==LOW && digitalRead(sensorBL)==LOW && MIDflag==0)
  {
    backward();
    MIDflag=1;
  }
  if((digitalRead(sensorFR)==HIGH || digitalRead(sensorFL)==HIGH) && MIDflag==2)
    MIDcounter++;
  
  if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW && MIDflag==1)
  {
    backward();
    MIDcounter++;
  }
  if(MIDcounter==20 && MIDflag==1)
  {
    MIDflag=2;
    MIDcounter=0;
  }
  
  if(MIDcounter==150 && MIDflag==2)
  {
   forward();
   pause();
   MIDflag=3;
   MIDcounter=0;
   break;
  }
  
  }
  
  while(MIDflag==3)
  {
   if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH && digitalRead(sensorFR)==HIGH && digitalRead(sensorFL)==HIGH)
    forward();
  else if(digitalRead(sensorBL)==LOW && digitalRead(sensorBR)==LOW)  
    forward();
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorFL)==LOW)
    MIDcounter++;
  else if(digitalRead(sensorFR)==LOW && digitalRead(sensorBR)==LOW)
  {
   _motor1.run(RELEASE);      // turn it on go right
   _motor2.run(FORWARD);
  }
  else if(digitalRead(sensorFL)==LOW && digitalRead(sensorBL)==LOW)
  {
   _motor1.run(FORWARD);      // turn it on go left
   _motor2.run(RELEASE);
  }
  else if(digitalRead(sensorBL)==LOW)
    right();
  else if(digitalRead(sensorBR)==LOW)
    left();
  else if(digitalRead(sensorFR)==LOW)
    right();
  else if(digitalRead(sensorFL)==LOW)
    left();
  
  if(digitalRead(sensorBL)==HIGH && digitalRead(sensorBR)==HIGH)
    forward();
  //if(digitalRead(sensorFL)==HIGH && digitalRead(sensorFR)==HIGH)
  //  forward();
  if(MIDcounter==10)
  {
    backward();
    pause();
    //Count The Number of Grids & Print on Serial Monitor
    gridCounter++; 
    Serial.print("GRID#: ");
    Serial.println(gridCounter);
    break;
  }
  }
}

void LineFollower::pause()
{
  _motor1.run(RELEASE);      // stopped
  _motor2.run(RELEASE);
}

void LineFollower::readSensors()
//Show Sensor Readings on the Serial Monitor (_senBLACK=1 & WHITE=0)
{

 if (digitalRead(sensorFR)==1) _senFR=0;
 else _senFR=1;

  if(digitalRead(sensorFL)==1) _senFL=0;
  else _senFL=1;

  if(digitalRead(sensorBR)==1) _senBR=0;
  else _senBR=1;

  if(digitalRead(sensorBL)==1) _senBL=0;
  else _senBL=1;


  Serial1.print('\n');
  Serial1.print("_senFL:");
  Serial1.print(_senFL);
  Serial1.print("    _senFR:");
  Serial1.print(_senFR);
  Serial1.print("    _senBL:");
  Serial1.print(_senBL);
  Serial1.print("    _senBR:");
  Serial1.print(_senBR);


  Serial.print('\n');
  Serial.print("_senFL:");
  Serial.print(_senFL);
  Serial.print("    _senFR:");
  Serial.print(_senFR);
  Serial.print("    _senBL:");
  Serial.print(_senBL);
  Serial.print("    _senBR:");
  Serial.print(_senBR);
}

int LineFollower::getGridCount()
{
	return gridCounter;
}

/*** Helper Functions ***/

void LineFollower::forward()
{
  _motor1.run(FORWARD);        // turn it on go forward
  _motor2.run(FORWARD);
}

void LineFollower::backward()
{
  _motor1.run(BACKWARD);      // turn it on go back
  _motor2.run(BACKWARD);
}

void LineFollower::left()
{
  _motor1.run(FORWARD);      // turn it on go right
  _motor2.run(BACKWARD);
}

void LineFollower::right()
{
  _motor1.run(BACKWARD);      // turn it on go left
  _motor2.run(FORWARD);
}

/*
void slowForward()
{
  delay(30);
  forward();
  delay(30);
  pause();
}

/*
int _senFRontonWhite()
{
  int result;
  if(digitalRead(_pinFL)==HIGH || digitalRead(_pinFR)==HIGH)
  result=1;
  else
  result=0;
  return result;
}
*/