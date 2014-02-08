/* LineFollower Header File
 * 
 * @author: Navid Fattahi 	<navid.fattahi (at) alumni.ubc.ca>
 *			Alireza Afshar
 * 
 * August 2013
 * this code is public domain, enjoy!
 *
 */


#ifndef _LINEFOLLOWER_H_
#define _LINEFOLLOWER_H_

#include <SoftwareSerial.h>
#include <AFMotor.h>

class LineFollower
{
  public:
    LineFollower(int pinFR, int pinFL, int pinBR, int pinBL);
    void moveForward();
	void moveBackward();
	void turnLeft();
	void turnRight();
	void turnAround();
	void readSensors();
	void pause();
	
	void setMotorSpeed(int m1speed, int m2speed);
	
	int getGridCount();
	
  private:
    int _pinFR, _pinFL, _pinBR, _pinBL;
    int _senFR, _senFL, _senBR, _senBL;
	int gridCounter;
	AF_DCMotor _motor1; // create motor #1, 64KHz pwm
    AF_DCMotor _motor2; // create motor #2, 64KHz pwm
	
    void forward();
    void backward();
    void right();
    void left();
};

#endif
