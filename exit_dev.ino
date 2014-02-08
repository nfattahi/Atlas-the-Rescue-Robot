#include <SoftwareSerial.h>
#include <AFMotor.h>
#include <LineFollower.h>
#include <GridMap.h>

#define SD_PWR_BTN  0  //Power Button
#define SD_MOTOR_R  1  //Motor Right
#define SD_MOTOR_L  2  //Motor Left
#define SD_IRSEN_R  3  //IR Right
#define SD_IRSEN_C  4  //IR Centre
#define SD_IRSEN_L  5  //IR Left
#define SD_IR_TOPS  6  //IR Top: Beacon Detector
#define SD_LINESEN  7  //Line Sensors
#define SD_ENCDR_R  8  //Encoder Right
#define SD_ENCDR_L  9  //Encoder Left
#define SD_RBT_DIR  10 //Robot's Direction
#define SD_NEW_MOV  11

/*** Pin Assignments ***/
//Transceiver:Analogue
#define PIN_BECN_N  8
#define PIN_BECN_E  9
#define PIN_BECN_S  10
#define PIN_BECN_W  11 
//IR Sensors: Analogue
#define PIN_IR_CTR  12
#define PIN_IR_LFT  13
#define PIN_IR_RGT  14
#define PIN_IR_TOP  15
//Line Sensors:Digital
#define PIN_LNS_FR  51
#define PIN_LNS_FL  53
#define PIN_LNS_BR  49
#define PIN_LNS_BL  47
#define PIN_LNS_MD  45

/*** Constants ***/
#define MOVFWD  0
#define MOVRGT  1
#define MOVBWD  2
#define MOVLFT  3
#define MOVARD	4
#define IR_RGT  0
#define IR_CTR  1
#define IR_LFT  2
#define RIGHT	0
#define LEFT	1
#define OFF	    0
#define ON	    1
#define BLOCKED 1
#define DIR_NORTH	0
#define DIR_EAST	1
#define DIR_SOUTH	2
#define DIR_WEST	3
#define BEACON_N	0
#define BEACON_NE	1
#define BEACON_E	2
#define BEACON_SE	3
#define BEACON_S	4
#define BEACON_SW	5
#define BEACON_W	6
#define BEACON_NW	7
#define BLOCK_LIMIT  10
#define BEACON_LIMIT 19
#define MISSION_ACCOMPLISHED	99

/* Robot State */
int frontSensor = 0;
int rightSensor = 0;
int leftSensor  = 0;
int deadEndDetected = 0;

//Transceiver Direction Mode//
int northCount = 0;
int southCount = 0;
int westCount  = 0;
int eastCount  = 0;

/* Global Variables */
int gridCounter  = 0;
int returnMode   = 0;
int loopFlag     = 0;
int switchcase	 = 0;
int hugcondition = 0;
int passoverself = 0;
int stickyWall   = 0;
int lastturnx    = -1;
int lastturny    = -1;
int beaconposx;
int beaconposy;
const int startposx = 7;
const int startposy = 7;

/* Path Memory */
int pathMem[128];
int pathMemCount = 0;
int pathMemPos   = 0;

/* GUI Stuff */
unsigned char serialData[10];
byte distLft = 0xFF;
byte distRgt = 0xFF;
byte distCtr = 0xFF;
byte bDetect = 0xFF;
byte lineSen = 0xFF;
byte nextMov = 0xFF;

/* Global Custom Objects */
LineFollower rover(PIN_LNS_FR, PIN_LNS_FL, PIN_LNS_BR, PIN_LNS_BL);
GridMap mapper;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  rover.setMotorSpeed(200, 200);
}

void loop()
{
  char keypress1;
  char keypress2;
  int  beaconDir = 0;
  int  homeDir = 0;

  if(Serial1.available())
  {
   keypress1 = (char)Serial1.read();
   Serial.print(keypress1);

   if(keypress1=='w')
   {
      rover.moveForward();
      pathMem[pathMemCount] = 0;
      pathMemCount++;
   }
   if(keypress1=='k')
   {
      rover.pause();
      pathMemPos=0;
      pathMemCount=0;
   }
   if(keypress1=='a')
   {
      rover.turnLeft();
      pathMem[pathMemCount] = 3;
      pathMemCount++;
   }
   if(keypress1=='d')
   {
      rover.turnRight();
      pathMem[pathMemCount] = 1;
      pathMemCount++;
   }
   if(keypress1=='u')
   {
     int i = 1;
     while (i==1)
     {
      rover.readSensors();
      keypress1 = (char)Serial1.read();
      if (keypress1=='i')
      break;
     }
   }
   if(keypress1=='o')
   {
     int i = 1;
     while (i==1)
     {
      printRangefinders();
      keypress1 = (char)Serial1.read();
      if (keypress1=='i')
       break;
     }
   }
   if(keypress1=='r')
   {
      returnMode = ON;
      pathMemPos = pathMemCount;
   }
   if(keypress1=='s')
      rover.moveBackward();
   if(keypress1=='t')
      rover.turnAround();
   if(keypress1=='b')
   {
     int i=1;
     while(i==1)
     {
      if (beaconDetected() == 1 && returnMode == OFF)
      {
        returnMode = ON;
        pathMemPos = pathMemCount;
		beaconposx = mapper.getCurrPosx();
		beaconposy = mapper.getCurrPosy();
        // Memory-based Return: back-off one step and turn 180 degrees
		/*
        rover.moveBackward();
        rover.turnAround(); */
        delay(3000);
      }

      if (returnMode == OFF)
      {
        //printBeacon();
		//printRangefinders();
        beaconDir = getBeaconDirection();
        navigate(beaconDir);
      }
	  else
	  {
        homeDir = getStartPosDirection();
		if (homeDir == MISSION_ACCOMPLISHED)
		{
			Serial.println("Got Back Home. Safe and Sound!");
			Serial1.println("Got Back Home. Safe and Sound!");
			break;
		}
        navigate(homeDir);
	  }
	  /*
      else
        returnHome();
	  */
      if ((char)Serial.read() == 'k')
        break;
     }
   }
  }

  if(Serial.available())
  {
    keypress2 = (char)Serial.read();
    // Send any characters the Serial monitor prints to the bluetooth
    Serial1.print(keypress2);

   if(keypress2=='w')
   {
      rover.moveForward();
      pathMem[pathMemCount] = 0;
      pathMemCount++;
   }
   if(keypress2=='k')
   {
      rover.pause();
      pathMemPos=0;
      pathMemCount=0;
   }
   if(keypress2=='a')
   {
      rover.turnLeft();
      pathMem[pathMemCount] = 3;
      pathMemCount++;
   }
   if(keypress2=='d')
   {
      rover.turnRight();
      pathMem[pathMemCount] = 1;
      pathMemCount++;
   }
   if(keypress2=='u')
   {
     int i = 1;
     while (i==1)
     {
      rover.readSensors();
      keypress2 = (char)Serial.read();
      if (keypress2=='i')
      break;
     }
   }

   if(keypress2=='r')
   {
      returnMode = 1;
      pathMemPos = pathMemCount;
   }
   if(keypress2=='o')
   {
     int i = 1;
     while (i==1)
     {
      printRangefinders();
      keypress2 = (char)Serial.read();
      if (keypress2=='i')
       break;
     }
   }
   if(keypress2=='s')
      rover.moveBackward();
   if(keypress2=='t')
      rover.turnAround();
   if(keypress2=='b')
   {
     int i=1;
     while(i==1)
     {
      //printPathMem();

      if (beaconDetected() == 1 && returnMode == 0)
      {
        returnMode = 1;
        pathMemPos = pathMemCount;
		beaconposx = mapper.getCurrPosx();
		beaconposy = mapper.getCurrPosy();
        //Back-off one step and turn 180 degrees
        /*
		rover.moveBackward();
        rover.turnAround();*/
        delay(3000);
      }

      if (returnMode == 0)
      {
        //printBeacon();
		//printRangefinders();
        beaconDir = getBeaconDirection();
        navigate(beaconDir);
      }
	  else
	  {
        homeDir = getStartPosDirection();
		if (homeDir == MISSION_ACCOMPLISHED)
		{
			Serial.println("Got Back Home. Safe and Sound!");
			Serial1.println("Got Back Home. Safe and Sound!");
			break;
		}
        navigate(homeDir);
	  }
	  /*
      else
        returnHome();
	  */

      if ((char)Serial.read() == 'k')
        break;
     }
   }
  }
  //updateGUI();

}

/* Takes in direction of beacon as a parameter
   and decides where to go next. The decision
   is made based on the beacon direction as well
   as the nearby obstacles.
   @param: Beacon Direction
   @return: none
*/
void navigate(int beaconDirection)
{
  int nextMove = MOVFWD;

  updateIRSensors();
  int inLoop = checkForLoop();
  
  mapper.printMap();
  
  // If in a dead-end, get out of function:
  // (let the checkForLoop() function take care of the situation)
  if(deadEndDetected==1)
    return;
  // If stuck in a loop, get out of function:
  // (let the checkForLoop() function take care of the situation)
  if (inLoop == 1)
	return;
	
  updateIRSensors();

  if (beaconDirection == 0)
  {
    if ( rangefinder(analogRead(PIN_IR_CTR)) > 10 )
      nextMove = MOVFWD;
    else
    {
      if ( rangefinder(analogRead(PIN_IR_RGT)) >= rangefinder(analogRead(PIN_IR_LFT)) )
        nextMove = MOVRGT;
      else
        nextMove = MOVLFT;
    }
  }
  else if (beaconDirection == 2)
  {
    if ( rangefinder(analogRead(PIN_IR_RGT)) > 10 )
      nextMove = MOVRGT;
    else
    {
      if ( rangefinder(analogRead(PIN_IR_CTR)) >= 10 )
        nextMove = MOVFWD;
      else
        nextMove = MOVLFT;
    }
  }
  else if (beaconDirection == 6)
  {
    if ( rangefinder(analogRead(PIN_IR_LFT)) > 10 )
      nextMove = MOVLFT;
    else
    {
      if ( rangefinder(analogRead(PIN_IR_CTR)) >= 10 )
        nextMove = MOVFWD;
      else
        nextMove = MOVRGT;
    }
  }
  else if (beaconDirection == 1)  // North-East (1st quadrent)
  {
    if ( rangefinder(analogRead(PIN_IR_CTR)) >= rangefinder(analogRead(PIN_IR_RGT)) )
      nextMove = MOVFWD;
    else
      nextMove = MOVRGT;
  }
  else if (beaconDirection == 7)  // North-East (2nd quadrent)
  {
    if ( rangefinder(analogRead(PIN_IR_CTR)) >= rangefinder(analogRead(PIN_IR_LFT)) )
      nextMove = MOVFWD;
    else
      nextMove = MOVLFT;
  }
  // Cases: Beacon is behind the robot
  else if (beaconDirection == 5)
  {
    if ( rangefinder(analogRead(PIN_IR_LFT)) >= BLOCK_LIMIT )
      nextMove = MOVLFT;
    else
      nextMove = MOVFWD;
  }
  else if (beaconDirection == 3)
  {
    if ( rangefinder(analogRead(PIN_IR_RGT)) >= BLOCK_LIMIT)
      nextMove = MOVRGT;
    else
      nextMove = MOVFWD;
  }
  else if (beaconDirection == 4)
  {
    if ( rangefinder(analogRead(PIN_IR_LFT)) <= rangefinder(analogRead(PIN_IR_RGT)) )
      nextMove = MOVRGT;
    else
      nextMove = MOVLFT;
  }
  else
  {
    if ( rangefinder(analogRead(PIN_IR_RGT)) >= rangefinder(analogRead(PIN_IR_LFT)) )
      nextMove = MOVRGT;
    else
      nextMove = MOVLFT;
  }
  
  /*
  Serial1.print("nextMove: ");
  Serial1.println(nextMove);
  
	Serial1.print("switchcase: ");
	Serial1.println(switchcase);
	Serial1.print("lastturnyx: ");
	Serial1.println(lastturnx);
	Serial1.print("lastturny: ");
	Serial1.println(lastturny);
	Serial1.print("mapper.getCurrPosx(): ");
	Serial1.println(mapper.getCurrPosx());
	Serial1.print("mapper.getCurrPosy(): ");
	Serial1.println(mapper.getCurrPosy());
  */
  
  if (nextMove == MOVFWD)      //Forward
  {
    rover.moveForward();
	mapper.updateMap(MOVFWD);
    pathMem[pathMemCount] = 0;
    nextMov = 0;
    pathMemCount++;
  }
  else if (nextMove == MOVRGT)  //Right
  {
    lastturnx = mapper.getCurrPosx();
    lastturny = mapper.getCurrPosy();
    rover.turnRight();
	mapper.updateMap(MOVRGT);
	deadEndDetected = 0;
    pathMem[pathMemCount] = 1;
    nextMov = 1;
    pathMemCount++;
  }
  else if (nextMove == MOVBWD)  //Backward
  {
    rover.turnAround();
	mapper.updateMap(MOVBWD);
    pathMem[pathMemCount] = 2;
    nextMov = 2;
    pathMemCount++;
  }
  else if (nextMove == MOVLFT)  //Left
  {
    lastturnx = mapper.getCurrPosx();
    lastturny = mapper.getCurrPosy();
    rover.turnLeft();
	mapper.updateMap(MOVLFT);
	deadEndDetected = 0;
    pathMem[pathMemCount] = 3;
    nextMov = 3;
    pathMemCount++;
  }
}

/* returnHome function:
   Takes the robot back home after detection of
   the beacon using the step-by-step memory.
   This is our basic Return Home strategy. The
   advanced return home strategy uses our map
   and tries to get back to the home grid after
   beacon detection.
   @param: None
   @return: None
*/
void returnHome()
{
  int nextMove = 0;

  if (pathMemPos >= 2)
  {
    nextMove = pathMem[pathMemPos - 1];
    if (nextMove == 0)
    {
      rover.moveForward();
	  mapper.updateMap(MOVFWD);
      Serial.print("Return Mode: Move Forward");
    }
    else if (nextMove == 1)
    {
      rover.turnLeft();
	  mapper.updateMap(MOVLFT);
      Serial.print("Return Mode: Turn Left");
    }
    else if (nextMove == 3)
    {
      rover.turnRight();
	  mapper.updateMap(MOVRGT);
      Serial.print("Return Mode: Turn Right");
    }
    else
    {
      rover.pause();
      Serial.print("Return Mode: Skip!");
    }

    pathMemPos--;
  }
  else
  {
    Serial.println("Got back home. Safe and Sound!");
    Serial1.println("Got back home. Safe and Sound!");
  }
}

/* hugWall() function:
   Takes in which side to hug as a parameter,
   and makes the robot stick to the wall on the
   specified side until wall is clear.
   @param: Side to Hug
   @return: None
*/
void hugWall(int side)
{
	int turnCounter = 0;
	int forwardCounter = 0;
	
	// Case 1: Hug the Right Wall
	if ( side == RIGHT )
	{
		forwardCounter = 0;
		while (turnCounter < 2 && forwardCounter < 6)
		{
			updateIRSensors();
			/*
			Serial1.print("switchcase: ");
			Serial1.println(switchcase);
			Serial1.print("lastturnx: ");
			Serial1.println(lastturnx);
			Serial1.print("lastturny: ");
			Serial1.println(lastturny);
			Serial1.print("mapper.getCurrPosx(): ");
			Serial1.println(mapper.getCurrPosx());
			Serial1.print("mapper.getCurrPosy(): ");
			Serial1.println(mapper.getCurrPosy());
			Serial1.println("Hugging Right");
			*/
			//Dead-End Case
			if ( leftSensor == BLOCKED && rightSensor == BLOCKED && frontSensor == BLOCKED)
			{
				Serial1.println("Dead End!");
				Serial.println("Dead End!");
				deadEndDetected = 1;
				rover.moveBackward();
				// Remove last step from return memory
				pathMemCount--;
				mapper.updateDeadEnd();
				mapper.updateMap(MOVBWD);
				blockIRSensor(IR_CTR);
			}
			else if (rightSensor != BLOCKED)
			{
				rover.turnRight();
				mapper.updateMap(MOVRGT);
				deadEndDetected = 0;
				forwardCounter = 0;
				turnCounter++;
				if (turnCounter == 2)
				{
					updateIRSensors();
					if (frontSensor == BLOCKED && rightSensor == BLOCKED && leftSensor != BLOCKED)
					{
						rover.turnLeft();
						mapper.updateMap(MOVLFT);
					}
				}
				hugcondition = 0;
				pathMem[pathMemCount] = 1;
				pathMemCount++;
			}
			else if (frontSensor == BLOCKED)
			{
				rover.turnLeft();
				mapper.updateMap(MOVLFT);
				deadEndDetected = 0;
				forwardCounter = 0;
				pathMem[pathMemCount] = 3;
				pathMemCount++;
			}
			else if (rightSensor == BLOCKED)
			{
				rover.moveForward();
				forwardCounter++;
				deadEndDetected = 0;
				mapper.updateMap(MOVFWD);
				pathMem[pathMemCount] = 0;
				pathMemCount++;
			}
			
			updateIRSensors();
			
			if (switchcase == 1)	// consider same-column exit condition
			{
			/*
			Serial.print("switchcase: ");
			Serial.println(switchcase);
			Serial1.print("lastturnx: ");
			Serial1.println(lastturnx);
			Serial.print("lastturny: ");
			Serial.println(lastturny);
			Serial.print("mapper.getCurrPosx(): ");
			Serial.println(mapper.getCurrPosx());
			Serial.print("mapper.getCurrPosy(): ");
			Serial.println(mapper.getCurrPosy());
			*/
				if (mapper.getCurrPosx() == lastturnx)
				{
				    if (frontSensor != BLOCKED)
					{
						rover.moveForward();
						mapper.updateMap(MOVFWD);
					}
					else if (frontSensor == BLOCKED && leftSensor == BLOCKED && rightSensor == BLOCKED)
					{
						rover.moveBackward();
						mapper.updateDeadEnd();
						mapper.updateMap(MOVBWD);
					}
					else
					{
						if (rangefinder(analogRead(PIN_IR_LFT)) <= rangefinder(analogRead(PIN_IR_RGT)))
						{
							rover.turnRight();
							mapper.updateMap(MOVRGT);
						}
						else
						{
							rover.turnLeft();
							mapper.updateMap(MOVLFT);							
						}
					}
					hugcondition = 1;
					pathMem[pathMemCount] = 0;
					pathMemCount++;
					break;
				}
			}
			
			if (beaconDetected() == 1 && returnMode == 0)
				break;
				
			if (getStartPosDirection() == MISSION_ACCOMPLISHED && returnMode == 1)
				break;
				
			mapper.printMap();
			updateIRSensors();
		}
        Serial1.println("Got out of hugWall(right)");
	}
	// Case 2: Hug the Left Wall
	else if ( side == LEFT )
	{
		forwardCounter = 0;
		while (turnCounter < 2 && forwardCounter < 6)
		{
			updateIRSensors();
			Serial1.println("Hugging Left");
			if ( leftSensor == BLOCKED && rightSensor == BLOCKED && frontSensor == BLOCKED)
			{
				Serial1.println("Dead End!");
				Serial.println("Dead End!");
				deadEndDetected = 1;
				rover.moveBackward();
				// Remove last step from return memory
				pathMemCount--;
				mapper.updateDeadEnd();
				mapper.updateMap(MOVBWD);
				blockIRSensor(IR_CTR);
			}
			else if (leftSensor  != BLOCKED)
			{
				rover.turnLeft();
				mapper.updateMap(MOVLFT);
				deadEndDetected = 0;
				forwardCounter = 0;
				turnCounter++;
				if (turnCounter == 2)
				{
					updateIRSensors();
					if (frontSensor == BLOCKED && leftSensor == BLOCKED && rightSensor != BLOCKED)
					{
						rover.turnRight();
						mapper.updateMap(MOVRGT);
					}
				}
				hugcondition = 0;
				pathMem[pathMemCount] = 3;
				pathMemCount++;
			}
			else if (frontSensor == BLOCKED)
			{
				rover.turnRight();
				mapper.updateMap(MOVRGT);
				deadEndDetected = 0;
				forwardCounter = 0;
				pathMem[pathMemCount] = 1;
				pathMemCount++;
			}
			else if (leftSensor  == BLOCKED)
			{
				rover.moveForward();
				forwardCounter++;
				deadEndDetected = 0;
				mapper.updateMap(MOVFWD);
				pathMem[pathMemCount] = 0;
				pathMemCount++;
			}
			
			updateIRSensors();
			
			if (switchcase == 1)	// consider same-column exit condition
			{
			/*
			Serial.print("switchcase: ");
			Serial.println(switchcase);
			Serial.print("lastturny: ");
			Serial.println(lastturny);
			Serial.print("mapper.getCurrPosx(): ");
			Serial.println(mapper.getCurrPosx());
			Serial.print("mapper.getCurrPosy(): ");
			Serial.println(mapper.getCurrPosy());
			*/
			
				if (mapper.getCurrPosx() == lastturnx)
				{
				    if (frontSensor != BLOCKED)
					{
						rover.moveForward();
						mapper.updateMap(MOVFWD);
					}
					else if (frontSensor == BLOCKED && leftSensor == BLOCKED && rightSensor == BLOCKED)
					{
						rover.moveBackward();
						mapper.updateDeadEnd();
						mapper.updateMap(MOVBWD);
					}
					else
					{
						if (rangefinder(analogRead(PIN_IR_LFT)) <= rangefinder(analogRead(PIN_IR_RGT)))
						{
							rover.turnRight();
							mapper.updateMap(MOVRGT);
						}
						else
						{
							rover.turnLeft();
							mapper.updateMap(MOVLFT);							
						}
					}		
					hugcondition = 1;
					pathMem[pathMemCount] = 0;
					pathMemCount++;
					break;
				}
			}
			
			if (beaconDetected() == 1 && returnMode == 0)
				break;
				
			if (getStartPosDirection() == MISSION_ACCOMPLISHED && returnMode == 1)
				break;
				
			mapper.printMap();
			updateIRSensors();
		}
        Serial1.println("Got out of hugWall(left)");
	}
	
	loopFlag = 0;
}

int checkForLoop()
{
//	int stickyWall;
//	printIRSensor();
	
	
	if ( frontSensor == BLOCKED )
	{
		if ( leftSensor == BLOCKED && rightSensor == BLOCKED )
		{
			Serial1.println("Dead End!");
			Serial.println("Dead End!");
			deadEndDetected = 1;
			loopFlag = OFF;
			rover.moveBackward();
			updateIRSensors();
			// Remove last step from return memory
			pathMemCount--;
			mapper.updateDeadEnd();
			mapper.updateMap(MOVBWD);
			blockIRSensor(IR_CTR);
		}
		else if ( rightSensor == BLOCKED )
		{
			loopFlag = ON;
			if ( lastturnx == -1)
			{
				lastturnx = mapper.getCurrPosx();
				lastturny = mapper.getCurrPosy();
				passoverself = ON;
			}
			else
			{
				passoverself = OFF;
			}
		}
		else if ( leftSensor == BLOCKED )
		{
			loopFlag = ON;
			if ( lastturnx == -1)
			{
				lastturnx = mapper.getCurrPosx();
				lastturny = mapper.getCurrPosy();
				passoverself = ON;
			}
			else
			{
				passoverself = OFF;
			}
		}
		else if ( leftSensor != BLOCKED && rightSensor != BLOCKED && deadEndDetected == ON)
		{
			rover.turnRight();
			mapper.updateMap(MOVRGT);
			deadEndDetected = OFF;
			pathMem[pathMemCount] = 1;
			pathMemCount++;
		}
		
		updateIRSensors();
		 
		if (hugcondition == 1 && frontSensor == BLOCKED)
		{
			if (stickyWall == RIGHT)
			{
				if (rightSensor == BLOCKED)
				{
					rover.turnAround();
				}
				else
				{
					rover.turnRight();
					mapper.updateMap(MOVRGT);
				}
					
				stickyWall = LEFT;
				hugcondition = 0;
				loopFlag = ON;
				updateIRSensors();
				/*
				if (leftSensor != BLOCKED)
				{
					rover.turnLeft();
					mapper.updateMap(MOVLFT);
				}
				*/
				
			}
			else if (stickyWall == LEFT)
			{
				if (leftSensor == BLOCKED)
				{
					rover.turnAround();
				}
				else
				{
					rover.turnLeft();
					mapper.updateMap(MOVLFT);
				}
					
				stickyWall = RIGHT;
				hugcondition = 0;
				loopFlag = ON;
				updateIRSensors();
				/*
				if (rightSensor != BLOCKED)
				{
					rover.turnRight();
					mapper.updateMap(MOVRGT);
                }
				*/
			}
		}
		
		if (loopFlag == ON && hugcondition == 0)
		{
			if (mapper.getCurrPosx() == lastturnx && passoverself == OFF)	// if robot is already in the same column that it started don't consider the exception condition
				switchcase = 0;
			else
			{
				switchcase = 1;
				passoverself = OFF;
			}
			/*
			Serial.print("switchcase: ");
			Serial.println(switchcase);
			Serial.print("lastturny: ");
			Serial.println(lastturny);
			Serial.print("mapper.getCurrPosx(): ");
			Serial.println(mapper.getCurrPosx());
			Serial.print("mapper.getCurrPosy(): ");
			Serial.println(mapper.getCurrPosy());
			
			Serial1.print("switchcase: ");
			Serial1.println(switchcase);
			Serial1.print("lastturny: ");
			Serial1.println(lastturny);
			Serial1.print("mapper.getCurrPosx(): ");
			Serial1.println(mapper.getCurrPosx());
			Serial1.print("mapper.getCurrPosy(): ");
			Serial1.println(mapper.getCurrPosy());
			*/
			
		}
	}
	/*
	Serial1.print("switchcase: ");
	Serial1.println(switchcase);
	Serial1.print("hugcondition: ");
	Serial1.println(hugcondition);
	Serial.print("switchcase: ");
	Serial.println(switchcase);
	Serial.print("hugcondition: ");
	Serial.println(hugcondition);
	*/
	// Loop Found! Let's get out of it...
	if (loopFlag == ON)
	{
        Serial1.println("Stuck in a loop! Let's get out... (loopFlag = ON)");
		hugWall(stickyWall);
		return 1;
	}
	else
		return 0;
}

/* converts analogue voltage from IR sensors
   to distance in centimetres */
double rangefinder(int voltage)
{
  /*
  double distance = 0;  // in cm
  if (voltage >= 80 && voltage <= 500)
    distance = 4800/(double)(voltage - 20);
  else if (voltage < 80)
    distance = 99;
  else if (voltage > 500)
    distance = -1;*/
  int voltage_mv = voltage / 1023.0 * 5000;
  float distance_mm = 1085534.81 * (float)pow((float)voltage_mv, -1.2);
  float distance=distance_mm/10;
  return distance;
}

int beaconDetected()
{
  if (rangefinder(analogRead(PIN_IR_TOP)) > BEACON_LIMIT)
    return 0;
  else
  {
    Serial1.print("Beacon Detected!");
    return 1;
  }
}

void printPathMem()
{
  Serial.print("[");
  for (int i = 0; i <pathMemCount; i++)
  {
    Serial.print(pathMem[i]);
    Serial.print(" ");
  }
  Serial.print("]");
}

void updateGUI()
{
  distLft = byte(rangefinder(analogRead(PIN_IR_LFT)));
  distRgt = byte(rangefinder(analogRead(PIN_IR_RGT)));
  distCtr = byte(rangefinder(analogRead(PIN_IR_CTR)));
  bDetect = byte(rangefinder(analogRead(PIN_IR_TOP)));

  lineSen = digitalRead(PIN_LNS_FR) * 16 + digitalRead(PIN_LNS_FL) * 8 + digitalRead(PIN_LNS_BR) * 4 + digitalRead(PIN_LNS_BL) * 2 + digitalRead(PIN_LNS_MD);

  serialData[SD_IRSEN_L] = distLft;
  serialData[SD_IRSEN_R] = distRgt;
  serialData[SD_IRSEN_C] = distCtr;
  serialData[SD_LINESEN] = lineSen;
  serialData[SD_IR_TOPS] = bDetect;
  serialData[SD_MOTOR_L] = 0;	//TODO: Read motor value
  serialData[SD_MOTOR_L] = 0;
  serialData[SD_RBT_DIR] = nextMov;

  //Serial communication
  for(int i = 0; i < 11; i++)
  {
    //trim all the data into double hex display
    if (serialData[i]<=0xF)
    {
      Serial1.print(0,HEX);
    }
    Serial1.print(serialData[i],HEX);
  }

  if (serialData[15]<=0xF)
  {
    Serial1.print(0,HEX);
  }
  Serial1.println(serialData[15],HEX);
}

//Beacon Array Count //
int getBeaconDirection()
{
  northCount = 0;
  southCount = 0;
  westCount  = 0;
  eastCount  = 0;
  
  for (int index = 0; index < 10 ; index++)
  {
    findPrimaryDir();
    delay(50);
  }
	
  int beaconDir = BEACON_N;
	
  // North
  if (northCount > southCount && northCount > eastCount && northCount > westCount)
  {
    if (northCount > 0 && eastCount > 0 && southCount < 1 && westCount < 1)
      beaconDir = BEACON_NE; 
    else if (northCount > 0 && eastCount < 1 && southCount < 1 && westCount > 0)
      beaconDir = BEACON_NW;
    else 
      beaconDir = BEACON_N;
  }
  
  // South
  if (southCount > northCount && southCount > eastCount && southCount > westCount)
  {
      if (northCount < 1 && eastCount > 0 && southCount > 0 && westCount < 1)
        beaconDir = BEACON_SE;
      else if (northCount < 1 && eastCount < 1 && southCount > 0 && westCount > 0)
        beaconDir = BEACON_SW;
      else 
        beaconDir = BEACON_S;
  }
  
  // East
  if (eastCount > northCount && eastCount > southCount && eastCount > westCount) 
  {
    if (northCount > 0 && eastCount > 0 && southCount < 1 && westCount < 1)
      beaconDir = BEACON_NE; 
    else if (northCount < 1 && eastCount > 0 && southCount > 0 && westCount < 1)
      beaconDir = BEACON_SE;
    else
      beaconDir = BEACON_E;
  }
    
  // West
  if (westCount > northCount && westCount > southCount && westCount > eastCount) 
  {
    if (northCount > 0 && eastCount < 1 && southCount < 1 && westCount > 0)
      beaconDir = BEACON_NW;
    else if (northCount < 1 && eastCount < 1 && southCount > 0 && westCount > 0)
      beaconDir = BEACON_SW;
    else
      beaconDir = BEACON_W;
  }
  
  return beaconDir;
}

/* getStartPosDirection() function:
   Gives the direction of the Home Grid based on
   current position of the robot (using the map).
   This function is used to direct the robot from
   beacon back to Home.
   @param: none
   @return: Home Direction (or mission-accomplished signal)
*/
int getStartPosDirection()
{
	int robotDir = mapper.getRobotDir();
	int currposx = mapper.getCurrPosx();
	int currposy = mapper.getCurrPosy();
	
	if (currposx == startposx && currposy == startposy)
		return MISSION_ACCOMPLISHED;
	
	if (robotDir == DIR_NORTH)
	{
		if (currposx < startposx && currposy < startposy)
			return BEACON_SE;
		else if (currposx < startposx && currposy > startposy)
			return BEACON_NE;
		else if (currposx > startposx && currposy > startposy)
			return BEACON_NW;
		else if (currposx > startposx && currposy < startposy)
			return BEACON_SW;
		else if (currposx == startposx && currposy < startposy)
			return BEACON_S;
		else if (currposx == startposx && currposy > startposy)
			return BEACON_N;
		else if (currposx > startposx && currposy == startposy)
			return BEACON_W;
		else if (currposx > startposx && currposy == startposy)
			return BEACON_E;
	}
	else if (robotDir == DIR_EAST)
	{
		if (currposx < startposx && currposy < startposy)
			return BEACON_NE;
		else if (currposx < startposx && currposy > startposy)
			return BEACON_NW;
		else if (currposx > startposx && currposy > startposy)
			return BEACON_SW;
		else if (currposx > startposx && currposy < startposy)
			return BEACON_SE;
		else if (currposx == startposx && currposy < startposy)
			return BEACON_E;
		else if (currposx == startposx && currposy > startposy)
			return BEACON_W;
		else if (currposx > startposx && currposy == startposy)
			return BEACON_S;
		else if (currposx > startposx && currposy == startposy)
			return BEACON_N;
	}
	else if (robotDir == DIR_WEST)
	{
		if (currposx < startposx && currposy < startposy)
			return BEACON_SW;
		else if (currposx < startposx && currposy > startposy)
			return BEACON_SE;
		else if (currposx > startposx && currposy > startposy)
			return BEACON_NE;
		else if (currposx > startposx && currposy < startposy)
			return BEACON_NW;
		else if (currposx == startposx && currposy < startposy)
			return BEACON_W;
		else if (currposx == startposx && currposy > startposy)
			return BEACON_E;
		else if (currposx > startposx && currposy == startposy)
			return BEACON_N;
		else if (currposx > startposx && currposy == startposy)
			return BEACON_S;
	}
	else if (robotDir == DIR_SOUTH)
	{
		if (currposx < startposx && currposy < startposy)
			return BEACON_NW;
		else if (currposx < startposx && currposy > startposy)
			return BEACON_SW;
		else if (currposx > startposx && currposy > startposy)
			return BEACON_SE;
		else if (currposx > startposx && currposy < startposy)
			return BEACON_NE;
		else if (currposx == startposx && currposy < startposy)
			return BEACON_N;
		else if (currposx == startposx && currposy > startposy)
			return BEACON_S;
		else if (currposx > startposx && currposy == startposy)
			return BEACON_E;
		else if (currposx > startposx && currposy == startposy)
			return BEACON_W;
	}
}

void findPrimaryDir()
{
  int north = analogRead(PIN_BECN_N);
  int east  = analogRead(PIN_BECN_E);
  int south = analogRead(PIN_BECN_S);
  int west  = analogRead(PIN_BECN_W);
  
  // Case 1: BEACON north
  if (north < 500 && south > 500 && east > 500 && west > 500) 
    northCount++;
    
  // CASE 2: BEACON south
  else if (north > 500 && south < 500 && east > 500 && west > 500)
     southCount++;
     
  // CASE 3: BEACON east
  else if (north > 500 && south > 500 && east < 500 && west > 500)
     eastCount++;

  // CASE 4: BEACON west
  else if (north > 500 && south > 500 && east > 500 && west < 500)
     westCount++;
}

/* blockIRSensor() function:
   Takes in an integer as an indication of
   one of the IR sensors and forces the
   specified sensor to get blocked.
   @param: sensor value (right=0; center=1; left=2)
   @return: none
*/
void blockIRSensor(int sensor)
{
  if (sensor == IR_CTR)
    frontSensor = 1;
  else if (sensor == IR_RGT)
    rightSensor = 1;
  else if (sensor == IR_LFT)
    leftSensor = 1;
}

/* updateIRSensors() function:
   Reads the value of IR sensors and updates
   the global variables corresponding to each
   IR sensor.
*/
void updateIRSensors()
{
  Serial.print("deadEndDetected: ");
  Serial.print(deadEndDetected);
  if (deadEndDetected == 0)
  {
	  if (rangefinder(analogRead(PIN_IR_CTR)) <= BLOCK_LIMIT)
		frontSensor = 1;
	  else
		frontSensor = 0;
  }
  /*
  else
  {
     deadEndDetected = 0;
  }
  */
	
  if (rangefinder(analogRead(PIN_IR_RGT)) <= BLOCK_LIMIT)
    rightSensor = 1;
  else
    rightSensor = 0;
	
  if (rangefinder(analogRead(PIN_IR_LFT)) <= BLOCK_LIMIT)
    leftSensor = 1;
  else
    leftSensor = 0;
}

void printRangefinders()
{
  Serial1.print("IR --- front: ");
  Serial1.print(rangefinder(analogRead(PIN_IR_CTR)));
  Serial1.print("       right: ");
  Serial1.print(rangefinder(analogRead(PIN_IR_RGT)));
  Serial1.print("       left: ");
  Serial1.print(rangefinder(analogRead(PIN_IR_LFT)));
  Serial1.print("       top: ");
  Serial1.println(rangefinder(analogRead(PIN_IR_TOP)));

  Serial.print("IR --- front: ");
  Serial.print(rangefinder(analogRead(PIN_IR_CTR)));
  Serial.print("       right: ");
  Serial.print(rangefinder(analogRead(PIN_IR_RGT)));
  Serial.print("       left: ");
  Serial.print(rangefinder(analogRead(PIN_IR_LFT)));
  Serial.print("       top: ");
  Serial.println(rangefinder(analogRead(PIN_IR_TOP)));
}

void printIRSensor()
{
  Serial1.print("IR --- front: ");
  Serial1.print(frontSensor);
  Serial1.print("       right: ");
  Serial1.print(rightSensor);
  Serial1.print("       left: ");
  Serial1.println(leftSensor);

  Serial.print("IR --- front: ");
  Serial.print(frontSensor);
  Serial.print("       right: ");
  Serial.print(rightSensor);
  Serial.print("       left: ");
  Serial.println(leftSensor);
}

void printBeacon()
{
  Serial.print("North: ");
  Serial.print(analogRead(PIN_BECN_N));
  Serial.print(" South: ");
  Serial.print(analogRead(PIN_BECN_S));
  Serial.print(" East: ");
  Serial.print(analogRead(PIN_BECN_E));
  Serial.print(" West: ");
  Serial.println(analogRead(PIN_BECN_W));
  
  Serial1.print("North: ");
  Serial1.print(analogRead(PIN_BECN_N));
  Serial1.print(" South: ");
  Serial1.print(analogRead(PIN_BECN_S));
  Serial1.print(" East: ");
  Serial1.print(analogRead(PIN_BECN_E));
  Serial1.print(" West: ");
  Serial1.println(analogRead(PIN_BECN_W));
}
