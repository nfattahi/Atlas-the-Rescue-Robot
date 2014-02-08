/* GridMap Class
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

#include <GridMap.h>

#define DIR_NORTH	0
#define DIR_EAST	1
#define DIR_SOUTH	2
#define DIR_WEST	3

#define MOVFWD  0
#define MOVRGT  1
#define MOVBWD  2
#define MOVLFT  3
#define MOVARD	4

GridMap::GridMap()
{
  direction = 0;
  currposx  = 7;
  currposy  = 7;

  int i, j;
  for (i = 0; i < 15; i++)
    for (j = 0; j < 8; j++)
      map[i][j] = 0;
	
  // Robot's starting position: map[8][7]
  map[currposx][currposy] = 1;
}

/* Takes in the nextStep and fills the
   corresponding cell in the map according
   to the facing direction of the robot 
   
   @param: nextStep (moving direction)
   @return: 1 for success, 0 for failure
 */
int GridMap::updateMap (int nextStep)
{
	int nextposx = currposx;
	int nextposy = currposy;
		
	if (direction == 0)			// Facing North
	{
	  if (nextStep == 0)		// Moving Forward
		nextposy--;
	  else if (nextStep == 1)	  // Turning Right
	  {
		nextposx++;
		direction = DIR_EAST;
	  }
	  else if (nextStep == 2)	// Moving Backward
	    nextposy++;
	  else if (nextStep == 3)	// Turning Left
	  {
		nextposx--;
		direction = DIR_WEST;
	  }
	  else if (nextStep == 4)
	  {
		direction = DIR_SOUTH;
	  }
	}
	else if (direction == 1)	// Facing East
	{
	  if (nextStep == 0)		// Moving Forward
		nextposx++;
	  else if (nextStep == 1)	// Turning Right
	  {
		nextposy++;
		direction = DIR_SOUTH;
	  }
	  else if (nextStep == 2)	// Moving Backward
	    nextposx--;
	  else if (nextStep == 3)	// Turning Left
	  {
		nextposy--;
		direction = DIR_NORTH;
	  }
	  else if (nextStep == 4)
	  {
		direction = DIR_WEST;
	  }
	}
	else if (direction == 2)	// Facing South
	{
	  if (nextStep == 0)		// Moving Forward
		nextposy++;
	  else if (nextStep == 1)	// Turning Right
	  {
		nextposx--;
		direction = DIR_WEST;
	  }
	  else if (nextStep == 2)	// Moving Backward
	    nextposy--;
	  else if (nextStep == 3)	// Turning Left
	  {
		nextposx++;
		direction = DIR_EAST;
	  }
	  else if (nextStep == 4)
	  {
		direction = DIR_NORTH;
	  }
	}
	else if (direction == 3)	// Facing West
	{
	  if (nextStep == 0)		// Moving Forward
		nextposx--;
	  else if (nextStep == 1)	// Turning Right
	  {
		nextposy--;
		direction = DIR_NORTH;
	  }
	  else if (nextStep == 2)	// Moving Backward
	    nextposx++;
	  else if (nextStep == 3)	// Turning Right
	  {
		nextposy++;
		direction = DIR_SOUTH;
	  }
	  else if (nextStep == 4)
	  {
		direction = DIR_EAST;
	  }
	}
	
	// Validate Cell
	if ((nextposx >= 0 && nextposx <= 14) && (nextposy >= 0 && nextposy <= 7))
	{
	    currposx = nextposx;
	    currposy = nextposy;
	    map[currposx][currposy] = 1;
		
		Serial.print("map[");
		Serial.print(currposx);
		Serial.print("][");
		Serial.print(currposy);
		Serial.println("] = 1");
		Serial1.print("map[");
		Serial1.print(currposx);
		Serial1.print("][");
		Serial1.print(currposy);
		Serial1.println("] = 1");
		
	    return 1;	// Success!
	}
	else
	{
		return 0;	// Failed: Not a valid cell!
		Serial.println("Failed to update map!");
		Serial1.println("Failed to update map!");
	}
		
}

void GridMap::updateDeadEnd()
{
  map[currposx][currposy] = 2;
}

int GridMap::getCurrPosx()
{
  return currposx;
}

int GridMap::getCurrPosy()
{
  return currposy;
}

int GridMap::getRobotDir()
{
  return direction;
}
	
void GridMap::printMap()
{
  int i, j;

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 15; j++)
    {
      Serial.print(map[j][i]);
      Serial.print(" ");
      Serial1.print(map[j][i]);
      Serial1.print(" ");
    }
    Serial.println("");
    Serial1.println("");
  }
}