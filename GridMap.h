/* GridMap Header File
 * 
 * @author: Navid Fattahi 	<navid.fattahi (at) alumni.ubc.ca>
 *			Alireza Afshar
 * 
 * August 2013
 * this code is public domain, enjoy!
 *
 */

#ifndef _GRIDMAP_H_
#define _GRIDMAP_H_

class GridMap
{
  public:
    GridMap();
    void printMap();
	void updateDeadEnd();
    int  updateMap(int nextStep);
	int  getCurrPosx();
	int  getCurrPosy();
	int  getRobotDir();
	
  private:
    int map[15][8];
/*
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 ]
*/
    int direction;
    int currposx;
    int currposy;
};

#endif
