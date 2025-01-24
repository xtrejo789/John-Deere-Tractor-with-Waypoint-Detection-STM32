/*
 * WayPointNotify.c
 *
 *  Created on: Nov 19, 2024
 *      Author: aavila
 */


#include <WayPointNotify.h>

// Waypoints and current index
#define NUM_WAYPOINTS 4
#define WAYPOINT_RADIUS 5

float waypoints[NUM_WAYPOINTS][2] = {
    {10, 10},
	{50, 30},
	{90, 60},
	{150, 120}
};

int currentWaypointIndex = 0;
enum State {START, GOTOwp, ATwp, OFF};   //define speed constant values
static enum State CurrState = START; // initialize state machine to START
static float wpCurrX;
static float wpCurrY;
static float wpNextX;
static float wpNextY;

static uint32_t stateId = 0;

uint32_t WaypointUpdateState(float wpEstX, float wpEstY) { // run periodically from main loop
switch (CurrState) {
   case START:   /* state 0 */
	  currentWaypointIndex = 0;
      wpCurrX = waypoints[currentWaypointIndex][0];   // Setup Initial WayPoints
      wpCurrY = waypoints[currentWaypointIndex][1];
	  currentWaypointIndex = 1;
      wpNextX = waypoints[currentWaypointIndex][0];   // Setup Initial WayPoints
      wpNextY = waypoints[currentWaypointIndex][1];
      CurrState = GOTOwp;
      stateId = 1;

      break;

   case GOTOwp:   /* state 1 */
	  wpCurrX = wpEstX;
	  wpCurrY = wpEstY;
	  if (isCarAtWaypoint()){   /* distance (next - current) way points*/
         CurrState = ATwp;
         stateId = 2;
	  }
      else {
         CurrState = GOTOwp;
      }
      break;

   case ATwp:  /* state 2 */
      if(updateNextWaypoint()) {
         CurrState = GOTOwp;
         stateId = 1;
      }
      else {
         CurrState = OFF;
         stateId = 3;
      }
      break;

   case OFF:   /* state 3*/
      stateId = 3;
      CurrState = OFF;
      break;

   default:   /* state 4 error: invalid state */
      stateId = 4;
      CurrState = OFF;
      break;
    }

   return(stateId);
}



// Function to check if the car is near a way point
uint8_t isCarAtWaypoint() {
    float distance = sqrt(pow(wpNextX - wpCurrX, 2) + pow(wpNextY - wpCurrY, 2));
    return (distance <= WAYPOINT_RADIUS);
}

// Function to update the target way point
uint8_t updateNextWaypoint() {
	uint8_t retval = 0;
	currentWaypointIndex = currentWaypointIndex + 1;
    if (currentWaypointIndex < NUM_WAYPOINTS) {
        wpNextX = waypoints[currentWaypointIndex][0];
        wpNextY = waypoints[currentWaypointIndex][1];
        retval = 1; /* process next way point*/
        }
    else{
      retval = 0; /* no way points left */
    }
    return(retval);
}


