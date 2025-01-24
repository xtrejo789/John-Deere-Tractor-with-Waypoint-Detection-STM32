/*
 * WayPointNotify.h
 *
 *  Created on: Nov 19, 2024
 *      Author: aavila
 */

#ifndef INC_WAYPOINTNOTIFY_H_
#define INC_WAYPOINTNOTIFY_H_


#include <math.h>
#include <stdbool.h>
#include <stdint.h>


uint8_t isCarAtWaypoint(void);
uint8_t updateNextWaypoint(void);
uint32_t WaypointUpdateState(float wpEstX, float wpEstY);

#endif /* INC_WAYPOINTNOTIFY_H_ */
