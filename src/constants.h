//
// Created by Bassam Sayed on 2017-11-06.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#include <math.h>
// priority levels for costs
#define COLLISION pow(10, 6)
#define DANGER pow(10, 6)
#define REACH_GOAL pow(10, 2)
#define COMFORT pow(10, 1)
#define EFFICIENCY pow(10, 1)

#define DESIRED_BUFFER 20.0  // timesteps
#define PLANNING_HORIZON 1

#define DEBUG true
#define LANE_WIDTH 4
#define MAX_ACCEL 10.0
#define NUM_OF_LANES 3
#define LANE_CHANGE_DURATION 2.2
#define SAFE_DISTANCE_S 22.0
#define DELTA_T 0.02
#define TARGET_SPEED 49.5
#define DANGER_DISTANCE_S 3.0


#define MAXFLOAT    0x1.fffffep+127f /* max value for floats */
#define	INTMAX		2147483647	/* max value for an int */
#define	INTMIN		-2147483647	/* min value for an int */

#endif //PATH_PLANNING_CONSTANTS_H
