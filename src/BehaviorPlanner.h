//
// Created by Bassam Sayed on 2017-11-06.
//

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H

#include <vector>
#include <map>
#include "vehicle.h"

using namespace std;

class BehaviorPlanner {

  map<int, vector<vector < int>>> predictions;
  map<int, Vehicle> vehicles;
  Vehicle ego;
  int ego_key = -1;
  vector<int> road_data;

public:
  /**
  * Constructor
  */
  BehaviorPlanner(vector <vector<double>> sensor_fusion, Vehicle ego_, vector<int> road_data);

  /**
  * Destructor
  */
  virtual ~BehaviorPlanner();

  void advance(vector<string> states);
  string get_next_state();

};


#endif //PATH_PLANNING_BEHAVIORPLANNER_H
