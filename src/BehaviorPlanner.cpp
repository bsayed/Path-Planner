//
// Created by Bassam Sayed on 2017-11-06.
//

#include "BehaviorPlanner.h"
#include "constants.h"

BehaviorPlanner::BehaviorPlanner(vector<vector<double>> sensor_fusion, Vehicle ego_, vector<int> road_data) : ego(
        ego_), road_data(road_data) {


  for (int i = 0; i < sensor_fusion.size(); i++) {
    int d = static_cast<int>(sensor_fusion[i][6]);
    int lane = d % LANE_WIDTH;
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    int speed = static_cast<int>(sqrt(vx * vx + vy * vy));
    int s = static_cast<int>(sensor_fusion[i][5]);

    Vehicle v = Vehicle(lane, s, speed, 0);
    v.configure(this->road_data);
    this->vehicles.insert(std::pair<int, Vehicle>(i, v));
  }

  this->ego.configure(this->road_data);
  this->vehicles.insert(std::pair<int, Vehicle>(ego_key, ego));
}


BehaviorPlanner::~BehaviorPlanner() {}

void BehaviorPlanner::advance(vector<string> states) {

  map<int, vector<vector<int> > > predictions;
  auto it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;
    vector<vector<int> > preds = it->second.generate_predictions(10);
    predictions[v_id] = preds;
    it++;
  }

  it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;
    if (v_id == ego_key) {
      it->second.update_state(predictions, states);
      it->second.realize_state(predictions);
    }
    it->second.increment(1);

    it++;
  }

}

string BehaviorPlanner::get_next_state() {
  return this->vehicles.at(ego_key).state;
}
