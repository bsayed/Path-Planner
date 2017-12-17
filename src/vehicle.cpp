//
// Created by Bassam Sayed on 2017-11-05.
//

#include "vehicle.h"
#include "constants.h"


using namespace std;

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  state = "CS";
  max_acceleration = -1;

}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(map<int, vector<vector<int> > > predictions, vector<string> state) {
  /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
  //state = "KL"; // this is an example of how you change state.
  this->state = get_next_state(predictions, state);

}

void Vehicle::configure(vector<int> road_data) {
  /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}

string Vehicle::display() {

  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(int dt) {

  this->s += this->v * dt;
  this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

  /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
  int s = this->s + this->v * t + this->a * t * t / 2;
  int v = this->v + this->a * t;
  return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

  /*
    Simple collision detection.
    */
  vector<int> check1 = state_at(at_time);
  vector<int> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps + 1; t++) {
    if (collides_with(other, t)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(map<int, vector<vector<int> > > predictions) {

  /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
  string state = this->state;
  if (state.compare("CS") == 0) {
    realize_constant_speed();
  } else if (state.compare("KL") == 0) {
    realize_keep_lane(predictions);
  } else if (state.compare("LCL") == 0) {
    realize_lane_change(predictions, "L");
  } else if (state.compare("LCR") == 0) {
    realize_lane_change(predictions, "R");
  } else if (state.compare("PLCL") == 0) {
    realize_prep_lane_change(predictions, "L");
  } else if (state.compare("PLCR") == 0) {
    realize_prep_lane_change(predictions, "R");
  }

}

void Vehicle::realize_constant_speed() {
  a = 0;
}

int Vehicle::_max_accel_for_lane(map<int, vector<vector<int> > > predictions, int lane, int s) {

  int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > in_front;
  while (it != predictions.end()) {

    int v_id = it->first;

    vector<vector<int> > v = it->second;

    if ((v[0][0] == lane) && (v[0][1] > s)) {
      in_front.push_back(v);

    }
    it++;
  }

  if (in_front.size() > 0) {
    int min_s = INTMAX;
    vector<vector<int>> leading = {};
    for (int i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0][1] - s) < min_s) {
        min_s = (in_front[i][0][1] - s);
        leading = in_front[i];
      }
    }

    int next_pos = leading[1][1];
    int my_next = s + this->v;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }

  return max_acc;

}

void Vehicle::realize_keep_lane(map<int, vector<vector<int> > > predictions) {
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector<vector<int> > > predictions, string direction) {
  int delta = 1;
  if (direction.compare("L") == 0) {
    delta = -1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<int> > > predictions, string direction) {
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  int lane = this->lane + delta;

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > at_behind;
  while (it != predictions.end()) {
    int v_id = it->first;
    vector<vector<int> > v = it->second;

    if ((v[0][0] == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);

    }
    it++;
  }
  if (at_behind.size() > 0) {

    int max_s = -1000;
    vector<vector<int> > nearest_behind = {};
    for (int i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }
    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    int delta_v = this->v - target_vel;
    int delta_s = this->s - nearest_behind[0][1];
    if (delta_v != 0) {

      int time = -2 * delta_s / delta_v;
      int a;
      if (time == 0) {
        a = this->a;
      } else {
        a = delta_v / time;
      }
      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }
      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }
      this->a = a;
    } else {
      int my_min_acc = max(-this->max_acceleration, -delta_s);
      this->a = my_min_acc;
    }

  }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

  vector<vector<int> > predictions;
  for (int i = 0; i < horizon; i++) {
    vector<int> check1 = state_at(i);
    vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;

}

void Vehicle::restore_state(std::tuple<int, int, int, int, string> snapshot) {

  this->lane = std::get<0>(snapshot);
  this->s = std::get<1>(snapshot);
  this->v = std::get<2>(snapshot);
  this->a = std::get<3>(snapshot);
  this->state = std::get<4>(snapshot);

}

std::tuple<int, int, int, int, string> Vehicle::get_snapshot() {
  return std::make_tuple(this->lane, this->s, this->v, this->a, this->state);
}

vector<tuple<int, int, int, int, string>>
Vehicle::trajectory_for_state(string state, map<int, vector<vector<int> > > predictions, int horizon) {
  // remember current state
  auto snapshot = this->get_snapshot();


  // pretend to be in new proposed state
  this->state = state;
  vector<tuple<int, int, int, int, string>> trajectory = {snapshot};

  for (int i = 0; i < horizon; i++) {
    this->restore_state(snapshot);
    this->state = state;
    this->realize_state(predictions);
    assert(0 <= this->lane < this->lanes_available);
    this->increment();
    trajectory.push_back(this->get_snapshot());

    for (auto it = predictions.begin(); it != predictions.end(); ++it) {
      // need to remove first prediction for each vehicle.
      it->second.erase(it->second.begin());
    }
  }
  // restore state from snapshot
  this->restore_state(snapshot);

  return trajectory;
}

string Vehicle::get_next_state(map<int, vector<vector<int> > > predictions, vector<string> states) {
  // states = ["KeepLane", "LaneChaneLeft", "LaneChangeRight"]
//  vector<string> states = {"KL", "LCL", "LCR"};
//  if (this->lane == 0)
//    states.erase(states.begin() + 1);
//  if (this->lane == (this->lanes_available - 1))
//    states.erase(states.begin() + 2);

  std::pair<string, double> min_cost = std::make_pair("", MAXFLOAT);

  for (string state: states) {
    auto predictions_copy = predictions;
    auto trajectory = this->trajectory_for_state(state, predictions_copy);
    double cost = calculate_cost((*this), trajectory, predictions);
    if (cost < min_cost.second)
      min_cost = make_pair(state, cost);
  }

  cout<<"Decision is to "<< min_cost.first<<endl;
  return min_cost.first;
}

Vehicle::TrajectoryData Vehicle::get_helper_data(Vehicle vehicle, vector<tuple<int, int, int, int, string>> t,
                                                 map<int, vector<vector<int> > > predictions) {
  tuple<int, int, int, int, string> current_snapshot = t[0];
  tuple<int, int, int, int, string> first = t[1];
  tuple<int, int, int, int, string> last = t[t.size()-1];
  int y = std::get<1>(last);
  int end_distance_to_goal = vehicle.goal_s - std::get<1>(last);
  int x = std::get<0>(last);
  int end_lanes_from_goal = abs(vehicle.goal_lane - std::get<0>(last));
  double dt = (double) t.size();
  int proposed_lane = std::get<0>(first);
  double avg_speed = (std::get<1>(last) - std::get<1>(current_snapshot)) / dt;


  // initialize a bunch of variables
  vector<int> accels;
  int closest_approach = 999999;
  bool collides = false;
  auto last_snap = t[0];
  auto filtered = filter_predictions_by_lane(predictions, proposed_lane);
  int collides_at = 0;

  for (int i = 1; i <= PLANNING_HORIZON; i++) {
    int lane, s, v, a;
    std::tie(lane, s, v, a, std::ignore) = t[i];
    auto snapshot = t[i];
    accels.push_back(a);
    for (auto it = filtered.begin(); it != filtered.end(); it++) {
      auto state = it->second[i];
      auto last_state = it->second[i - 1];
      bool vehicle_collides = check_collision(snapshot, last_state[1], state[1]);
      if (vehicle_collides)
        collides_at = i;
      int dist = abs(state[1] - s);
      if (dist < closest_approach)
        closest_approach = dist;
    }
  }
  int max_accel = INTMIN;
  auto get_max = [&max_accel](const int &a) { if (abs(a) > max_accel) max_accel = abs(a); };
  std::for_each(accels.begin(), accels.end(), get_max);

  vector<int> rms_accels;
  for (auto acc : accels)
    rms_accels.push_back(acc * acc);

  int num_accels = rms_accels.size();
  double rms_acceleration = ((double) std::accumulate(rms_accels.begin(), rms_accels.end(), 0)) / (double) num_accels;

  TrajectoryData trajectoryData;
  trajectoryData.proposed_lane = proposed_lane;
  trajectoryData.avg_speed = avg_speed;
  trajectoryData.closest_approach = closest_approach;
  trajectoryData.max_accel = max_accel;
  trajectoryData.rms_acceleration = rms_acceleration;
  trajectoryData.end_distance_to_goal = end_distance_to_goal;
  trajectoryData.end_lanes_from_goal = end_lanes_from_goal;
  trajectoryData.collides_at = collides_at;

  return trajectoryData;
}

map<int, vector<vector<int> > >
Vehicle::filter_predictions_by_lane(map<int, vector<vector<int>>> predictions, int lane) {
  map<int, vector<vector<int>>> filtered;

  for (auto it = predictions.begin(); it != predictions.end(); it++) {
    if (it->second[0][0] == lane && it->first != -1) {
      filtered[it->first] = it->second;
    }
  }
  return filtered;
}

bool Vehicle::check_collision(std::tuple<int, int, int, int, string> snapshot, int s_previous, int s_now) {
  int s = std::get<1>(snapshot);
  int v = std::get<2>(snapshot);
  int v_target = s_now - s_previous;
  if (s_previous < s) {
    return s_now >= s;
  }

  if (s_previous > s) {
    return s_now <= s;
  }

  if (s_previous == s) {
    return v_target <= v;
  }

  throw "We Should not reach hear.";

}

double Vehicle::calculate_cost(Vehicle vehicle, vector<tuple<int, int, int, int, string>> trajectory,
                               map<int, vector<vector<int>>> predictions, bool verbose) {

  TrajectoryData trajectoryData = get_helper_data(vehicle, trajectory, predictions);
  double cost = 0.0;

  cost += distance_from_goal_lane(trajectoryData);
  cost += collision_cost(trajectoryData);
//  cost += buffer_cost(trajectoryData);
  cost += inefficiency_cost(vehicle, trajectoryData);
  cost += change_lane_cost(trajectory, trajectoryData);

  return cost;
}

double Vehicle::distance_from_goal_lane(Vehicle::TrajectoryData trajectoryData) {
  int distance = std::max(abs(trajectoryData.end_distance_to_goal), 1);
  double time_to_goal = (float) distance / (float) trajectoryData.avg_speed;
  int lanes = trajectoryData.end_lanes_from_goal;
  double multiplier = ((float) 5 * lanes) / time_to_goal;
  double cost = multiplier * REACH_GOAL;
  if(DEBUG)
    cout<<"Cost of distance_from_goal_lane: "<<cost<<endl;

  return cost;
}

double Vehicle::collision_cost(Vehicle::TrajectoryData trajectoryData) {
  double cost = 0.0;
  if (trajectoryData.collides_at != 0) {
    double exponent = (double) trajectoryData.collides_at * trajectoryData.collides_at;
    double multiplier = exp(-exponent);
    cost = multiplier * COLLISION;
  }
  if(DEBUG)
    cout<<"Cost of Collision: "<<cost<<endl;
  return cost;
}

double Vehicle::buffer_cost(Vehicle::TrajectoryData trajectoryData) {
  int closest = trajectoryData.closest_approach;
  if (closest == 0)
    return 10 * DANGER;

  double timesteps_away = closest / trajectoryData.avg_speed;
  if (timesteps_away > DESIRED_BUFFER)
    return 0.0;

  double multiplier = 1.0 - (timesteps_away / DESIRED_BUFFER) * (timesteps_away / DESIRED_BUFFER);
  double cost = multiplier * DANGER;

  if(DEBUG)
    cout<<"Cost of buffer: "<<cost<<endl;
  return cost;
}

double Vehicle::inefficiency_cost(Vehicle vehicle, Vehicle::TrajectoryData trajectoryData) {
  double speed = trajectoryData.avg_speed;
  double target_speed = vehicle.target_speed;
  double diff = target_speed - speed;
  double pct = ((float)diff) / target_speed;
  double multiplier = pct * pct;
  double cost = multiplier * EFFICIENCY;
  if(DEBUG)
    cout<<"Cost of inefficiency: "<<cost<<endl;
  return cost;

}

double Vehicle::change_lane_cost(vector<tuple<int, int, int, int, string>> trajectory,
                                 Vehicle::TrajectoryData trajectoryData) {
  /*
  Penalizes lane changes AWAY from the goal lane and rewards
  lane changes TOWARDS the goal lane.
  */
  int proposed_lanes = trajectoryData.end_lanes_from_goal;
  int cur_lanes = std::get<0>(trajectory[0]);
  double cost = 0.0;
  if (proposed_lanes > cur_lanes)
    cost = COMFORT;
  if (proposed_lanes < cur_lanes)
    cost = -COMFORT;
  if (cost != 0)
    cout<<"cost for lane change is "<<cost<<endl;
  return cost;
}
