/**
 * @file agvs.cpp
 * @brief The implement of CL-CBS
 * @date 2024/09/03
 *
 */

#include <fstream>
#include <iostream>
#include <chrono>

#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>


#include "agvs_plannerv2/cl_cbs.hpp"
#include "agvs_plannerv2/agvs.hpp"
#include "agvs_plannerv2/environment.hpp"
#include "agvs_plannerv2/timer.hpp"

using libMultiRobotPlanning::CL_CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;

// calculate agent collision more precisely BUT need LONGER time
// #define PRCISE_COLLISION

struct Location {
  Location(double x, double y) : x(x), y(y) {}
  double x;
  double y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct State {
  State(double x, double y, double yaw, int time = 0)
      : time(time), x(x), y(y), yaw(yaw) {
    rot.resize(2, 2);
    rot(0, 0) = cos(-this->yaw);
    rot(0, 1) = -sin(-this->yaw);
    rot(1, 0) = sin(-this->yaw);
    rot(1, 1) = cos(-this->yaw);
#ifdef PRCISE_COLLISION
    corner1 = Point(
        this->x -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LB) - this->yaw),
        this->y -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LB) - this->yaw));
    corner2 = Point(
        this->x -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LB) + this->yaw),
        this->y +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LB) + this->yaw));
    corner3 = Point(
        this->x +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LF) - this->yaw),
        this->y +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LF) - this->yaw));
    corner4 = Point(
        this->x +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LF) + this->yaw),
        this->y -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LF) + this->yaw));
#endif
  }

  State() = default;

  bool operator==(const State& s) const {
    return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
  }

  bool agentCollision(const State& other) const {
#ifndef PRCISE_COLLISION
    if (pow(this->x - other.x, 2) + pow(this->y - other.y, 2) <
        pow(2 * Constants::LF, 2) + pow(Constants::carWidth, 2))
      return true;
    return false;
#else
    std::vector<Segment> rectangle1{Segment(this->corner1, this->corner2),
                                    Segment(this->corner2, this->corner3),
                                    Segment(this->corner3, this->corner4),
                                    Segment(this->corner4, this->corner1)};
    std::vector<Segment> rectangle2{Segment(other.corner1, other.corner2),
                                    Segment(other.corner2, other.corner3),
                                    Segment(other.corner3, other.corner4),
                                    Segment(other.corner4, other.corner1)};
    for (auto seg1 = rectangle1.begin(); seg1 != rectangle1.end(); seg1++)
      for (auto seg2 = rectangle2.begin(); seg2 != rectangle2.end(); seg2++) {
        if (boost::geometry::intersects(*seg1, *seg2)) return true;
      }
    return false;
#endif
  }

  bool obsCollision(const Location& obstacle) const {
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x - this->x;
    obs(0, 1) = obstacle.y - this->y;

    auto rotated_obs = boost::numeric::ublas::prod(obs, rot);
    if (rotated_obs(0, 0) > -Constants::LB - Constants::obsRadius &&
        rotated_obs(0, 0) < Constants::LF + Constants::obsRadius &&
        rotated_obs(0, 1) > -Constants::carWidth / 2.0 - Constants::obsRadius &&
        rotated_obs(0, 1) < Constants::carWidth / 2.0 + Constants::obsRadius)
      return true;
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@" << s.time;
  }

  int time;
  double x;
  double y;
  double yaw;

 private:
  boost::numeric::ublas::matrix<double> rot;
  Point corner1, corner2, corner3, corner4;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    return seed;
  }
};
}  // namespace std

using Action = int;  // int<7 int ==6 wait

struct Conflict {
  int time;
  size_t agent1;
  size_t agent2;

  State s1;
  State s2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    os << c.time << ": Collision [ " << c.agent1 << c.s1 << " , " << c.agent2
       << c.s2 << " ]";
    return os;
  }
};

struct Constraint {
  Constraint(int time, State s, size_t agentid)
      : time(time), s(s), agentid(agentid) {}
  Constraint() = default;
  int time;
  State s;
  size_t agentid;

  bool operator<(const Constraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) <
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  bool operator==(const Constraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) ==
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraint& c) {
    return os << "Constraint[" << c.time << "," << c.s << "from " << c.agentid
              << "]";
  }

  bool satisfyConstraint(const State& state) const {
    if (state.time < this->time ||
        state.time > this->time + Constants::constraintWaitTime)
      return true;
    return !this->s.agentCollision(state);
  }
};

namespace std {
template <>
struct hash<Constraint> {
  size_t operator()(const Constraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.s.x);
    boost::hash_combine(seed, s.s.y);
    boost::hash_combine(seed, s.s.yaw);
    boost::hash_combine(seed, s.agentid);
    return seed;
  }
};
}  // namespace std

// FIXME: modidy data struct, it's not the best option
struct Constraints {
  std::unordered_set<Constraint> constraints;

  void add(const Constraints& other) {
    constraints.insert(other.constraints.begin(), other.constraints.end());
  }

  bool overlap(const Constraints& other) {
    for (const auto& c : constraints) {
      if (other.constraints.count(c) > 0) return true;
    }
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& cs) {
    for (const auto& c : cs.constraints) {
      os << c << std::endl;
    }
    return os;
  }
};

//@brief 读取该cpp文件同目录下config.yaml配置文件 
void readAgentConfig() {
  YAML::Node car_config;
  std::string test(__FILE__); //获取当前cpp文件路径
  boost::replace_all(test, "src/agvs.cpp", "config/planner_param.yaml"); //将路径中的cpp文件名替换为config.yaml TODO：修改文件名
  try {
    car_config = YAML::LoadFile(test.c_str());
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[33mWARNING: Failed to load agent config file: "
              << test << "\033[0m , Using default params. \n";
  }
  // int car_r = car_config["r"].as<int>();
  Constants::r = car_config["r"].as<double>();
  Constants::deltat = car_config["deltat"].as<double>();
  Constants::penaltyTurning = car_config["penaltyTurning"].as<double>();
  Constants::penaltyReversing = car_config["penaltyReversing"].as<double>();
  Constants::penaltyCOD = car_config["penaltyCOD"].as<double>();
  // map resolution
  Constants::mapResolution = car_config["mapResolution"].as<double>();
  // change to set calcIndex resolution
  Constants::xyResolution = Constants::r * Constants::deltat;
  Constants::yawResolution = Constants::deltat;

  Constants::carWidth = car_config["carWidth"].as<double>();
  Constants::LF = car_config["LF"].as<double>();
  Constants::LB = car_config["LB"].as<double>();
  // obstacle default radius
  Constants::obsRadius = car_config["obsRadius"].as<double>();
  // least time to wait for constraint
  Constants::constraintWaitTime = car_config["constraintWaitTime"].as<double>();

  Constants::dx = {Constants::r * Constants::deltat,
                   Constants::r * sin(Constants::deltat),
                   Constants::r * sin(Constants::deltat),
                   -Constants::r * Constants::deltat,
                   -Constants::r * sin(Constants::deltat),
                   -Constants::r * sin(Constants::deltat)};
  Constants::dy = {0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat)),
                   0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat))};
  Constants::dyaw = {0, Constants::deltat,  -Constants::deltat,
                     0, -Constants::deltat, Constants::deltat};
}



// namespace agvs_ns
// {
//   std::vector<std::vector<xyyaw>> agvs::agvs_route_plan(const std::vector<xyyaw>& positions, const std::vector<std::pair<xyyaw, bool>>& targets)
//   {
//     int count = targets.size();
//     if(count == 0)
//     {
//       std::cout << "\033[31m[agv] Please set targets first!!\033[0m" << std::endl;
//       return std::vector<std::vector<xyyaw>>();
//     }

//     readAgentConfig();
//     int dimx = 20;
//     int dimy = 30;//TODO:无人车地图大小
//     std::unordered_set<Location> obstacles;
//     std::multimap<int, State> dynamic_obstacles;
//     std::vector<State> goals;
//     std::vector<State> startStates;
//     State goal, startState;
//     int batchSize = positions.size();
    
//     for(int i = 0; i < count; i++)
//     {
//       std::pair<xyyaw, bool> target = targets[i];
//       xyyaw position = positions[i];
//       startState = State(position.x - 180.0, position.y - 420.0, position.yaw);
//       if(target.second)
//       {
//         goal = State(target.first.x - 180.0, target.first.y - 420.0, target.first.yaw);
//       }
//       else
//       {
//         goal = State(position.x - 180.0 + 1e-5, position.y - 420.0, position.yaw);
//       }
//       goals.emplace_back(goal);
//       startStates.emplace_back(startState);
//     }

//     std::cout << "Calculating Solution...\n";
//     double timer = 0;
//     bool success = false;
//     std::vector<PlanResult<State, Action, double>> solution;
//     for (size_t iter = 0; iter < (double)goals.size() / batchSize; iter++) {
//       size_t first = iter * batchSize;
//       size_t last = first + batchSize;
//       if (last >= goals.size()) last = goals.size();
//       std::vector<State> m_goals(goals.begin() + first, goals.begin() + last);
//       // for(size_t i =0; i < m_goals.size(); ++i)
//       // {
//       //   ROS_INFO("goal_%d: %f, %f", int(i), m_goals[i].x, m_goals[i].y);
//       // }
//       std::vector<State> m_starts(startStates.begin() + first,
//                                   startStates.begin() + last);

//       Environment<Location, State, Action, double, Conflict, Constraint,
//                   Constraints>
//           mapf(dimx, dimy, obstacles, dynamic_obstacles, m_goals);
//       if (!mapf.startAndGoalValid(m_starts, iter, batchSize)) {
//         success = false;
//         break;
//       }
//       for (auto goal = goals.begin() + last; goal != goals.end(); goal++) {
//         dynamic_obstacles.insert(
//             std::pair<int, State>(-1, State(goal->x, goal->y, goal->yaw)));
//       }//除了当前批次的车，其他的均为动态障碍物
//       CL_CBS<State, Action, double, Conflict, Constraints,
//             Environment<Location, State, Action, double, Conflict, Constraint,
//                         Constraints>>
//           cbsHybrid(mapf);
//       std::vector<PlanResult<State, Action, double>> m_solution;
//       Timer iterTimer;
//       success = cbsHybrid.search(m_starts, m_solution);
//       iterTimer.stop();

//       if (!success) {
//         std::cout << "\033[1m\033[31m No." << iter
//                   << "iter fail to find a solution \033[0m\n";
//         break;
//       } else {
//         solution.insert(solution.end(), m_solution.begin(), m_solution.end());
//         for (size_t a = 0; a < m_solution.size(); ++a) {
//           for (const auto& state : m_solution[a].states)
//             dynamic_obstacles.insert(std::pair<int, State>(
//                 state.first.time,
//                 State(state.first.x, state.first.y, state.first.yaw)));
//           State lastState = m_solution[a].states.back().first;
//           dynamic_obstacles.insert(std::pair<int, State>(
//               -lastState.time, State(lastState.x, lastState.y, lastState.yaw)));
//         }
//         timer += iterTimer.elapsedSeconds();
//         std::cout << "Complete " << iter
//                   << " iter. Runtime:" << iterTimer.elapsedSeconds()
//                   << " Expand high-level nodes:" << mapf.highLevelExpanded()
//                   << " Average Low-level-search time:"
//                   << iterTimer.elapsedSeconds() / mapf.highLevelExpanded() /
//                         m_goals.size()
//                   << std::endl;
//       }
//       dynamic_obstacles.erase(-1);
//     }

//     // std::ofstream out;
//     // out = std::ofstream(outputFile);

//     // if (success) {
//     //   std::cout << "\033[1m\033[32m Successfully find solution! \033[0m\n";

//     //   double makespan = 0, flowtime = 0, cost = 0;
//     //   for (const auto& s : solution) cost += s.cost;

//     //   for (size_t a = 0; a < solution.size(); ++a) {
//     //     // calculate makespan
//     //     double current_makespan = 0;
//     //     for (size_t i = 0; i < solution[a].actions.size(); ++i) {
//     //       // some action cost have penalty coefficient

//     //       if (solution[a].actions[i].second < Constants::dx[0])
//     //         current_makespan += solution[a].actions[i].second;
//     //       else if (solution[a].actions[i].first % 3 == 0)
//     //         current_makespan += Constants::dx[0];
//     //       else
//     //         current_makespan += Constants::r * Constants::deltat;
//     //     }
//     //     flowtime += current_makespan;
//     //     if (current_makespan > makespan) makespan = current_makespan;
//     //   }
//       // std::cout << " Runtime: " << timer << std::endl
//       //           << " Makespan:" << makespan << std::endl
//       //           << " Flowtime:" << flowtime << std::endl
//       //           << " cost:" << cost << std::endl;
//       // // output to file
//       // out << "statistics:" << std::endl;
//       // out << "  cost: " << cost << std::endl;
//       // out << "  makespan: " << makespan << std::endl;
//       // out << "  flowtime: " << flowtime << std::endl;
//       // out << "  runtime: " << timer << std::endl;
//       // out << "schedule:" << std::endl;
//       // for (size_t a = 0; a < solution.size(); ++a) {
//       //   out << "  agent" << a << ":" << std::endl;
//       //   for (const auto& state : solution[a].states) {
//       //     out << "    - x: " << state.first.x << std::endl
//       //         << "      y: " << state.first.y << std::endl
//       //         << "      yaw: " << state.first.yaw << std::endl
//       //         << "      t: " << state.first.time << std::endl;
//       //   }
//       // }
//     // } else {
//     //   std::cout << "\033[1m\033[31m Fail to find paths \033[0m\n";
//     // }
//     //--------------------------------------------------------
//     std::vector<std::vector<xyyaw>> resultss;
//     std::vector<xyyaw> results;
//         for (size_t a = 0; a < solution.size(); ++a) 
//         {
//           results.clear();
//           for (const auto& state : solution[a].states) 
//           {
//             xyyaw result = xyyaw(state.first.x+180.0, state.first.y+420.0, state.first.yaw);
//             results.push_back(result);
//           }
//           resultss.push_back(results);
//         }
//     return resultss;
//     //----------------------------------------------------原内容全部注释掉，没有进行修改，这一段是自己加的
//   }
// }



/**
 *                                                   CAR_INIT
 *                                                      |
 *                                                      |
 *                                                      v
 *                  CAR_IN_BUFFER0    <--------   CAR_IN_FLIGHT   <----------   CAR_IN_BUFFER3
 *                         |                         |  |  ^                           ^
 *                         |                         |  |  |                           |
 *                         V                         |  |  |                           |
 *                  CAR_IN_BUFFER1    <--------------   |   -----------------   CAR_IN_BUFFER2                     
 *                         |                            |                             ^
 *                         |                            |                             |
 *                         |                            V                             |
 *                          ------------------->   CAR_IN_LOAD   --------------------- 
 *                                
 *                               
 *                              
 */
namespace agvs_ns{

void agvs::FSM_loop(const ros::TimerEvent& )
{   
    auto start = std::chrono::steady_clock::now();
    //判断车是否全部初始化成功，初始化成功才开始主循环
    if(!this->is_init_)
    {
        if(!std::all_of(this->cars_work_state_.begin(), this->cars_work_state_.end(), [](car_work_state state){return state == car_work_state::CAR_READY;}))
        {
            ROS_WARN("[agvs]: Not all cars are ready, env init failed");
            return;
        }
        std::cout <<"[agvs]: env init succuss!! Start to run agvs loop!!!!!!!!!!!!!!" << std::endl;
        this->is_init_ = true;
    }

    std::cout <<"\033[32m" << "loop:" << ++this->main_loop_count_ <<std::endl;
    std::cout << "---------------------------------------------------------------------------------------------" << "\033[0m" << std::endl;

    for (size_t i = 0; i < this->agvs_.size(); ++i)
    {
        std::cout <<"  "<<"\033[36m" << "agv_" << i <<":" << "\033[0m" << std::endl;
        //检查无人车是否错误状态，并打印无人车当前工作状态

        //读取全局信息-----------------------------------
        std::vector<user_pkg::DronePhysicalStatus> drones;
        std::vector<agv> agvs;
        std::vector<user_pkg::BillStatus> bills;
        std::vector<car_work_state> cars_work_state;
        while(this->info_lock_.test_and_set());
        agvs.assign(this->agvs_.begin(), this->agvs_.end());
        drones.assign(this->drones_.begin(), this->drones_.end());
        bills.assign(this->bills_.begin(), this->bills_.end());
        this->info_lock_.clear();
        //----------------------------------------------
        std::vector<xyyaw> route;

        switch (this->cars_work_state_[i])
        {
        case car_work_state::CAR_ERROR:
        {
            std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_ERROR state" << "\033[0m" << std::endl;
            return;
            break;
        } 
        case car_work_state::CAR_READY:
        {
            std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_READY state" << "\033[0m" << std::endl;
            break;
        }
        case car_work_state::CAR_RUNNING:
        {
            std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_RUNNING state" << "\033[0m" << std::endl;
            break;
        }
        case car_work_state::CAR_UNKOWN:
        {
            std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_UNKOWN state" << "\033[0m" << std::endl;
            break;
        }
        }
        //正式开始状态机
        switch (this->cars_state_[i])
        {
            case car_state::CAR_INIT:
            {   
                std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_INIT state" << "\033[0m" << std::endl;

                if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.init_point[i], 0.5) && this->cars_work_state_[i] == CAR_READY)
                {
                    this->set_state(car_state::CAR_IN_FLIGHT, i);
                    #ifdef DEBUG
                    ROS_INFO("[Transform]: Car %s from CAR_INIT to CAR_IN_FLIGHT", agvs[i].get_sn().c_str());
                    #endif
                }

                if(this->cars_work_state_[i] == car_work_state::CAR_READY && !this->is_init_aaaa[i])
                {
                    route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                    route.push_back(this->key_points_.init_point[i]);
                    this->pub_traj(route, i);
                    this->is_init_aaaa[i] = true;
                    ros::Duration(0.1).sleep();
                    // ROS_INFO("agv_%d's target is %f,%f", int(i),this->load_position_.x, this->load_position_.y);
                }

                break;
            }
            case car_state::CAR_IN_BUFFER_0:
            {
                std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_IN_BUFFER_0 state" << "\033[0m" << std::endl;
                if(i == 2) //只有小车2和小车5才能进入该状态
                {
                    if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.5), this->key_points_.left_buffer[0], 0.5) && this->cars_work_state_[i] == car_work_state::CAR_READY)//判断agv是否真的到达buffer_0
                    {
                        if(this->key_points_.is_occupyed_left_buffer[1] == -1)//判断buffer_1是否被占据
                        {
                            route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                            route.push_back(this->key_points_.left_buffer[1]);
                            this->pub_traj(route, i);

                            this->key_points_.is_occupyed_left_buffer[0] = -1;
                            this->key_points_.is_occupyed_left_buffer[1] = i;
                            this->key_points_.in_which_buffer[i] = 1;
                            set_state(car_state::CAR_IN_BUFFER_1, i);
                            #ifdef DEBUG
                            ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_0 to CAR_IN_BUFFER_1", agvs[i].get_sn().c_str());
                            #endif
                        }
                        else//buffer_1被占据
                        {
                            ROS_INFO("[CAR_IN_BUFFER_0]: Car %s can not going to BUFFER_1 occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->key_points_.is_occupyed_left_buffer[1]].get_sn().c_str());
                            break;
                        }
                    }
                    else//没有真正到达buffer_0
                    {
                        std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_BUFFER_0 state" << "\033[0m" << std::endl;
                        break;
                    }
                }
                else if(i == 5)
                {
                    if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.right_buffer[0], 0.5) && this->cars_work_state_[i] == car_work_state::CAR_READY)//判断agv是否真的到达buffer_0
                    {
                        if(this->key_points_.is_occupyed_right_buffer[1] == -1)//判断buffer_1是否被占据
                        {
                            route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                            route.push_back(this->key_points_.right_buffer[1]);
                            this->pub_traj(route, i);

                            this->key_points_.is_occupyed_right_buffer[0] = -1;
                            this->key_points_.is_occupyed_right_buffer[1] = i;
                            this->key_points_.in_which_buffer[i] = 1;
                            set_state(car_state::CAR_IN_BUFFER_1, i);
                            #ifdef DEBUG
                            ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_0 to CAR_IN_BUFFER_1", agvs[i].get_sn().c_str());
                            #endif
                        }
                        else//buffer_1被占据
                        {
                            ROS_INFO("[CAR_IN_BUFFER_0]: Car %s can not going to BUFFER_1 occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->key_points_.is_occupyed_right_buffer[1]].get_sn().c_str());
                            break;
                        }
                    }
                    else//没有真正到达buffer_0
                    {
                        std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_BUFFER_0 state" << "\033[0m" << std::endl;
                        break;
                    }
                }
                break;
            }
            case car_state::CAR_IN_BUFFER_1:
            {
                std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_IN_BUFFER_1 state" << "\033[0m" << std::endl;

                if(i == 1 || i == 2)//左边的buffer_1
                {
                    if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.left_buffer[1], 0.5) && this->cars_work_state_[i] == car_work_state::CAR_READY)//判断agv是否真的到达buffer_1
                    {
                        if(this->index_to_load_ == -1)//判断是否没有车在上货区
                        {
                            route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                            route.push_back(this->key_points_.load_position);
                            this->pub_traj(route, i);

                            this->key_points_.is_occupyed_left_buffer[1] = -1;
                            this->index_to_load_ = i;
                            this->key_points_.in_which_buffer[i] = -1;
                            set_state(car_state::CAR_IN_LOAD, i);
                            #ifdef DEBUG
                            ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_1 to CAR_IN_LOAD", agvs[i].get_sn().c_str());
                            #endif
                        }
                        else//上货区被占据
                        {
                            ROS_INFO("[CAR_IN_BUFFER_1]: Car %s can not going to LOADING POSITION occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->index_to_load_].get_sn().c_str());
                            break;
                        }
                    }
                    else//没有真正到达buffer_1
                    {
                        std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_BUFFER_1 state" << "\033[0m" << std::endl;
                        break;
                    }
                }
                else if(i == 4 || i == 5)//右边的buffer_1
                {
                    if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.right_buffer[1], 0.5) && this->cars_work_state_[i] == car_work_state::CAR_READY)//判断agv是否真的到达buffer_2
                    {
                        if(this->index_to_load_ == -1)//判断是否没有车在上货区
                        {
                            route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                            route.push_back(this->key_points_.load_position);
                            this->pub_traj(route, i);

                            this->key_points_.is_occupyed_right_buffer[1] = -1;
                            this->index_to_load_ = i;
                            this->key_points_.in_which_buffer[i] = -1;
                            set_state(car_state::CAR_IN_LOAD, i);
                            #ifdef DEBUG
                            ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_1 to CAR_IN_LOAD", agvs[i].get_sn().c_str());
                            #endif
                        }
                        else//上货区被占据
                        {
                            ROS_INFO("[CAR_IN_BUFFER_1]: Car %s can not going to LOADING POSITION occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->index_to_load_].get_sn().c_str());
                            break;
                        }
                    }
                    else//没有真正到达buffer_1
                    {
                        std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_BUFFER_1 state" << "\033[0m" << std::endl;
                        break;
                    }
                }
                break;
            }
            case car_state::CAR_IN_BUFFER_2:
            {
                std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_IN_BUFFER_2 state" << "\033[0m" << std::endl;

                if(i == 1 || i == 2 || i ==0)//左边的buffer_2
                {
                    if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.left_buffer[2], 0.5) && this->cars_work_state_[i] == car_work_state::CAR_READY)//判断agv是否真的到达buffer_2
                    {   
                        if(i == 0)//判断是否为agv_0  则回到起飞点
                        {
                            route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                            route.push_back(this->key_points_.init_point[i]);
                            this->pub_traj(route, i);

                            this->key_points_.is_occupyed_left_buffer[2] = -1;
                            this->key_points_.in_which_buffer[i] = -1;
                            set_state(car_state::CAR_IN_FLIGHT, i);
                            #ifdef DEBUG
                            ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_2 to CAR_IN_FLIGHT", agvs[i].get_sn().c_str());
                            #endif
                        }
                        else//不是agv_0 需要前往buffer_3
                        {
                            if(this->key_points_.is_occupyed_left_buffer[3] == -1)//判断是否没有车在buffer_3
                            {
                                route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                                route.push_back(this->key_points_.left_buffer[3]);
                                this->pub_traj(route, i);
 
                                this->key_points_.is_occupyed_left_buffer[2] = -1;
                                this->key_points_.is_occupyed_left_buffer[3] = i;
                                this->key_points_.in_which_buffer[i] = 3;
                                set_state(car_state::CAR_IN_BUFFER_3, i);
                                #ifdef DEBUG
                                ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_2 to CAR_IN_BUFFER_3", agvs[i].get_sn().c_str());
                                #endif
                            }
                            else//buffer_3被占据
                            {
                                ROS_INFO("[CAR_IN_BUFFER_2]: Car %s can not going to BUFFER_3 occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->key_points_.is_occupyed_left_buffer[3]].get_sn().c_str());
                                break;
                            }
                        }
                      
                    }
                    else//没有真正到达buffer_2
                    {
                        std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_BUFFER_2 state" << "\033[0m" << std::endl;
                        break;
                    }
                }
                else if(i == 4 || i == 5 || i == 3)//右边的buffer_2
                {
                    if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.right_buffer[2], 0.5) && this->cars_work_state_[i] == car_work_state::CAR_READY)//判断agv是否真的到达buffer_2
                    {   
                        if(i == 3)//判断是否为agv_3  则回到起飞点
                        {
                            route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                            route.push_back(this->key_points_.init_point[i]);
                            this->pub_traj(route, i);

                            this->key_points_.is_occupyed_right_buffer[2] = -1;
                            this->key_points_.in_which_buffer[i] = -1;
                            set_state(car_state::CAR_IN_FLIGHT, i);
                            #ifdef DEBUG
                            ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_2 to CAR_IN_FLIGHT", agvs[i].get_sn().c_str());
                            #endif
                        }
                        else//不是agv_3 需要前往buffer_3
                        {
                            if(this->key_points_.is_occupyed_right_buffer[3] == -1)//判断是否没有车在buffer_3
                            {
                                route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                                route.push_back(this->key_points_.right_buffer[3]);
                                this->pub_traj(route, i);

                                this->key_points_.is_occupyed_right_buffer[2] = -1;
                                this->key_points_.is_occupyed_right_buffer[3] = i;
                                this->key_points_.in_which_buffer[i] = 3;
                                set_state(car_state::CAR_IN_BUFFER_3, i);
                                #ifdef DEBUG
                                ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_2 to CAR_IN_BUFFER_3", agvs[i].get_sn().c_str());
                                #endif
                            }
                            else//buffer_3被占据
                            {
                                ROS_INFO("[CAR_IN_BUFFER_2]: Car %s can not going to BUFFER_3 occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->key_points_.is_occupyed_right_buffer[3]].get_sn().c_str());
                                break;
                            }
                        }
                      
                    }
                    else//没有真正到达buffer_2
                    {
                        std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_BUFFER_2 state" << "\033[0m" << std::endl;
                        break;
                    }
                }
                break;

            }
            case car_state::CAR_IN_BUFFER_3:
            {
                std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_IN_BUFFER_3 state" << "\033[0m" << std::endl;

                if(i == 1 || i == 2)//左边的buffer_3
                {
                    if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.left_buffer[3], 0.5) && this->cars_work_state_[i] == car_work_state::CAR_READY)//判断agv是否真的到达buffer_3
                    {   
                        route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                        if(i == 1)//根据是agv_1还是agv_2来决定是否经过buffer_4回到起飞点
                        {
                            route.push_back(this->key_points_.init_point[i]);
                        }
                        else
                        {
                            route.push_back(this->key_points_.left_buffer[4]);
                            route.push_back(this->key_points_.init_point[i]);
                        }
                        this->pub_traj(route, i);
    
                        this->key_points_.is_occupyed_left_buffer[3] = -1;
                        this->key_points_.in_which_buffer[i] = -1;
                        set_state(car_state::CAR_IN_FLIGHT, i);
                        #ifdef DEBUG
                        ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_3 to CAR_IN_FLIGHT", agvs[i].get_sn().c_str());
                        #endif
                    }
                    else//没有真正到达buffer_3
                    {
                        std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_BUFFER_3 state" << "\033[0m" << std::endl;
                        break;
                    }
                }
                else if(i == 4 || i == 5)//右边的buffer_3
                {
                    if(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.right_buffer[3], 0.5) && this->cars_work_state_[i] == car_work_state::CAR_READY)//判断agv是否真的到达buffer_3
                    {   
                        route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                        if(i == 4)//根据是agv_4还是agv_5来决定是否经过buffer_4回到起飞点
                        {
                            route.push_back(this->key_points_.init_point[i]);
                        }
                        else
                        {
                            route.push_back(this->key_points_.right_buffer[4]);
                            route.push_back(this->key_points_.init_point[i]);
                        }
                        this->pub_traj(route, i);

                        this->key_points_.is_occupyed_right_buffer[3] = -1;
                        this->key_points_.in_which_buffer[i] = -1;
                        set_state(car_state::CAR_IN_FLIGHT, i);
                        #ifdef DEBUG
                        ROS_INFO("[Transform]: Car %s from CAR_IN_BUFFER_3 to CAR_IN_FLIGHT", agvs[i].get_sn().c_str());
                        #endif
                    }
                    else//没有真正到达buffer_3
                    {   
                        std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_BUFFER_3 state" << "\033[0m" << std::endl;
                        break;
                    }
                }
                break;
            }
            case car_state::CAR_IN_LOAD:
            {
                std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_IN_LOAD state" << "\033[0m" << std::endl;

                size_t start = 5 * i;
                size_t end = std::min(5 * i + 5, static_cast<size_t>(this->drones_count_));
                std::vector<user_pkg::DronePhysicalStatus> temp_drones(drones.begin() + start, drones.begin() + end);//提取出来的与当前无人车相关的五台无人机
                if(!(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.load_position, 0.3) && this->cars_work_state_[i] == car_work_state::CAR_READY))//判断agv是否真的到达上货区 没到就break 到了就继续运行
                {
                    ROS_WARN("Now_Position: %f, %f", agvs[i].get_position().x, agvs[i].get_position().y);
                    ROS_WARN("load_Position: %f, %f", this->key_points_.load_position.x, this->key_points_.load_position.y);
                    std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_LOAD state" << "\033[0m" << std::endl;
                    break;
                }

                int drone_index;
                if(agvs[i].get_drone_sn().empty())//此时车上没有无人机 上一台
                {
                    //找到位于出生点且处于ready状态的temp_drones中的第一台无人机
                    xyyaw temp_position;
                    auto it = std::find_if(temp_drones.begin(), temp_drones.end(), [&temp_position](user_pkg::DronePhysicalStatus drone)
                    {
                        temp_position = xyyaw(drone.pos.position.x, drone.pos.position.y, 0.0);
                        return judge_distance(temp_position, xyyaw(185.0, 425.0, 0.0), 0.5) && drone.drone_work_state == 1;
                    });
                    // int index = std::distance(temp_drones.begin(), it);
                    //将满足条件的无人机绑定到无人车上s
                    user_pkg::UserCmdRequest msg;
                    msg.peer_id = this->config.getPeerId();
                    msg.task_guid = this->config.getGuid();
                    msg.type = user_pkg::UserCmdRequest::USER_CMD_MOVE_DRONE_ON_CAR;
                    msg.binding_drone.car_sn = agvs[i].get_sn();
                    msg.binding_drone.drone_sn = it->sn;
                    ROS_WARN("before to agv, uav's state is %d", it->drone_work_state);
                    std::cout << "Publishing UserCmdRequest message for bind drone to car" << std::endl;
                    std::cout << "  car_sn: " << msg.binding_drone.car_sn << std::endl;
                    std::cout << "  drone_sn: " << msg.binding_drone.drone_sn << std::endl;
                    this->cmd_pub_.publish(msg);
                    // ROS_WARN("after to agv, uav's state is %d", it->drone_work_state);
                    std::cout << "Waiting for drone to bind to car in 3 seconds" << std::endl;
                    ros::Duration(0.5).sleep();//TODO：等待时间可以去做其他车的决策 这个地方怎么处理
                    break;
                    //等待无人机绑定成功
                }
                else//有无人机返回drone_index
                {
                    drone_index = findDroneIndexBySN(temp_drones, agvs[i].get_drone_sn());
                }
                
                if(temp_drones[drone_index].drone_work_state == 5)
                {
                    this->is_charging_[i] = false;
                }
                if(temp_drones[drone_index].drone_work_state == 6)
                {
                    this->is_loading_cargo_[i] = false;
                }

                int forSwitch = 0;
                if(this->is_charging_[i] || temp_drones[drone_index].drone_work_state == 5)//判断是否正在换电
                {
                    forSwitch = 1;
                }
                else if(this->is_loading_cargo_[i] || temp_drones[drone_index].drone_work_state == 6)//判断是否正在上货
                {
                    forSwitch =2;
                }
                else if(temp_drones[drone_index].drone_work_state == 1)
                {
                    forSwitch = 3;
                }

                switch (forSwitch) //判断无人机此时的状态
                {
                    case 1: //正在换电
                    {
                        ROS_INFO("[CAR_IN_LOAD]: Car %s's drone %s is charging battery",
                                agvs[i].get_sn().c_str(),
                                temp_drones[drone_index].sn.c_str());
                        
                        break;
                    }                                
                    
                    case 2://正在上货
                    {
                        ROS_INFO("[CAR_IN_LOAD]: Car %s's drone %s is loading cargo",
                                this->agvs_[i].get_sn().c_str(),
                                temp_drones[drone_index].sn.c_str());
                        
                        break;
                    }                

                    case 3://无人机is ready
                    {   

                        if(this->cars_work_state_[i] == car_work_state::CAR_READY &&
                            this->drones_[start + drone_index].bind_cargo_id == 0 &&
                            this->is_loading_cargo_[i] == false) //车上有无人机 且车和无人机均为ready 并且无人机上没有货物 则进行上货
                        {   
                            user_pkg::UserCmdRequest msg;
                            msg.peer_id = this->config.getPeerId();
                            msg.task_guid = this->config.getGuid();
                            msg.type = user_pkg::UserCmdRequest::USER_CMD_MOVE_CARGO_IN_DRONE;
                            msg.binding_cargo.cargo_id = this->waybill_[i].cargoParam.index;
                            msg.binding_cargo.drone_sn = agvs[i].get_drone_sn();
                            
                            std::cout << "Publishing UserCmdRequest message for bind cargo to drone" << std::endl;
                            std::cout << "  cargo_id: " << msg.binding_cargo.cargo_id << std::endl;
                            std::cout << "  drone_sn: " << msg.binding_cargo.drone_sn << std::endl;
                            this->cmd_pub_.publish(msg);
                            this->is_loading_cargo_[i] = true;
                            this->waybill_[i] = BILL_TakeOne(this->waybillCarRelation_[i]);
                            std::cout << "Need to wait for cargo to bind to drone in 10 seconds" << std::endl;
                            break;
                        }

                        if(this->battery_progress_[start + drone_index] == 2)//TODO:接到无人机记得更新这个进度 以及无人车前往放飞点的一系列判断 无人机电量是否满足条件 是否有货物等等
                        {
                            //无人机需要换电
                            user_pkg::UserCmdRequest msg;
                            msg.peer_id = this->config.getPeerId();
                            msg.task_guid = this->config.getGuid();
                            msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_BATTERY_REPLACEMENT;
                            msg.drone_msg.drone_sn = drones[start + drone_index].sn;
                            std::cout << "Publishing UserCmdRequest message for drone to battery replacement" << std::endl;
                            std::cout << "  drone_sn: " << msg.drone_msg.drone_sn << std::endl;
                            this->cmd_pub_.publish(msg);
                            this->is_charging_[i] = true;
                            this->battery_progress_[start + drone_index] = 0;
                            std::cout << "Need to wait for drone to battery replacement in 10 seconds" << std::endl;
                            break;
                        }

                        if(((this->battery_progress_[start + drone_index] == 0 && temp_drones[drone_index].remaining_capacity == 100.0) || this->battery_progress_[start + drone_index] == 1) &&
                            temp_drones[drone_index].bind_cargo_id > 0 )//有货 有电 ready状态 则出发前往buffer2
                        {
                            route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                            if(i < 3)//i = 0 1 2 为左边的agv 前往左边的buffer_2
                            {   
                                if(this->key_points_.is_occupyed_left_buffer[2] == -1)//判断buffer_2是否被占据
                                {
                                    route.push_back(this->key_points_.left_buffer[2]);
                                    pub_traj(route, i);

                                    this->key_points_.is_occupyed_left_buffer[2] = i;
                                    this->key_points_.in_which_buffer[i] = 2;
                                    this->index_to_load_ = -1;
                                    set_state(CAR_IN_BUFFER_2, i);
                                    #ifdef DEBUG
                                    ROS_INFO("[Transform]: Car %s from CAR_IN_LOAD to CAR_IN_BUFFER_2", agvs[i].get_sn().c_str());
                                    #endif
                                }
                                else//buffer_2被占据
                                {
                                    ROS_INFO("[CAR_IN_LOAD]: Car %s can not going to BUFFER_2 occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->key_points_.is_occupyed_left_buffer[2]].get_sn().c_str());
                                    break;
                                }
                            }
                            else//i == 3 4 5 为右边的agv 前往左边的buffer_2
                            {
                                if(this->key_points_.is_occupyed_right_buffer[2] == -1)//判断buffer_2是否被占据
                                {
                                    route.push_back(this->key_points_.right_buffer[2]);
                                    pub_traj(route, i);

                                    this->key_points_.is_occupyed_right_buffer[2] = i;
                                    this->key_points_.in_which_buffer[i] = 2;
                                    this->index_to_load_ = -1;
                                    set_state(CAR_IN_BUFFER_2, i);
                                    #ifdef DEBUG
                                    ROS_INFO("[Transform]: Car %s from CAR_IN_LOAD to CAR_IN_BUFFER_2", agvs[i].get_sn().c_str());
                                    #endif
                                }
                                else//buffer_2被占据
                                {
                                    ROS_INFO("[CAR_IN_LOAD]: Car %s can not going to BUFFER_2 occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->key_points_.is_occupyed_right_buffer[2]].get_sn().c_str());
                                    break;
                                }
                            }
                        }
                    
                        break;
                    }                
                }
                break;
            }
            case car_state::CAR_IN_FLIGHT:
            {
                std::cout << "    " << "Agv " << agvs[i].get_sn() << " is in " << "\033[31m" << "CAR_IN_FLIGHT state" << "\033[0m" << std::endl;

                size_t start = 5 * i;
                size_t end = std::min(5 * i + 5, static_cast<size_t>(this->drones_count_));
                std::vector<user_pkg::DronePhysicalStatus> temp_drones(this->drones_.begin() + start, this->drones_.begin() + end);
                if(!(judge_distance(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0), this->key_points_.init_point[i], 0.5) && (this->cars_work_state_[i] == car_work_state::CAR_READY)))
                {
                    std::cout << "    " << " being to " << "\033[31m" << "CAR_IN_FLIGHT state" << "\033[0m" << std::endl;
                    break;
                }
                if(!agvs[i].get_drone_sn().empty()) //车上有无人机
                {
                    int index = findDroneIndexBySN(temp_drones, this->agvs_[i].get_drone_sn());
                    if(temp_drones[index].drone_work_state == 1 &&
                        this->cars_work_state_[i] == car_work_state::CAR_READY )
                        {
                            if(this->deliverRoute_[i]->check_add_uav(index + 1))//无人机起飞判断
                            {
                                //无人机起飞
                                this->deliverRoute_[i]->set_car_ready(index + 1);
                                ROS_WARN("[CAR_IN_FLIGHT]: Uav is to fly in route_%d, line_%d", int(i + 1), index + 1);
                                break;
                            }
                            else
                            {
                                ROS_WARN("[CAR_IN_FLIGHT]: Uav ROUTE %d can not receieve a new drone", int(i + 1));
                                break;
                            }
                        }
                    else 
                    {
                        ROS_WARN("[CAR_IN_FLIGHT]: Car %s or drone %s is not ready, drone can not take off", 
                                    this->agvs_[i].get_sn().c_str(),
                                    temp_drones[index].sn.c_str());
                    }
                }
                else //车上没有无人机
                {

                    uint8_t lineIndex = this->deliverRoute_[i]->choose_uav_line();
                    
                    if(this->deliverRoute_[i]->check_add_uav(lineIndex, this->carPeriod_[i])) //如果航线是空的或者 接一个新的无人机 来回+上货物+搬运无人机+起飞来得及接下一个无人机则 允许去接 
                    {
                        switch (i)
                        {
                            case 0:
                            {
                                if(this->index_to_load_ == -1)//如果没有车前往上货点
                                {
                                    route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                                    route.push_back(this->key_points_.load_position);
                                    ROS_WARN("load_position: %f, %f", this->key_points_.load_position.x, this->key_points_.load_position.y);
                                    pub_traj(route, i);

                                    this->index_to_load_ = i;
                                    set_state(CAR_IN_LOAD, i);
                                    #ifdef DEBUG
                                    ROS_INFO("[Transform]: Car %s from CAR_IN_FLIGHT to CAR_IN_LOAD", agvs[i].get_sn().c_str());
                                    #endif
                                }
                                else
                                {
                                    ROS_INFO("[CAR_IN_FLIGHT]: Car %s can not going to LOADING POSITION occupyed by %s", agvs[i].get_sn().c_str(), agvs[this->index_to_load_].get_sn().c_str());
                                }
                                break;
                            }
                            case 1:
                            {
                                if(this->key_points_.is_occupyed_left_buffer[1] == -1)
                                {
                                    route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                                    route.push_back(this->key_points_.left_buffer[1]);
                                    pub_traj(route, i);

                                    this->key_points_.is_occupyed_left_buffer[1] = i;
                                    this->key_points_.in_which_buffer[i] = 1;
                                    set_state(CAR_IN_BUFFER_1, i);
                                    #ifdef DEBUG
                                    ROS_INFO("[Transform]: Car %s from CAR_IN_FLIGHT to CAR_IN_BUFFER_1", agvs[i].get_sn().c_str());
                                    #endif
                                }
                                break;
                            }
                            case 2:
                            {
                                if(this->key_points_.is_occupyed_left_buffer[0] == -1)
                                {
                                    route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                                    route.push_back(this->key_points_.left_buffer[0]);
                                    pub_traj(route, i);

                                    this->key_points_.is_occupyed_left_buffer[0] = i;
                                    this->key_points_.in_which_buffer[i] = 0;
                                    set_state(CAR_IN_BUFFER_0, i);
                                    #ifdef DEBUG
                                    ROS_INFO("[Transform]: Car %s from CAR_IN_FLIGHT to CAR_IN_BUFFER_0", agvs[i].get_sn().c_str());
                                    #endif
                                }
                                break;
                            }
                            case 3:
                            {
                                
                                if(this->index_to_load_ == -1)//如果没有车前往上货点
                                {
                                    route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                                    route.push_back(this->key_points_.load_position);
                                    pub_traj(route, i);

                                    this->index_to_load_ = i;
                                    set_state(CAR_IN_LOAD, i);
                                    #ifdef DEBUG
                                    ROS_INFO("[Transform]: Car %s from CAR_IN_FLIGHT to CAR_IN_LOAD", agvs[i].get_sn().c_str());
                                    #endif
                                }
                                break;
                            }
                            case 4:
                            {
                                if(this->key_points_.is_occupyed_right_buffer[1] == -1)
                                {
                                    route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                                    route.push_back(this->key_points_.right_buffer[1]);
                                    pub_traj(route, i);

                                    this->key_points_.is_occupyed_right_buffer[1] = i;
                                    this->key_points_.in_which_buffer[i] = 1;
                                    set_state(CAR_IN_BUFFER_1,i);
                                    #ifdef DEBUG
                                    ROS_INFO("[Transform]: Car %s from CAR_IN_FLIGHT to CAR_IN_BUFFER_1", agvs[i].get_sn().c_str());
                                    #endif
                                }
                                break;
                            }
                            case 5:
                            {
                                if(this->key_points_.is_occupyed_right_buffer[0] == -1)
                                {
                                    route.push_back(xyyaw(agvs[i].get_position().x, agvs[i].get_position().y, 0.0));
                                    route.push_back(this->key_points_.right_buffer[0]);
                                    pub_traj(route, i);

                                    this->key_points_.is_occupyed_right_buffer[0] = i;
                                    this->key_points_.in_which_buffer[i] = 0;
                                    set_state(CAR_IN_BUFFER_0, i);
                                    #ifdef DEBUG
                                    ROS_INFO("[Transform]: Car %s from CAR_IN_FLIGHT to CAR_IN_BUFFER_0", agvs[i].get_sn().c_str());
                                    #endif
                                }
                                break;
                            }
                        }
                    }
                    else
                    {
                        int index = this->deliverRoute_[i]->choose_uav_landing();
                        this->deliverRoute_[i]->set_car_ready(index);
                        ROS_INFO("[CAR_IN_FLIGHT]: Car %s can not go to receive a new drone, wait for uav %s to landing", agvs[i].get_sn().c_str(), drones[start + index -1].sn.c_str());
                        break;
                    }
                }

                break;
            }   
        }

        this->deliverRoute_[i]->route_update(drones, this->cmd_pub_);
        
    }
    std::cout <<"\033[32m" << "---------------------------------------------------------------------------------------------" << "\033[0m" << std::endl;
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "\033[34m" << "Time used: " << duration.count() << "ms" << "\033[0m" << std::endl;
}




}

