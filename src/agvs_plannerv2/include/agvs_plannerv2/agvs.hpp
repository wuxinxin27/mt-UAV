/**
 * @file agvs.hpp
 * @brief  agv class and agvs class depend on agv class
 * @author RuiXin Wu
 * @date 2024/09/03              
 * @version 1.0
 */




#ifndef AGVS_H
#define AGVS_H

#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <ctime>
#include <fstream>
#include <cmath>

#include <ros/ros.h>

#include "user_pkg/PanoramicInfo.h"
#include "user_pkg/CarPhysicalStatus.h"
#include "user_pkg/DynamicPosition.h"
#include "user_pkg/Position.h"
#include "user_pkg/EulerAngle.h"
#include "user_pkg/UserCmdRequest.h"
#include "user_pkg/UserCmdResponse.h"



#include "agvs_plannerv2/Config.h"
#include "agvs_plannerv2/uav_route.h"
#define DEBUG

/**
 * @brief for debug
 * @param str: the string you want to print
 * @param i: 1 for ROS_INFO, 2 for ROS_WARN
 */
std::vector<std::string> car_sn = {
    "SIM-MAGV-0001",
    "SIM-MAGV-0002",
    "SIM-MAGV-0003",
    "SIM-MAGV-0004",
    "SIM-MAGV-0005",
    "SIM-MAGV-0006",
};
std::vector<std::string> drone_sn = {
    "SIM-DRONE-0001", "SIM-DRONE-0002", "SIM-DRONE-0003", "SIM-DRONE-0004", "SIM-DRONE-0005",
    "SIM-DRONE-0006", "SIM-DRONE-0007", "SIM-DRONE-0008", "SIM-DRONE-0009", "SIM-DRONE-0010",
    "SIM-DRONE-0011", "SIM-DRONE-0012", "SIM-DRONE-0013", "SIM-DRONE-0014", "SIM-DRONE-0015",
    "SIM-DRONE-0016", "SIM-DRONE-0017", "SIM-DRONE-0018", "SIM-DRONE-0019", "SIM-DRONE-0020",
    "SIM-DRONE-0021", "SIM-DRONE-0022", "SIM-DRONE-0023", "SIM-DRONE-0024", "SIM-DRONE-0025",
    "SIM-DRONE-0026", "SIM-DRONE-0027", "SIM-DRONE-0028", "SIM-DRONE-0029", "SIM-DRONE-0030",
};


// 由于读取到的全景信息里，cars数组和drones数组并不是严格按照sn排序的，因此需要自己实现查找car和drone的索引
static int findCarIndexBySN(const std::vector<user_pkg::CarPhysicalStatus>& car_physical_status, const std::string& sn)
{
    auto it = std::find_if(car_physical_status.begin(), car_physical_status.end(),
                           [&sn](const user_pkg::CarPhysicalStatus& status) { return status.sn == sn; });

    if (it != car_physical_status.end())
    {
        // std::cout << "findCarIndexBySN: " << sn << " index: " << std::distance(car_physical_status.begin(), it);
        return std::distance(car_physical_status.begin(), it);
    }
    else
    {
        std::cout << "findCarIndexBySN: " << sn << " not found" << std::endl;
        return -1;  // 未找到
    }
}

static int findDroneIndexBySN(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status, const std::string& sn)
{
    auto it = std::find_if(drone_physical_status.begin(), drone_physical_status.end(),
                           [&sn](const user_pkg::DronePhysicalStatus& status) { return status.sn == sn; });

    if (it != drone_physical_status.end())
    {
        // std::cout << "finddroneIndexBySN: " << sn << " index: " << std::distance(drone_physical_status.begin(), it);
        return std::distance(drone_physical_status.begin(), it);
    }
    else
    {
        std::cout << "findDroneIndexBySN: " << sn << " not found" << std::endl;
        return -1;  // 未找到
    }
}

// /**
//  * @brief 根据无人车的索引找到对应的无人机(5架,按照索引顺序)
//  * @param drone_physical_status: 全局信息中的无人机状态(乱序无人机)
//  * @param index: 无人车的索引
//  * @return 无人机状态向量
//  */
// static std::vector<user_pkg::DronePhysicalStatus> findDronesByindex(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status, const int& index)
// {
//     std::vector<user_pkg::DronePhysicalStatus> drones;
//     int index5 = 5 * index;
//     for(int i = index5; i < index5 + 5; i++)
//     {
//         drones.push_back(drone_physical_status[findDroneIndexBySN(drone_physical_status, drone_sn[i])]);
//     }
//     return drones;
// }

namespace agvs_ns{


    template <class T>
    bool judge_distance(const T& a, const T& b, const double& distance)
    {
        return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) < distance*distance;
    }

/** @brief a list of AGV's state*/
    enum car_work_state
    {
        CAR_UNKOWN = 0,
        CAR_READY = 1,
        CAR_RUNNING = 2,
        CAR_ERROR = 10
    };
    enum car_state
    {
        CAR_INIT = 0,
        CAR_IN_BUFFER_0 = 1,
        CAR_IN_BUFFER_1 = 2,
        CAR_IN_BUFFER_2 = 3,
        CAR_IN_BUFFER_3 = 4,
        CAR_IN_LOAD = 5,
        CAR_IN_FLIGHT = 6, 

    };


    std::string print_agv_work_state(const car_work_state& state)
    {
        switch (state)
        {
        case car_work_state::CAR_UNKOWN:
            return "CAR_UNKOWN";
        case car_work_state::CAR_READY:
            return "CAR_READY";
        case car_work_state::CAR_RUNNING:
            return "CAR_RUNNING";
        case car_work_state::CAR_ERROR:
            return "CAR_ERROR";
        default:
            return "ERROR";
        }
    }

    std::string print_agv_state(const car_state& state)
    {
        switch (state)
        {
        case car_state::CAR_INIT:
            return "CAR_INIT";
        case car_state::CAR_IN_BUFFER_0:
            return "CAR_IN_BUFFER_0";
        case car_state::CAR_IN_BUFFER_1:
            return "CAR_IN_BUFFER_1";
        case car_state::CAR_IN_BUFFER_2:
            return "CAR_IN_BUFFER_2";
        case car_state::CAR_IN_BUFFER_3:
            return "CAR_IN_BUFFER_3";
        case car_state::CAR_IN_LOAD:
            return "CAR_IN_LOAD";
        case car_state::CAR_IN_FLIGHT:
            return "CAR_IN_FLIGHT";
        default:
            return "ERROR";
        }
    }


/**
 *  @struct xyyaw agvs.hpp
 *  @brief for planning AGV's path
 */
    struct xyyaw
    {
    public:
        double x;
        double y;
        double yaw;
        xyyaw(){};
        xyyaw(double x, double y, double yaw):x(x), y(y), yaw(yaw){};
        ~xyyaw(){};
    };


/**
 *  @class agv agvs.hpp
 *  @brief includeing some property of agv 
 */
    class agv
    {
    private:
        std::string sn_;
        // ros::Time timestamp_;
        int peer_id_;
        std::string task_guid_;

        user_pkg::Position position_;
        user_pkg::EulerAngle euler_angle_;
        double v_;

        enum car_work_state car_work_state_;


        std::string description_;
        std::string drone_sn_;
    public:

        agv(){};
        ~agv(){};

        agv(const user_pkg::CarPhysicalStatus& car)
        {
            this->sn_ = car.sn;
            // this->timestamp_ = ros::Time(car.timestamp);//TODO:要具体看timestamp表示的物理意义 单位 毫秒 纳秒 还是unix时间戳
            this->peer_id_ = car.peer_id;
            this->task_guid_ = car.task_guid;
            this->position_ = car.pos.position;
            this->euler_angle_ = car.pos.euler_angle;
            this->v_ = car.pos.v;
            this->car_work_state_ = (enum car_work_state)car.car_work_state;
            this->description_ = car.description;
            this->drone_sn_ = car.drone_sn;

        };

        agv& update_state(const user_pkg::CarPhysicalStatus& car)
        {   
            this->sn_ = car.sn;
            // this->timestamp_ = ros::Time(car.timestamp);//TODO:要具体看timestamp表示的物理意义 单位 毫秒 纳秒 还是unix时间戳
            this->peer_id_ = car.peer_id;  //TODO:peer_id和task_guid如果保持不变，则不需要更新
            this->task_guid_ = car.task_guid;
            this->position_ = car.pos.position;
            this->euler_angle_ = car.pos.euler_angle;
            this->v_ = car.pos.v;
            this->car_work_state_ = (enum car_work_state)car.car_work_state;
            this->description_ = car.description;
            this->drone_sn_ = car.drone_sn;
            return *this;
        }
        user_pkg::UserCmdRequest move(const std::vector<xyyaw>& route) const//TODO:如果写了状态机，可以在这里加入一个切换状态的变量,看看后续要不要加入yaw角
        {
            user_pkg::UserCmdRequest msg = user_pkg::UserCmdRequest();
            msg.peer_id = this->peer_id_;
            msg.task_guid = this->task_guid_;
            msg.type = user_pkg::UserCmdRequest::USER_CMD_CAR_EXEC_ROUTE;
            msg.car_route_info.carSn = this->sn_;
            for(const auto& point:route)
            {
                user_pkg::Position point_temp;
                point_temp.x = point.x;
                point_temp.y = point.y;
                point_temp.z = (this->position_).z;
                msg.car_route_info.way_point.push_back(point_temp);
            }
            msg.car_route_info.yaw = 0.0;
            return msg;
        }
        

        inline std::string get_sn() const
        {
            return this->sn_;
        }
        inline std::string get_drone_sn() const 
        {
            return this->drone_sn_;
        }

        inline user_pkg::Position get_position() 
        {
            return this->position_;
        }

        inline user_pkg::EulerAngle get_euler() 
        {
            return this->euler_angle_;
        }


    };



/**
 *  @class agvs agvs.hpp
 *  @brief includeing all of agvs 
 */
    class agvs
    {
        typedef std::vector<enum car_state> cars_state;
        typedef std::vector<enum car_work_state> cars_work_state;
        struct key_points
        {   
            int agvs_count = 6;

            xyyaw load_position;
            xyyaw drone_birth_position;
            std::vector<xyyaw> init_position;

            std::vector<xyyaw> init_point;
            std::vector<xyyaw> left_buffer;
            std::vector<int> is_occupyed_left_buffer;
            std::vector<xyyaw> right_buffer;
            std::vector<int> is_occupyed_right_buffer; //每个int代表对应的left_buffer被哪个agv所占用，-1代表未被占用
            std::vector<int> in_which_buffer; //每个int代表在哪个buffer内，-1代表不在buffer内
            key_points(){};
            key_points(const int& count):agvs_count(count)
            {
                this->load_position =  xyyaw(190.0, 425.0, 0.0);
                this->drone_birth_position = xyyaw(185.0, 425.0, 0.0);
                this->init_position.push_back(xyyaw(184.0, 434.0, 0.0));
                this->init_position.push_back(xyyaw(184.0, 440.0, 0.0));
                this->init_position.push_back(xyyaw(184.0, 446.0, 0.0));
                this->init_position.push_back(xyyaw(196.0, 434.0, 0.0));
                this->init_position.push_back(xyyaw(196.0, 440.0, 0.0));
                this->init_position.push_back(xyyaw(196.0, 446.0, 0.0));

                this->init_point.push_back(xyyaw(184.0, 434.0, 0.0));
                this->init_point.push_back(xyyaw(184.0, 440.0, 0.0));
                this->init_point.push_back(xyyaw(184.0, 448.0, 0.0));
                this->init_point.push_back(xyyaw(196.0, 434.0, 0.0));
                this->init_point.push_back(xyyaw(196.0, 440.0, 0.0));
                this->init_point.push_back(xyyaw(196.0, 448.0, 0.0));

                this->left_buffer.push_back(xyyaw(188.25, 440.0, 0.0));
                this->left_buffer.push_back(xyyaw(188.25, 434.0, 0.0));
                this->left_buffer.push_back(xyyaw(180.5, 425.0, 0.0));
                this->left_buffer.push_back(xyyaw(180.5, 434.0, 0.0));
                this->left_buffer.push_back(xyyaw(180.5, 440.0, 0.0));
                this->is_occupyed_left_buffer.resize(this->left_buffer.size(), -1);
                this->right_buffer.push_back(xyyaw(191.75, 440.0, 0.0));
                this->right_buffer.push_back(xyyaw(191.75, 434.0, 0.0));
                this->right_buffer.push_back(xyyaw(199.5, 425.0, 0.0));
                this->right_buffer.push_back(xyyaw(199.5, 434.0, 0.0));
                this->right_buffer.push_back(xyyaw(199.5, 440.0, 0.0));
                this->is_occupyed_right_buffer.resize(this->right_buffer.size(), -1);
                this->in_which_buffer.resize(this->agvs_count, -1);
            }
        };
        
    private:
        ros::NodeHandle nh_;
        ros::Publisher cmd_pub_;
        ros::Subscriber info_sub_;
        ros::Subscriber cmdres_sub_;
        ros::Timer cmd_timer_;
        ros::Timer timer_;

        bool is_init_ = false;
        std::vector<bool> is_charging_;
        std::vector<bool> is_loading_cargo_;
        //drone and bill information
        int drones_count_ = 30;
        std::vector<user_pkg::DronePhysicalStatus> drones_;
        std::vector<int> battery_progress_;
        std::vector<user_pkg::BillStatus> bills_;

        int agvs_count_ = 6;
        std::vector<agv> agvs_;
        std::vector<std::pair<xyyaw, bool>> targets_;//just for planner

        std::vector<ros::Duration> carPeriod_;

        Config config;

        //位置参数
        key_points key_points_;
        // std::vector<xyyaw> init_position_;
        // std::vector<xyyaw> wait_load_position_;
        // xyyaw load_position_;//TODO:传入接口
        // xyyaw drone_birth_position_ = xyyaw(185.0, 425.0, 0.0);

        cars_state cars_state_; //all agvs state
        cars_work_state cars_work_state_;

        //前往load区域的car的index
        int index_to_load_ = -1;

        //命令执行结果
        int cmd_res_ = 0;
        std::string cmd_res_description_;

        //随机数生成器
        std::mt19937 gen_;
        std::uniform_real_distribution<> real_distrib_;

        unsigned int main_loop_count_ = 0;
        bool is_init_aaaa[6] ={false};

        std::vector<std::shared_ptr<DeliverRoute>> deliverRoute_;
        std::vector<TaskParam::WaybillParam> waybill_;    
        std::vector<int> waybillCarRelation_;

        //线程锁
        std::atomic_flag targets_lock_ = ATOMIC_FLAG_INIT;
        std::atomic_flag info_lock_ = ATOMIC_FLAG_INIT;

    public:
        agvs(ros::NodeHandle& nh):nh_(nh), gen_(std::random_device{}()), real_distrib_(0.0, 1.0)
        {
            this->key_points_ = key_points(this->agvs_count_);
            this->agvs_.resize(this->agvs_count_);
            this->targets_.resize(this->agvs_count_, std::pair<xyyaw, bool>(xyyaw(0.0, 0.0, 0.0), false));
            this->drones_.resize(this->drones_count_);
            this->battery_progress_.resize(this->drones_count_, 0);
            this->cars_state_.resize(this->agvs_count_, car_state::CAR_INIT);
            this->cars_work_state_.resize(this->agvs_count_);
            this->is_charging_.resize(this->agvs_count_, false);
            this->is_loading_cargo_.resize(this->agvs_count_, false);
            this->carPeriod_.resize(this->agvs_count_, ros::Duration(30.0));//TODO:更新准确时间
            // 读取 JSON 文件
            std::ifstream jsonFile("/config/config.json", std::ifstream::binary);
            if (!jsonFile.is_open()) {
                ROS_WARN("Unable to open json file");
                ros::shutdown();
            }
            ROS_INFO("Successfully open user config file");
            Json::Value root;
            jsonFile >> root;
            this->config.LoadFromJson(root);
            this->key_points_.load_position = xyyaw(config.getLoadingCargoPoint().x, config.getLoadingCargoPoint().y, 0.0);

            std::vector<TaskParam::WaybillParam> waybill_params = config.getWaybillParamList();
            BILL_GroupInit(waybill_params);
            this->waybillCarRelation_ = {2, 1, 4, 3, 6, 5};
            this->waybill_.push_back(BILL_TakeOne(2));
            this->waybill_.push_back(BILL_TakeOne(1));
            this->waybill_.push_back(BILL_TakeOne(4));
            this->waybill_.push_back(BILL_TakeOne(3));
            this->waybill_.push_back(BILL_TakeOne(6));
            this->waybill_.push_back(BILL_TakeOne(5));

            UAV_MapInit();
            for(int i = 0; i < this->agvs_count_; i++)
            {
                std::vector<std::string> drones_sn_temp(drone_sn.begin() + 5 * i, drone_sn.begin() + 5 * (i + 1)); 
                user_pkg::Position position_temp;
                user_pkg::Position waybill_position_temp;
                waybill_position_temp.x = this->waybill_[i].targetPosition.x;
                waybill_position_temp.y = this->waybill_[i].targetPosition.y;
                waybill_position_temp.z = this->waybill_[i].targetPosition.z;

                position_temp.x = this->key_points_.init_point[i].x;
                position_temp.y = this->key_points_.init_point[i].y;
                position_temp.z = -16.0;
                // ROS_WARN("")
                this->deliverRoute_.push_back(std::make_shared<DeliverRoute>(this->waybillCarRelation_[i], position_temp, waybill_position_temp, drones_sn_temp, this->config));
            }


            this->cmd_pub_ = nh_.advertise<user_pkg::UserCmdRequest>("/cmd_exec", 10000);
            this->info_sub_ = nh_.subscribe("/panoramic_info", 10, &agvs::update_state, this);
            this->cmdres_sub_ = nh_.subscribe("/cmd_resp", 10, &agvs::cmd_res_callback, this);
            this->timer_ = nh_.createTimer(ros::Duration(1.0), &agvs::FSM_loop, this);
            // this->cmd_timer_ = nh_.createTimer(ros::Duration(0.5), &agvs::cmd_loop, this);
            
            ROS_INFO("Please wait for 10s to initialize node");
            ros::Duration(10).sleep();
            ROS_WARN("[agvs_planner_node]: Node initialize succuss!!!");

        };
        ~agvs(){};

        void cmd_res_callback(const user_pkg::UserCmdResponseConstPtr& msg)
        {
            this->cmd_res_ = msg->type;
            this->cmd_res_description_ = msg->description;
        }

        inline std::vector<xyyaw> get_positions() 
        {
            std::vector<xyyaw> temp_positions;
            for(auto& agv:(this->agvs_))
            {   
                user_pkg::Position position = agv.get_position();
                temp_positions.push_back(xyyaw(position.x, position.y, agv.get_euler().yaw));
            }
            return temp_positions;
        }

        inline std::vector<enum car_state> get_state() const
        {
            return this->cars_state_;
        }
        inline void set_state(enum car_state car_state, int index)
        {
            this->cars_state_[index] = car_state;
        }
        inline void set_state(std::vector<car_state> cars_state)
        {
            this->cars_state_.assign(cars_state.begin(), cars_state.end());
        }

        // void cmd_loop(const ros::TimerEvent& )
        // {   
        //     return;
        //     double sleep_time = 2.0;
        //     std::vector<std::pair<xyyaw, bool>> temp_targets;
        //     while (this->targets_lock_.test_and_set());
        //     temp_targets.assign(this->targets_.begin(), this->targets_.end());
        //     this->targets_lock_.clear();
        //     for(size_t i = 0; i < temp_targets.size(); ++i)
        //     {
        //         if(temp_targets[i].second)
        //         {
        //             break;
        //         }
        //         else if(i == temp_targets.size()-1)
        //         {
        //             ROS_INFO("[CAR_cmd_loop]: Dont have any target to move");
        //             return;
        //         }
        //     }

        //     std::vector<xyyaw> temp_positions = this->get_positions();
            
        //     std::vector<std::vector<xyyaw>> route = this->agvs_route_plan(temp_positions, temp_targets);
        //     user_pkg::UserCmdRequest msg;
        //     for(size_t i = 0; i < temp_targets.size(); ++i)
        //     {
        //         if(temp_targets[i].second)
        //         {
        //             msg = this->agvs_[i].move(route[i]);
        //             this->cmd_pub_.publish(msg);
        //             ROS_INFO("[CAR_cmd_loop]: Car %s is moving, from [%f, %f] to [%f, %f]",
        //                 this->agvs_[i].get_sn().c_str(), 
        //                 temp_positions[i].x, temp_positions[i].y,
        //                 route[i].back().x, route[i].back().y);
        //         }
        //     }
        //     ros::Duration(sleep_time).sleep();
        //     this->clear_targets();
        // }


/**
 * @brief public_traj for designated agv
 * @param route The agv's route
 * @param agv_index The index of agv that releases the trajectory
 * @since 1.0
 */    
        agvs& pub_traj(const std::vector<xyyaw>& route, const int& agv_index)
        {
            user_pkg::UserCmdRequest msg = this->agvs_[agv_index].move(route);
            ROS_WARN("[CAR_cmd_loop]: msg: car %s from [%f, %f] to [%f, %f]", msg.car_route_info.carSn.c_str(), msg.car_route_info.way_point.front().x, msg.car_route_info.way_point.front().y, msg.car_route_info.way_point.back().x, msg.car_route_info.way_point.back().y);
            if(fabs(route.front().x - route.back().x) < 0.1 && fabs(route.front().y - route.back().y) < 0.1)
            {
                ROS_WARN("[CAR_cmd_loop]: route is too short, ignore");
                return *this;
            }
            this->cmd_pub_.publish(msg);
            ROS_INFO("[CAR_cmd_loop]: Car %s is moving, from [%f, %f] to [%f, %f]",
            this->agvs_[agv_index].get_sn().c_str(), 
            route.front().x, route.front().y,
            route.back().x, route.back().y);
            return *this;
        }

/**
 * @brief useing this funcition before move(time_est)
 * @param targets agvs's targets 
 * @since 1.0
 */    
        agvs& clear_targets()
        {   
            std::pair<xyyaw, bool> target_temp = std::make_pair(xyyaw(0.0, 0.0, 0.0), false);
            while(this->targets_lock_.test_and_set());
            std::fill(this->targets_.begin(), this->targets_.end(), target_temp);
            this->targets_lock_.clear();
            return *this;
        }  
        agvs& set_targets(const xyyaw& target, const bool& is_pub_traj, int index)
        {
            while(this->targets_lock_.test_and_set());
            this->targets_[index].first = target;
            this->targets_[index].second = is_pub_traj;
            this->targets_lock_.clear();
            return *this;
        }
        agvs& set_targets(const xyyaw& target, int index)
        {
            while(this->targets_lock_.test_and_set());

            this->targets_[index].first = target;

            this->targets_lock_.clear();
            return *this;
        }
        agvs& set_targets(const bool& is_pub_traj, int index)
        {
            while(this->targets_lock_.test_and_set());

            this->targets_[index].second = is_pub_traj;

            this->targets_lock_.clear();
            return *this;
        }
        agvs& set_targets(const std::vector<bool>& is_put_traj)
        {
            while(this->targets_lock_.test_and_set());
            for(size_t i = 0; i < is_put_traj.size(); ++i)
            {
                this->targets_[i].second = is_put_traj[i];
            }
            this->targets_lock_.clear();
            return *this;
        }
        agvs& set_targets(const std::vector<xyyaw>& targets, const std::vector<bool>& is_put_traj)
        {   
            while(this->targets_lock_.test_and_set());

            this->targets_.clear();
            for(size_t i = 0; i<targets.size(); ++i)
            {
                this->targets_.push_back(std::make_pair(targets[i], is_put_traj[i]));
            }

            this->targets_lock_.clear();
            return *this;
        }


        agvs& set_targets(const std::vector<xyyaw>& targets, int i, const xyyaw& target, const std::vector<bool>& is_put_traj)
        {
            while(this->targets_lock_.test_and_set());    

            this->targets_.clear();
            for(size_t i = 0; i<targets.size(); ++i)
            {
                this->targets_.push_back(std::make_pair(targets[i], is_put_traj[i]));
            }
            this->targets_[i] = std::make_pair(target, is_put_traj[i]);

            this->targets_lock_.clear();
            return *this;
        }

    private:

/**
 * @brief update AGVs's state by info_sub_
 * @param info 
 * @since 1.0
 */
        inline void update_state(const user_pkg::PanoramicInfoConstPtr& info)
        {   
            while(this->info_lock_.test_and_set());
            std::vector<user_pkg::CarPhysicalStatus> cars = info->cars;
            for(int i = 0; i < this->drones_count_; ++i)
            {
                this->drones_[i] = info->drones[findDroneIndexBySN(info->drones, drone_sn[i])];
            }
            this->bills_ = info->bills;
            this->info_lock_.clear();

            for(decltype(cars.size()) i = 0; i < cars.size(); ++i)
            {
                int index = findCarIndexBySN(cars, car_sn[i]);
                user_pkg::CarPhysicalStatus car = cars[index];
                this->agvs_[i].update_state(car);
                this->cars_work_state_[i] = (enum car_work_state)car.car_work_state;
            }

        }


/**
 * @brief planning AGVs's path
 * 
 * @since 1.0
 */
        // std::vector<std::vector<xyyaw>> agvs_route_plan(const std::vector<xyyaw>& positions, const std::vector<std::pair<xyyaw, bool>>& targets);



/** 
 * @brief main loop
 * @since 1.0
 * 
 */
        void FSM_loop(const ros::TimerEvent& );


    };

}     













#endif

