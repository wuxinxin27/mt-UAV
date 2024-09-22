// ros msg
#include "ros/ros.h"
#include "user_pkg/UserCmdRequest.h"
#include "user_pkg/DynamicPosition.h"
#include "user_pkg/EulerAngle.h"
#include "user_pkg/Position.h"
#include "user_pkg/DronePhysicalStatus.h"
#include "user_pkg/CarPhysicalStatus.h"
#include "user_pkg/BillStatus.h"
#include "user_pkg/PanoramicInfo.h"
#include "user_pkg/DroneMsg.h"
#include "user_pkg/UserCmdResponse.h"
#include "user_pkg/Voxel.h"
#include "user_pkg/QueryVoxel.h"

// config
#include "race_demo_cpp/Config.h"
#include <fstream>
#include <iostream>
#include <string>


// user

#include <map_generator/map_generator.h>
#include <race_demo_cpp/uav_route.h>

// MapGenerator *mapGenerator;
// DeliverUav *test;


DeliverRoute *deliverRoute1;





user_pkg::PanoramicInfo::ConstPtr current_panoramic_info;
int cmd_response_type = 100;
// 通过cmd_exec_type修改指令流程
int cmd_exec_type = 100;  
bool recevie_panoramic_info = false;
std::vector<user_pkg::CarPhysicalStatus> car_physical_status;
std::vector<user_pkg::DronePhysicalStatus> drone_physical_status;
std::vector<std::string> car_sn = {
    "SIM-MAGV-0001",
    "SIM-MAGV-0002",
    "SIM-MAGV-0003",
    "SIM-MAGV-0004",
    "SIM-MAGV-0005",
    "SIM-MAGV-0006"
};
std::vector<std::string> drone_sn = {
    "SIM-DRONE-0001",
    "SIM-DRONE-0001",
    "SIM-DRONE-0001",
    "SIM-DRONE-0001",
    "SIM-DRONE-0001"
};
user_pkg::Position car_start_pos;
user_pkg::Position car_current_pos;
user_pkg::Position drone_current_pos;
user_pkg::Position loading_cargo_point; 
user_pkg::Position waybill_pos;  // 这里只取了第一个订单
int car_work_state = user_pkg::CarPhysicalStatus::CAR_ERROR;
int drone_work_state = user_pkg::DronePhysicalStatus::ERROR;


std::string carSn = car_sn[0];
TaskParam::WaybillParam billNow = {0};

// 由于读取到的全景信息里，cars数组和drones数组并不是严格按照sn排序的，因此需要自己实现查找car和drone的索引
int findCarIndexBySN(const std::vector<user_pkg::CarPhysicalStatus>& car_physical_status, const std::string& sn)
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

int findDroneIndexBySN(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status, const std::string& sn)
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

void panoramicInfoCallback(const user_pkg::PanoramicInfo::ConstPtr& msg)
{
  //std::cout << "Ros Received panoramic info" << std::endl;
  recevie_panoramic_info = true;
  current_panoramic_info = msg;
  car_physical_status = msg->cars;
  drone_physical_status = msg->drones;
  //只读取了一个无人机状态
  car_work_state = msg->cars[findCarIndexBySN(car_physical_status, carSn)].car_work_state;
  drone_work_state = msg->drones[findDroneIndexBySN(drone_physical_status, drone_sn[0])].drone_work_state;
}

void cmdResponseCallback(const user_pkg::UserCmdResponse::ConstPtr& msg)
{
    cmd_response_type = msg->type;
    std::cout << "******************* cmd response callback start *******************" << std::endl; 
    std::cout << "Ros Received cmd response:" << cmd_response_type << std::endl;
    std::cout << " description: " << msg->description << std::endl;
    std::cout << "*******************  cmd response callback end  *******************" << std::endl;
}

void printPanoramicInfo(const ros::TimerEvent& event)
{
  if (current_panoramic_info)
  {
    // 这里只打印了car1 drone1 bill1 event1的状态
    std::cout << "******************* print panoramic info start *******************" << std::endl;
    // std::cout << "cars: " << std::endl;
    // std::cout << current_panoramic_info->cars[findCarIndexBySN(current_panoramic_info->cars, carSn)] << std::endl;
    std::cout << "drones: " << std::endl;
    std::cout << current_panoramic_info->drones[findDroneIndexBySN(current_panoramic_info->drones, drone_sn[0])]<< std::endl;
    // std::cout << "bills: " << std::endl;
    // std::cout << current_panoramic_info->bills[billNow.index]<< std::endl;
    std::cout << "current cmd exec type: " << cmd_exec_type<< std::endl;
    if (!current_panoramic_info->events.empty())
    {
        std::cout << "events: "<< std::endl;
        std::cout << current_panoramic_info->events[0]<< std::endl;
    }
    std::cout << "total score: " << current_panoramic_info->score << std::endl;
    std::cout << "*******************  print panoramic info end  *******************" << std::endl;
  }
  else
  {
    std::cout << "No panoramic info received yet." << std::endl;
  }
}

bool judgeOnePosition(const user_pkg::Position& pos1, const user_pkg::Position& pos2, double threshold)
{
  double distance = sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) + pow(pos1.z - pos2.z, 2));
  if (distance < threshold)
  {
    std::cout << "judgeOnePosition: distance < " << threshold << std::endl;
    return true;
  }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "race_demo_cpp_node");

  ros::NodeHandle nh;
  ros::Publisher cmd_pub = nh.advertise<user_pkg::UserCmdRequest>("/cmd_exec", 10000);
  ros::Subscriber info_sub = nh.subscribe("/panoramic_info", 10, panoramicInfoCallback);
  ros::Subscriber cmd_resp_sub = nh.subscribe("/cmd_resp", 10, cmdResponseCallback);
  // 每5s打印一次全景信息
  ros::Timer timer = nh.createTimer(ros::Duration(5.0), printPanoramicInfo);
  ros::ServiceClient map_client = nh.serviceClient<user_pkg::QueryVoxel>("query_voxel");



  std::cout << "Please wait for 5s to initialize node" << std::endl;
  ros::Duration(5).sleep(); 
  if (info_sub.getTopic().empty() && cmd_resp_sub.getTopic().empty()) {
    std::cout << "Failed to subscribe to /panoramic_info topic or /cmd_resp topic" << std::endl;
  } else {
    std::cout << "Successfully subscribed to /panoramic_info topic and /cmd_resp topic" << std::endl;
  }
  ros::Rate loop_rate(1);  // 每1s发送一次

  // 读取 JSON 文件
  std::ifstream jsonFile("/config/config.json", std::ifstream::binary);
  if (!jsonFile.is_open()) {
      std::cout << "Unable to open json file" << std::endl;
      return 1;
  }
  std::cout << "Successfully open user config file" << std::endl;
  Json::Value root;
  jsonFile >> root;
  Config config;
  config.LoadFromJson(root);
  loading_cargo_point.x = config.getLoadingCargoPoint().x;
  loading_cargo_point.y = config.getLoadingCargoPoint().y;
  loading_cargo_point.z = config.getLoadingCargoPoint().z;

  int waybill_count = 1;
    // std::vector<TaskParam::WaybillParam>
  std::vector<TaskParam::WaybillParam> waybillParams = config.getWaybillParamList();
  BILL_GroupInit(waybillParams);
  billNow = BILL_TakeOne(6);

  while (ros::ok())
{
    // waybill_pos.x = config.getWaybillParamList()[waybill_count].targetPosition.x;
    // waybill_pos.y = config.getWaybillParamList()[waybill_count].targetPosition.y;
    // waybill_pos.z = config.getWaybillParamList()[waybill_count].targetPosition.z;

    waybill_pos.x = billNow.targetPosition.x;
    waybill_pos.y = billNow.targetPosition.y;
    waybill_pos.z = billNow.targetPosition.z;
    // 测试地图服务
    if (cmd_exec_type == 100)
    {
        user_pkg::QueryVoxel srv;
        srv.request.x = 1.0;
        srv.request.y = 2.0;
        srv.request.z = -3.0;

        if (map_client.call(srv))
        {
            if (srv.response.success)
            {
                std::cout << "Query successful:" << std::endl;
                std::cout << "Distance: " << srv.response.voxel.distance << std::endl;
                std::cout << "Current Height: " << srv.response.voxel.cur_height << std::endl;
                std::cout << "Height: " <<  srv.response.voxel.height << std::endl;
                std::cout << "Semantic: " << srv.response.voxel.semantic << std::endl;
            }
            else
            {
                std::cout << "Query failed." << std::endl;
            }
        }
        else
        {
            std::cout << "Failed to call service query_voxel" << std::endl;
        }

        cmd_exec_type = 0;
    }
    // 小车前往上货点
    else if(recevie_panoramic_info && cmd_exec_type == 0)
    {
        user_pkg::UserCmdRequest msg;
        msg.peer_id = config.getPeerId();
        msg.task_guid = config.getGuid();

        msg.type = user_pkg::UserCmdRequest::USER_CMD_CAR_EXEC_ROUTE;
        msg.car_route_info.carSn = carSn;

        car_start_pos = car_physical_status[findCarIndexBySN(car_physical_status, carSn)].pos.position;
    
        msg.car_route_info.way_point.push_back(car_start_pos);
        msg.car_route_info.way_point.push_back(loading_cargo_point);
        msg.car_route_info.yaw = 0.0;   
        std::cout << "Publishing UserCmdRequest message for car1 to load cargo point" << std::endl;
        std::cout << "  car_sn: " << msg.car_route_info.carSn << std::endl;
        std::cout << "  car route info:" << msg.car_route_info;
        cmd_pub.publish(msg);
        cmd_exec_type = 1;
        std::cout << "Waiting for car to loading cargo point in 5 seconds" << std::endl;
        ros::Duration(5.0).sleep();
    }
    // 将drone1绑定在car1
    else if (cmd_exec_type == 1 && car_work_state == user_pkg::CarPhysicalStatus::CAR_READY)
    {
        user_pkg::UserCmdRequest msg;
        msg.peer_id = config.getPeerId();
        msg.task_guid = config.getGuid();

        car_current_pos = car_physical_status[findCarIndexBySN(car_physical_status, carSn)].pos.position;
        if (judgeOnePosition(car_current_pos, loading_cargo_point, 0.5))
        {
            msg.type = user_pkg::UserCmdRequest::USER_CMD_MOVE_DRONE_ON_CAR;
            msg.binding_drone.car_sn = carSn;
            msg.binding_drone.drone_sn = drone_sn[0];
            std::cout << "Publishing UserCmdRequest message for bind drone to car" << std::endl;
            std::cout << "  car_sn: " << msg.binding_drone.car_sn << std::endl;
            std::cout << "  drone_sn: " << msg.binding_drone.drone_sn << std::endl;
            cmd_pub.publish(msg);
            cmd_exec_type = 2;
            std::cout << "Waiting for drone to bind to car in 3 seconds" << std::endl;
            ros::Duration(3.0).sleep();
        }
    }
    // 将货物绑定在drone1
    else if (cmd_exec_type == 2)
    {
        user_pkg::UserCmdRequest msg;
        msg.peer_id = config.getPeerId();
        msg.task_guid = config.getGuid();

        msg.type = user_pkg::UserCmdRequest::USER_CMD_MOVE_CARGO_IN_DRONE;
        // msg.binding_cargo.cargo_id = config.getWaybillParamList()[waybill_count].cargoParam.index;
        msg.binding_cargo.cargo_id = billNow.cargoParam.index;
        msg.binding_cargo.drone_sn = drone_sn[0];
        std::cout << "Publishing UserCmdRequest message for bind cargo to drone" << std::endl;
        std::cout << "  cargo_id: " << msg.binding_cargo.cargo_id << std::endl;
        std::cout << "  drone_sn: " << msg.binding_cargo.drone_sn << std::endl;
        cmd_pub.publish(msg);
        cmd_exec_type = 3;
        std::cout << "Waiting for cargo to bind to drone in 10 seconds" << std::endl;
        ros::Duration(10.0).sleep();
    }
    // 小车载着飞机和货物返回到航空作业区
    else if (cmd_exec_type == 3 && drone_work_state == user_pkg::DronePhysicalStatus::READY)
    {
        user_pkg::UserCmdRequest msg;
        msg.peer_id = config.getPeerId();
        msg.task_guid = config.getGuid();

        msg.type = user_pkg::UserCmdRequest::USER_CMD_CAR_EXEC_ROUTE;
        msg.car_route_info.carSn = carSn;
        
        msg.car_route_info.way_point.push_back(loading_cargo_point);
        msg.car_route_info.way_point.push_back(car_start_pos);
        msg.car_route_info.yaw = 0.0;   
        std::cout << "Publishing UserCmdRequest message for car to return to takeoff point" << std::endl;
        std::cout << "  car_sn: " << msg.car_route_info.carSn << std::endl;
        std::cout << "  car route info:" << msg.car_route_info;
        cmd_pub.publish(msg);
        cmd_exec_type = 4;
        std::cout << "Waiting for car to return to takeoff point in 5 seconds" << std::endl;
        ros::Duration(5.0).sleep();
    }

    else if (cmd_exec_type == 4 && car_work_state == user_pkg::CarPhysicalStatus::CAR_READY)
    {
        static bool firstFlag = true;
        
        if(firstFlag)
        {
            UAV_MapInit();
            deliverRoute1 =new DeliverRoute(1, car_start_pos, waybill_pos, drone_sn, config);
            firstFlag = false;
        }
        deliverRoute1->set_car_ready(5);
        // deliverRoute1->active_uav_line(drone_sn[0], 1, config);
        if(deliverRoute1->route_update(drone_physical_status, cmd_pub))
        {
            cmd_exec_type = 7;

        }


    }


    // 下发航线 飞机前往订单目的地
    // else if (cmd_exec_type == 4 && car_work_state == user_pkg::CarPhysicalStatus::CAR_READY)
    // {
    //     user_pkg::UserCmdRequest msg;
    //     msg.peer_id = config.getPeerId();
    //     msg.task_guid = config.getGuid();

    //     car_current_pos = car_physical_status[findCarIndexBySN(car_physical_status, car_sn[0])].pos.position;
    //     if (judgeOnePosition(car_current_pos, car_start_pos, 0.5))
    //     {
    //         msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_EXEC_ROUTE;
    //         msg.drone_way_point_info.droneSn = drone_sn[0];

    //         user_pkg::DroneWayPoint takeoff_point;
    //         takeoff_point.type = user_pkg::DroneWayPoint::POINT_TAKEOFF;
    //         takeoff_point.timeoutsec = 1000;
    //         msg.drone_way_point_info.way_point.push_back(takeoff_point);

    //         //起飞点
    //         user_pkg::DroneWayPoint takeoff_air_point;
    //         takeoff_air_point.type = user_pkg::DroneWayPoint::POINT_FLYING;
    //         takeoff_air_point.pos.x = car_start_pos.x;
    //         takeoff_air_point.pos.y = car_start_pos.y;
    //         takeoff_air_point.pos.z = -70;
    //         takeoff_air_point.v = 15.0;
    //         takeoff_air_point.timeoutsec = 1000;
    //         msg.drone_way_point_info.way_point.push_back(takeoff_air_point);

    //         user_pkg::DroneWayPoint flying_point1;
    //         flying_point1.type = user_pkg::DroneWayPoint::POINT_FLYING;
    //         flying_point1.pos.x = waybill_pos.x;
    //         flying_point1.pos.y = waybill_pos.y;
    //         flying_point1.pos.z = -100;
    //         flying_point1.v = 15.0;
    //         flying_point1.timeoutsec = 1000;
    //         msg.drone_way_point_info.way_point.push_back(flying_point1);

    //         user_pkg::DroneWayPoint last_flying_point;
    //         last_flying_point.type = user_pkg::DroneWayPoint::POINT_FLYING;
    //         last_flying_point.pos.x = waybill_pos.x;
    //         last_flying_point.pos.y = waybill_pos.y;
    //         last_flying_point.pos.z = waybill_pos.z - 5;
    //         last_flying_point.v = 15.0;
    //         last_flying_point.timeoutsec = 1000;
    //         msg.drone_way_point_info.way_point.push_back(last_flying_point);

    //         user_pkg::DroneWayPoint landing_point;
    //         landing_point.type = user_pkg::DroneWayPoint::POINT_LANDING;
    //         landing_point.timeoutsec = 1000;
    //         msg.drone_way_point_info.way_point.push_back(landing_point);

    //         std::cout << "Publishing UserCmdRequest message for drone to takeoff and flying and land" << std::endl;
    //         std::cout << "  drone_sn: " << msg.drone_way_point_info.droneSn << std::endl;
    //         std::cout << "  drone way point info:" << msg.drone_way_point_info;
    //         cmd_pub.publish(msg);
    //         cmd_exec_type = 5;
    //         std::cout << "Waiting for drone to takeoff and flying and land" << std::endl;
    //     }
    //     std::cout << msg.drone_way_point_info.droneSn << " is delivering cargo to " << waybill_pos.x << " " << waybill_pos.y << " " << waybill_pos.z << std::endl;
    // }
    // // 判断是否到达目的地 卸货 成功后 总分+1
    // else if (cmd_exec_type == 5 && drone_work_state == user_pkg::DronePhysicalStatus::READY)
    // {
    //     user_pkg::UserCmdRequest msg;
    //     msg.peer_id = config.getPeerId();
    //     msg.task_guid = config.getGuid();

    //     //无人机的位置
    //     drone_current_pos = drone_physical_status[findDroneIndexBySN(drone_physical_status, drone_sn[0])].pos.position;
    //     if (judgeOnePosition(drone_current_pos, waybill_pos, 1.0))
    //     {
    //         msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_RELEASE_CARGO;
    //         msg.drone_msg.drone_sn = drone_sn[0];
    //         std::cout << "Publishing UserCmdRequest message for drone to release cargo" << std::endl;
    //         std::cout << "  drone_sn: " << msg.drone_msg.drone_sn << std::endl;
    //         cmd_pub.publish(msg);
    //         cmd_exec_type = 6;
    //         std::cout << "Waiting for drone to release cargo in 5 seconds" << std::endl;
    //         ros::Duration(5.0).sleep();
    //     }
    // }
    // // 下发航线 飞机返回航空作业区（car1）
    // else if (cmd_exec_type == 6 && drone_work_state == user_pkg::DronePhysicalStatus::READY)
    // {
    //     user_pkg::UserCmdRequest msg;
    //     msg.peer_id = config.getPeerId();
    //     msg.task_guid = config.getGuid();

    //     msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_EXEC_ROUTE;
    //     msg.drone_way_point_info.droneSn = drone_sn[0];

    //     user_pkg::DroneWayPoint takeoff_point;
    //     takeoff_point.type = user_pkg::DroneWayPoint::POINT_TAKEOFF;
    //     takeoff_point.timeoutsec = 1000;
    //     msg.drone_way_point_info.way_point.push_back(takeoff_point);

    //     user_pkg::DroneWayPoint takeoff_air_point;
    //     takeoff_air_point.type = user_pkg::DroneWayPoint::POINT_FLYING;
    //     takeoff_air_point.pos.x = waybill_pos.x;
    //     takeoff_air_point.pos.y = waybill_pos.y;
    //     takeoff_air_point.pos.z = -100;
    //     takeoff_air_point.v = 15.0;
    //     takeoff_air_point.timeoutsec = 1000;
    //     msg.drone_way_point_info.way_point.push_back(takeoff_air_point);

    //     user_pkg::DroneWayPoint flying_point1;
    //     flying_point1.type = user_pkg::DroneWayPoint::POINT_FLYING;
    //     flying_point1.pos.x = car_start_pos.x;
    //     flying_point1.pos.y = car_start_pos.y;
    //     flying_point1.pos.z = -100;
    //     flying_point1.v = 15.0;
    //     flying_point1.timeoutsec = 1000;
    //     msg.drone_way_point_info.way_point.push_back(flying_point1);

    //     user_pkg::DroneWayPoint last_flying_point;
    //     last_flying_point.type = user_pkg::DroneWayPoint::POINT_FLYING;
    //     last_flying_point.pos.x = car_start_pos.x;
    //     last_flying_point.pos.y = car_start_pos.y;
    //     last_flying_point.pos.z = -20;
    //     last_flying_point.v = 15.0;
    //     last_flying_point.timeoutsec = 1000;
    //     msg.drone_way_point_info.way_point.push_back(last_flying_point);

    //     user_pkg::DroneWayPoint landing_point;
    //     landing_point.type = user_pkg::DroneWayPoint::POINT_LANDING;
    //     landing_point.timeoutsec = 1000;
    //     msg.drone_way_point_info.way_point.push_back(landing_point);

    //     std::cout << "Publishing UserCmdRequest message for drone to return" << std::endl;
    //     std::cout << "  drone_sn: " << msg.drone_way_point_info.droneSn << std::endl;
    //     std::cout << "  drone way point info:" << msg.drone_way_point_info;
    //     cmd_pub.publish(msg);
    //     cmd_exec_type = 7;
    // }
    // 小车带着飞机前往上货点 
    else if (cmd_exec_type == 7 && drone_work_state == user_pkg::DronePhysicalStatus::READY && car_work_state == user_pkg::CarPhysicalStatus::CAR_READY)
    {
        user_pkg::UserCmdRequest msg;
        msg.peer_id = config.getPeerId();
        msg.task_guid = config.getGuid();

        drone_current_pos = drone_physical_status[findDroneIndexBySN(drone_physical_status, drone_sn[0])].pos.position;
        if (judgeOnePosition(drone_current_pos, car_start_pos, 0.5))
        {
            msg.type = user_pkg::UserCmdRequest::USER_CMD_CAR_EXEC_ROUTE;
            msg.car_route_info.carSn = carSn;

            msg.car_route_info.way_point.push_back(car_start_pos);
            msg.car_route_info.way_point.push_back(loading_cargo_point);
            msg.car_route_info.yaw = 0.0;   
            std::cout << "Publishing UserCmdRequest message for car to load cargo point" << std::endl;
            std::cout << "  car_sn: " << msg.car_route_info.carSn << std::endl;
            std::cout << "  car route info:" << msg.car_route_info;
            cmd_pub.publish(msg);
            cmd_exec_type = 8;
            std::cout << "Waiting for car to load cargo point in 5 seconds" << std::endl;
            ros::Duration(5.0).sleep();
        }
    }
    // 为drone1 换电
    else if (cmd_exec_type == 8 && car_work_state == user_pkg::CarPhysicalStatus::CAR_READY)
    {
        user_pkg::UserCmdRequest msg;
        msg.peer_id = config.getPeerId();
        msg.task_guid = config.getGuid();

        car_current_pos = car_physical_status[findCarIndexBySN(car_physical_status, carSn)].pos.position;
        if (judgeOnePosition(car_current_pos, loading_cargo_point, 0.5))
        {
            msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_BATTERY_REPLACEMENT;
            msg.drone_msg.drone_sn = drone_sn[0];
            std::cout << "Publishing UserCmdRequest message for drone to battery replacement" << std::endl;
            std::cout << "  drone_sn: " << msg.drone_msg.drone_sn << std::endl;
            cmd_pub.publish(msg);
            cmd_exec_type = 9;
            std::cout << "Waiting for drone to battery replacement in 10 seconds" << std::endl;
            ros::Duration(10.0).sleep();
        }
    }
    // 将drone1与car1解绑 drone1回到出生点
    else if (cmd_exec_type == 9 && drone_work_state == user_pkg::DronePhysicalStatus::READY)
    {
        user_pkg::UserCmdRequest msg;
        msg.peer_id = config.getPeerId();
        msg.task_guid = config.getGuid();

        msg.type = user_pkg::UserCmdRequest::USER_CMD_MOVE_DRONE_ON_BIRTHPLACE;
        msg.unbind_info.drone_sn = drone_sn[0];
        msg.unbind_info.car_sn = carSn;
        std::cout << "Publishing UserCmdRequest message for drone to move on birthplace" << std::endl;
        std::cout << "  drone_sn: " << msg.unbind_info.drone_sn << std::endl;
        std::cout << "  car_sn: " << msg.unbind_info.car_sn << std::endl;
        cmd_pub.publish(msg);
        // 状态循环 回到 将drone1绑定到car1的状态 送下一单货物
        cmd_exec_type = 1;
        std::cout << "Begin to deliver next cargo!" << std::endl;
        waybill_count++;
        billNow = BILL_TakeOne(6);
        ros::Duration(5.0).sleep();
    }
    else
    {
        std::cout << "No cmd message to publish" << std::endl;
    }

    ros::spinOnce();
    loop_rate.sleep();
    }
  return 0;
}
