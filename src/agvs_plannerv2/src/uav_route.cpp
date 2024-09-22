#include <agvs_plannerv2/uav_route.h>
#include <map_generator/map_generator.h>

std::vector<TaskParam::WaybillParam> billGroup1, billGroup2, billGroup3, billGroup4, billGroup5, billGroup6;
MapGenerator *mapGenerator;

//前三段路线较相近，2，3，4不准
std::vector<std::vector<int>> getRoutePeriod(uint8_t routeIndex) 
{
    static const std::vector<std::vector<int>> ROUTE1_PERIOD = 
    {
        {23, 82, 22, 260}, {24, 82, 23, 264}, {25, 82, 24, 268}, {26, 90, 24, 285}, {27, 96, 24, 299}
    };
    static const std::vector<std::vector<int>> ROUTE2_PERIOD = 
    {
        {24, 41, 16, 169}, {25, 41, 17, 172}, {26, 42, 18, 177}, {27, 80, 22, 263}, {29, 111, 24, 344}
    };
    static const std::vector<std::vector<int>> ROUTE3_PERIOD = 
    {
        {23, 115, 33, 346}, {24, 115, 34, 350}, {25, 115, 35, 354}, {26, 137, 37, 400}, {27, 167, 29, 454}
    };

    static const std::vector<std::vector<int>> ROUTE4_PERIOD = 
    {
        {23, 78, 24, 264}, {24, 78, 25, 268}, {25, 78, 26, 272}, {26, 101, 27, 313}, {27, 125, 26, 369}
    };

    static const std::vector<std::vector<int>> ROUTE5_PERIOD = 
    {
        {23, 77, 35, 277}, {24, 77, 36, 281}, {25, 77, 37, 285}, {26, 111, 36, 345}, {27, 147, 24, 408}
    };

    static const std::vector<std::vector<int>> ROUTE6_PERIOD = 
    {
        {23, 68, 31, 246}, {24, 68, 31, 250}, {25, 68, 32, 254}, {26, 87, 33, 297}, {27, 107, 34, 341}
    };    
    switch (routeIndex)
    {
    case 1:
        return ROUTE1_PERIOD;
        break;

    case 2:
        return ROUTE2_PERIOD;
        break;

    case 3:
        return ROUTE3_PERIOD;
        break;

    case 4:
        return ROUTE4_PERIOD;
        break;

    case 5:
        return ROUTE5_PERIOD;
        break;
    
    case 6:
        return ROUTE6_PERIOD;
        break;    
    default:
        break;
    }
    return {};
}





void BILL_GroupInit(const std::vector<TaskParam::WaybillParam, std::allocator<TaskParam::WaybillParam>>& billALl)
{
    for (const TaskParam::WaybillParam& bill : billALl) 
    {
        switch (bill.targetPosition.x)
        {
        case 146:
            billGroup1.push_back(bill);
            break;

        case 430:
            billGroup2.push_back(bill);
            break;

        case 528:
            billGroup3.push_back(bill);
            break;
        case 508:
            billGroup4.push_back(bill);
            break;   

        case 564:
            billGroup5.push_back(bill);
            break;     
        case 490:
            billGroup6.push_back(bill);
            break; 
        default:
            printf("订单目的地错误\n");
            break;
        }

    }   
    printf("订单分组:%ld, %ld, %ld, %ld, %ld, %ld\n", billGroup1.size(), billGroup2.size(),
        billGroup3.size(), billGroup4.size(), billGroup5.size(), billGroup6.size());
}

TaskParam::WaybillParam BILL_TakeOne(uint8_t routeIndex)
{
    static uint8_t billCounter[6] = {0}; 
    TaskParam::WaybillParam billTaken = {0};
    switch (routeIndex)
    {
    case 1:
        if(billCounter[0] >= billGroup1.size())
        {
            printf("订单组1已经派完!");
            return billTaken;
        }
        billTaken = billGroup1[billCounter[0]];
        billCounter[0] ++; 
        printf("分配组1订单%d, %d\n", billCounter[0], billTaken.cargoParam.index);   
        break;

    case 2:
        if(billCounter[1] >= billGroup2.size())
        {
            printf("订单组2已经派完!");
            return billTaken;
        }
        billTaken = billGroup2[billCounter[1]];
        billCounter[1] ++; 

        break;
    case 3:
        if(billCounter[2] >= billGroup3.size())
        {
            printf("订单组3已经派完!");
            return billTaken;
        }
        billTaken = billGroup3[billCounter[2]];
        billCounter[2] ++;    
        break;

    case 4:
        if(billCounter[3] >= billGroup4.size())
        {
            printf("订单组4已经派完!");
            return billTaken;
        }
        billTaken = billGroup4[billCounter[3]];
        billCounter[3] ++;    
        break;

    case 5:
        if(billCounter[4] >= billGroup5.size())
        {
            printf("订单组5已经派完!");
            return billTaken;
        }
        billTaken = billGroup5[billCounter[4]];
        billCounter[4] ++;    
        break;
    case 6:
        if(billCounter[5] >= billGroup6.size())
        {
            printf("订单组6已经派完!");
            return billTaken;
        }
        billTaken = billGroup6[billCounter[5]];
        billCounter[5] ++;    
        break;

    default:
        break;
    }

    

    return billTaken;

}


void UAV_MapInit(void)
{
    mapGenerator = new MapGenerator();
}


int findUavIndexBySN(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status, const std::string& sn)
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

bool checkUavPosition(const user_pkg::Position& pos1, const user_pkg::Position& pos2, double threshold)
{
  double distance = sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) + pow(pos1.z - pos2.z, 2));
  if (distance < threshold)
  {
    std::cout << "judgeOnePosition: distance < " << threshold << std::endl;
    return true;
  }
  return false;
}

//检查水平方向位置
bool checkUavHorizPosition(const user_pkg::Position& pos1, const user_pkg::Position& pos2, double threshold)
{
  double distance = sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
  if (distance < threshold)
  {
    std::cout << "judgeHorizPosition: distance < " << threshold << std::endl;
    return true;
  }
  return false;
}

//
DeliverUav::DeliverUav(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status, const std::string& sn)
{
    this->uavSn = sn;
    this->uavIndex = findUavIndexBySN(drone_physical_status, sn);
    if(this->uavIndex >= 0)
    {
        this->activateFlag = true;

    }
    else
    {
        this->activateFlag = false;
    }

}

DeliverUav::DeliverUav(const std::string& sn)
{
    this->uavSn = sn;
    this->activateFlag = false;
}

void DeliverUav::uav_bind_line(uint8_t lineIndex, uint8_t routeIndex)
{
    this->workState = 1;
    this->airLineIndex = lineIndex;
    this->deliverRouteIndex = routeIndex;

}


void DeliverUav::uav_status_update(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status)
{

    this->uavIndex = findUavIndexBySN(drone_physical_status, this->uavSn);
    if(this->uavIndex == -1)
    {
        this->activateFlag = false;
        std::cout << "无法获取UAV-" << this->uavSn << "信息" << std::endl;
        return;
    }

    if(this->airLineIndex == 0 || this->deliverRouteIndex == 0)
    {
        this->workState = 0;
        std::cout << "无人机-" << this->uavSn << "暂未使用" << std::endl;
        // return;
    }

    
    this->uavStatus = drone_physical_status[this->uavIndex];
    // std::cout << "更新" << this->uavIndex << "无人机:" << this->uavStatus << std::endl;


}

void UavAirLine::line_switch_uav(const std::string& uavSn)
{
    this->deliverUav->uavSn = uavSn;

}


//初始化一条无人机路线
UavAirLine::UavAirLine(const std::string& uavSn, uint8_t routeIndex, uint8_t lineIndex, user_pkg::Position takeOffPoint, user_pkg::Position landPoint)
{
    int zDiff = 0;
    //相邻航线高度差
    if(routeIndex % 2 == 0)
    {
        zDiff = 5;

    }
    
    this->deliverUav = new DeliverUav(uavSn);
    this->airLineIndex = lineIndex;
    this->deliverRouteIndex =routeIndex;

    

    this->deliverUav->uav_bind_line(lineIndex, routeIndex);

    this->takeOffPoint = takeOffPoint;
    this->landPoint = landPoint;

    // this->landPoint.z -= 5;

    this->startPoint = takeOffPoint;
    this->startPoint.z = -(75 + 7*lineIndex + zDiff);

    this->endPoint = landPoint;
    this->endPoint.z = -(75 + 7*lineIndex + zDiff);

    Vector3d startPt(this->startPoint.y, this->startPoint.x, -this->startPoint.z);
    Vector3d endPt(this->endPoint.y, this->endPoint.x, -this->endPoint.z);

    this->startPt = startPt;
    this->endPt = endPt;

    // std::cout << "line-" << this->airLineIndex << "-Pt:" << this->takeOffPoint << "~" << this->landPoint << std::endl;


    this->line_generate_waypoints();

    this->uavCargoMsg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_RELEASE_CARGO;
    this->uavCargoMsg.drone_msg.drone_sn = this->deliverUav->uavSn;



    this->takeOffPeriod = ros::Duration(getRoutePeriod(this->deliverRouteIndex)[this->airLineIndex-1][0]);
    this->flyPeriod = ros::Duration(getRoutePeriod(this->deliverRouteIndex)[this->airLineIndex-1][1]);
    this->landPeriod = ros::Duration(getRoutePeriod(this->deliverRouteIndex)[this->airLineIndex-1][2]);
    this->deliverTimePeriod = ros::Duration(getRoutePeriod(this->deliverRouteIndex)[this->airLineIndex-1][3]);


    this->airLineState = 1;
    printf("航线%d线路%d初始化完成\n", this->deliverRouteIndex, this->airLineIndex);
    printf("预计耗时：%.2f, %.2f, %.2f, %.2f\n", this->takeOffPeriod.toSec(), this->flyPeriod.toSec(), this->landPeriod.toSec(), this->deliverTimePeriod.toSec());

}


void UavAirLine::path_to_waypoints(void)
{
    this->uavLineMsg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_EXEC_ROUTE;
    this->uavLineMsg.drone_way_point_info.droneSn = this->deliverUav->uavSn;

    user_pkg::DroneWayPoint wayPoint;
    wayPoint.type = user_pkg::DroneWayPoint::POINT_TAKEOFF;
    wayPoint.timeoutsec = 1000;
    this->uavLineMsg.drone_way_point_info.way_point.push_back(wayPoint);

    //录入第一个点
    wayPoint.type = user_pkg::DroneWayPoint::POINT_FLYING;
    wayPoint.timeoutsec = 1000;
    wayPoint.pos = this->startPoint;
    wayPoint.v = 15.0;
    this->uavLineMsg.drone_way_point_info.way_point.push_back(wayPoint);

    uint8_t trajSize = this->uavPath.size();

    for(int i = 0; i < trajSize; ++ i)
    {
        wayPoint.type = user_pkg::DroneWayPoint::POINT_FLYING;
        wayPoint.timeoutsec = 1000;

        wayPoint.pos.x = this->uavPath[i].y();
        wayPoint.pos.y = this->uavPath[i].x();
        wayPoint.pos.z = -this->uavPath[i].z();
        wayPoint.v = 15.0;
        this->uavLineMsg.drone_way_point_info.way_point.push_back(wayPoint);
    }

    wayPoint.type = user_pkg::DroneWayPoint::POINT_FLYING;
    wayPoint.timeoutsec = 1000;

    wayPoint.pos.x = this->landPoint.x;
    wayPoint.pos.y = this->landPoint.y;
    wayPoint.pos.z = this->landPoint.z - 5;
    wayPoint.v = 15.0;
    this->uavLineMsg.drone_way_point_info.way_point.push_back(wayPoint);

    user_pkg::DroneWayPoint landPoint;
    landPoint.type = user_pkg::DroneWayPoint::POINT_LANDING;
    landPoint.timeoutsec = 1000;
    this->uavLineMsg.drone_way_point_info.way_point.push_back(landPoint);

    std::cout << " 送货航路-" << this->airLineIndex << this->uavLineMsg.drone_way_point_info.droneSn << std::endl;

    if(this->deliverRouteIndex == 2) 
    {
        std::cout << "  航点:" << this->uavLineMsg.drone_way_point_info << std::endl;
    }
    


    //生成返回轨迹点
    vector<Vector3d> returnPath = this->uavPath;

    reverse(returnPath.begin(), returnPath.end());


    this->uavReturnMsg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_EXEC_ROUTE;
    this->uavReturnMsg.drone_way_point_info.droneSn = this->deliverUav->uavSn;

    wayPoint.type = user_pkg::DroneWayPoint::POINT_TAKEOFF;
    wayPoint.timeoutsec = 1000;
    this->uavReturnMsg.drone_way_point_info.way_point.push_back(wayPoint);

    // uint8_t trajSize = this->uavPath.size();

    for(int i = 0; i < trajSize; ++ i)
    {
        wayPoint.type = user_pkg::DroneWayPoint::POINT_FLYING;
        wayPoint.timeoutsec = 1000;

        wayPoint.pos.x = returnPath[i].y();
        wayPoint.pos.y = returnPath[i].x();
        wayPoint.pos.z = -returnPath[i].z();
        wayPoint.v = 15.0;
        this->uavReturnMsg.drone_way_point_info.way_point.push_back(wayPoint);
    }

    //录入去程第一个点
    wayPoint.type = user_pkg::DroneWayPoint::POINT_FLYING;
    wayPoint.timeoutsec = 1000;
    wayPoint.pos = this->startPoint;
    wayPoint.v = 15.0;
    this->uavReturnMsg.drone_way_point_info.way_point.push_back(wayPoint);

    wayPoint.type = user_pkg::DroneWayPoint::POINT_FLYING;
    wayPoint.timeoutsec = 1000;

    wayPoint.pos.x = this->takeOffPoint.x;
    wayPoint.pos.y = this->takeOffPoint.y;
    wayPoint.pos.z = this->takeOffPoint.z - 5;
    wayPoint.v = 15.0;
    this->uavReturnMsg.drone_way_point_info.way_point.push_back(wayPoint);

    user_pkg::DroneWayPoint returnPoint;
    returnPoint.type = user_pkg::DroneWayPoint::POINT_LANDING;
    returnPoint.timeoutsec = 1000;
    this->uavReturnMsg.drone_way_point_info.way_point.push_back(returnPoint);

    // std::cout << " 返程航路: " << this->uavReturnMsg.drone_way_point_info.droneSn << std::endl;
    // std::cout << "  航点:" << this->uavReturnMsg.drone_way_point_info << std::endl;


    // this->airLineState = 1;

}
//返回当前是否需要发布状态MSG
bool UavAirLine::line_state_update(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status)
{
    bool needPubMsg = false;
    if(this->airLineState == 0)
    {
        std::cout << "航线暂未使用" << std::endl;
    }
    
    this->deliverUav->uav_status_update(drone_physical_status);
    // if(this->airLineState == 1 && this->carState && checkUavPosition(this->deliverUav->uavStatus.pos.position, this->takeOffPoint, 4))
    if(this->airLineState == 1 && this->carState)
    {
        //开始起飞，需要发布
        this->airLineState = 2;
        needPubMsg = true;

        printf("航线%d线路%d状态%d准备起飞\n", this->deliverRouteIndex, this->airLineIndex, this->airLineState);

        this->takeOffTime = ros::Time::now();
        this->returnTime = this->takeOffTime + this->deliverTimePeriod;
        this->beginPower = this->deliverUav->uavStatus.remaining_capacity;

        return needPubMsg;
    } 

    // printf("LineState:%d, %d\n", this->airLineState, this->deliverUav->uavStatus.drone_work_state);

    if(this->airLineState == 2 )
    {
        
        //起飞过程中
        if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::TAKEOFF 
        || checkUavHorizPosition(this->deliverUav->uavStatus.pos.position, this->takeOffPoint, 2) )
        {
            this->startWayBusy = true;
        }
        if(!checkUavHorizPosition(this->deliverUav->uavStatus.pos.position, this->takeOffPoint, 2))
        {
            this->startWayBusy = false;
            std::cout << "Taking Off END" << std::endl;
            this->beginLineTime = ros::Time::now();

            this->takeOffPeriod = (this->beginLineTime - this->takeOffTime);
            this->airLineState = 3;

            printf("航线%d线路%d状态%d起飞结束\n", this->deliverRouteIndex, this->airLineIndex, this->airLineState);

        }

        return needPubMsg;
        
    } 
//
    if(this->airLineState == 3 && checkUavHorizPosition(this->deliverUav->uavStatus.pos.position, this->landPoint, 2))
    {
        if(!this->endWayBusy && this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::FLYING)
        {
            this->endWayBusy = true;
            this->endLineTime = ros::Time::now();
            this->flyPeriod = (this->endLineTime - this->beginLineTime);
        }

        if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::LANDING)
        {
            this->endWayBusy = true;
        }
        //降落成功,准备卸货,需要发布
        if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::READY 
            && checkUavPosition(this->deliverUav->uavStatus.pos.position, this->landPoint, 1))
        {
            
            this->landTime = ros::Time::now();
            this->landPeriod = (this->landTime - this->endLineTime);
            needPubMsg = true;
            this->airLineState = 4;
            printf("航线%d线路%d状态%d准备卸货\n", this->deliverRouteIndex, this->airLineIndex, this->airLineState);
        }

        return needPubMsg;

    } 

    if(this->airLineState == 4)
    {

        if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::LOADING_CARGO)
        {
            printf("航线%d线路%d状态%d正在卸货\n", this->deliverRouteIndex, this->airLineIndex, this->airLineState);
        }
        //卸货完毕，准备返航
        if (this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::READY)
        {
            this->airLineState = 5;
            this->lineScore += 1;
            needPubMsg = true;
            printf("航线%d线路%d状态%d准备返航\n", this->deliverRouteIndex, this->airLineIndex, this->airLineState);
        }
        return needPubMsg;
        
        
    }

    if(this->airLineState == 5)
    {
        //回程起飞过程中
        if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::TAKEOFF)
        
        {
            // std::cout << "Return Taking Off" << std::endl;
            this->endWayBusy = true;
        }
        if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::FLYING)
        {
            this->endWayBusy = false;
            this->airLineState = 6;
            printf("航线%d线路%d状态%d返航起飞结束\n", this->deliverRouteIndex, this->airLineIndex, this->airLineState);

        }

        return needPubMsg;
    }

    if(this->airLineState == 6 && checkUavHorizPosition(this->deliverUav->uavStatus.pos.position, this->takeOffPoint, 1))
    {

        // std::cout << "航路-" << this->deliverRouteIndex << "路线-" << this->airLineIndex 
        //         << "无人机-" << this->deliverUav->uavSn << "无人机状态" << std::endl;
        this->startWayBusy = true;
        if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::FLYING && !this->needCarWait)
        {
            
            this->needCarWait = true;
            // this->carState = false;

        }
        //测试未出现landing和takeoff状态
        // if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::LANDING)
        if(checkUavPosition(this->deliverUav->uavStatus.pos.position, this->takeOffPoint, 5))
        {
            if(this->carState)
            {
                this->airLineState = 7;
                // std::cout << "航路-" << this->deliverRouteIndex << "路线-" << this->airLineIndex 
                // << "无人机-" << this->deliverUav->uavSn << "正在接机" << std::endl;
                printf("航线%d线路%d状态%d正在接机\n", this->deliverRouteIndex, this->airLineIndex, this->airLineState);
            }
            else
            {
                std::cout << "航路-" << this->deliverRouteIndex << "路线-" << this->airLineIndex 
                << "无人机-" << this->deliverUav->uavSn << "暂未等到车" << std::endl;
            }
            
        }

        return needPubMsg;

    }

    if(this->airLineState == 7)
    {
        if(this->deliverUav->uavStatus.drone_work_state == user_pkg::DronePhysicalStatus::READY 
            && checkUavPosition(this->deliverUav->uavStatus.pos.position, this->takeOffPoint, 1))
        {
            this->startWayBusy = false;
            //由小车操作
            // this->carState = false;
            this->airLineState = 1;
            this->endPower = this->deliverUav->uavStatus.remaining_capacity;
            this->consumePower = this->beginPower - this->endPower;
            this->check_uav_power();

            this->deliverTimePeriod = (ros::Time::now() - this->takeOffTime);
            printf("航线%d线路%d状态%d完成送货%d\n", this->deliverRouteIndex, this->airLineIndex, this->airLineState, this->lineScore);
            printf("路线-起飞:%.2f 飞行:%.2f 降落:%.2f 总共:%.2f, 耗电:%.2f\n", this->takeOffPeriod.toSec(), this->flyPeriod.toSec(), this->landPeriod.toSec(), 
                            this->deliverTimePeriod.toSec(), this->consumePower);
        }
        return needPubMsg;
        
        
    }


    return needPubMsg;

}

void UavAirLine::check_uav_power(void)
{

    if(this->endPower <= this->consumePower)
    {
        this-> uavPowerLow = true;
    }
    else
    {
        this-> uavPowerLow = false;
    }
    

}

void UavAirLine::line_generate_waypoints(void)
{
    if(!mapGenerator->mapReadFlag)
    {
        std::cout << "无地图生成" << std::endl;
        return;
    }

    this->uavPath = mapGenerator->uav_generate_path(this->startPt, this->endPt);
    this->path_to_waypoints();


}


void UavAirLine::line_get_config(Config raceConfig)
{
    this->uavLineMsg.peer_id = raceConfig.getPeerId();
    this->uavLineMsg.task_guid = raceConfig.getGuid();

    this->uavReturnMsg.peer_id = raceConfig.getPeerId();
    this->uavReturnMsg.task_guid = raceConfig.getGuid();

    this->uavCargoMsg.peer_id = raceConfig.getPeerId();
    this->uavCargoMsg.task_guid = raceConfig.getGuid();


}


DeliverRoute::DeliverRoute(uint8_t routeIndex, user_pkg::Position takeOffPoint, user_pkg::Position landPoint)
{
    this->uavLineList.resize(5);
    if(routeIndex <=0 || routeIndex > 6)
    {
        printf("当前路线序列设置-%d违规(1~6)\n", routeIndex);
        return;
    }

    this->deliverRouteIndex = routeIndex;
    this->takeOffPoint = takeOffPoint;
    this->landPoint = landPoint;


}


DeliverRoute::DeliverRoute(uint8_t routeIndex, user_pkg::Position takeOffPoint, 
user_pkg::Position landPoint, const std::vector<std::string>& droneSns, Config raceConfig)
{
    int lineIndex = 0;
    this->uavLineList.resize(5);
    if(routeIndex <=0 || routeIndex > 6)
    {
        printf("当前航线序列设置-%d违规(1~6)\n", routeIndex);
        return;
    }
    this->deliverRouteIndex = routeIndex;
    this->takeOffPoint = takeOffPoint;
    this->landPoint = landPoint;

    for (const std::string& sn : droneSns) 
    {
        lineIndex ++;
        this->active_uav_line(sn, lineIndex, raceConfig);
    }




    ROS_WARN("linecounter: %d", this->lineCounter);
}


void DeliverRoute::set_car_ready(uint8_t index)
{
    for(int i = 0; i<=4; i++)
    {
        this->carReadyFlag[i] = false;
    }
    
    this->carReadyFlag[index-1] = true;


}

bool DeliverRoute::active_uav_line(std::string uav_sn, uint8_t lineIndex, Config raceConfig)
{
    if(this->lineCounter >= 5)
    {
        printf("当前航线-%d激活路线已满!\n", this->deliverRouteIndex);
        return false;
    }
    if(lineIndex <=0 || lineIndex > 5 || this->airLineFlag[lineIndex - 1])
    {
        // printf("当前航线-%d线路-%d违规或或者已激活!\n", this->deliverRouteIndex, lineIndex);
        return false;
    }
    // printf("初始化航线-%d线路-%d...\n", this->deliverRouteIndex, this->lineCounter);

    UavAirLine *line = new UavAirLine(uav_sn, this->deliverRouteIndex, lineIndex, this->takeOffPoint, this->landPoint);

    this->uavLineList[this->lineCounter] = line;

    this->uavLineList[this->lineCounter]->line_get_config(raceConfig);

    //准备运送
    this->uavLineList[lineCounter]->airLineState = 1;
    // this->uavLineList[lineCounter]->carState = true;

    // printf("test1\n");

    this->airLineFlag[lineCounter] = true;
    this->lineCounter ++;

    printf("初始化航线-%d线路-%d完毕\n", this->deliverRouteIndex, this->lineCounter);

    return true;
}

bool DeliverRoute::route_update(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status, ros::Publisher& msgPub)
{
    bool uavReturn = false;
    if(this->lineCounter == 0)
    {
        printf("航点未分配路线!\n");
        return false;
    }
    uint8_t index = 0;
    while (index < this->lineCounter)
    {
        // if(!this->airLineFlag[index])
        // {
        //     printf("该线路未激活！\n");
        //     break;
        // }
        this->uavLineList[index]->carState = this->carReadyFlag[index];
        if (this->uavLineList[index]->line_state_update(drone_physical_status))
        {
                            
            switch (this->uavLineList[index]->airLineState)
            {
            case 2:
                msgPub.publish(this->uavLineList[index]->uavLineMsg);
                std::cout << "起飞Msg:" << this->uavLineList[index]->uavLineMsg << std::endl;
                ros::Duration(5.0).sleep();
                break;
            case 4:
                std::cout << "卸货Msg:" << this->uavLineList[index]->uavCargoMsg << std::endl;
                msgPub.publish(this->uavLineList[index]->uavCargoMsg);
                break;

            case 5:
                std::cout << "返回Msg:" << this->uavLineList[index]->uavReturnMsg << std::endl;
                msgPub.publish(this->uavLineList[index]->uavReturnMsg);
                break;
            
            default:
                break;
            }
        }

        if(this->uavLineList[index]->startWayBusy)
        {
            this->takeOffBusy = true;
        }

        if(this->uavLineList[index]->endWayBusy)
        {
            this->landBusy = true;
        }

        //需要搬运无人机回去
        if(this->uavLineList[index]->needCarWait && this->uavLineList[index]->airLineState == 1 )
        {
            // cmd_exec_type = 7;
            this->uavLineList[index]->needCarWait = false;
            uavReturn =  true;
        }

        index ++;
    
    }
    return uavReturn;
}

uint8_t DeliverRoute::choose_uav_line()
{
    for(uint8_t i = 1; i <= this->lineCounter; ++i)
    {
        if(this->uavLineList[i - 1]->airLineState == 1)
        {
            return i; 
        }
    }
    return 1;
}
uint8_t DeliverRoute::choose_uav_landing()
{
    ros::Time minReturnTime = this->uavLineList[0]->returnTime; 
    uint8_t index = 1;
    for(uint8_t i = 1; i <= this->lineCounter; ++i)
    {
        if(this->uavLineList[i - 1]->returnTime.isZero())
        {
            continue;
        }
        else
        {
            if(this->uavLineList[i - 1]->returnTime < minReturnTime)
            {
                minReturnTime = this->uavLineList[i - 1]->returnTime;
                index = i;
            }
        }
    }
    return index;
}
bool DeliverRoute::check_add_uav(uint8_t lineIndex)
{
    bool routeFreeFlag = true;
    ros::Time uavAddTime = ros::Time::now();
    ros::Time uavbeginFlyTime, uavBeginLandTime;

    uavbeginFlyTime = uavAddTime + this->uavLineList[lineIndex - 1]->takeOffPeriod;
    uavBeginLandTime = uavbeginFlyTime + this->uavLineList[lineIndex - 1]->flyPeriod;
    
    //安全时间
    ros::Duration safeGapTime(5.0);
    //卸货时间
    ros::Duration deliverTime(5.0);


    ros::Time uavLandTime1, uavTakeOffTime2, uavLandTime2;

    //无无人机起飞，可以添加
    if(this->lineCounter == 0)
    {
        return routeFreeFlag;
    }

    if(this->takeOffBusy)
    {
        routeFreeFlag = false;
        return routeFreeFlag;
    }
    uint8_t index = 0;
    while (index < this->lineCounter)
    {
        if(index == lineIndex - 1)
        {
            index ++;
            break;
        }
        //线路上有无人机
        if(this->uavLineList[index]->airLineState > 1)
        {
            uavLandTime1 = this->uavLineList[index]->takeOffTime + this->uavLineList[index]->takeOffPeriod 
                    + this->uavLineList[index]->flyPeriod  - safeGapTime;
            uavTakeOffTime2 = uavLandTime1 + this->uavLineList[index]->landPeriod + deliverTime + safeGapTime + safeGapTime;
            uavLandTime2 = uavTakeOffTime2 + this->uavLineList[index]->flyPeriod; 

            //在送货点碰撞

            if(uavBeginLandTime <= uavTakeOffTime2)
            {
                return false;
            }
            //起飞遇到回来的
            if(uavbeginFlyTime >= uavLandTime2)
            {
                return false;
            }



        }
        index ++;

    }

    return routeFreeFlag;

}


bool DeliverRoute::check_add_uav(uint8_t lineIndex, ros::Duration carPeriod)
{
    ROS_WARN("%d", lineIndex);
    bool routeFreeFlag = true;
    ros::Time uavAddTime = ros::Time::now() + carPeriod;
    ros::Time uavbeginFlyTime, uavBeginLandTime;
    uavbeginFlyTime = uavAddTime + this->uavLineList[lineIndex - 1]->takeOffPeriod;
    uavBeginLandTime = uavbeginFlyTime + this->uavLineList[lineIndex - 1]->flyPeriod;
    
    //安全时间
    ros::Duration safeGapTime(5.0);
    //卸货时间
    ros::Duration deliverTime(5.0);

    ros::Time uavLandTime1, uavTakeOffTime2, uavLandTime2;

    //无无人机起飞，可以添加
    if(this->lineCounter == 0)
    {
        return routeFreeFlag;
    }

    if(this->takeOffBusy)
    {
        routeFreeFlag = false;
        return routeFreeFlag;
    }
    uint8_t index = 0;
    while (index < this->lineCounter)
    {
        if(index == lineIndex - 1)
        {
            index ++;
            break;
        }
        //线路上有无人机
        if(this->uavLineList[index]->airLineState > 1)
        {
            uavLandTime1 = this->uavLineList[index]->takeOffTime + this->uavLineList[index]->takeOffPeriod 
                    + this->uavLineList[index]->flyPeriod  - safeGapTime;
            uavTakeOffTime2 = uavLandTime1 + this->uavLineList[index]->landPeriod + deliverTime + safeGapTime + safeGapTime;
            uavLandTime2 = uavTakeOffTime2 + this->uavLineList[index]->flyPeriod; 
            //在送货点碰撞

            if(uavBeginLandTime <= uavTakeOffTime2)
            {
                return false;
            }
            //起飞遇到回来的
            if(uavbeginFlyTime >= uavLandTime2)
            {
                return false;
            }



        }
        index ++;

    }

    return routeFreeFlag;

}







