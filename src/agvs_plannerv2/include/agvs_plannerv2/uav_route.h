#ifndef _UAV_ROUTE_H
#define _UAV_ROUTE_H


#include <Eigen/Dense>
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

#include "agvs_plannerv2/Config.h"
#include "ros/ros.h"

using namespace std;
using namespace Eigen;

void UAV_MapInit(void);
void BILL_GroupInit(const std::vector<TaskParam::WaybillParam, std::allocator<TaskParam::WaybillParam>>& billALl);
TaskParam::WaybillParam BILL_TakeOne(uint8_t routeIndex);

//运送无人机
class DeliverUav
{
    public:

        bool activateFlag = false;
        //0 未用 1 小车上 2 起飞 3 来时航线 4 降落货物 5 返程航线 6 降落
        uint8_t workState = 0;
        uint8_t airLineIndex = 0;
        uint8_t deliverRouteIndex = 0;

        //sn号搜索索引值
        std::string uavSn;
        //无人机在全局消息里的索引
        int uavIndex = -1;
        //无人机状态 
        user_pkg::DronePhysicalStatus uavStatus;

    public:
        DeliverUav(const std::string& sn);
        DeliverUav(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status, const std::string& sn);
        ~DeliverUav();

        void uav_bind_line(uint8_t lineIndex, uint8_t routeIndex);
        void uav_status_update(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status);

};

//无人机路线
class UavAirLine
{
    public:

        uint8_t airLineIndex;
        uint8_t deliverRouteIndex;
        //0 空闲 1 等待小车 2 准备起飞 3 送货中 4 卸货 5准备返航 6 返航中 7 接机中 
        uint8_t airLineState = 0;
        //记录线路状态
        bool startWayBusy = false;
        bool endWayBusy = false;
        bool carState = false;
        bool needCarWait = false;
        
        user_pkg::Position startPoint, endPoint;
        user_pkg::Position takeOffPoint, landPoint;
        Vector3d startPt, endPt;

        //起飞降落时间点
        ros::Time takeOffTime, landTime;
        ros::Time beginLineTime, endLineTime;

        ros::Time returnTime;

        //起飞、飞行、降落耗时
        ros::Duration takeOffPeriod, flyPeriod, landPeriod;
        ros::Duration deliverTimePeriod;

        //航线耗电
        bool uavPowerLow = false;
        float beginPower;
        float endPower;
        float consumePower;
        
        //无人机对应的索引
        uint8_t uavUserIndedx;
        DeliverUav *deliverUav;

        //路线得分
        uint16_t lineScore = 0;

        //航线轨迹
        vector<Vector3d> uavPath;
        user_pkg::UserCmdRequest uavLineMsg;
        user_pkg::UserCmdRequest uavCargoMsg;
        user_pkg::UserCmdRequest uavReturnMsg;

    public:

        UavAirLine(const std::string& uavSn, uint8_t routeIndex, uint8_t lineIndex, user_pkg::Position takeOffPoint, user_pkg::Position landPoint);
        ~UavAirLine();

        void line_switch_uav(const std::string& uavSn);

        bool line_state_update(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status);
        //计算运送一单需要时间voi
        uint32_t deliver_time_calib(void);
        void path_to_waypoints(void);
        void line_generate_waypoints(void);
        //判断无人机电量是否支持运送下一单，不够则需要换电
        void check_uav_power(void);

        void line_get_config(Config raceConfig);

};

//运送航线，一条运送路线最多五条无人机路线
class DeliverRoute
{
    public:
        //当前航线索引
        uint8_t deliverRouteIndex = 0;

        uint8_t carLoadIndex = 0;
        //使用的路线数量
        uint8_t lineCounter = 0;

        user_pkg::Position takeOffPoint, landPoint;
        bool airLineFlag[5] = {0};

        bool carReadyFlag[5] = {0};

        bool takeOffBusy = false;
        bool landBusy = false;

        vector<UavAirLine*> uavLineList;



    public:

        DeliverRoute(uint8_t routeIndex, user_pkg::Position takeOffPoint, user_pkg::Position landPoint);
        DeliverRoute(uint8_t routeIndex, user_pkg::Position takeOffPoint, user_pkg::Position landPoint, const std::vector<std::string>& droneSns, Config raceConfig);
        ~DeliverRoute(){};
        bool active_uav_line(std::string uav_sn, uint8_t lineIndex, Config raceConfig);
        void set_car_ready(uint8_t index);
        bool route_update(const std::vector<user_pkg::DronePhysicalStatus>& drone_physical_status, ros::Publisher& msgPub);
        
        bool check_add_uav(uint8_t lineIndex);
        bool check_add_uav(uint8_t lineIndex, ros::Duration carPeriod);
        
        uint8_t choose_uav_line();
        uint8_t choose_uav_landing();

};











#endif
