#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>
#include <json/json.h>

class TaskParam {
public:
    struct Point {
        int x;
        int y;
        int z;
        void LoadFromJson(const Json::Value& value);
    };
    struct DroneLimitConfig {
        int maxCargoSlots;
        int maxFlightRange;
        int maxHoriAcc;
        int maxHoriFlySpeed;
        int maxTakeOffHeight;
        int maxVertAcc;
        int maxVertFlySpeed;
        int maxWeight;
        int minTakeOffHeight;
        void LoadFromJson(const Json::Value& value);
    };
    struct MagvLimitConfig {
        int maxHoriAcc;
        int maxHoriMoveSpeed;
        void LoadFromJson(const Json::Value& value);
    };
    struct MagvParam {
        Point birthplace;
        std::string currentDir;
        std::string guid;
        std::string ip;
        long lastReportTime;
        std::string magvSn;
        std::string mapServerIp;
        int mapServerPort;
        int peerId;
        std::string sceneIp;
        int scenePeerId;
        int scenePort;
        std::string status;
        int taskId;
        int tcpPort;
        int udpPort;
        void LoadFromJson(const Json::Value& value);
    };
    struct DroneParam {
        Point birthplace;
        std::string currentDir;
        std::string droneSn;
        std::string guid;
        std::string ip;
        long lastReportTime;
        std::string mapServerIp;
        int mapServerPort;
        int peerId;
        std::string sceneIp;
        int scenePeerId;
        int scenePort;
        std::string status;
        int taskId;
        int tcpPort;
        int udpPort;
        void LoadFromJson(const Json::Value& value);
    };
    struct BoundaryInfo {
        Point bottomLeft;
        Point bottomRight;
        Point topLeft;
        Point topRight;
        int heightLimit;
        int lowLimit;
        void LoadFromJson(const Json::Value& value);
    };
    struct UnloadingCargoStation {
        int index;
        std::string name;
        Point position;

        void LoadFromJson(const Json::Value& value);
    };
    struct CargoParam {
        Point birthplace;
        int index;
        std::string name;
        float weight;

        void LoadFromJson(const Json::Value& value);
    };
    struct WaybillParam {
        int betterTime;
        CargoParam cargoParam;
        int index;
        Point loadingStationInfo;
        int orderTime;
        Point targetPosition;
        int timeout;

        void LoadFromJson(const Json::Value& value);
    };

    bool batteryConsuming;
    DroneLimitConfig droneLimitConfig;
    std::string guid;
    bool hasObstacles;
    Point loadingCargoPoint;
    BoundaryInfo magvAviationOperationsBoundaryInfo;
    BoundaryInfo magvBoundaryInfo;
    MagvLimitConfig magvLimitConfig;
    std::vector<DroneParam> droneParamList;
    std::vector<MagvParam> magvParamList;
    BoundaryInfo mapBoundaryInfo;
    std::string mapServerIp;
    int mapServerPort;
    int taskId;
    std::vector<UnloadingCargoStation> unloadingCargoStationList;
    std::vector<WaybillParam> waybillParamList;


    void LoadFromJson(const Json::Value& value);
};


class Config {
public:
    std::string controlType;
    uint32_t peerId;
    int32_t port;
    std::string sceneIp;
    TaskParam taskParam;

    void LoadFromJson(const Json::Value& json);
    void print() const;
    
    // 访问器方法
    std::string getControlType() const { return controlType; }
    uint32_t getPeerId() const { return peerId; }
    int32_t getPort() const { return port; }
    std::string getSceneIp() const { return sceneIp; }
    TaskParam getTaskParam() const { return taskParam; }

    bool getBatteryConsuming() const { return taskParam.batteryConsuming; }
    TaskParam::DroneLimitConfig getDroneLimitConfig() const { return taskParam.droneLimitConfig; }
    std::string getGuid() const { return taskParam.guid; }
    bool getHasObstacles() const { return taskParam.hasObstacles; }
    TaskParam::Point getLoadingCargoPoint() const { return taskParam.loadingCargoPoint; }
    TaskParam::BoundaryInfo getMagvAviationOperationsBoundaryInfo() const { return taskParam.magvAviationOperationsBoundaryInfo; }
    TaskParam::BoundaryInfo getMagvBoundaryInfo() const { return taskParam.magvBoundaryInfo; }
    TaskParam::MagvLimitConfig getMagvLimitConfig() const { return taskParam.magvLimitConfig; }
    std::vector<TaskParam::MagvParam> getMagvParamList() const { return taskParam.magvParamList; }
    TaskParam::BoundaryInfo getMapBoundaryInfo() const { return taskParam.mapBoundaryInfo; }
    std::string getMapServerIp() const { return taskParam.mapServerIp; }
    int getMapServerPort() const { return taskParam.mapServerPort; }
    int getTaskId() const { return taskParam.taskId; }
    std::vector<TaskParam::UnloadingCargoStation> getUnloadingCargoStationList() const { return taskParam.unloadingCargoStationList; }
    std::vector<TaskParam::WaybillParam> getWaybillParamList() const { return taskParam.waybillParamList; }
};

#endif // CONFIG_H
 