// src/Config.cpp

#include "agvs_plannerv2/Config.h"
#include <iostream>

void TaskParam::Point::LoadFromJson(const Json::Value& value) {
    x = value["x"].asInt();
    y = value["y"].asInt();
    z = value["z"].asInt();
}

void TaskParam::DroneLimitConfig::LoadFromJson(const Json::Value& value) {
    maxCargoSlots = value["maxCargoSlots"].asInt();
    maxFlightRange = value["maxFlightRange"].asInt();
    maxHoriAcc = value["maxHoriAcc"].asInt();
    maxHoriFlySpeed = value["maxHoriFlySpeed"].asInt();
    maxTakeOffHeight = value["maxTakeOffHeight"].asInt();
    maxVertAcc = value["maxVertAcc"].asInt();
    maxVertFlySpeed = value["maxVertFlySpeed"].asInt();
    maxWeight = value["maxWeight"].asInt();
    minTakeOffHeight = value["minTakeOffHeight"].asInt();
}

void TaskParam::MagvLimitConfig::LoadFromJson(const Json::Value& value) {
    maxHoriAcc = value["maxHoriAcc"].asInt();
    maxHoriMoveSpeed = value["maxHoriMoveSpeed"].asInt();
}

void TaskParam::MagvParam::LoadFromJson(const Json::Value& value) {
    birthplace.LoadFromJson(value["birthplace"]);
    currentDir = value["currentDir"].asString();
    guid = value["guid"].asString();
    ip = value["ip"].asString();
    lastReportTime = value["lastReportTime"].asInt64();
    magvSn = value["magvSn"].asString();
    mapServerIp = value["mapServerIp"].asString();
    mapServerPort = value["mapServerPort"].asInt();
    peerId = value["peerId"].asInt();
    sceneIp = value["sceneIp"].asString();
    scenePeerId = value["scenePeerId"].asInt();
    scenePort = value["scenePort"].asInt();
    status = value["status"].asString();
    taskId = value["taskId"].asInt();
    tcpPort = value["tcpPort"].asInt();
    udpPort = value["udpPort"].asInt();
}

void TaskParam::DroneParam::LoadFromJson(const Json::Value& value) {
    birthplace.LoadFromJson(value["birthplace"]);
    currentDir = value["currentDir"].asString();
    guid = value["guid"].asString();
    ip = value["ip"].asString();
    lastReportTime = value["lastReportTime"].asInt64();
    droneSn = value["droneSn"].asString();
    mapServerIp = value["mapServerIp"].asString();
    mapServerPort = value["mapServerPort"].asInt();
    peerId = value["peerId"].asInt();
    sceneIp = value["sceneIp"].asString();
    scenePeerId = value["scenePeerId"].asInt();
    scenePort = value["scenePort"].asInt();
    status = value["status"].asString();
    taskId = value["taskId"].asInt();
    tcpPort = value["tcpPort"].asInt();
    udpPort = value["udpPort"].asInt();
}

void TaskParam::BoundaryInfo::LoadFromJson(const Json::Value& value) {
    bottomLeft.LoadFromJson(value["bottomLeft"]);
    bottomRight.LoadFromJson(value["bottomRight"]);
    topLeft.LoadFromJson(value["topLeft"]);
    topRight.LoadFromJson(value["topRight"]);
    heightLimit = value.get("heightLimit", 0).asInt();
    lowLimit = value.get("lowLimit", 0).asInt();
}

void TaskParam::UnloadingCargoStation::LoadFromJson(const Json::Value& value) {
    index = value["index"].asInt();
    name = value["name"].asString();
    position.LoadFromJson(value["position"]);
}

void TaskParam::CargoParam::LoadFromJson(const Json::Value& value) {
    birthplace.LoadFromJson(value["birthplace"]);
    index = value["index"].asInt();
    name = value["name"].asString();
    weight = value["weight"].asFloat();
}

void TaskParam::WaybillParam::LoadFromJson(const Json::Value& value) {
    betterTime = value["betterTime"].asInt();
    cargoParam.LoadFromJson(value["cargoParam"]);
    index = value["index"].asInt();
    loadingStationInfo.LoadFromJson(value["loadingStationInfo"]);
    orderTime = value["orderTime"].asInt();
    targetPosition.LoadFromJson(value["targetPosition"]);
    timeout = value["timeout"].asInt();
}

void TaskParam::LoadFromJson(const Json::Value& value) {
    batteryConsuming = value["batteryConsuming"].asBool();
    droneLimitConfig.LoadFromJson(value["droneLimitConfig"]);
    guid = value["guid"].asString();
    hasObstacles = value["hasObstacles"].asBool();
    loadingCargoPoint.LoadFromJson(value["loadingCargoPoint"]);
    magvAviationOperationsBoundaryInfo.LoadFromJson(value["magvAviationOperationsBoundaryInfo"]);
    magvBoundaryInfo.LoadFromJson(value["magvBoundaryInfo"]);
    magvLimitConfig.LoadFromJson(value["magvLimitConfig"]);

    for (const auto& param : value["magvParamList"]) {
        MagvParam magvParam;
        magvParam.LoadFromJson(param);
        magvParamList.push_back(magvParam);
    }

    for (const auto& param : value["droneParamList"]) {
        DroneParam droneParam;
        droneParam.LoadFromJson(param);
        droneParamList.push_back(droneParam);
    }

    for (const auto& param : value["waybillParamList"]) {
        WaybillParam waybillParam;
        waybillParam.LoadFromJson(param);
        waybillParamList.push_back(waybillParam);
    }
    mapBoundaryInfo.LoadFromJson(value["mapBoundaryInfo"]);
    mapServerIp = value["mapServerIp"].asString();
    mapServerPort = value["mapServerPort"].asInt();
    taskId = value["taskId"].asInt();
}

void Config::LoadFromJson(const Json::Value& json) {
    controlType = json["controlType"].asString();
    peerId = json["peerId"].asInt();
    port = json["port"].asInt();
    sceneIp = json["sceneIp"].asString();
    taskParam.LoadFromJson(json["taskParam"]);
}

void Config::print() const {
    std::cout << "Control Type: " << controlType << std::endl;
    std::cout << "Peer ID: " << peerId << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "Scene IP: " << sceneIp << std::endl;
    std::cout << "Task Param:" << std::endl;
    std::cout << "  Battery Consuming: " << taskParam.batteryConsuming << std::endl;
    std::cout << "  Map Server IP: " << taskParam.mapServerIp << std::endl;
    std::cout << "  Map Server Port: " << taskParam.mapServerPort << std::endl;
    std::cout << "  Task ID: " << taskParam.taskId << std::endl;
}
