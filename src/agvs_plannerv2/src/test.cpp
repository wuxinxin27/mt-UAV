#include <vector>

#include <ros/ros.h>

#include "user_pkg/PanoramicInfo.h"
#include "user_pkg/CarPhysicalStatus.h"
#include "user_pkg/UserCmdRequest.h"

#include "agvs_plannerv2/agvs.hpp"
 

using namespace agvs_ns;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");
    agvs agvs_test(nh);

    xyyaw pos1(182.0, 422.0, 0.0);
    xyyaw pos2(190.0, 422.0, 0.0);
    xyyaw pos3(198.0, 422.0, 0.0);
    xyyaw pos4(182.0, 432.0, 0.0);
    xyyaw pos5(190.0, 432.0, 0.0);
    xyyaw pos6(198.0, 432.0, 0.0);

    std::vector<xyyaw> targets;

    targets.push_back(pos1);
    targets.push_back(pos2);
    targets.push_back(pos3);
    targets.push_back(pos4);
    targets.push_back(pos5);
    targets.push_back(pos6);


    agvs_test.set_targets(targets);
    agvs_test.move(2.0);


    ros::spin();



    return 0;
}