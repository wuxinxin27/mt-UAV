#include <ros/ros.h>
#include "agvs_plannerv2/agvs.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agvs_node");
    ros::NodeHandle nh;

    agvs_ns::agvs agvs_node(nh);
    ros::spin();
    
    return 0;
}