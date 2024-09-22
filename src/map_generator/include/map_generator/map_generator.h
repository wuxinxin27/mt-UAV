#ifndef _MAP_GENERATOR_H
#define _MAP_GENERATOR_H


#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


#include "map_generator/Astar_searcher.h"
#include "map_generator/JPS_searcher.h"
#include "map_generator/backward.hpp"

using namespace std;
using namespace Eigen;

class Test
{
    public:
        int a[5];
};

class MapGenerator
{
    public:
        bool mapReadFlag = false;
        pcl::PointCloud<pcl::PointXYZL> mapRaw;
        sensor_msgs::PointCloud2 mapRawMsg;

        vector<Vector3d> gridPath;
        vector<Vector3d> pathNodes;

        AstarPathFinder *mapPathFinder = new AstarPathFinder();
        JPSPathFinder *jumpPathFinder = new JPSPathFinder();

    public:
        MapGenerator();
        MapGenerator(const std::string& file_path);
        ~MapGenerator();

        void generate_grid_map(void);
        vector<Vector3d> uav_generate_path(const Vector3d start_pt, const Vector3d target_pt);

};



#endif