
#include "map_generator/map_generator.h"

MapGenerator *mapGenerator;

pcl::PointCloud<pcl::PointXYZL>::Ptr mapPointCloud(new pcl::PointCloud<pcl::PointXYZL>);
sensor_msgs::PointCloud2 mapRosMsg;
sensor_msgs::PointCloud2 mapGridVis;

ros::Publisher race_map_pub;
ros::Publisher grid_map_pub;
ros::Publisher _grid_path_vis_pub, _visited_nodes_vis_pub;

ros::Subscriber uav_point_sub;

bool map_flag = false;

AstarPathFinder *_astar_path_finder = new AstarPathFinder();
JPSPathFinder *_jps_path_finder = new JPSPathFinder();
void visGridPath(vector<Vector3d> nodes, bool is_use_jps);
void visVisitedNode(vector<Vector3d> nodes);
void gridMapGenerate(const sensor_msgs::PointCloud2 &pointcloud_map);
void uavPathGenerate(const Vector3d start_pt, const Vector3d target_pt);

double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

Vector3d start_point, end_point;


// pcl::PointXYZ 并不包含语义信息的参数。它仅包含点的三维坐标 (x, y, z)。如果你需要在点云中包含语义信息，可以使用 pcl::PointXYZL
void readPointCloudFromTXT(const std::string& file_path, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud) {
    std::ifstream file(file_path.c_str());
    std::string line;
    pcl::PointXYZL point;

    if (!file.is_open()) 
    {
      std::cerr << "无法打开点云文件: " << file_path << std::endl;
      map_flag = false;
      return;
    }
    else
    {
      while (getline(file, line)) 
      {
         std::stringstream ss(line);
         char delimiter; // 用于处理逗号分隔符
         ss >> point.x >> delimiter >> point.y >> delimiter >> point.z >> delimiter >> point.label;
         if(point.z >= 60 && point.z <= 120)
         {
            // printf("1xyz:%d, %d, %d, %d\n", (int)point.x, (int)point.y, (int)point.z, (int) point.label);
            cloud->push_back(point);

         }

         
      }
        file.close();
        map_flag = true;
    } 
}

void publishRaceMap(void)
{
   if(!map_flag)
      return;
//    race_map_pub.publish(mapRosMsg);
   grid_map_pub.publish(mapGridVis);

   static int counter = 0;
   counter ++;
   if(counter % 10 ==0)
   {

    end_point.y() -= 50;
    if(end_point.y() <= 0)
    {
        end_point.y() = 800;
    }
        mapGenerator->uav_generate_path(start_point, end_point);
        // visGridPath(mapGenerator->uav_generate_path(start_point, end_point), false);
        visGridPath(mapGenerator->gridPath, false);
        visVisitedNode(mapGenerator->pathNodes);
   }
}


//收到随机点云地图信息的回调函数
void gridMapGenerate(const sensor_msgs::PointCloud2 &pointcloud_map) // 接受点云地图
{
    if (!map_flag)
        return;

    pcl::PointCloud<pcl::PointXYZL> cloud;
    pcl::PointCloud<pcl::PointXYZL> cloud_vis;


    pcl::fromROSMsg(pointcloud_map, cloud); // 从收到的消息中转换为具体使用的点云

    if ((int)cloud.points.size() == 0)
        return;

    pcl::PointXYZL pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++) // 能够返回点云即代表是有障碍物
    {
        pt = cloud.points[idx];

        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
        // _jps_path_finder->setObs(pt.x, pt.y, pt.z);

        // for visualize only
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1; // 1 表示二维空间
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, mapGridVis);

    mapGridVis.header.frame_id = "world";


}



void uavPathGenerate(const Vector3d start_pt, const Vector3d target_pt)
{
    //Call A* to search for a path
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
    // Retrieve the path
    auto grid_path = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //Visualize the result
    visGridPath(grid_path, false);
    visVisitedNode(visited_nodes);

    _astar_path_finder->resetUsedGrids();
}


void visGridPath(vector<Vector3d> nodes, bool is_use_jps)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    if (is_use_jps)
        node_vis.ns = "map_generator/jps_path";
    else
        node_vis.ns = "map_generator/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if (is_use_jps)
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode(vector<Vector3d> nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "map_generator/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}



MapGenerator::MapGenerator()
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr mapRaw(new pcl::PointCloud<pcl::PointXYZL>);
    // readPointCloudFromTXT("/home/xxy/Development/Competition/MTuav/xxy_ws/src/map_generator/map_utmm/voxel.txt", mapRaw);
    readPointCloudFromTXT("/home/sdk_for_user/map_utmm/voxel.txt", mapRaw);
    this->mapRaw = *mapRaw;
    this->mapReadFlag = map_flag;
    std::cout << "点云数量" <<  this->mapRaw.points.size() << std::endl;
    pcl::toROSMsg(this->mapRaw, this->mapRawMsg);

    this->mapRawMsg.header.frame_id = "map";

    //地图边界大小
    _x_size = 1652;
    _y_size = 1400;
    _z_size = 60;
    _resolution = 15;

    _map_lower << 0, 0, 60;
    _map_upper << 1652, 1400, 120;

    start_point << 300, 300, 65;
    end_point << 0, 500, 65;

    _inv_resolution = 1.0 / _resolution;

    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    this->mapPathFinder = new AstarPathFinder();
    this->mapPathFinder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    this->jumpPathFinder = new JPSPathFinder();
    this->jumpPathFinder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    this->generate_grid_map();

}


vector<Vector3d> MapGenerator::uav_generate_path(const Vector3d start_pt, const Vector3d target_pt)
{

    //Call A* to search for a path
    this->mapPathFinder->AstarGraphSearch(start_pt, target_pt);
    // Retrieve the path
    auto grid_path = this->mapPathFinder->getPath();
    auto visited_nodes = this->mapPathFinder->getVisitedNodes();
    this->mapPathFinder->resetUsedGrids();

    this->gridPath = grid_path;
    this->pathNodes = visited_nodes;
    //换成跳点搜索
    // this->jumpPathFinder->JPSGraphSearch(start_pt, target_pt);

    // this->gridPath = this->jumpPathFinder->getPath();
    // this->pathNodes = this->jumpPathFinder->getVisitedNodes();

    // this->jumpPathFinder->resetUsedGrids();

    return this->gridPath;


}

void MapGenerator::generate_grid_map(void)
{
    if (!this->mapReadFlag)
    return;

    pcl::PointCloud<pcl::PointXYZL> cloud;
    pcl::PointCloud<pcl::PointXYZL> cloud_vis;


    cloud = this->mapRaw;

    if ((int)cloud.points.size() == 0)
    {
        std::cerr << "无点云地图！" << std::endl;
        return;
    }
        

    pcl::PointXYZL pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++) // 能够返回点云即代表是有障碍物
    {
        pt = cloud.points[idx];

        this->mapPathFinder->setObs(pt.x, pt.y, pt.z);
        this->jumpPathFinder->setObs(pt.x, pt.y, pt.z);

        Vector3d cor_round = this->mapPathFinder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }


    //只做可视化用
    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1; // 1 表示二维空间
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, mapGridVis);
    mapGridVis.header.frame_id = "world";

}


int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "map_node");
   ros::NodeHandle n( "~" );

   race_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1); 
   grid_map_pub   = n.advertise<sensor_msgs::PointCloud2>("grid_map", 1);  

   _grid_path_vis_pub = n.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
   _visited_nodes_vis_pub = n.advertise<visualization_msgs::Marker>("visited_nodes_vis", 1);
 
    mapGenerator = new MapGenerator();


   ros::Rate loop_rate(10);
   ROS_INFO("MAP_GENERATOR INIT SUCCESS\n");

   while (ros::ok())
   {
      publishRaceMap();
      ros::spinOnce();
      loop_rate.sleep();
   }

}

