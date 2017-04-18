#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "adventure_slam/geometry.h"
#include "geometry_msgs/Pose.h"

class MapMaker{

private:
ros::Subscriber laser_sub;
ros::Publisher occu_pub;
nav_msgs::OccupancyGrid* occ_grid;
tf::TransformListener tf_listener;
tf::StampedTransform transform;

double origin_x, origin_y;
float resolution;
unsigned int size_x, size_y;
std::string fixed_frame;

public:
MapMaker(ros::NodeHandle n_, double origin_x, double origin_y, float resolution, unsigned int size_x, unsigned int size_y, bool use_vo = false); 
void process_scan(const sensor_msgs::LaserScan& scan);

private:
void to_grid(double world_x, double world_y, int &grid_x, int& grid_y);

};
