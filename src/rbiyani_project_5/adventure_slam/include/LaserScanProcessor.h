#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include "geometry_msgs/Point.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "adventure_slam/LinesCurrentFrame.h"
#include "adventure_slam/Localizer.h"

#include <cmath>
#include <vector>
#include <math.h>
#include <sstream>
#include <string>

class LaserScanProcessor {
private:
   ros::NodeHandle n;
   ros:Subscriber scan_sub;
   LinesCurrentFrame curr_line_state;
   Localizer loc;

public:
   LaserScanProcessor(ros::NodeHandle n_);
   void laser_callback(const sensor_msgs::LaserScan& scan);

};
