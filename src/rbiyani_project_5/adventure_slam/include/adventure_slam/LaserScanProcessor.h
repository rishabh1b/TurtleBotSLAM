#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "adventure_slam/LinesCurrentFrame.h"
#include "adventure_slam/Localizer.h"
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <vector>
#include <math.h>
#include <sstream>
#include <string>

class LaserScanProcessor {
private:
   ros::NodeHandle n;
   ros::Subscriber scan_sub;
   ros::Publisher vis_pub, vo_pub;
   LinesCurrentFrame curr_line_state;
   Localizer loc;
   tf::TransformListener tf_listener_depth_footprint;
   tf::TransformListener tf_listener_odom_footprint;
   tf::TransformBroadcaster br_vo_bf;
   tf::TransformBroadcaster br_vo_o;
   // Transform Visual Odometry Data from camera_depth_frame to base_footprint
   tf::StampedTransform vo_fixed_to_base;

public:
   LaserScanProcessor(ros::NodeHandle n_);
   void laser_callback(const sensor_msgs::LaserScan& scan);

};
