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

#include <cmath>
#include <vector>
#include <math.h>
#include <sstream>
#include <string>

ros::Publisher vis_pub;
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ModelCoefficients::Ptr coefficients;
pcl::PointIndices::Ptr inliers;
pcl::ExtractIndices<pcl::PointXYZ> eifilter(true);

visualization_msgs::Marker getLine(std::vector<float> end_pts, std::vector<float> dir, int id_)
{
   visualization_msgs::Marker marker;
   marker.header.frame_id = "camera_depth_frame";
   marker.header.stamp = ros::Time();
   marker.lifetime = ros::Duration(1);
   marker.ns = "slam_debug_ns";
   marker.id = id_;
   marker.type = visualization_msgs::Marker::LINE_STRIP;
   marker.action = visualization_msgs::Marker::ADD;
   marker.pose.position.x = 0;
   marker.pose.position.y = 0;
   marker.pose.position.z = 0;
   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 0.0;
   marker.scale.x = 0.01;
   marker.scale.y = 0.1;
   marker.scale.z = 0.1;
   marker.color.a = 1.0; // Don't forget to set the alpha!
   marker.color.r = 0.0;
   marker.color.g = 1.0;
   marker.color.b = 0.0;

   geometry_msgs::Point p1, p2;
   p1.x = end_pts[0] + 100 * dir[0];
   p1.y = end_pts[1] + 100 * dir[1];
   p1.z = 0;

   p2.x = end_pts[0] - 100 * dir[0];
   p2.y = end_pts[1] - 100 * dir[1];
   p2.z = 0;

   marker.points.push_back(p1);
   marker.points.push_back(p2);
 
   return marker;

}
void laser_callback(const sensor_msgs::LaserScan& scan)
{
   visualization_msgs::MarkerArray mrkArr;
   
   // Convert the laserscan to coordinates
    float angle = scan.angle_min;
    std::vector<pcl::PointXYZ> points;
    std::vector<float> ranges = scan.ranges;
    float theta, r;      

    for (int i = 0; i < ranges.size(); i++ )
    {
	theta = angle;
        angle += scan.angle_increment;
        r = ranges[i];
	pcl::PointXYZ point;
        if ((r > scan.range_max) || (r < scan.range_min))
            continue;
        if (isnan(r))
            continue;
        point.x = (r * std::sin(theta)); //camera_depth_optical_frame has actually Z direction pointing forwards. 
        point.z = (r * std::cos(theta)); //Initially, We took rather y direction pointing forwards. i.e. swap z and y and get all
        point.y = 0;                     //co-ordinates in first two-components of the PCLXYZ object and work with 
                                         //all are equations in X-Y Plane(rather than X-Z). It is better to work in X-Z so that
                                         //RANSAC data can be transformed directly from camera_depth_optical frame to /odom frame
        points.push_back(point);
     }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
 	cloud->points[i] = points[i];
    }

    // Direct Ransac Approach requires use of Eigen Library
    /*
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
    model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
    ransac.setDistanceThreshold (.003);
    ransac.computeModel();
    */
    //std::vector<int> inliers;
    //std::vector<int> model;
    //ransac.getInliers(inliers);
    //ransac.getModelCoefficients(model); // Here -> requires model is returned as Eigen Vector

    coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients ());
    inliers = pcl::PointIndices::Ptr (new pcl::PointIndices ());

    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.003); // TODO: Put a parameter in param server for this
    seg.setOptimizeCoefficients (true);

    int num_points = (int) cloud->points.size ();

    int count = 0;
    std::string s, s2, s3;
    std::stringstream ss, ss2, ss3;
 
    // Filtering Code Inspired From http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    while (cloud->points.size() > 0.001 * num_points)
    {
      count++; 

      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size() <= 20) // TODO: put a parameter in the param server for this
        {
	  break;  
        }    
      
      std::vector<float> end_pt, dir; // TODO: Modification of DS to Array?
      end_pt.push_back(coefficients->values[2]); //model[2]  //Modified from 1->2 see description on camera_depth_optical_frame
      end_pt.push_back(coefficients->values[0]); //model[0] 
      dir.push_back(coefficients->values[5]); //model[5]  //Modified from 4->5 see description on camera_depth_optical_frame
      dir.push_back(coefficients->values[3]); //model[3]
      mrkArr.markers.push_back(getLine(end_pt, dir, count));
   
      eifilter.setInputCloud (cloud);
      eifilter.setIndices (inliers);
      eifilter.setNegative (true);
      eifilter.filter (*cloud);
    }

     //Debug
     /* ss << count;
     ss >> s;   
     ROS_INFO_STREAM(s);
     */
     //ss3 << cloud->header.frame_id;
     //ROS_INFO_STREAM("Frame Id: " + s3);

    vis_pub.publish(mrkArr);

}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "adventure_slam");
   ros::NodeHandle n;
   vis_pub = n.advertise<visualization_msgs::MarkerArray>("/slam_debug", 1);

   ros::Subscriber sub = n.subscribe("/scan", 1, &laser_callback);

   ros::spin();

   return 0;
}
