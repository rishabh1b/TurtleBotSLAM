#include "adventure_slam/LinesCurrentFrame.h"
#include "ros/ros.h" // ROS_INFO_STREAM
#include <sstream>
LinesCurrentFrame::LinesCurrentFrame(bool visualize)
{
  this->coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients ());
  this->inliers = pcl::PointIndices::Ptr (new pcl::PointIndices ());

  this->seg.setModelType (pcl::SACMODEL_LINE);
  this->seg.setMethodType (pcl::SAC_RANSAC);
  this->seg.setDistanceThreshold (distance_threshold); 
  this->seg.setOptimizeCoefficients (true);
  this->curr_cloud =  pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  
  this->vis_lines = visualize;
}

void LinesCurrentFrame::update(std::vector<pcl::PointXYZ> points)
{
    renewcloud(points);
    int num_points = (int) curr_cloud->points.size ();

     int count = 0;
  // Filtering Code Inspired From http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    while (curr_cloud->points.size() > 0.001 * num_points)
    {
      count++; 

      seg.setInputCloud (curr_cloud);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size() <= minimum_no_inliers) 
	  break;  
      
      Line l;
      l.setParams(coefficients);
      lines.push_back(l);
   
      eifilter.setInputCloud (curr_cloud);
      eifilter.setIndices (inliers);
      eifilter.setNegative (true);
      eifilter.filter (*curr_cloud);
   
      if (count > 6)
         break;

      if (vis_lines)
     {
       std::vector<float> end_pt, dir;
       end_pt.push_back(coefficients->values[2]); //model[2] //Modified from 1->2 see description on camera_depth_optical_frame
       end_pt.push_back(coefficients->values[0]); //model[0] 
       dir.push_back(coefficients->values[5]); //model[5]  //Modified from 4->5 see description on camera_depth_optical_frame
       dir.push_back(coefficients->values[3]); //model[3]
       mrkArr.markers.push_back(getLine(end_pt, dir, count, colorindices[count - 1]));  
       //ROS_INFO_STREAM("Able to Push markers");
    }

    }
    //if (vis_lines)
     //   vis_pub.publish(mrkArr);

}

void LinesCurrentFrame::renewcloud(std::vector<pcl::PointXYZ> points)
{
  (*curr_cloud).clear();
   curr_cloud->width = points.size();
   curr_cloud->height = 1;
   curr_cloud->is_dense = false;
   curr_cloud->points.resize (curr_cloud->width * curr_cloud->height);

   for (size_t i = 0; i < curr_cloud->points.size (); ++i)
   {
      curr_cloud->points[i] = points[i];
   }

   // Clear the Lines vector & vis markers
      lines.clear();
      mrkArr.markers.clear();
}

void LinesCurrentFrame::setColorIndices(std::vector<int> indices)
{
   //this->colorindices.clear(); 
   for (int i =0 ; i < indices.size(); i++)
       this->colorindices.push_back(indices[i]);
}

visualization_msgs::Marker LinesCurrentFrame::getLine(std::vector<float> end_pts, std::vector<float> dir, int id_, int color_ind)
{
   float col[3];
   switch(color_ind)
  {
    case 0:
        col[0] = 1.0;
        col[1] = 0.0;
        col[2] = 0.0;
        break;
    case 1:
        col[0] = 0.0;
        col[1] = 1.0;
        col[2] = 0.0;
        break;
    case 2:
        col[0] = 0.0;
        col[1] = 0.0;
        col[2] = 1.0;
        break;
    case 3:
        col[0] = 1.0;
        col[1] = 1.0;
        col[2] = 0.0;
        break;
    case 4:
        col[0] = 0.0;
        col[1] = 1.0;
        col[2] = 1.0;
        break;
    case 5:
        col[0] = 1.0;
        col[1] = 0.0;
        col[2] = 1.0;
        break;
  }

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
   
   marker.color.r = col[0];
   marker.color.g = col[1];
   marker.color.b = col[2];

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
