#include "adventure_slam/LinesCurrentFrame.h"

LinesCurrentFrame::LinesCurrentFrame()
{
  this->coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients ());
  this->inliers = pcl::PointIndices::Ptr (new pcl::PointIndices ());

  this->seg.setModelType (pcl::SACMODEL_LINE);
  this->seg.setMethodType (pcl::SAC_RANSAC);
  this->seg.setDistanceThreshold (0.003); // TODO: Put a parameter in param server for this
  this->seg.setOptimizeCoefficients (true);

  this->curr_cloud =  pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  
}

void LinesCurrentFrame::update(std::vector<pcl::PointXYZ> points)
{

    renewcloud(points);
    int num_points = (int) curr_cloud->points.size ();

  // Filtering Code Inspired From http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    while (curr_cloud->points.size() > 0.001 * num_points)
    {
      // count++; For debugging 

      seg.setInputCloud (curr_cloud);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size() <= 20) // TODO: put a parameter in the param server for this
	  break;  
      
      Line l;
      l.setParams(coefficients);
      lines.push_back(l);
   
      eifilter.setInputCloud (curr_cloud);
      eifilter.setIndices (inliers);
      eifilter.setNegative (true);
      eifilter.filter (*curr_cloud);
    }

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
  
}
