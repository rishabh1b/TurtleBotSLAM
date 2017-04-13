#include "adventure_slam/Line.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
//#include "geometry_msgs/Point.h"

class LinesCurrentFrame {
private:
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ModelCoefficients::Ptr coefficients;
pcl::PointIndices::Ptr inliers;
pcl::ExtractIndices<pcl::PointXYZ> eifilter;
pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud;

public:
LinesCurrentFrame();
std::vector<Line> lines; //Lines visible currently
void update(std::vector<pcl::PointXYZ> points);

private:
void renewcloud(std::vector<pcl::PointXYZ> points);
};
