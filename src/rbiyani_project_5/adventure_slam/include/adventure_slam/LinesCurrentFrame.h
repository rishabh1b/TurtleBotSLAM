#include "adventure_slam/Line.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "visualization_msgs/MarkerArray.h"
#include <vector>
//#include "geometry_msgs/Point.h"


extern double distance_threshold;
extern int minimum_no_inliers;

class LinesCurrentFrame {
private:
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ModelCoefficients::Ptr coefficients;
pcl::PointIndices::Ptr inliers;
pcl::ExtractIndices<pcl::PointXYZ> eifilter;
pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud;

// Visualization stuff
std::vector<int> colorindices;
bool vis_lines; // Parameter to control whether or not to display markers

public:
LinesCurrentFrame(bool visualize = false);
std::vector<Line> lines; //Lines visible currently
void update(std::vector<pcl::PointXYZ> points);
void setColorIndices(std::vector<int> indices);
visualization_msgs::MarkerArray mrkArr;

private:
void renewcloud(std::vector<pcl::PointXYZ> points);
visualization_msgs::Marker getLine(std::vector<float> end_pts, std::vector<float> dir, int id_, int color_ind);
};
