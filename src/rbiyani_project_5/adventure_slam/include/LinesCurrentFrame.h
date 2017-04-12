#include "Line.h"
#include <vector>

class LinesCurrentFrame {
private:
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ModelCoefficients::Ptr coefficients;
pcl::PointIndices::Ptr inliers;
pcl::ExtractIndices<pcl::PointXYZ> eifilter;

public:
LinesCurrentFrame();
std::vector<Line> lines;
void update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
};
