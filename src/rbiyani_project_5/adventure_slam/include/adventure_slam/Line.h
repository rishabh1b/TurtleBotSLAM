#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#ifndef LINE_H
#define LINE_H
class Line {
public:
double angle;
double distance;
pcl::PointXYZ pt_on_line;
pcl::PointXYZ line_dir;

void setParams(pcl::ModelCoefficients::Ptr coefficients);
double getError(Line l2);

private:
double distance_to_point(pcl::PointXYZ p1);
};

#endif
