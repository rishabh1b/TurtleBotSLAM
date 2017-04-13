#include <pcl/point_types.h>

class Line {
public:
double angle;
double distance;
pcl::PointXYZ pt_on_line;
pcl::PointXYZ line_dir;

Line();

void setParams(pcl::ModelCoefficients::Ptr coefficients);

private:
void correct_representation(pcl::ModelCoefficients::Ptr coefficients);
double distance_to_point(pcl::PointXYZ p1);
};
