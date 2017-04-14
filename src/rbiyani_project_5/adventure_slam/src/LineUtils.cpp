#include "adventure_slam/LineUtils.h"
#include <cmath>

void LineUtils::normalize(pcl::PointXYZ &l)
{
  double norm = mag(l);
  l.x = l.x / norm;
  l.y = l.y / norm;
  l.z = l.z / norm;
}

double LineUtils::dot(pcl::PointXYZ l1, pcl::PointXYZ l2)
{ 
  return l1.x * l2.x + l1.y * l2.y + l1.z*l2.z;
}

double LineUtils::mag(pcl::PointXYZ l)
{
  return std::sqrt(dot(l, l));
} 

pcl::PointXYZ LineUtils::cross(pcl::PointXYZ l1, pcl::PointXYZ l2) {
    pcl::PointXYZ result;
    result.x = l1.y * l2.z - l1.z * l2.y;
    result.y = l1.z * l2.x - l1.x * l2.z;
    result.z = l1.x * l2.y - l1.y * l2.x;
    return result;
};
