#include <pcl/point_types.h>

#ifndef LINE_UTILS_H
#define LINE_H
class LineUtils {

  public:

   // Add Some Vector Math Stuff here
   static void normalize(pcl::PointXYZ &l);
   static double dot(pcl::PointXYZ l1, pcl::PointXYZ l2);
   static double mag(pcl::PointXYZ l);
   static pcl::PointXYZ cross(pcl::PointXYZ l1, pcl::PointXYZ l2);
};
#endif
