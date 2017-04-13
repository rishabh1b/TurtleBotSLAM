#include "Line.h"

class LineUtils {

  public:
   static double getError(Line l1_, Line l2_);

   // Add Some Vector Math Stuff here
   static void normalize(pcl::PointXYZ &l);
   static double dot(pcl::PointXYZ l1, pcl::PointXYZ l2);
   static double mag(pcl::PointXYZ l);
   pcl::PointXYZ LineUtils::cross(pcl::PointXYZ l1, pcl::PointXYZ l2);
};
