#include "adventure_slam/Line.h"
#include <cmath.h>

// function borrowed from https://github.com/ncos/mipt-airdrone/blob/master/src/localization/ransac_slam/advanced_types.cpp
void Line::correct_representation(pcl::ModelCoefficients::Ptr coefficients)
{
    // line direction coordinates in kinect frame:
    double lx = coefficients->values[3]; // x is to the right
    double ly = coefficients->values[5]; // y is forward !FOR KINECT!

    // point on the line in kinect frame:
    double rx = coefficients->values[0]; // x is to the right
    double ry = coefficients->values[2]; // y is forward !FOR KINECT!

    if (rx*ly - ry*lx > 0) {
        lx = - lx;
        ly = - ly;
    }

    coefficients->values[3] = lx;
    coefficients->values[5] = ly;
};

void Line::setParams(pcl::ModelCoefficients::Ptr coefficients)
{
   normalize(coefficients);
   pt_on_line.x = coefficients[0];
   pt_on_line.y = coefficients[1];
   pt_on_line.z = coefficients[2];

   line_dir.x = coefficients[3];
   line_dir.y = coefficients[4];
   line_dir.z = coefficients[5];

   LineUtils::normalize(line_dir);

   // Following Code Borrowed from https://github.com/ncos/mipt-airdrone/blob/master/src/localization/ransac_slam/advanced_types.cpp
   if (line_dir.x == 0) {
        if(line_dir.z < 0) this->angle = -90; // 5 = forward, 3 - to the right
        else this->angle = 90;
    }
    else this->angle = atan(line_dir.z / fabs(line_dir.x))*180.0/M_PI;
    if (this->angle > 0 && line_dir.x < 0) this->angle += 90;
    if (this->angle < 0 && this->line_dir.x < 0) this->angle -= 90;

    this->distance = distance_to_point(pcl::PointXYZ(0, 0, 0));
 
}

double Line::distance_to_point(pcl::PointXYZ p1) {
    pcl::PointXYZ dist;
    dist.x = pt_on_line.x - p1.x;
    dist.y = pt_on_line.y - p1.y;
    dist.z = pt_on_line.z - p1.z;
    pcl::PointXYZ d_vec = LineUtils::cross(dist, line_dir);
    return LineUtils::mag(d_vec);
};

