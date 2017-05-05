#include "adventure_slam/Localizer.h"
#include "adventure_slam/Line.h"
#include <cmath>
#include "ros/ros.h"

#define PI 3.14159265

Localizer::Localizer()
{
  this->shift_x = 0;
  this->shift_y = 0; //TODO: Should be shift_z actually
  this->delta_yaw = 0;
}

void Localizer::estimate()
{
  int sz = matched_pairs.size();
  int count = 0;
  double del_yaw = 0;
  for (int i = 0; i < sz; i++)
  {
    LineMatcher::Pair curr_pair = matched_pairs[i];
    if (std::abs(curr_pair.L2.angle - curr_pair.L1.angle) > 35 || std::abs(curr_pair.L2.angle - curr_pair.L1.angle) < 0.01) // False values
    {
      continue;
    }
   del_yaw += (curr_pair.L2.angle - curr_pair.L1.angle);
   count++;
  }  
 
  if (count != 0)
     del_yaw = del_yaw / count;

  delta_yaw += del_yaw;

  if (std::abs(delta_yaw) >= 360)
  {
    delta_yaw = delta_yaw - 360;
  }

  // If it is rotating avoid calculating the shift
  if (std::abs(del_yaw) > 0.5)
    return;
  
  double curr_shift_x = 0, curr_shift_y = 0;
  int i, j, count_y = 0;
  double first_distance, first_angle;
  int size_max = std::min(sz,2);
  for (i = 0; i < size_max ; i++) //sz limited to 2
  {
    //Simple Averaging approach
    LineMatcher::Pair curr_pair = matched_pairs[i];

     if (std::abs(curr_pair.L2.angle - curr_pair.L1.angle) > 35)// False Value
    {
      continue;
    }

    first_distance = curr_pair.L1.distance - curr_pair.L2.distance;

    if (first_distance > 0.5)
	continue;

    first_angle = (curr_pair.L1.angle + curr_pair.L2.angle) / 2;
    first_angle = first_angle * PI / 180;

    curr_shift_y += first_distance * std::sin(first_angle);
    if(!(std::abs(first_angle) < 10 * PI/ 180 || std::abs(first_angle) > 170 * PI / 180 ))
       count_y++;
  }

  if (count_y != 0)
    curr_shift_y = curr_shift_y / count_y;

  estimateGlobalPosition(curr_shift_x, curr_shift_y);

}

void Localizer::estimateGlobalPosition(double curr_shift_x, double curr_shift_y)
{
  if (std::abs(delta_yaw) < 0.1)
     delta_yaw = 0; 
  double curr_ang = delta_yaw * M_PI / 180;
  double glob_curr_shift_x = -(std::cos(curr_ang) * curr_shift_x - std::sin(curr_ang) * curr_shift_y);
  double glob_curr_shift_y = std::sin(curr_ang) * curr_shift_x + std::cos(curr_ang) * curr_shift_y;
  
  shift_x += glob_curr_shift_x;
  shift_y += glob_curr_shift_y;
}

// Utility Methods for Line Intersection Logic - Was not chosen, instead simple averaging gave better results
/*
double Localizer::getXEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle)
{
  return  (sec_dist * std::sin(fir_angle) - fir_dist  * std::sin(sec_angle)) / std::sin(fir_angle - sec_angle);
}

double Localizer::getYEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle)
{
  return (sec_dist * std::cos(fir_angle) - fir_dist * std::cos(sec_angle)) / std::sin(sec_angle - fir_angle);
}*/
