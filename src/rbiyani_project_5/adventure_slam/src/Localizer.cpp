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
  //rotating = false;
  int sz = matched_pairs.size();
  int count = 0;
  ROS_INFO("Matched Pairs: %d", sz);
  double del_yaw = 0;
  for (int i = 0; i < sz; i++)
  {
    LineMatcher::Pair curr_pair = matched_pairs[i];
    if (std::abs(curr_pair.L2.angle - curr_pair.L1.angle) > 35 || std::abs(curr_pair.L2.angle - curr_pair.L1.angle) < 0.01) // False values
    {
      /*
      double d2 = curr_pair.L2.distance;
      double d1 = curr_pair.L1.distance;
      double ang2 = curr_pair.L2.angle;
      double ang1 = curr_pair.L1.angle;
      ROS_INFO("Bad_Pair(d_2): %lf", d2);
      ROS_INFO("Bad_Pair(d_1): %lf", d1);
      ROS_INFO("Bad_Pair(ang_2): %lf", ang2);
      ROS_INFO("Bad_Pair(ang_1): %lf", ang1);*/
      continue;
    }
   del_yaw += (curr_pair.L2.angle - curr_pair.L1.angle);
   count++;
  }  
 
  if (count != 0)
     del_yaw = del_yaw / count;

  //if (std::abs(del_yaw) > 0.1)
      delta_yaw += del_yaw;

  if (std::abs(delta_yaw) >= 360)
  {
    delta_yaw = delta_yaw - 360;
  }
  if (std::abs(del_yaw) > 1)
    return;

  int num_pairs = 0;
  
  double curr_shift_x = 0, curr_shift_y = 0;
  int i, j, count_x = 0, count_y = 0;
  double first_distance, first_angle, second_distance, sec_angle;
  int size_max = std::min(sz,2);
  for (i = 0; i < size_max ; i++) //sz limited to 2
  {
    //Simple Averaging approach
    LineMatcher::Pair curr_pair = matched_pairs[i];

    first_distance = curr_pair.L1.distance - curr_pair.L2.distance;

    first_angle = (curr_pair.L1.angle + curr_pair.L2.angle) / 2;
    first_angle = first_angle * PI / 180;

       curr_shift_x += first_distance * std::cos(first_angle);  
       if (std::abs(curr_shift_x) > 0.1)
          count_x++;
       curr_shift_y += first_distance * std::sin(first_angle);
       if(std::abs(curr_shift_y) > 0.1)
          count_y++;
  }

    // Following logic was based on Line Intersection - Didn't work out very well
   /*if (sz == 1)
    {
       curr_shift_x += first_distance * std::cos(first_angle);
       curr_shift_y += first_distance * std::sin(first_angle);
       count++;
       break;
    }
    for (j = i+1; j < sz; j ++)
    {
       LineMatcher::Pair sec_pair = matched_pairs[j];
       second_distance = sec_pair.L1.distance - sec_pair.L2.distance;
       sec_angle = (sec_pair.L1.angle + sec_pair.L2.angle) / 2;
       sec_angle = sec_angle * PI / 180;

       curr_shift_x += getXEstimate(first_distance, first_angle, second_distance, sec_angle);
       curr_shift_y += getYEstimate(first_distance, first_angle, second_distance, sec_angle);
       count++;
    }
   }*/
  
  if (count_x != 0)    
    curr_shift_x = curr_shift_x / count_x;
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

// Utility Methods for Line Intersection Logic
/*
double Localizer::getXEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle)
{
  return  (sec_dist * std::sin(fir_angle) - fir_dist  * std::sin(sec_angle)) / std::sin(fir_angle - sec_angle);
}

double Localizer::getYEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle)
{
  return (sec_dist * std::cos(fir_angle) - fir_dist * std::cos(sec_angle)) / std::sin(sec_angle - fir_angle);
}*/
