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
  ROS_INFO("Matched Pairs: %d", sz);
  double del_yaw = 0;
  for (int i = 0; i < sz; i++)
  {
    LineMatcher::Pair curr_pair = matched_pairs[i];
    if (std::abs(curr_pair.L2.angle - curr_pair.L1.angle) > 35 || std::abs(curr_pair.L2.angle - curr_pair.L1.angle) < 0.01) // False values
    {
      /* Debugging
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
  count = 0;
  /*for (i = 0; i < sz ; i++) 
  {
    //Simple Averaging approach
    LineMatcher::Pair curr_pair = matched_pairs[i];

     if (std::abs(curr_pair.L2.angle - curr_pair.L1.angle) > 35 || std::abs(curr_pair.L2.angle - curr_pair.L1.angle) < 0.01) // False values
    {
      if (std::abs(curr_pair.L2.angle - curr_pair.L1.angle) > 35)
      {
       double d2 = curr_pair.L2.distance;
       double d1 = curr_pair.L1.distance;
       ROS_INFO("Bad_Pair(d_2): %lf", d2);
       ROS_INFO("Bad_Pair(d_1): %lf", d1);
      }
      continue;
    }

    first_distance = curr_pair.L1.distance - curr_pair.L2.distance;

    first_angle = (curr_pair.L1.angle + curr_pair.L2.angle) / 2;
    ROS_INFO("first_angle :%f", first_angle);
    first_angle = first_angle * PI / 180;

    double d_x = first_distance * std::cos(first_angle); 
    double d_y = first_distance * std::sin(first_angle);

       //curr_shift_x += first_distance * std::cos(first_angle);  
       //count_x++;
       curr_shift_y += first_distance * std::sin(first_angle);
       if(!(std::abs(first_angle) < 10 * PI/ 180 || std::abs(first_angle) > 170 * PI / 180 ))
          count_y++;
 
   ROS_INFO("curr_d_x: %lf", d_x);
   ROS_INFO("curr_d_y: %lf", d_y);
  }*/

  
    for (int i = 0; i < sz; i++)
    {
	LineMatcher::Pair curr_pair = matched_pairs[i];

     if ((std::abs(curr_pair.L2.angle - curr_pair.L1.angle) > 35) || std::abs(curr_pair.L2.distance - curr_pair.L1.distance) > 10) // False values
    {
      continue;
    }


    if (sz == 1)
    {
      first_distance = curr_pair.L1.distance - curr_pair.L2.distance;
      first_angle = (curr_pair.L1.angle + curr_pair.L2.angle) / 2;
      first_angle = first_angle * PI / 180;
      curr_shift_x += first_distance * std::cos(first_angle);
      curr_shift_y += first_distance * std::sin(first_angle);
      break;
    }
  
    for (j = i+1; j < sz; j ++)
    {
       LineMatcher::Pair sec_pair = matched_pairs[j];

       if ((std::abs(sec_pair.L2.angle - sec_pair.L1.angle) > 35) || std::abs(sec_pair.L2.distance - sec_pair.L1.distance) > 10)// False values
      {
         continue;
      }

      if (std::abs(sec_pair.L1.angle - curr_pair.L1.angle) < 10)
	continue;

      double x_old = getXEstimate(curr_pair.L1.distance, curr_pair.L1.angle * PI / 180, sec_pair.L1.distance, sec_pair.L1.angle * PI / 180);
      double x_new = getXEstimate(curr_pair.L2.distance, curr_pair.L2.angle * PI / 180, sec_pair.L2.distance, sec_pair.L2.angle * PI / 180);
      double y_old = getYEstimate(curr_pair.L1.distance, curr_pair.L1.angle * PI / 180, sec_pair.L1.distance, sec_pair.L1.angle * PI / 180);
      double y_new = getYEstimate(curr_pair.L2.distance, curr_pair.L2.angle * PI / 180, sec_pair.L2.distance, sec_pair.L2.angle * PI / 180);

      ROS_INFO("(x_old): %lf", x_old);
      ROS_INFO("(x_new): %lf", x_new);
      ROS_INFO("(y_old): %lf", y_old);
      ROS_INFO("(y_new ): %lf", y_new );

       curr_shift_x += (x_old - x_new);
       curr_shift_y += (y_old - y_new);

       //ROS_INFO("curr_shift_x: %f", curr_shift_x);
       //ROS_INFO("curr_shift_y: %f", curr_shift_y);
       count++;
    }
   }
  


  /*if (count_x != 0)    
    curr_shift_x = curr_shift_x / count_x;
  if (count_y != 0)
    curr_shift_y = curr_shift_y / count_y;*/

  if (count != 0)
  {
     curr_shift_x = curr_shift_x / count;
     curr_shift_y = curr_shift_y / count;
  }

  estimateGlobalPosition(curr_shift_x, curr_shift_y);

}

void Localizer::estimateGlobalPosition(double curr_shift_x, double curr_shift_y)
{
  if (std::abs(delta_yaw) < 0.1)
     delta_yaw = 0; 
  double curr_ang = delta_yaw * M_PI / 180;
  double glob_curr_shift_x = -(std::cos(curr_ang) * curr_shift_x - std::sin(curr_ang) * curr_shift_y);
  double glob_curr_shift_y = (std::sin(curr_ang) * curr_shift_x + std::cos(curr_ang) * curr_shift_y);
  
  shift_x += glob_curr_shift_x;
  shift_y += glob_curr_shift_y;
}

// Utility Methods for Line Intersection Logic

double Localizer::getXEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle)
{
  return  (sec_dist * std::sin(fir_angle) - fir_dist  * std::sin(sec_angle)) / std::sin(fir_angle - sec_angle);
}

double Localizer::getYEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle)
{
  return (sec_dist * std::cos(fir_angle) - fir_dist * std::cos(sec_angle)) / std::sin(sec_angle - fir_angle);
}
