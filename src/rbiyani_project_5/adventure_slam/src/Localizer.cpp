#include "adventure_slam/Localizer.h"
#include "adventure_slam/Line.h"
#include <cmath>

#define PI 3.14159265

Localizer::Localizer()
{
  this->shift_x = 0;
  this->shift_y = 0;
  this->delta_yaw = 0;
}

void Localizer::estimateRotation()
{
  int sz = matched_pairs.size();
  double del_yaw = 0;
  for (int i = 0; i < sz; i++)
  {
    LineMatcher::Pair curr_pair = matched_pairs[i];
    del_yaw += curr_pair.L2.angle - curr_pair.L1.angle;
  }  
 
  if (sz != 0)
     del_yaw = del_yaw / sz;

  delta_yaw += del_yaw;
  if (delta_yaw >= 360)
     delta_yaw = delta_yaw - 360;
  //return delta_yaw;
}

void Localizer::estimateTranslation()
{
  int sz = matched_pairs.size();
  int num_pairs = 0;
  
  double curr_shift_x = 0, curr_shift_y = 0;
  int i, j, count = 0;
  double first_distance, first_angle, second_distance, sec_angle;
  
  for (i = 0; i < sz ; i++)
  {
    LineMatcher::Pair curr_pair = matched_pairs[i];
    first_distance = curr_pair.L1.distance - curr_pair.L2.distance;
    first_angle = (curr_pair.L1.angle + curr_pair.L2.angle) / 2;
    first_angle = first_angle * PI / 180;
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
   }
  
  if (count != 0)    
  { 
    curr_shift_x = curr_shift_x / count;  // TODO: Simple Average For now, should become weighted
    curr_shift_y = curr_shift_y / count;
  }

  shift_x += curr_shift_x;
  shift_y += curr_shift_y;
 
}

double Localizer::getXEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle)
{
  return  (sec_dist * std::sin(fir_angle) - fir_dist  * std::sin(sec_angle)) / std::sin(fir_angle - sec_angle);
}

double Localizer::getYEstimate(double fir_dist, double fir_angle, double sec_dist, double sec_angle)
{
  return (sec_dist * std::cos(fir_angle) - fir_dist * std::cos(sec_angle)) / std::sin(sec_angle - fir_angle);
}
