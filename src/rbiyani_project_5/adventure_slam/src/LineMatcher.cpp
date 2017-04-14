#include "adventure_slam/LineMatcher.h"
#include "adventure_slam/LineUtils.h"
#include <sstream>
#include "ros/ros.h" //For ROS_INFO_STREAM

std::vector<int> LineMatcher::BruteForcePairs(std::vector<Line> l_old, std::vector<Line> l_new, std::vector<LineMatcher::Pair> &matched)
{
  int sz1 = l_old.size();
  int sz2 = l_new.size();
  
  int i = 0, j = 0;
  double curr_min_thresh;
  double thresh_matching_lines = 10; // TODO: Put a Parameter in the Parameter Server for thresh_matching_lines
  std::vector<int> indices;
  std::string s;
  for (i = 0; i < sz1; i++)
  {
     curr_min_thresh = 10;
     for (j = 0; j < sz2; j++)
     {
        double err_ = l_new[j].getError(l_old[i]);
        if (err_ < thresh_matching_lines && err_ < curr_min_thresh) 
           {
              LineMatcher::Pair new_pair(l_new[j], l_old[i]);
              matched.push_back(new_pair);
              curr_min_thresh = err_;
              indices.push_back(i);
              // Some way to remove this pair from further consideration? Should we?
             
              // debug
              std::stringstream ss;
              ss << curr_min_thresh;
              ss >> s;
              ROS_INFO_STREAM("Curr_Minimum_Threshold: " + s);
            }
      }

   }

   return indices;
}
