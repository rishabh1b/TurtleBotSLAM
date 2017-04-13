#include "adventure_slam/LineMatcher.h"
#include "adventure_slam/LineUtils.h"

void LineMatcher::BruteForceMatchingPairs(std::vector<Line> l_old, std::vector<Line> l_new, std::vector<LineMatcher::Pair> &matched)
{
  int sz1 = l_old.size();
  int sz2 = l_new.size();
  
 int i = 0, j = 0;
 double curr_min_threshold;
  for (i = 0; i < sz1; i++)
  {
     curr_min_thresh = 100;
     for (j = 0; j < sz2; j++)
     {
        double err_ = LineUtils::getError(l_new[j], l_old[i]);
        if (err_ < thresh_matching_line && err_ < curr_min_thresh) // TODO: Put a Parameter in the Parameter Server for thresh_matching_lines
           {
              LineMatcher::Pair new_pair(l_new, l_old);
              matched.push(new_pair);
              curr_min_thresh = err_;
              // Some way to remove this pair from further consideration? Should we?
            }
      }

   }

}
