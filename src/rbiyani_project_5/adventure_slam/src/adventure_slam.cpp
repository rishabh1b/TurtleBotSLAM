#include "adventure_slam/LaserScanProcessor.h"

bool turn_on_visualization;
double matching_line_threshold;


LaserScanProcessor::LaserScanProcessor(ros::NodeHandle n_)
{
  this->scan_sub = n_.subscribe("/scan", 1, &LaserScanProcessor::laser_callback, this);
  this->curr_line_state = LinesCurrentFrame(turn_on_visualization); 
  std::vector<int> v;
  v.push_back(0);
  v.push_back(1);
  v.push_back(2);
  v.push_back(3);
  v.push_back(4);
  v.push_back(5);

  curr_line_state.setColorIndices(v);

  //Visualization Publisher will come here
  this->vis_pub = n.advertise<visualization_msgs::MarkerArray>("/slam_debug", 1);

  //nav_msgs/odom publisher will come here

}

void LaserScanProcessor::laser_callback(const sensor_msgs::LaserScan& scan)
{
    std::stringstream ss;
    std::string s;
   // Convert the laserscan to coordinates
    float angle = scan.angle_min;
    std::vector<pcl::PointXYZ> points;
    std::vector<float> ranges = scan.ranges;
    float theta, r;      

    for (int i = 0; i < ranges.size(); i++ )
    {
	theta = angle;
        angle += scan.angle_increment;
        r = ranges[i];
	pcl::PointXYZ point;
        if ((r > scan.range_max) || (r < scan.range_min))
            continue;
        if (isnan(r))
            continue;
        point.x = (r * std::sin(theta)); //camera_depth_optical_frame has actually Z direction pointing forwards. 
        point.z = (r * std::cos(theta)); //Initially, We rather took y direction pointing forwards. i.e. swap z and y and get all
        point.y = 0;                     //co-ordinates in first two-components of the PCLXYZ object and work with 
                                         //all are equations in X-Y Plane(rather than X-Z). It is better to work in X-Z so that
                                         //RANSAC data can be transformed directly from camera_depth_optical frame to /odom frame
        points.push_back(point);
     }

    std::vector<LineMatcher::Pair> matched_pairs;
    std::vector<Line> old_lines = curr_line_state.lines;
    curr_line_state.update(points);
    //TODO: Visualization controlled via external flag
    vis_pub.publish(curr_line_state.mrkArr);
    //ROS_INFO_STREAM("Able to publish on topic");
    std::vector<Line> new_lines = curr_line_state.lines;

    std::vector<int> indices = LineMatcher::BruteForcePairs(old_lines, new_lines, matched_pairs);
    curr_line_state.setColorIndices(indices);
    //ROS_INFO_STREAM("Able to set Color Indices");
    loc.matched_pairs = matched_pairs;
    // Debug
    ss << loc.matched_pairs.size();
    ss >> s;
    ROS_DEBUG("matched_pairs size: ");
    /*ss << new_lines.size();
    ss >> s;
    ROS_INFO_STREAM("new_lines size: " + s);*/
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "adventure_slam");
   ros::NodeHandle n;
   if (!n.getParam("adventure_slam/turn_on_visualization", turn_on_visualization)) turn_on_visualization = true;
   if (!n.getParam("adventure_slam/matching_line_threshold", matching_line_threshold)) matching_line_threshold = 10;
   LaserScanProcessor lsp(n);
   ros::spin();

   return 0;
}
