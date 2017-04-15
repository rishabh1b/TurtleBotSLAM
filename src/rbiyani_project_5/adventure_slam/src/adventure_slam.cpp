#include "adventure_slam/LaserScanProcessor.h"

LaserScanProcessor::LaserScanProcessor(ros::NodeHandle n_)
{
  this->scan_sub = n_.subscribe("/scan", 1, &LaserScanProcessor::laser_callback, this);
  this->curr_line_state = LinesCurrentFrame(true); //TODO: A boolean in parameter server to turn visualization on or off

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
    std::stringstream ss, ss2, ss3, ss4;
    std::string s, s2, s3, s4;
   // Convert the laserscan to coordinates
    float angle = scan.angle_min;
    std::vector<pcl::PointXYZ> points;
    std::vector<float> ranges = scan.ranges;
    float theta, r; 

    //double shift_x, shift_y, delta_yaw;     

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

    loc.estimateRotation();
    loc.estimateTranslation();

    // Debug
    ss << loc.matched_pairs.size();
    ss >> s;
    ROS_INFO_STREAM("matched_pairs size: " + s);

    ss2 << loc.delta_yaw;
    ss2 >> s2;
    ROS_INFO_STREAM("Delta_Yaw: " + s2);

    ss3 << loc.shift_x;
    ss3 >> s3;
    ROS_INFO_STREAM("Shift_x: " + s3);

    ss4 << loc.shift_y;
    ss4 >> s4;
    ROS_INFO_STREAM("Shift_y: " + s4);
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "adventure_slam");
   ros::NodeHandle n;

   LaserScanProcessor lsp(n);
   ros::spin();

   return 0;
}
