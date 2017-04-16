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

  try {
         this->tf_listener.waitForTransform("/camera_depth_optical_frame", "/base_footprint",ros::Time(0), ros::Duration(10.0) );
     }
  catch (tf::TransformException &ex) {
            ROS_ERROR("[adventure_slam]: (wait) %s", ex.what());
            ros::Duration(1.0).sleep();
  }

  try {
        this->tf_listener.lookupTransform("/camera_depth_optical_frame", "/base_footprint", ros::Time(0), (this->vo_fixed_to_base));
  }
 catch (tf::TransformException &ex) {
      ROS_ERROR("[ransac_slam]: (lookup) %s", ex.what());
  }

  //Visualization Publisher will come here
  this->vis_pub = n.advertise<visualization_msgs::MarkerArray>("/slam_debug", 1);

  //nav_msgs/odom publisher will come here
  this->vo_pub = n.advertise<nav_msgs::Odometry>("/vo",1);
}

void LaserScanProcessor::laser_callback(const sensor_msgs::LaserScan& scan)
{
    std::stringstream ss, ss2, ss3, ss4;
    std::string s, s2, s3, s4;

    // tf broadcaster
    static tf::TransformBroadcaster br;
   // Visual Odom message
    nav_msgs::Odometry result_pose;

   // Convert the laserscan to coordinates
    float angle = scan.angle_min;
    std::vector<pcl::PointXYZ> points;
    std::vector<float> ranges = scan.ranges;
    float theta, r; 
    tf::Vector3 v, v_glob;

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

    /*
    geometry_msgs::PointStamped vo_pose, vo_pose_fixed;
    vo_pose.header.frame_id = "/camera_depth_optical_frame"; // TODO: Add Parameter for the link name in Param Server
    vo_pose.header.stamp = ros::Time();
    vo_pose.point.x = loc.shift_x;
    vo_pose.point.y = loc.shift_y;
    vo_pose.point.z = 0; */

     /*
    try {
       tf_listener.transformPoint("/base_footprint", vo_pose, vo_pose_fixed);
    }
    catch(tf::TransformException& ex) {
        ROS_ERROR("[adventure_slam]: Received an exception trying to transform a point from \"asus\" to \"base_footprint\": %s", ex.what());*/

    v.setValue(loc.shift_x, loc.shift_y, 0);
    v_glob = (this->vo_fixed_to_base) * v;

    result_pose.pose.pose.position.x = v_glob.getX();;
    result_pose.pose.pose.position.y = v_glob.getY();
    result_pose.pose.pose.position.z = 0;

    tf::Quaternion res = vo_fixed_to_base.getRotation() * tf::createQuaternionFromRPY(0, 0, loc.delta_yaw);
    tf::quaternionTFToMsg(res, result_pose.pose.pose.orientation);

    // EKF stuff
    /*
    double rotation_err = 1e-4;
    double transl_err   = 1e-2;
    if (matched.size() == 0) rotation_err = 1e6;
    if (matched.size() == 0) transl_err   = 1e6;

    result_pose.pose.covariance =   boost::assign::list_of(transl_err)  (0) (0)  (0)  (0)  (0)
                                                          (0) (transl_err)  (0)  (0)  (0)  (0)
                                                          (0)   (0)  (1e-4) (0)  (0)  (0)
                                                          (0)   (0)   (0) (1e6) (0)  (0)
                                                          (0)   (0)   (0)  (0) (1e6) (0)
                                                          (0)   (0)   (0)  (0)  (0)  (rotation_err);
    result_pose.twist.covariance =  boost::assign::list_of(1e6)  (0) (0)  (0)  (0)  (0)
                                                          (0) (1e6)  (0)  (0)  (0)  (0)
                                                          (0)   (0)  (1e6) (0)  (0)  (0)
                                                          (0)   (0)   (0) (1e6) (0)  (0)
                                                          (0)   (0)   (0)  (0) (1e6) (0)
                                                          (0)   (0)   (0)  (0)  (0)  (1e6);*/

    result_pose.twist.twist.angular.x = 0;
    result_pose.twist.twist.angular.y = 0;
    result_pose.twist.twist.angular.z = 0;
    result_pose.twist.twist.linear.x  = 0;
    result_pose.twist.twist.linear.y  = 0;
    result_pose.twist.twist.linear.z  = 0;
    result_pose.child_frame_id  = "/base_footprint";
    result_pose.header.frame_id = "/camera_depth_optical_frame";
    result_pose.header.stamp = ros::Time::now();  

    vo_pub.publish(result_pose);   

    tf::Transform transform;
    transform.setOrigin( v_glob);
    transform.setRotation(res);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom_visual", "/base_footprint"));
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "adventure_slam");
   ros::NodeHandle n;

   tf_map_to_odom_visual.stamp_ = ros::Time::now();
   tf_map_to_odom_visual.frame_id_ = std::string("map");
   tf_map_to_odom_visual.child_frame_id_ = std::string("odom_visual");

   tf_map_to_odom_visual.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
   tf_map_to_odom_visual.setRotation(tf::createQuaternionFromRPY(0, 0, 0));

   tf_br_.sendTransform(tf_map_to_odom_visual);
   LaserScanProcessor lsp(n);
   ros::spin();

   return 0;
}
