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
         this->tf_listener_depth_footprint.waitForTransform( "/base_footprint","/camera_depth_optical_frame",ros::Time(0), ros::Duration(20.0) );
     }
  catch (tf::TransformException &ex) {
            ROS_ERROR("[adventure_slam]: (wait) %s", ex.what());
            ros::Duration(1.0).sleep();
  }

    try {
        this->tf_listener_depth_footprint.lookupTransform("/base_footprint","/camera_depth_optical_frame", ros::Time(0), (this->vo_fixed_to_base));
     }
    catch (tf::TransformException &ex) {
        ROS_ERROR("[adventure_slam]: (lookup) %s", ex.what());
    }

   try {
         this->tf_listener_odom_footprint.waitForTransform( "/odom","/base_footprint",ros::Time(0), ros::Duration(10.0) );
     }
  catch (tf::TransformException &ex) {
            ROS_ERROR("[adventure_slam]: (wait) %s", ex.what());
            ros::Duration(1.0).sleep();
  }

  //Visualization Publisher will come here
  this->vis_pub = n.advertise<visualization_msgs::MarkerArray>("/slam_debug", 1);

  //nav_msgs/odom publisher will come here
  this->vo_pub = n.advertise<nav_msgs::Odometry>("/vo",1);

  //Initialize the odom_visual to odom Broadcaster
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  br_vo_o.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_visual", "odom"));
}

void LaserScanProcessor::laser_callback(const sensor_msgs::LaserScan& scan)
{
    std::stringstream ss, ss2, ss3, ss4;
    std::string s, s2, s3, s4;

    // tf broadcaster
    //static tf::TransformBroadcaster br_vo_bf;
    //static tf::TransformBroadcaster br_vo_o;
   // Visual Odom message
    nav_msgs::Odometry result_pose;

   // Convert the laserscan to coordinates
    float angle = scan.angle_min;
    std::vector<pcl::PointXYZ> points;
    std::vector<float> ranges = scan.ranges;
    float theta, r; 
    tf::Vector3 v, v_glob, v_glob_vo;

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

    //loc.estimateRotation();
    //loc.estimateTranslation();

    loc.estimate();
    // Debug
    /*ss << loc.matched_pairs.size();
    ss >> s;
    ROS_INFO_STREAM("matched_pairs size: " + s);*/

    ss2 << loc.delta_yaw;
    ss2 >> s2;
    ROS_INFO_STREAM("Delta_Yaw: " + s2);

    /*
    ss3 << loc.shift_x;
    ss3 >> s3;
    ROS_INFO_STREAM("Shift_x: " + s3);

    ss4 << loc.shift_y;
    ss4 >> s4;
    ROS_INFO_STREAM("Shift_y: " + s4);*/

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


    // Should be simply this-
    v_glob.setX(loc.shift_x);
    v_glob.setZ(loc.shift_y);
    v_glob.setY(0);

    // Publishing the other way round - base_footprint to odom
    //v_glob.setX(-loc.shift_y);
    //v_glob.setY(loc.shift_x);
    //v_glob.setZ(0);

    //TODO: Need to transform v_glob to base_footprint - static transform-will be only translation
    // For now robot's pose point is taken to be as the point on the optical depth sensor
    v_glob_vo = (this->vo_fixed_to_base) * v_glob;

    //Correct for the difference in origin
    v_glob_vo.setX(v_glob_vo.getX() - vo_fixed_to_base.getOrigin().x()); 
    v_glob_vo.setY(v_glob_vo.getY() - vo_fixed_to_base.getOrigin().y());
    v_glob_vo.setZ(0);

    ss << v_glob_vo.getX();
    ss >> s;
    ROS_INFO_STREAM("Glob_X " + s);

    ss3 << v_glob_vo.getY();
    ss3 >> s3;
    ROS_INFO_STREAM("Glob_Y: " + s3);

    /*
    ss3 << v_glob_vo.getZ();
    ss3 >> s3;
    ROS_INFO_STREAM("Glob_z: " + s3);*/

    result_pose.pose.pose.position.x = v_glob_vo.getX();
    result_pose.pose.pose.position.y = v_glob_vo.getY();
    result_pose.pose.pose.position.z = 0;

     //tf::Quaternion res = vo_fixed_to_base.getRotation() * tf::createQuaternionFromRPY(0,loc.delta_yaw,0);
     tf::Quaternion res = tf::createQuaternionFromRPY(0, 0, -loc.delta_yaw * M_PI / 180);

    //Publishing the other way round
    //tf::Quaternion res = tf::createQuaternionFromRPY(0, 0, -loc.delta_yaw);
    //tf::quaternionTFToMsg(res, result_pose.pose.pose.orientation);

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
    result_pose.header.frame_id =  "/odom_visual"; 
    result_pose.header.stamp = ros::Time::now();  

    vo_pub.publish(result_pose);   

    tf::Transform base_to_ov;
    base_to_ov.setOrigin(v_glob_vo);
    base_to_ov.setRotation(res);

    //Get the Odometry Stuff
    tf::StampedTransform base_to_odom;
 try {
        this->tf_listener_odom_footprint.lookupTransform("/odom","/base_footprint", ros::Time(0), base_to_odom);
     }
    catch (tf::TransformException &ex) {
        ROS_ERROR("[adventure_slam]: (lookup) %s", ex.what());
    }

    tf::Transform odom_to_ov;
    odom_to_ov = base_to_ov * base_to_odom.inverse();
    br_vo_o.sendTransform(tf::StampedTransform(odom_to_ov, ros::Time::now(), "/odom_visual", "/odom"));
    br_vo_bf.sendTransform(tf::StampedTransform(base_to_ov, ros::Time::now(), "/odom_visual", "/base_footprint"));
   
    //Attach base_footprint to odom_visual directly
    //br_vo_bf.sendTransform(tf::StampedTransform(base_to_ov.inverse(), ros::Time::now(), "/base_footprint", "/odom_visual"));
 
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "adventure_slam");
   ros::NodeHandle n;

   LaserScanProcessor lsp(n);
   ros::spin();

   return 0;
}
