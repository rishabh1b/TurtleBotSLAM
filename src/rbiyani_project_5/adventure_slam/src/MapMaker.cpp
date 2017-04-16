#include "adventure_slam/MapMaker.h"

MapMaker::MapMaker(ros::NodeHandle n_, double origin_x, double origin_y, float resolution, unsigned int size_x, unsigned int size_y, bool use_vo);
{
  this->laser_sub = n_.Subscribe("/scan", 1, &MapMaker::process_scan, this_;
  
  this->occu_pub = n_.advertise<nav_msgs::OccupancyGrid>("/map",10);

  this->origin_x = origin_x;
  this->orgin_y = origin_y;
  this->resolution = resolution;
  this->size_x = size_x;
  this->size_y = size_y;

  if (use_vo)
    fixed_frame = "/odom_visual";
  else
    fixed_frame = "/odom";

    grid.header.frame_id = fixed_frame;
    occ_grid.info.resolution = resolution
    occ_grid.info.width = size_x
    occ_grid.info.height = size_y
    occ_grid.info.origin.position.x = origin_x
    occ_grid.info.origin.position.y = origin_y
    occ_grid.info.origin.orientation.w = 1.0
    for (int i = 0; i < size_x; i++)
    {
       for(int j = 0; j < size_y; j++)
      {
        occ_grid.data.push_back(-1);
      }
    } 

    try {
         this->tf_listener.waitForTransform(fixed_frame,"/base_footprint",ros::Time(0), ros::Duration(10.0) );
     }
  catch (tf::TransformException &ex) {
            ROS_ERROR("[adventure_slam]: (wait) %s", ex.what());
            ros::Duration(1.0).sleep();
       }
 
}

void MapMaker::process_scan(const sensor_msgs::LaserScan& scan)
{
   // Get the Range data and corresponding scan-angles less than range_max and greater than min
    std::vector<float> ranges = scan.ranges;
    int sz = ranges.size();
    std::vector<float> correct_ranges;
    std::vector<float> scanner_angles;
    for(int i = 0; i < sz ; i++)
    {
       if ((ranges[i] > scan.range_max) || (ranges[i] < scan.range_min))
          continue;

       if (isnan(ranges[i]))
            continue;
       
       correct_ranges.push_back(ranges[i]);
       scanner_angles.push_back(scan.angle_min + i * scan.angle_increment);
     }

    // Get the Global Pose
    try {
        this->tf_listener.lookupTransform(fixed_frame, "/base_footprint", ros::Time(0), (this->transform));
      }
 
    catch (tf::TransformException &ex) {
      ROS_ERROR("[adventure_slam]: (lookup) %s", ex.what());
    } 
    
    double curr_state_x = transform.getOrigin().x();
    double curr_state_y = transform.getOrigin().y();
    double curr_state_theta = tf::getYaw(transform.getRotation());
  
    // Fill The Grid Data
    float occ_w_x, occ_w_y, dist_x, dist_y, angle;
    int occ_g_x, occ_g_y, grid_state_x, grid_state_y, occ_ind, free_ind;
    std::vector<int*> free_cells;
  
    sz = correct_ranges.size(); 
    for (int i = 0; i < sz; i++)
    {
        angle = curr_state_theta + scanner_angles[i];
        dist_x = correct_ranges[i] * std::sin(angle);
        dist_y = correct_ranges[i] * std::cos(angle);       
        occ_w_x = curr_state_x + dist_x;
        occ_w_y = curr_state_y + dist_y;

       // Occupied Grid Cell for Bresenham
        to_grid(curr_state_x, curr_state_y, grid_state_x, grid_state_y);
      
       //Grid cell for the laser range
        to_grid(occ_w_x, occ_w_y, occ_g_x, occ_g_y); //TODO: Check whether grid co-ords are outside the size of the map?
        
        occ_ind = to_index(occ_g_x, occ_g_y, this->size_x);
        this->occ_grid.data[occ_ind] = 100;
        free_cells = bresenham(grid_state_x, grid_state_y, occ_g_x,occ_g_y)
        for (int k = 0; k < free_cells.size(), k++)
        {
           free_ind = to_index(free_cells[k][0], free_cells[k][1], this->size_x)
           this->occ_grid.data[free_ind] = 0
        }
    }

    occ_pub.publish(this->occ_grid);

}

void MapMaker::to_grid(double world_x, double world_y, int& grid_x, int& grid_y)
{
  convert_to_grid();
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "map_maker");
   ros::NodeHandle n;

   //TODO: Get Params for Map(like size, resolution) from the parameter server

   MapMaker mm(n);
   ros::spin();
   return 0;
}
