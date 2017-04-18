#include "adventure_slam/MapMaker.h"

MapMaker::MapMaker(ros::NodeHandle n_, double origin_x, double origin_y, float resolution, unsigned int size_x, unsigned int size_y, bool use_vo)
{
  this->laser_sub = n_.subscribe("/scan", 10, &MapMaker::process_scan, this);
  
  this->occu_pub = n_.advertise<nav_msgs::OccupancyGrid>("/map",10);

  this->origin_x = origin_x;
  this->origin_y = origin_y;
  this->resolution = resolution;
  this->size_x = size_x;
  this->size_y = size_y;

  if (use_vo)
    fixed_frame = "/odom_visual";
  else
    fixed_frame = "/odom";

  //Initialize Occupancy Grid
    occ_grid = new nav_msgs::OccupancyGrid();
    occ_grid->info.width = size_x;
    occ_grid->info.height = size_y;
    occ_grid->header.frame_id = fixed_frame;
    occ_grid->info.resolution = resolution;
    occ_grid->info.origin.position.x = origin_x;
    occ_grid->info.origin.position.y = origin_y;
    occ_grid->info.origin.orientation.w = 1.0;
    
    char* data = new char[size_x*size_y];
    for (int i = 0; i < size_x*size_y; i++)
         data[i] = -1; //initialise to -1

    occ_grid->data = std::vector<int8_t>(data, data + size_x*size_y);
    ROS_INFO("Cell Count: %d", size_x*size_y);

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
    std::vector<mypoint> free_cells;
  
    sz = correct_ranges.size(); 
    // Occupied Grid Cell for Bresenham
     to_grid(curr_state_x, curr_state_y, grid_state_x, grid_state_y);

     if (isnan(grid_state_x) || isnan(grid_state_y))
     {
       occu_pub.publish(*(this->occ_grid));
       return;
      }

    for (int i = 0; i < sz; i++)
    {
        angle = curr_state_theta + scanner_angles[i];
        dist_x = correct_ranges[i] * std::sin(angle);
        dist_y = correct_ranges[i] * std::cos(angle);       
        occ_w_x = curr_state_x + dist_x;
        occ_w_y = curr_state_y + dist_y;

        //Grid cell for the laser range
        to_grid(occ_w_x, occ_w_y, occ_g_x, occ_g_y); 
        if (isnan(occ_g_x) || isnan(occ_g_y))
            continue;
            
       // Mark free cells as 0
        free_cells = bresenham(grid_state_x, grid_state_y, occ_g_x, occ_g_y);
        for (int k = 0; k < free_cells.size(); k++)
        {
           mypoint curr_point = free_cells[k];
           free_ind = to_index(curr_point.x, curr_point.y, this->size_x);
           if (free_ind >=0 && free_ind < size_x * size_y)
               this->occ_grid->data[free_ind] = 0;
        }
        
        // Mark Occupied cells as 100
        occ_ind = to_index(occ_g_x, occ_g_y, this->size_x);
        if (occ_ind >=0 && occ_ind < size_x * size_y)
             this->occ_grid->data[occ_ind] = 100;
    }
    occu_pub.publish(*(this->occ_grid));
}

void MapMaker::to_grid(double world_x, double world_y, int& grid_x, int& grid_y)
{
  //int grid_x_new, grid_y_new;
  convert_to_grid(grid_x , grid_y, world_x, world_y, this->origin_x, this->origin_y, this->size_x, this->size_y, this->resolution);
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "map_maker");
   ros::NodeHandle n;

   //TODO: Get Params for Map(like size, resolution) from the parameter server

   MapMaker mm(n, -20, -20, 0.5, 100, 100); //Passing dummy params for now
   ros::spin();
   return 0;
}
