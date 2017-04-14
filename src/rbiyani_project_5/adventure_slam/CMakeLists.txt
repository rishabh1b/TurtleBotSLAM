
cmake_minimum_required(VERSION 2.8.3)
project(adventure_slam)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
	     geometry_msgs
	     message_generation
             pcl_conversions
             pcl_ros
	     roscpp
	     sensor_msgs
             visualization_msgs
             tf
             tf_conversions
             laser_geometry
             std_msgs)
             

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package()


include_directories(	${catkin_INCLUDE_DIRS}
			${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES adventure_slam.cpp)

add_executable(adventure_slam_nd ${SOURCES})
target_link_libraries(adventure_slam_nd	 	${catkin_LIBRARIES} 
						${PCL_LIBRARIES})
add_dependencies(adventure_slam_nd ${PROJECT_NAME}_gencpp)

