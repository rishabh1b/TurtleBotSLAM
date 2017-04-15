
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
catkin_package(
INCLUDE_DIRS include
LIBRARIES LineMatcher LinesCurrentFrame Line LineUtils LaserScanProcessor
)

include_directories(include)
include_directories(	${catkin_INCLUDE_DIRS}
			${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(Line
  src/Line.cpp
)

target_link_libraries(Line LineUtils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(LineMatcher
  src/LineMatcher.cpp
)

target_link_libraries(LineMatcher Line ${catkin_LIBRARIES})

add_library(LinesCurrentFrame
  src/LinesCurrentFrame.cpp
)

target_link_libraries(LinesCurrentFrame Line ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(LineUtils
  src/LineUtils.cpp
)

target_link_libraries(LineUtils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(Localizer
  src/Localizer.cpp
)

target_link_libraries(Localizer Line LineMatcher ${catkin_LIBRARIES})

set(SOURCES src/adventure_slam.cpp)

add_executable(adventure_slam_nd ${SOURCES})
target_link_libraries(adventure_slam_nd	 LineMatcher LinesCurrentFrame Localizer ${catkin_LIBRARIES} 
						                       ${PCL_LIBRARIES})
add_dependencies(adventure_slam_nd ${PROJECT_NAME}_gencpp)

