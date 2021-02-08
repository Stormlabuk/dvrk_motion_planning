cmake_minimum_required(VERSION 3.0.2)
project(dvrk_moveit)

find_package(catkin REQUIRED
        COMPONENTS
        interactive_markers
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_perception
        pluginlib
        geometric_shapes
        pcl_ros
        pcl_conversions
        rosbag
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(single_point_test src/single_point_test.cpp)
target_link_libraries(single_point_test ${catkin_LIBRARIES})
