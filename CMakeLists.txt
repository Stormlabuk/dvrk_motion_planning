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

add_executable(basic_io src/basic_io.cpp)
target_link_libraries(basic_io ${catkin_LIBRARIES})

add_executable(basic_planning src/basic_planning.cpp)
target_link_libraries(basic_planning ${catkin_LIBRARIES})

add_executable(ros_planning src/ros_planning.cpp)
target_link_libraries(ros_planning ${catkin_LIBRARIES})

add_executable(move_planning src/move_planning.cpp)
target_link_libraries(move_planning ${catkin_LIBRARIES})
