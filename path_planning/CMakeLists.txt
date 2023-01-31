cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
    include
    ../VisiLibity1
    ../Interpolation
    ../OffsetTools
)

add_executable(path_planning_node 
                  src/path_planning.cpp
                  ../Interpolation/src/interpolation.cpp
                  ../VisiLibity1/src/visilibity.cpp
                  ../Interpolation/src/dubins.cpp
                  )
ament_target_dependencies(path_planning_node rclcpp nav_msgs nav2_msgs tf2 tf2_ros rclcpp_action std_srvs rclcpp_components )


install(TARGETS
  path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

