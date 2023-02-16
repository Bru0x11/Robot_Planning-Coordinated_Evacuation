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
find_package(obstacles_msgs REQUIRED)

include_directories(
    include
    ../VisiLibity1
    ../Interpolation
    ../OffsetTools
    ../obstacles_msgs
    ../CoordinationTask
)

add_executable(path_planning_node 
                  src/path_planning.cpp
                  ../Interpolation/src/interpolation.cpp
                  ../VisiLibity1/src/visilibity.cpp
                  ../Interpolation/src/dubins.cpp
                  ../OffsetTools/src/offsetFunctions.cpp
                  ../CoordinationTask/src/coordination.cpp
                  ../CoordinationTask/src/intersection.cpp
                  ../OffsetTools/include/CPP/Clipper2Lib/src/clipper.engine.cpp 
                  ../OffsetTools/include/CPP/Clipper2Lib/src/clipper.offset.cpp 
                  ../OffsetTools/include/CPP/Clipper2Lib/src/clipper.rectclip.cpp
                  ../OffsetTools/include/CPP/Utils/ClipFileLoad.cpp
                  ../OffsetTools/include/CPP/Utils/ClipFileSave.cpp
                  ../OffsetTools/include/CPP/Utils/clipper.svg.cpp
                  )


ament_target_dependencies(path_planning_node rclcpp nav_msgs nav2_msgs tf2 tf2_ros rclcpp_action std_srvs rclcpp_components obstacles_msgs)


install(TARGETS
  path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

