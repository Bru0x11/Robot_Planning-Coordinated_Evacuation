cmake_minimum_required(VERSION 3.8)
project(OffsetTools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
    include
    ../VisiLibity1
    include/CPP/src/
    include/CPP/Utils/
)

add_executable(offset_node
 src/offsetFunctions.cpp
 clipper.engine.cpp 
 clipper.offset.cpp 
 clipper.rectclip.cpp
 ClipFileLoad.cpp
 ClipFileSave.cpp
 clipper.svg.cpp
)


install(TARGETS
  offset_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
