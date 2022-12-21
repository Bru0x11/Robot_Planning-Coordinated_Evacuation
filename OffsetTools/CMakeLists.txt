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
)

add_executable(offset_node
 src/offsetFunctions.cpp
 include/CPP/src/clipper.engine.cpp 
 include/CPP/src/clipper.offset.cpp 
 include/CPP/src/clipper.rectclip.cpp
 include/CPP/Utils/ClipFileLoad.cpp
 include/CPP/Utils/ClipFileSave.cpp
 include/CPP/Utils/clipper.svg.cpp
)


install(TARGETS
  offset_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
