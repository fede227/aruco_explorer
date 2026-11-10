cmake_minimum_required(VERSION 3.8)
project(precision_landing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui aruco)

add_executable(precision_landing_node 
  src/test.cpp
)

target_include_directories(precision_landing_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(precision_landing_node
  rclcpp
  behaviortree_cpp
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  px4_msgs
)

target_link_libraries(precision_landing_node
  ${OpenCV_LIBS}
)

install(TARGETS
  precision_landing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
