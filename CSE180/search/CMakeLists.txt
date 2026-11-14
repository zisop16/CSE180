cmake_minimum_required(VERSION 3.8)
project(search)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(navigation REQUIRED)

include_directories(include)


add_executable(search src/search.cpp)
target_link_libraries(search navigation::navigator)
ament_target_dependencies(search rclcpp)
ament_target_dependencies(search geometry_msgs)
ament_target_dependencies(search nav_msgs)
ament_target_dependencies(search nav2_msgs)
ament_target_dependencies(search sensor_msgs)
ament_target_dependencies(search tf2)
ament_target_dependencies(search tf2_ros)
ament_target_dependencies(search tf2_geometry_msgs)

install(TARGETS
  search
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
