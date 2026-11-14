cmake_minimum_required(VERSION 3.8)
project(scout)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(navigation2 REQUIRED)

include_directories(include)


add_executable(scout src/scout.cpp)
ament_target_dependencies(scout rclcpp)
ament_target_dependencies(scout geometry_msgs)
ament_target_dependencies(scout nav_msgs)
ament_target_dependencies(scout sensor_msgs)
ament_target_dependencies(scout tf2)
ament_target_dependencies(scout tf2_ros)
ament_target_dependencies(scout tf2_geometry_msgs)

install(TARGETS
  scout
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
