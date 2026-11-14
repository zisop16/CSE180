cmake_minimum_required(VERSION 3.8)
project(gotogoal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

include_directories(include)


add_executable(goalgo src/gotogoal.cpp)
ament_target_dependencies(goalgo rclcpp)
ament_target_dependencies(goalgo geometry_msgs)
ament_target_dependencies(goalgo nav_msgs)


install(TARGETS
  goalgo
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
