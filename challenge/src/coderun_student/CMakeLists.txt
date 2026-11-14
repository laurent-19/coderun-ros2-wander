cmake_minimum_required(VERSION 3.8)
project(coderun_student)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++: every .cpp in src/ is part of the node, so you can split your code into several files
file(GLOB SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_executable(solution_cpp ${SOURCES})
target_compile_features(solution_cpp PUBLIC cxx_std_17)
ament_target_dependencies(solution_cpp rclcpp geometry_msgs nav_msgs sensor_msgs)
install(TARGETS solution_cpp DESTINATION lib/${PROJECT_NAME})

# Python: with --symlink-install this is a link to python/solution.py, so edits apply without a rebuild
install(PROGRAMS python/solution.py DESTINATION lib/${PROJECT_NAME})

ament_package()
