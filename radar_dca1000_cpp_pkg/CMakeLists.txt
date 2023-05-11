cmake_minimum_required(VERSION 3.5)
project(radar_dca1000_cpp_pkg)

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
find_package(radar_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

# find include directory
include_directories(include)

# executables
add_executable(radar_data 
  src/radar_data_node.cpp
  src/ringbuffer.cpp)
ament_target_dependencies(radar_data rclcpp radar_interfaces builtin_interfaces)
target_link_libraries(radar_data ${JSONCPP_LIBRARIES})
target_link_libraries(radar_data m2s2_cppdevclass)


# install
install(TARGETS
  radar_data
  DESTINATION lib/${PROJECT_NAME})

ament_package()
