cmake_minimum_required(VERSION 3.8)
project(ads_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(pluginlib REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


# Posix threads


# ADS library from Beckhoff (assumed pre-built, follow the instructions in git submodule ADS).
find_library(
    LIBADS_LIBRARY
    NAMES AdsLib
    HINTS "../ADS/build"
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ads_hardware.cpp
)

#add_executable(ads_hardware src/ads_hardware.cpp)


ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle lifecycle_msgs pluginlib hardware_interface Threads)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)



target_compile_features(ads_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(ads_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_include_directories(ads_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(ads_hardware PRIVATE "../ADS/AdsLib")

target_link_libraries(ads_hardware ${LIBADS_LIBRARY})



pluginlib_export_plugin_description_file(hardware_interface ads_hardware.xml)

install(TARGETS ads_hardware
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include/ads_hardware
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)
ament_package()
