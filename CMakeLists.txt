cmake_minimum_required(VERSION 3.8)
project(mocap_fake_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(mocap_fake_localizer_node src/mocap_fake_localizer_node.cpp)
add_executable(mocap_fake_ekf_node src/mocap_fake_ekf_node.cpp)
# target_link_libraries(mocap_fake_ekf_node
#   rclcpp::rclcpp
#   nav_msgs::nav_msgs
#   tf2_ros::tf2_ros
#   geometry_msgs::geometry_msgs
# )

target_include_directories(mocap_fake_localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(mocap_fake_localizer_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  mocap_fake_localizer_node
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "geometry_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "Eigen3"
)

target_include_directories(mocap_fake_ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(mocap_fake_ekf_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  mocap_fake_ekf_node
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "geometry_msgs"
)

install(TARGETS mocap_fake_localizer_node   
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mocap_fake_ekf_node   
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
