cmake_minimum_required(VERSION 3.8)
project(misc_tools_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)


include_directories(include ${EIGEN3_INCLUDE_DIR})



# Declare multiple executables for your nodes
add_executable(zedOdomRepublisherNode src/zedOdomRepublisher_main.cpp)
ament_target_dependencies(zedOdomRepublisherNode rclcpp tf2 tf2_ros nav_msgs geometry_msgs Eigen3 tf2_geometry_msgs)

# add_executable(node2 src/node2.cpp)
# ament_target_dependencies(node2 rclcpp tf2 tf2_ros nav_msgs geometry_msgs Eigen3)

# Install executables
install(TARGETS
  zedOdomRepublisherNode
  # node2
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS zedodom_transformer_component DESTINATION lib/${PROJECT_NAME})

add_library(zedOdom_transformer_component SHARED src/zedOdomRepublisher.cpp)
ament_target_dependencies(zedOdom_transformer_component rclcpp tf2 tf2_ros nav_msgs geometry_msgs Eigen3 tf2_geometry_msgs rclcpp_components)

rclcpp_components_register_nodes(zedOdom_transformer_component "ZedOdomTransformer")


# Install executables
install(TARGETS
  zedOdom_transformer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
