cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Create library
add_library(path_searching_lib
  src/kinodynamic_astar.cpp
)

ament_target_dependencies(path_searching_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  plan_env
)

target_include_directories(path_searching_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(path_searching_lib
  Eigen3::Eigen
)

# Install library
install(TARGETS path_searching_lib
  EXPORT path_searching_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Create executable
# add_executable(kinodynamic_astar_node
#   src/kinodynamic_astar_node.cpp
# )

# ament_target_dependencies(kinodynamic_astar_node
#   rclcpp
#   std_msgs
#   geometry_msgs
#   nav_msgs
#   visualization_msgs
#   plan_env
# )

# target_link_libraries(kinodynamic_astar_node
#   path_searching_lib
#   Eigen3::Eigen
# )

# Install executable
# install(TARGETS kinodynamic_astar_node
#   DESTINATION lib/${PROJECT_NAME}
# )

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
