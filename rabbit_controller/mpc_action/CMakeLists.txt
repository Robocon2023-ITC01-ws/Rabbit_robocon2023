cmake_minimum_required(VERSION 3.5)
project(mpc_action)

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

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(AUTO_MPC_SRC src/auto_omni.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
	"action/MPCAction.action"
	)

add_library(auto_mpc_lib src/auto_omni.cpp)

add_executable(pub_sub_node src/pub_sub.cpp)
ament_target_dependencies(pub_sub_node rclcpp std_msgs)

add_executable(mpc_omni src/mpc_node.cpp)
target_link_libraries(mpc_omni auto_mpc_lib casadi)
ament_target_dependencies(mpc_omni rclcpp sensor_msgs std_msgs)

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

install(TARGETS
  pub_sub_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#   mpc_omni
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME})

# ament_export_include_directories(include)
# ament_export_libraries(auto_mpc_lib)
# ament_export_libraries(mpc_omni)
# ament_export_dependencies(
#   rclcpp casadi Eigen3
# )
ament_package()


