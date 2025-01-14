cmake_minimum_required(VERSION 3.8)
project(bigomni_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
 find_package(rclcpp REQUIRED)
 find_package(std_msgs REQUIRED)
 find_package(geometry_msgs REQUIRED)

# add executable targets
add_executable(twist2bigomni_control src/twist2bigomni_control.cpp)
# add library targets
target_include_directories(twist2bigomni_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# link to dependencies
ament_target_dependencies(twist2bigomni_control
	rclcpp
	std_msgs
	geometry_msgs
)

# compile target
target_compile_features(twist2bigomni_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install target
install(TARGETS twist2bigomni_control
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
	config
	DESTINATION share/${PROJECT_NAME}/
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
