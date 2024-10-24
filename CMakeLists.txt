cmake_minimum_required(VERSION 3.8)
project(turtle_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

set(dependencies
    rclcpp 
    std_msgs
    geometry_msgs
    turtlesim
)

add_executable(turtle_command src/turtle_command.cpp)
target_include_directories(turtle_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtle_command PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  turtle_command
  "rclcpp"
  "geometry_msgs"
)

add_executable(pose_subscriber src/pose_subscriber.cpp)
ament_target_dependencies(pose_subscriber ${dependencies})

add_executable(straight_line_controller src/straight_line_controller.cpp)
target_include_directories(straight_line_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(straight_line_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  straight_line_controller
  ${dependencies}
)

add_executable(pid_controller src/pid_controller.cpp)
target_include_directories(pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pid_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pid_controller
  ${dependencies}
)

install(TARGETS 
  turtle_command
  pose_subscriber
  straight_line_controller
  pid_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()