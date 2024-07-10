cmake_minimum_required(VERSION 3.8)
project(kromek_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

set(DRIVER "~/Spectrometer/Kromek/Driver")
include_directories(
	${DRIVER}/KromekDriver
	${DRIVER}/SpectrometerDriver
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_planning_interfaces REQUIRED)

set(dependencies
rclcpp
rclcpp_action
robot_planning_interfaces
)

add_executable(action_server src/kromek_action_server.cpp)

ament_target_dependencies(action_server
${dependencies})

install(TARGETS
action_server
DESTINATION lib/${PROJECT_NAME})

set(ENV{LD_LIBRARY_PATH} "$ENV{LD_LIBRARY_PATH}:${DRIVER}/output/linux64/release")

target_link_libraries(action_server
  ${DRIVER}/output/linux64/release/libSpectrometerDriver.so
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
