cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDistance.srv"
  DEPENDENCIES builtin_interfaces
)

include_directories(include)

add_executable(path_planner_node
  src/main.cpp
  src/path_planner.cpp
)

ament_target_dependencies(path_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  example_interfaces
)

rosidl_target_interfaces(path_planner_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
