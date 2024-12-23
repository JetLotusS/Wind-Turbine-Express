cmake_minimum_required(VERSION 3.8)
project(wind_turbine_express_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(wind_turbine_express_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)

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

install(
  PROGRAMS 
    src/thruster_driver.py
    src/wte_localisation.py
    src/wte_aquabot.py
    src/wte_cv_module.py
    src/wte_planner.py
    src/tool_planner.py
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    images
    assets
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
