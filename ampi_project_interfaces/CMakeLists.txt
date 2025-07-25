cmake_minimum_required(VERSION 3.8)
project(ampi_project_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunStep.action"
  "action/AddRecipie.action"
  "action/TransferPlate.action"
  "action/SwapTool.action"
  "action/PickPlace.action"
  "action/BoltPickPlace.action"
  "msg/QuadraturePhotodiode.msg"
  "msg/QuadraturePhotodiodesStamped.msg"
  "msg/MoonrakerNotification.msg"
  "msg/Line.msg"
  "msg/Circle.msg"
  "msg/Datum.msg"
  "msg/PickPlace.msg"
  "srv/UpdateBeacons.srv"
  "srv/GDTPosition.srv"
  "srv/GDTCircularity.srv"
  "srv/GDTFlatness.srv"
  "srv/HemisphereRadialScan.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs moveit_msgs
)

ament_package()
