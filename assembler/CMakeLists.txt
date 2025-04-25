cmake_minimum_required(VERSION 3.5)
set(CMAKE_BUILD_TYPE Release)
project(assembler)

# Find the required dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(assembler_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(CGAL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCASCADE REQUIRED)

include_directories(include)

link_directories("/usr/lib")

# Declare the package
add_executable(assembler_node src/assembler_node.cpp src/ModelLoader.cpp src/Assembler.cpp src/Assembly.cpp src/Part.cpp src/MeshFunctions.cpp src/ARMSConfig.cpp)

# Link against the necessary libraries
ament_target_dependencies(assembler_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  assembler_msgs
  visualization_msgs
)

target_include_directories(assembler_node
  PRIVATE
  ${CGAL_INCLUDE_DIRS}
)


# Link CGAL, Assimp, and OpenGL libraries
target_link_libraries(assembler_node
  ${CGAL_LIBRARIES}
  ${OpenCASCADE_LIBRARIES}
  yaml-cpp::yaml-cpp
)

# Install executables and libraries
install(TARGETS assembler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
