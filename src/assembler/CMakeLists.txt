cmake_minimum_required(VERSION 3.5)
set(CMAKE_BUILD_TYPE Release)
project(assembler)

# Find the required dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(CGAL REQUIRED)
#find_package(OpenGL REQUIRED)
find_package(yaml-cpp REQUIRED)
#find_package(Qt5 REQUIRED)

set(ASSIMP_INCLUDE_DIR "/usr/include/assimp")

set(ASSIMP_LIBRARY "/usr/lib/x86_64-linux-gnu/libassimp.so")

include_directories(${ASSIMP_INCLUDE_DIR})

include_directories(include)

link_directories("/usr/lib")

# Declare the package
add_executable(assembler_node src/assembler_node.cpp src/ModelLoader.cpp src/Assembler.cpp src/Assembly.cpp src/Part.cpp)

# Link against the necessary libraries
ament_target_dependencies(assembler_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  visualization_msgs
)

target_include_directories(assembler_node
  PRIVATE
  ${CGAL_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

# Link CGAL, Assimp, and OpenGL libraries
target_link_libraries(assembler_node
  ${CGAL_LIBRARIES}
  ${ASSIMP_LIBRARY}
  #OpenGL::GL
  yaml-cpp::yaml-cpp
  #Qt5::Core
  #Qt5::Widgets
)

# Install executables and libraries
install(TARGETS assembler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
