cmake_minimum_required(VERSION 3.8)
project(csi_camera)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)        # ✅ ADDED - Required for Int32
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find libcamera (optional - for experimental node)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBCAMERA libcamera)
endif()

# Create executables
add_executable(csi_camera_node src/csi_camera_node.cpp)
add_executable(dual_csi_camera_node src/dual_csi_camera_node.cpp)

# EXPERIMENTAL: libcamera direct API node (optional)
if(LIBCAMERA_FOUND)
  message(STATUS "libcamera found - building experimental libcamera_dual_node")
  add_executable(libcamera_dual_node src/libcamera_dual_node.cpp)
  target_include_directories(libcamera_dual_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
  )
  ament_target_dependencies(libcamera_dual_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
  )
  target_link_libraries(libcamera_dual_node
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
  )
else()
  message(STATUS "libcamera NOT found - skipping experimental libcamera_dual_node")
endif()

# Include directories for csi_camera_node
target_include_directories(csi_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Include directories for dual_csi_camera_node
target_include_directories(dual_csi_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Link dependencies for csi_camera_node
ament_target_dependencies(csi_camera_node
  rclcpp
  sensor_msgs
  std_msgs          # ✅ ADDED - Required for Int32
  cv_bridge
)

# Link dependencies for dual_csi_camera_node
ament_target_dependencies(dual_csi_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(csi_camera_node 
  ${OpenCV_LIBS}
)

target_link_libraries(dual_csi_camera_node
  ${OpenCV_LIBS}
)

# Install executables
install(TARGETS 
  csi_camera_node
  dual_csi_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install experimental libcamera node if built
if(LIBCAMERA_FOUND)
  install(TARGETS libcamera_dual_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch files (optional)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

# Install config files (optional)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()