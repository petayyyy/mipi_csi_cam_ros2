cmake_minimum_required(VERSION 3.10)
project(mipi_csi_cam_ros2 CXX)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(MIPI_CSI_CAM_ROS2_ENABLE_NATIVE_TUNE
  "Enable -march=native for on-device builds; keep OFF for cross/buildx builds"
  OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

add_executable(mipi_csi_cam_ros2_node src/mipi_csi_cam_ros2_node.cpp)

target_compile_options(mipi_csi_cam_ros2_node PRIVATE
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Release>:-pipe>
)

if(MIPI_CSI_CAM_ROS2_ENABLE_NATIVE_TUNE)
    target_compile_options(mipi_csi_cam_ros2_node PRIVATE
        $<$<CONFIG:Release>:-march=native>
    )
endif()

target_include_directories(mipi_csi_cam_ros2_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(mipi_csi_cam_ros2_node
    rclcpp
    sensor_msgs
    camera_info_manager
    image_transport
    cv_bridge
)

target_link_libraries(mipi_csi_cam_ros2_node
    ${OpenCV_LIBS}
    pthread
)

install(TARGETS mipi_csi_cam_ros2_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
