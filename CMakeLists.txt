cmake_minimum_required(VERSION 3.8)
project(flir_boson_usb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create the Component Library
add_library(boson_camera_component SHARED src/BosonCamera.cpp)
target_compile_definitions(boson_camera_component PRIVATE "BOSON_CAMERA_BUILDING_DLL")

ament_target_dependencies(boson_camera_component
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  sensor_msgs
  camera_info_manager
)

target_link_libraries(boson_camera_component ${OpenCV_LIBRARIES})

# Register the Component and create the executable
rclcpp_components_register_node(boson_camera_component
  PLUGIN "flir_boson_usb::BosonCamera"
  EXECUTABLE boson_camera_node
)

# Install rules
install(TARGETS boson_camera_component boson_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch example_calibrations
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp rclcpp_components cv_bridge image_transport sensor_msgs camera_info_manager
)
ament_package()
