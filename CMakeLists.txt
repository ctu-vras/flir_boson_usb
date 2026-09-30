cmake_minimum_required(VERSION 3.10.2)
project(flir_boson_usb)

find_package(ament_cmake QUIET)
find_package(catkin QUIET)

if(catkin_FOUND)
  # ROS 1

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_EXTENSIONS OFF)

  find_package(catkin REQUIRED COMPONENTS
    camera_info_manager
    cras_cpp_common
    cv_bridge
    dynamic_reconfigure
    image_transport
    nodelet
    pluginlib
    roscpp
    sensor_msgs
  )

  generate_dynamic_reconfigure_options(
    cfg/BosonCamera.cfg
  )

  catkin_package(
    CATKIN_DEPENDS
      camera_info_manager
      cras_cpp_common
      cv_bridge
      dynamic_reconfigure
      image_transport
      nodelet
      pluginlib
      roscpp
      sensor_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_library(BosonCamera src/BosonCamera.cpp)
  add_dependencies(BosonCamera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(BosonCamera ${catkin_LIBRARIES})
  cras_node_from_nodelet(BosonCamera flir_boson_usb::BosonCamera OUTPUT_NAME ${PROJECT_NAME}_node)

  install(TARGETS BosonCamera
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN *.launch
  )

  install(DIRECTORY example_calibrations
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

elseif(ament_cmake_FOUND)
  # ROS 2

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_EXTENSIONS OFF)

  find_package(camera_info_manager REQUIRED)
  find_package(cras_cpp_common REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)

  include_directories(include)

  add_library(boson_camera_component SHARED src/BosonCamera.cpp)
  target_compile_definitions(boson_camera_component PRIVATE "BOSON_CAMERA_BUILDING_DLL" "ROS2")

  target_link_libraries(boson_camera_component PUBLIC
    camera_info_manager::camera_info_manager
    cras_cpp_common::cras_image_transport
    cv_bridge::cv_bridge
    image_transport::image_transport
    rclcpp::rclcpp
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
  )

  rclcpp_components_register_node(
    boson_camera_component PLUGIN "flir_boson_usb::BosonCamera" EXECUTABLE boson_camera_node)

  install(TARGETS boson_camera_component boson_camera_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN *.launch.xml
  )

  install(DIRECTORY example_calibrations
    DESTINATION share/${PROJECT_NAME}
  )

  ament_export_dependencies(
    camera_info_manager
    cras_cpp_common
    cv_bridge
    image_transport
    rclcpp
    rclcpp_components
    sensor_msgs
  )
  ament_package()

else()
  message(FATAL_ERROR "Build failed because neither catkin nor ament found.")
endif()
