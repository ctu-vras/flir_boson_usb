# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2018 FLIR Systems, INC
# SPDX-FileCopyrightText: 2018-2019 AutonomouStuff, LLC
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.11.0)
project(flir_boson_usb LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake QUIET)
#catkin_lint: ignore_once missing_required
find_package(catkin QUIET)

set(USE_BOSON_SDK OFF CACHE BOOL
  "Use the Boson SDK to access the camera. By turning on this variable, you agree to BOSON TOOLS and SDK License
  Agreement.pdf."
)

if(USE_BOSON_SDK)
  set(BOSON_SDK_DIR ${CMAKE_CURRENT_BINARY_DIR}/_deps/download_boson_sdk-src)

  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
  endif()
  include(FetchContent)
  fetchcontent_declare(download_boson_sdk
    URL https://flir.netx.net/file/asset/46046/original/attachment
    DOWNLOAD_NAME boson_sdk.zip
  )
  fetchcontent_makeavailable(download_boson_sdk)

  set(BOSON_SDK_ROOT "${BOSON_SDK_DIR}/SDK_USER_PERMISSIONS/SDK_USER_PERMISSIONS")
  set(BOSON_SDK_FSLP_DIR "${BOSON_SDK_ROOT}/FSLP_Files")
  set(BOSON_SDK_INCLUDE_DIRS "${BOSON_SDK_ROOT}/ClientFiles_C")
  get_filename_component(BOSON_SDK_LIBRARY "${BOSON_SDK_FSLP_DIR}/FSLP_64.so" ABSOLUTE)

  # The precompiled FSLP_64.so has no SONAME. If it is linked by its full path, the linker stores
  # exactly that path as the DT_NEEDED entry of the depending library, which cannot be resolved at
  # run time any more (the path only exists inside the build tree). Link therefore through a copy
  # with a conventional lib-prefixed name and declare the library as having no SONAME, so that CMake
  # links it with -L<dir> -lFSLP_64 and the DT_NEEDED entry contains just libFSLP_64.so.
  set(BOSON_SDK_FSLP_INSTALLED_NAME "libFSLP_64.so")
  set(BOSON_SDK_FSLP_LINK_DIR "${CMAKE_CURRENT_BINARY_DIR}/fslp_lib")
  file(MAKE_DIRECTORY "${BOSON_SDK_FSLP_LINK_DIR}")
  configure_file("${BOSON_SDK_LIBRARY}" "${BOSON_SDK_FSLP_LINK_DIR}/${BOSON_SDK_FSLP_INSTALLED_NAME}" COPYONLY)

  #catkin_lint: ignore_once order_violation
  add_library(FSLP_64 SHARED IMPORTED)
  set_target_properties(FSLP_64 PROPERTIES
    IMPORTED_LOCATION "${BOSON_SDK_FSLP_LINK_DIR}/${BOSON_SDK_FSLP_INSTALLED_NAME}"
    IMPORTED_NO_SONAME TRUE
  )

  file(GLOB BOSON_SDK_SOURCES "${BOSON_SDK_INCLUDE_DIRS}/*.c")
  list(FILTER BOSON_SDK_SOURCES EXCLUDE REGEX ".*I2C.*")
  set_property(SOURCE ${BOSON_SDK_SOURCES} PROPERTY LANGUAGE C)
  #catkin_lint: ignore_once order_violation
  add_library(boson_sdk SHARED ${BOSON_SDK_SOURCES})
  #catkin_lint: ignore_once external_interface_path missing_directory
  target_include_directories(boson_sdk PUBLIC "${BOSON_SDK_INCLUDE_DIRS}")
  target_link_libraries(boson_sdk PUBLIC FSLP_64)
  # libFSLP_64.so is installed next to this library, so look for it in the same directory.
  set_target_properties(boson_sdk PROPERTIES INSTALL_RPATH "$ORIGIN")
  target_compile_definitions(boson_sdk PUBLIC HAS_BOSON_SDK)

  message(STATUS "BOSON_SDK_INCLUDE_DIRS: ${BOSON_SDK_INCLUDE_DIRS}")
  message(STATUS "BOSON_SDK_LIBRARY: ${BOSON_SDK_FSLP_LINK_DIR}/${BOSON_SDK_FSLP_INSTALLED_NAME}")
else()
  #catkin_lint: ignore_once order_violation
  add_library(boson_sdk INTERFACE)
  message(
    "USE_BOSON_SDK is OFF, the driver will not have some extra functionality. Consider enabling it if you agree
    with its terms."
  )
endif()

#catkin_lint: ignore_once order_violation
add_library(boson_api SHARED src/boson_api.cpp)
target_link_libraries(boson_api PRIVATE boson_sdk)

if(catkin_FOUND)
  # ROS 1
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_EXTENSIONS OFF)

  find_package(catkin REQUIRED COMPONENTS
    camera_info_manager
    cras_cpp_common
    cv_bridge
    dynamic_reconfigure
    filters
    image_transport
    nodelet
    pluginlib
    roscpp
    sensor_msgs
  )

  generate_dynamic_reconfigure_options(
    cfg/BosonCamera.cfg
  )

  catkin_package()

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(boson_api PUBLIC ${catkin_LIBRARIES})

  # The image processing does not depend on ROS, so it is a separate library that can be tested
  # without a running ROS and without a camera.
  add_library(flir_boson_pipeline src/pipeline.cpp)
  target_link_libraries(flir_boson_pipeline ${catkin_LIBRARIES})

  add_library(BosonCamera src/BosonCamera.cpp)
  add_dependencies(BosonCamera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(BosonCamera flir_boson_pipeline boson_api ${catkin_LIBRARIES})
  cras_node_from_nodelet(BosonCamera ${PROJECT_NAME}::BosonCamera OUTPUT_NAME ${PROJECT_NAME}_node)

  install(TARGETS BosonCamera boson_api flir_boson_pipeline
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  if(USE_BOSON_SDK)
    install(FILES "${BOSON_SDK_LIBRARY}"
      DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RENAME ${BOSON_SDK_FSLP_INSTALLED_NAME}
    )
    install(TARGETS boson_sdk
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  endif()

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

  if(CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    find_package(roslint REQUIRED)
    find_package(rostest REQUIRED)

    catkin_add_gtest(test_pipeline test/test_pipeline.cpp)
    target_link_libraries(test_pipeline flir_boson_pipeline)

    roslint_custom(catkin_lint "-W2" .)

    # Roslint C++ - checks formatting and some other rules for C++ files

    file(GLOB_RECURSE ROSLINT_INCLUDE include/*.hpp)
    file(GLOB_RECURSE ROSLINT_SRC src/*.cpp)

    set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
    roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC})

    roslint_add_test()

    roslaunch_add_file_check(launch/flir_boson.launch)
    roslaunch_add_file_check(launch/flir_boson_320.launch)
  endif()

elseif(ament_cmake_FOUND)
  # ROS 2
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_EXTENSIONS OFF)

  #catkin_lint: ignore_once duplicate_find
  find_package(camera_info_manager REQUIRED)
  #catkin_lint: ignore_once duplicate_find
  find_package(cras_cpp_common REQUIRED)
  #catkin_lint: ignore_once duplicate_find
  find_package(cv_bridge REQUIRED)
  #catkin_lint: ignore_once duplicate_find
  find_package(filters REQUIRED)
  #catkin_lint: ignore_once duplicate_find
  find_package(image_transport REQUIRED)
  #catkin_lint: ignore_once duplicate_find
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  #catkin_lint: ignore_once duplicate_find
  find_package(sensor_msgs REQUIRED)

  include_directories(include)

  target_link_libraries(boson_api PUBLIC cras_cpp_common::cras_expected)

  add_library(flir_boson_pipeline SHARED src/pipeline.cpp)
  target_link_libraries(flir_boson_pipeline PUBLIC cv_bridge::cv_bridge)

  add_library(radiometric_filter SHARED src/filters/radiometric_filter.cpp)
  target_link_libraries(radiometric_filter PUBLIC
    flir_boson_pipeline
    # cras_type_utils is not pulled in by cras_filter_base, but it is needed by it: the templated getParam()
    # instantiates cras::typeName<T>() in the translation unit that calls it, and that is this one.
    cras_cpp_common::cras_filter_base
    cras_cpp_common::cras_type_utils
    cv_bridge::cv_bridge
    filters::filter_base
    pluginlib::pluginlib
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
  )
  pluginlib_export_plugin_description_file(filters radiometric_filter.xml)

  add_library(boson_camera_component SHARED src/BosonCamera.cpp)
  target_compile_definitions(boson_camera_component PRIVATE "ROS2")
  target_link_libraries(boson_camera_component PUBLIC
    boson_api
    flir_boson_pipeline
    camera_info_manager::camera_info_manager
    cras_cpp_common::cras_image_transport
    cras_cpp_common::cras_string_utils
    cv_bridge::cv_bridge
    filters::filter_chain
    image_transport::image_transport
    rclcpp::rclcpp
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
  )
  add_dependencies(boson_camera_component
    radiometric_filter
  )
  rclcpp_components_register_node(
    boson_camera_component PLUGIN "${PROJECT_NAME}::BosonCamera" EXECUTABLE boson_camera_node)

  install(TARGETS boson_api boson_camera_component flir_boson_pipeline radiometric_filter
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  if(USE_BOSON_SDK)
    install(FILES "${BOSON_SDK_LIBRARY}"
      DESTINATION lib
      RENAME ${BOSON_SDK_FSLP_INSTALLED_NAME}
    )

    install(TARGETS boson_sdk
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN *.launch EXCLUDE
  )

  install(DIRECTORY example_calibrations
    DESTINATION share/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(cras_lint REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    cras_lint_common()
    cras_lint_cpp()

    ament_add_gtest(test_pipeline test/test_pipeline.cpp)
    target_link_libraries(test_pipeline flir_boson_pipeline)

    ament_add_gtest(test_radiometric_filter test/test_radiometric_filter.cpp)
    target_link_libraries(test_radiometric_filter
      # Link the filter itself, not just its header, so that the test runs the very code the driver loads
      # through pluginlib. It brings the pipeline library, cras_filter_base and cv_bridge with it.
      radiometric_filter
      rclcpp::rclcpp
      ${sensor_msgs_TARGETS}
    )

    ament_add_gtest(test_radiometric_filter_chain test/test_radiometric_filter_chain.cpp)
    target_link_libraries(test_radiometric_filter_chain
      # The filter is deliberately not linked here: the chain loads it through pluginlib, and a class that is
      # linked into the executable as well makes class_loader report a namespace collision. The pipeline
      # library is only there for the unit table the expected values are taken from.
      flir_boson_pipeline
      filters::filter_chain
      rclcpp::rclcpp
      ${sensor_msgs_TARGETS}
    )
    # The chain looks the filter up in the plugin description of this package, so it has to be built first, and
    # it has to know where the package was installed to when the tests are run without the package on
    # AMENT_PREFIX_PATH (which colcon test puts there, and plain ctest does not).
    add_dependencies(test_radiometric_filter_chain radiometric_filter)
    target_compile_definitions(test_radiometric_filter_chain
      PRIVATE FLIR_BOSON_USB_PACKAGE_PREFIX="${CMAKE_INSTALL_PREFIX}")
  endif()

  ament_package()

else()
  message(FATAL_ERROR "Build failed because neither catkin nor ament found.")
endif()
