cmake_minimum_required(VERSION 3.10)

project(camera_aravis)

## Enable simple buffer processing benchmark output
#add_compile_definitions(ARAVIS_BUFFER_PROCESSING_BENCHMARK)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  sensor_msgs
  message_generation
  image_transport
  camera_info_manager
  dynamic_reconfigure
  tf
  tf2_ros
)

# Version >= 0.8.17 -> ARV_UV_USB_MODE_ASYNC
# Version >= 0.8.25 -> multipart support and fixes
find_package(Aravis 0.8 REQUIRED)
find_package(GLIB2 REQUIRED)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/CameraAravis.cfg)

add_message_files(
   FILES
   CameraAutoInfo.msg
   ExtendedCameraInfo.msg
)

add_service_files(
  FILES
  get_boolean_feature_value.srv
  get_float_feature_value.srv
  get_integer_feature_value.srv
  get_string_feature_value.srv
  set_boolean_feature_value.srv
  set_float_feature_value.srv
  set_integer_feature_value.srv
  set_string_feature_value.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

catkin_package(
    DEPENDS Aravis GLIB2 OpenCV
    CATKIN_DEPENDS roscpp nodelet std_msgs sensor_msgs message_runtime image_transport camera_info_manager dynamic_reconfigure tf tf2_ros
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(cfg
  include
  ${catkin_INCLUDE_DIRS}
  ${Aravis_INCLUDE_DIRS}
  ${GLIB2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${Aravis_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/camera_aravis_nodelet.cpp
  src/camera_buffer_pool.cpp
  src/conversion_utils.cpp
)

target_link_libraries(${PROJECT_NAME} ${Aravis_LIBRARIES} glib-2.0 gmodule-2.0 gobject-2.0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(cam_aravis
  src/camera_aravis_node.cpp
)

target_link_libraries(cam_aravis ${PROJECT_NAME})
add_dependencies(cam_aravis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS cam_aravis
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY calibration/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/calibration
)

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
