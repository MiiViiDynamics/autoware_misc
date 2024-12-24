cmake_minimum_required(VERSION 3.8)
project(autoware_misc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(can_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
set(TMP_CMAKE_SIZEOF_VOID_P ${CMAKE_SIZEOF_VOID_P})
unset(CMAKE_SIZEOF_VOID_P)
find_package(CycloneDDS REQUIRED COMPONENTS ddsc idlc)
set(CMAKE_SIZEOF_VOID_P ${TMP_CMAKE_SIZEOF_VOID_P})
autoware_package()


# find dependencies
find_package(ament_cmake REQUIRED)

file(GLOB IDL_FILES msg/*.idl)
list(TRANSFORM IDL_FILES REPLACE "\.idl" "\.c" OUTPUT_VARIABLE SOURCES)
list(TRANSFORM SOURCES REPLACE ".*/" "${CMAKE_CURRENT_BINARY_DIR}/")
list(TRANSFORM SOURCES REPLACE "\.c$" "\.h" OUTPUT_VARIABLE HEADERS)

add_custom_command(
  OUTPUT ${SOURCES} ${HEADERS}
  COMMAND find ${CMAKE_CURRENT_SOURCE_DIR}/msg/ -maxdepth 1 -name "*.idl" -exec idlc {} "\;"
  COMMAND sed -i -- "'s/\\(\\/.\\+\\/\\)\\(.\\+\\.idl\\)/\\2/'" *.h *.c
  DEPENDS ${IDL_FILES}
)

add_custom_target(generate_idl ALL
  DEPENDS ${SOURCES} ${HEADERS}
)

file(GLOB C_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/*.c)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

ament_auto_add_library(image_publisher_lib SHARED
  src/miivii_static_image_publisher/miivii_image_publisher_node.cpp
)

rclcpp_components_register_node(image_publisher_lib
  PLUGIN "autoware_misc::ImagePublisherNode"
  EXECUTABLE miivii_image_publisher_node
)

ament_auto_add_library(status_monitor_lib SHARED
  src/miivii_status_monitor/miivii_status_monitor.cpp
)

target_link_libraries(status_monitor_lib
  ${can_msgs_LIBRARY}
  ${radar_msgs_LIBRARY}
  ${diagnostic_msgs_LIBRARY}
  ${camera_info_manager_LIBRARY}
)

rclcpp_components_register_node(status_monitor_lib
  PLUGIN "autoware_misc::MiiviiStatusMonitorNode"
  EXECUTABLE miivii_status_monitor_node
)

ament_auto_add_library(converter_lib SHARED
  src/actuation_message_converter/actuation_message_converter_node.cpp
  ${C_SOURCES}
)

target_link_libraries(converter_lib
  CycloneDDS::ddsc
)

target_include_directories(converter_lib PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})

rclcpp_components_register_node(converter_lib
  PLUGIN "autoware::actuation_message_converter::MessageConverterNode"
  EXECUTABLE actuation_message_converter_node_exe
)

ament_auto_package()