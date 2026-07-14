cmake_minimum_required(VERSION 3.5)
project(qr_bpu_detector)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc dnn)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)

set(DNN_INCLUDE_PATH "/usr/include/dnn")
set(DNN_LIB_PATH "/usr/lib")

include_directories(
  include
  ${DNN_INCLUDE_PATH}
)

link_directories(
  ${DNN_LIB_PATH}
  /usr/hobot/lib
)

find_path(ZBAR_INCLUDE_DIR
  NAMES zbar.h
  PATHS /usr/include /usr/local/include /usr/include/zbar
)

find_library(ZBAR_LIBRARY
  NAMES zbar
  PATHS
    /usr/lib
    /usr/local/lib
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
)

if(NOT ZBAR_INCLUDE_DIR OR NOT ZBAR_LIBRARY)
  message(FATAL_ERROR
    "ZBar not found. Install it with: sudo apt install libzbar-dev")
endif()

add_executable(qr_bpu_detector_node
  src/qr_bpu_detector_node.cpp
)

add_executable(qr_roi_decoder_node
  src/qr_roi_decoder_node.cpp
)

ament_target_dependencies(qr_bpu_detector_node
  ai_msgs
  OpenCV
  rclcpp
  sensor_msgs
)

target_link_libraries(qr_bpu_detector_node
  ${OpenCV_LIBS}
  dnn
  pthread
  rt
  dl
)

ament_target_dependencies(qr_roi_decoder_node
  ai_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
  message_filters
)

target_include_directories(qr_roi_decoder_node PRIVATE
  ${ZBAR_INCLUDE_DIR}
)

target_link_libraries(qr_roi_decoder_node
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARY}
  pthread
)

install(TARGETS
  qr_bpu_detector_node
  qr_roi_decoder_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES README.md CHANGELOG.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
