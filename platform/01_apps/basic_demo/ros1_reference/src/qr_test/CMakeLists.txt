cmake_minimum_required(VERSION 3.0.2)
project(qr_test)

## 编译选项（你上面写注释是 C++17，但实际是 C++14，这里统一成 C++14）
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  tf
)

## OpenCV
find_package(OpenCV REQUIRED)

## zbar（通过 pkg-config）
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES qr_test
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  #  DEPENDS system_lib
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

add_executable(qr_test
  src/qr_test.cpp
)

target_link_libraries(qr_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)


install(TARGETS qr_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)


