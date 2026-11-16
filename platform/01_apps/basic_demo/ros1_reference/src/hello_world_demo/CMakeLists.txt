cmake_minimum_required(VERSION 3.0.2) # 指定最低 CMake 版本，适用于 ROS Noetic。
project(hello_world_demo) # 设置项目名，必须与 package.xml 中的功能包名称一致。
find_package(catkin REQUIRED COMPONENTS roscpp rospy) # 查找 catkin 以及本例需要的 ROS 库。
catkin_package(CATKIN_DEPENDS roscpp rospy) # 声明这是 catkin 功能包，并导出 ROS 依赖信息。
include_directories(${catkin_INCLUDE_DIRS}) # 添加 ROS 头文件的搜索路径。
add_executable(hello_cpp src/hello_cpp.cpp) # 将 C++ 源文件编译成名为 hello_cpp 的可执行程序。
target_link_libraries(hello_cpp ${catkin_LIBRARIES}) # 为 C++ 可执行程序链接 ROS 库。
catkin_install_python(PROGRAMS scripts/hello_python.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}) # 登记 Python 节点，并使用 catkin 配置的解释器。
install(TARGETS hello_cpp RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}) # 执行安装构建时，将 C++ 节点安装到功能包的可执行目录。
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}) # 执行安装构建时，将 launch 文件夹安装到功能包共享目录。
