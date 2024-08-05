cmake_minimum_required(VERSION 2.8.3)

project(cam_pose_converter)




## 找到catkin宏和库
find_package(catkin REQUIRED COMPONENTS
 roscpp
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
)

## 定义catkin包
catkin_package()

## 设置包含路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 声明一个可执行文件
add_executable(cam_pose_converter src/camera_pose_resived.cpp)

## 链接catkin库
target_link_libraries(cam_pose_converter
  ${catkin_LIBRARIES}
)

## 添加依赖项
add_dependencies(cam_pose_converter ${catkin_EXPORTED_TARGETS})


#3 拷贝 launch目录           (PATH: install/share/slam_karto/launch/)
install(DIRECTORY 
    launch         pose         #注意,不能使用 launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS 
  cam_pose_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)