cmake_minimum_required(VERSION 2.8.3)
project(velodyne_receiver)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs 
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES velodyne_receiver
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs geometry_msgs message_runtime
  DEPENDS eigen
)

set(ChoreonoidPath ~/choreonoid)
set(graspPluginPath ${ChoreonoidPath}/extplugin/graspPlugin)
# set(rostmsPluginPath ${ChoreonoidPath}/extplugin/rostmsPlugin)
set(prmPath ${ChoreonoidPath}/extplugin/graspPlugin/PRM)
set(ChoreonoidLibPath ${ChoreonoidPath}/lib/choreonoid-1.4)

set(libname CnoidVelodynePlugin)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ChoreonoidPath}
  ${ChoreonoidPath}/include
  ${graspPluginPath}
  ${prmPath}
  /usr/include/eigen3
)

link_directories(
  ${ChoreonoidLibPath}
)

add_library(
  ${libname}
  SHARED
  src/velodyne_bar.cpp
  src/velodyne_plugin.cpp
  #~/choreonoid/extplugin/graspPlugin/Grasp/PlanBase.h
  #./extplugin/graspPlugin/Grasp/PlanBase.h
)

# 追加
add_executable(receiver src/receiver.cpp)
target_link_libraries(receiver ${catkin_LIBRARIES} ) 

include_directories(include ${catkin_INCLUDE_DIRS})

target_link_libraries(
  ${libname}
  CnoidBodyPlugin
  CnoidPoseSeqPlugin
  CnoidGraspPlugin
  CnoidPRMPlugin
  ${catkin_LIBRARIES}
)

set_target_properties(
  ${libname}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY 
  ${ChoreonoidLibPath}
)

add_definitions(-DCNOID_BACKWARD_COMPATIBILITY)

find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${QT_OPENGL_INCLUDE_DIR})
add_definitions(-DQT_NO_KEYWORDS)

add_dependencies(${libname} velodyne_receiver_generate_messages_cpp)


