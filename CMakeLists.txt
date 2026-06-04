
cmake_minimum_required(VERSION 3.0.2)
project(radar_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  message_filters
  message_generation
  pcl_ros
  pcl_conversions
  diagnostic_msgs
  dynamic_reconfigure
  geodesy
  serial
)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_message_files(
  FILES
  RTKData.msg
  RTKEvent.msg
  RemapCommand.msg
  PathPlanning.msg
  Obstacle3D.msg
  Obstacle3DArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    tf2     
    tf2_ros      
    message_filters  
    pcl_ros
    pcl_conversions  
    diagnostic_msgs
    dynamic_reconfigure  
    geodesy       
    serial        
    message_runtime  
  DEPENDS
    PCL
    EIGEN3  
)


# 第一阶段：点云处理器
add_executable(pointcloud_processor src/pointcloud_processor.cpp)
target_link_libraries(pointcloud_processor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# 第二阶段：特征检测
add_executable(feature_detector_node 
  src/feature_detector.cpp
  src/feature_detector_main.cpp
)
target_link_libraries(feature_detector_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(feature_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 第三阶段：RTK-GPS
add_executable(rtk_gps_node 
  src/rtk_gps_parser.cpp
  src/rtk_gps_main.cpp
)
target_link_libraries(rtk_gps_node ${catkin_LIBRARIES} ${serial_LIBRARIES})
add_dependencies(rtk_gps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(time_sync_node src/time_sync_node.cpp)
target_link_libraries(time_sync_node ${catkin_LIBRARIES} ${message_filters_LIBRARIES})
add_dependencies(time_sync_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 第四阶段：重新建图管理
add_executable(remap_manager_node 
  src/remap_manager.cpp
  src/remap_manager_main.cpp
)
target_link_libraries(remap_manager_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(remap_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 系统监控
add_executable(system_monitor_node src/system_monitor.cpp)
target_link_libraries(system_monitor_node ${catkin_LIBRARIES})
add_dependencies(system_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch config rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(pointcloud_relay src/pointcloud_relay.cpp)
target_link_libraries(pointcloud_relay ${catkin_LIBRARIES})
