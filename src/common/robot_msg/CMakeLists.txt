cmake_minimum_required(VERSION 3.8)  # ROS2 推荐 3.8 及以上
project(robot_msg)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成接口（消息、服务、动作）
rosidl_generate_interfaces(${PROJECT_NAME}
  # msg
  "msg/LocalizationState.msg"
  "msg/LocationStatus.msg"
  "msg/ActionState.msg"
  "msg/DockingState.msg"
  "msg/BaseInfo.msg"
  "msg/BatteryState.msg"
  "msg/ChassisControl.msg"
  "msg/LeanMap.msg"
  "msg/FeedBack.msg"
  "msg/NavHsmFlexState.msg"
  "msg/ScoredPose.msg"
  "msg/SlamStatus.msg"
  "msg/StringOccupancyGrid.msg"
  "msg/Restrict.msg"
  "msg/ElevatorState.msg"
  "msg/BaseDiagnostics.msg"
  "msg/Gate.msg"
  "msg/StateChassis.msg"
  "msg/GateArray.msg"
  "msg/Humidifier.msg"
  "msg/DepthMM.msg"
  "msg/Region.msg"
  "msg/RegionArray.msg"
  "msg/SetVirtualWall.msg"
  "msg/RestrictArray.msg"
  "msg/Elevator.msg"
  "msg/SpeedAndAcceleration.msg"
  "msg/RegionClean.msg"
  "msg/ObstacleItem.msg"
  "msg/VirtualBumperItem.msg"
  "msg/WallFollowInfoItem.msg"
  "msg/VirtualBumperInfo.msg"
  "msg/ChargingState.msg"
  "msg/Parameters.msg"
  "msg/IntPoint2D.msg"
  "msg/IntPolygon.msg"

  # srv
  "srv/SetAction.srv"
  "srv/SetMap.srv"
  "srv/MapLoading.srv"
  "srv/MapSaving.srv"
  "srv/TrainScan.srv"
  "srv/SetSlam.srv"
  "srv/SetDevice.srv"
  "srv/EraseMap.srv"
  "srv/CheckPosition.srv"
  "srv/GetPlan.srv"
  "srv/GetDepthState.srv"
  "srv/SetGate.srv"
  "srv/SetElevatorCenter.srv"
  "srv/SetPathLimit.srv"
  "srv/GetMoveBaseState.srv"
  "srv/SetCalibrateDepthCameraCmd.srv"
  "srv/SetRegion.srv"
  "srv/SetElevator.srv"
  "srv/GetMapInfo.srv"
  "srv/SetChargeStation.srv"
  "srv/GetStaticPlan.srv"
  "srv/SetDepthSliceMethod.srv"
  "srv/GetFreeRatio.srv"
  "srv/GetElevatorPolygon.srv"
  "srv/CoverMap.srv"
  "srv/CoveragePathMove.srv"
  "srv/CoveragePathPlan.srv"
  "srv/IpaServerTest.srv"
  "srv/ChassisControlSrv.srv"
  "srv/GetVirtualBumper.srv"
  "srv/MotionCmd.srv"
  "srv/CoveragePathEstimate.srv"
  "srv/SetDepthCamera.srv"
  "srv/SetGlobalParams.srv"
  "srv/SetNodeParams.srv"
  "srv/SetCalibrateLineLaserCmd.srv"
  "srv/SetLaserOdomCalibrate.srv"
  "srv/SetOptModeCmd.srv"
  "srv/RoomSegmentation.srv"
  "srv/CoveragePathSegmentation.srv"

  # action
  "action/AutoElevator.action"
  "action/AutoDock.action"
  "action/AuxiliaryPositioning.action"
  "action/InoutElevator.action"
  "action/Navigation.action"
  "action/LaserOdomCalibration.action"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    action_msgs
)

# 导出消息运行时依赖
ament_export_dependencies(rosidl_default_runtime)

# 如果后续有 C++ 节点，可以取消下面注释
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node
#   rclcpp
#   std_msgs
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
# )
# install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

ament_package()
