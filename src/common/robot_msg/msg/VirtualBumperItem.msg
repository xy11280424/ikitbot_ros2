# msg definition
# virtual bumper type enum
uint8   VBT_UNKONW    = 0 # 未知
uint8   VBT_FRONT          = 1 
uint8   VBT_FRONT_LEFT         = 2
uint8   VBT_LEFT          = 3
uint8   VBT_BACK_LEFT            = 4
uint8   VBT_BACK           = 5 
uint8   VBT_BACK_RIGHT           = 6 
uint8   VBT_RIGHT           = 7 
uint8   VBT_FRONT_RIGHT           = 8 

std_msgs/Header header
uint8  vmt_type      # 障碍类型来源
robot_msg/ObstacleItem  obs_item


