# msg definition
# obs source type enum
uint8   OST_UNKOWN    = 0 # 未知
uint8   OST_LIDAR          = 1 # 激光雷达
uint8   OST_LINE_LASER         = 2 # 线激光
uint8   OST_IR            = 3 # 红外
uint8   OST_ULTRASONIC       = 4 # 超声波 ultrasonic
uint8   OST_DEPTH_CAMERA            = 5 # 深度相机

uint8 obs_source_type      # 障碍数据来源
float32 obs_distance         # 障碍的距离
float32 obs_angle               # 障碍的角度(弧度)
float32 obs_height            # 障碍的高度


