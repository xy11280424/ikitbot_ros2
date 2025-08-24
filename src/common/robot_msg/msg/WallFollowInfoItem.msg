# msg definition
# wallfollow item type enum
uint8   WFI_UNKONW    = 0 # 未知
uint8   WFI_FW_R_UPPER          = 1   # Findwall right upper
uint8   WFI_FW_R_MIDDLE         = 2
uint8   WFI_FW_R_LOWER          = 3
uint8   WFI_CR_R_UPPER            = 4   # Circle right upper
uint8   WFI_CR_R_MIDDLE           = 5 
uint8   WFI_CR_R_LOWER           = 6
uint8   WFI_R_TRACKING           = 7     # 右侧沿边实时传感器距离
uint8   WFI_FW_L_UPPER          = 8   # Findwall left upper
uint8   WFI_FW_L_MIDDLE         = 9
uint8   WFI_FW_L_LOWER          = 10
uint8   WFI_CR_L_UPPER            = 11   # Circle left upper
uint8   WFI_CR_L_MIDDLE           = 12
uint8   WFI_CR_L_LOWER           = 13
uint8   WFI_L_TRACKING           = 14     # 左侧沿边实时传感器距离

std_msgs/Header header
uint8  wfi_type      # 沿边类型来源
robot_msg/ObstacleItem  obs_item


