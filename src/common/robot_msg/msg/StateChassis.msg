# msg definition

# charging State enum
uint8   UNCHARGING          = 0 # 未充电
uint8   CHARGING            = 1 # 充电中
uint8   DOCK_IN             = 2 # 接触，未充电
uint8   CHARGE_OK           = 3 # 充电完成
uint8   DC_CHARGING         = 4 # 直充-充电中
uint8   DC_DOCK_IN          = 5 # 直充-接触，未充电
uint8   DC_CHARGE_OK        = 6 # 直充-充电完成

std_msgs/Header                 header
robot_msg/LocalizationState     localization_state
uint8                           charging_state
float32                         bat_percentage
string[]                        error_code