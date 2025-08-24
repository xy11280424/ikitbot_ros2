# msg definition

# Charging_State State enum
uint8   UNCHARGING          = 0 # 未充电
uint8   CHARGING            = 1 # 充电中
uint8   DOCK_IN             = 2 # 接触，未充电
uint8   CHARGE_OK           = 3 # 充电完成
uint8   DC_CHARGING         = 4 # 直充-充电中
uint8   DC_DOCK_IN          = 5 # 直充-接触，未充电
uint8   DC_CHARGE_OK        = 6 # 直充-充电完成

std_msgs/Header                 header
uint16      charging_vol             #   充电电压  10mv
uint8       charging_state           #   充电状态
uint8       charge_ctrl              #   充电控制状态