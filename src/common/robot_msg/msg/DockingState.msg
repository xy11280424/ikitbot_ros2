# msg definition

# constants state enum
int32 NONE              = 0 #初始化状态
int32 SEARCHING_STATION = 1 # 1-寻找充电桩
int32 DOCKING_STATION   = 2 # 2-对接中
int32 DOCKING_SUCCESS   = 3 # 3-对接成功
int32 SEARCHING_FAILURE = 4 # 4-失败-未找到充电桩
int32 DOCKING_FAILURE   = 5 # 5-失败-对接失败
int32 CANCLED           = 6 # 6-成功-动作被取消

# fields
int32 		docking_state  #机器人回充状态

