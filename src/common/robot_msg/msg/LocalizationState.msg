# msg definition 对Android

# localization state enum
int32 NONE         = 0 # 初始化
int32 MAP_BUILDING = 1 # 建图中
int32 LOCATING     = 2 # 定位中
int32 SUCCESS      = 3 # 定位成功
int32 FAILURE      = 4 # 定位失败
int32 MAP_SAVING   = 5 # 地图保存中
int32 MAP_LOADING  = 6 # 地图加载中

std_msgs/Header         header
int32 		        localization_state      # 机器人定位状态
string                  map_name                # 机器人当前使用地图名称
int32                   floor                   # 机器人当前所处楼层
geometry_msgs/Vector3 	global_pose             # 机器人全局位置坐标，(x，y，thete)
float32                 score                   # 得分
float32                 progress                # 地图保存进度，100%为已保存状态
