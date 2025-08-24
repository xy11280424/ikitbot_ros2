# msg definition

#   action_state_id: 								
#	"idle"			    - 空闲
#	"sleep"			    - 休眠
#	"movebase_goal"             - 导航到目标点
#	"rotate"		    - 旋转一定角度（body系）
#       "rotate_to"		    - 旋转到指定朝向（world系）
#	"go_charging"               - 返回充电
# 	"separate_docker"           - 脱离充电桩

# action_feedback
int32 NONE              = 0 # none
int32 SET_NEW_GOAL      = 1 # 收到指令
int32 RUNNING           = 2 # 动作执行中
int32 PREEMPTED         = 3 # 执行中 - 目标被抢占
int32 GOAL_REACHED      = 4 # 成功 - 目标达到
int32 GOAL_CANCLE       = 5 # 成功 - 目标被取消
int32 FAILURE           = 6 # 失败

# sub_action_feedback
# 100~199 navigation
int32   NAV_RECV_CMD                   = 1100    # recv cmd
int32   NAV_PLANNING                   = 2101    # planning
int32   NAV_CONTROLLING                = 2102    # controlling
int32   NAV_RECOVERY                   = 2103    # recovery
int32   NAV_WAIT                       = 2104    # wait
int32   NAV_HIJACKED_OUT_OF_ELEVATOR   = 2105    # robot is hijacked out of the elevator
int32   NAV_HITTING_AN_OBSTACLE        = 2106    # robot hitting an obstacle
int32   NAV_GOAL_PREEMPTED             = 3104    # preempted
int32   NAV_SUCCESS                    = 4105    # nav successfully
int32   NAV_TO_WAITING_ELEVATOR_AREA_SUCCESS  = 4106 # nav to waiting elevator area successfully
int32   NAV_CANCLE                     = 5106    # nav cancle

int32   NAV_GOAL_INVALID               = 6110    # goal invalid
int32   NAV_LOCATION_LOST              = 6111    # location lost
int32   NAV_PLANNING_FAILURE           = 6112    # planning failure
int32   NAV_CONTROLLING_FAILURE        = 6113    # controlling failure
int32   NAV_OSCILLATION_FAILURE        = 6114    # oscillation failure
int32   NAV_EMERGENCY_STOP             = 6115    # emergency stop
int32   NAV_IN_VIRTUAL_WALL            = 6116    # in virtual wall
int32   NAV_MOTOR_ERROR                = 6117    # motor error
int32   NAV_IDLING                     = 6118    # wheel idling
int32   NAV_NO_TRANSFER_ELEVATOR       = 6119    # have no transfer elevator to target floor
int32   NAV_CLIFF                      = 6120    # cliff
int32   NAV_NO_POSE                    = 6121    # can not get robot pose
int32   NAV_DEPTH_LAUNCH_FAIL          = 6122    # depth camera launch failed
int32   NAV_LIDAR_SENSOR_FAULT         = 6123    # lidar sensor fault
int32   NAV_WIRED_CHARGING             = 6124    # wired charging
int32   NAV_EMBEDDED_DISCONNECT        = 6125    # embedded disconnect
int32   NAV_NO_ELEVATOR_INFO           = 6126    # have no elevator info
int32   MOTOR_SERVICE_CALL_FAILURE     = 6127    # motor service call failure
int32   NAV_HITTIGING_OBSTACLE_FAILURE = 6128    # robot hitting obstavle failure
int32   FAILURE_OUT_OF_DISTANCE        = 6129    # failure, out of distance
int32   NAV_ROS_SERVICE_CALL_FAILURE   = 6198    # ros service call failure
int32   NAV_MISSION_TIMEOUT            = 6199    # mission time out

# 200~299 auto_docking
int32   DOCKING_SEARCHING_STATION         = 2200 # searching docker
int32   DOCKING_STATION                   = 2201 # docking station
int32   DOCKING_OBSTACLE                  = 2202 # docking obstacle
int32   DOCKING_SUCCESS                   = 4203 # docking successfully
int32   DOCKING_SEPERATING                = 2204 # seperatiing station
int32   DOCKING_SEPERATE_SUCCESS          = 4205 # seperate successfully
int32   DOCKING_CANCLE                    = 5206 # cancle
#退桩 
int32   DOCKING_SEPERATE_MISSION_TIMEOUT = 6209 # seperate timeout
int32   DOCKING_SEPERATE_FAILURE          = 6210 # seperate failure
#上桩
int32   DOCKING_FAILURE                   = 6211 # retry docking failure
#回充自由导航
int32   DOCKING_NAV_FAILURE               = 6212 # docking navigation failure 回充导航没到位，与目标点>0.8m
#找充电桩
int32   DOCKING_SEARCHING_FAILURE         = 6213 # searching failure，no detect docker pose,没有检测到充电桩，
int32   DOCKING_POS_FAIL                  = 6214 # detect error docker pose 检测到不合法的充电桩，距离很远>1.5m
#上桩
int32   DOCKING_OBSTACLE_FAILURE          = 6215 # auto_docking obstacle failure,上桩过程遇到障碍物
int32   DOCKING_FAILURE_NO_POWER          = 6216 # docking failure，no power，充电桩没电
int32   DOCKING_FAILURE_OUT_OF_DISTANCE   = 6217 # docking failure，out of distance,充电桩被推着走等


int32   DOCKING_SERVER_TIMEOUT            = 6298 # auto_docking action server connect timeout
int32   DOCKING_MISSION_TIMEOUT           = 6299 # mission time out

# 300~399 auto_elevator
int32   ELEVATOR_CALLING                 = 2300 # call elevator
int32   ELEVATOR_WAITING                 = 2301 # waitting elevator arrived
int32   ELEVATOR_GETTING_ON              = 2302 # getting on elevator
int32   ELEVATOR_GETTING_ON_SUCCESS      = 2303 # getting on successfully
int32   ELEVATOR_TAKING                  = 2304 # take the elevator
int32   ELEVATOR_GETTING_OFF             = 2305 # getting off elevator
int32   ELEVATOR_GETTING_ON_FULL         = 2306 # elevator full
int32   ELEVATOR_ARRIVED                 = 2307 # elevator arrived
int32   ELEVATOR_GETTING_OFF_SUCCESS     = 4306 # getting off successfully
int32   ELEVATOR_CANCLE                  = 5307 # cancle

int32   ELEVATOR_SERVICE_CALL_FAILURE   = 6310 # elevator service call failure
int32   ELEVATOR_GET_STATE_FAILURE      = 6311 # get elevator state failed
int32   ELEVATOR_GETTING_ON_FAILURE     = 6312 # getting on elevator failed
int32   ELEVATOR_GETTING_OFF_FAILURE    = 6313 # getting off elevator failed
int32   ELEVATOR_SWITCH_MAP_FAILURE     = 6314 # switch map failure
int32   ELEVATOR_WAITTING_TIMEOUT       = 6398 # waitting elevator timeout
int32   ELEVATOR_TAKING_TIMEOUT         = 6399 # taking elevator timeout


string                      action_state_id  	# 动作命令id
string                      task_id             # 任务id
geometry_msgs/Vector3       goal                # 当前处理目标值
int32                       action_feedback 	# 运行主状态反馈
int32                       sub_action_feedback # 运行子状态反馈


