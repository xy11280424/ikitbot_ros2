# msg definition

# nav_hsm_flex_state enum
int32   NONE                           = 0 # none

# 100~199 navigation
int32   NAV_RECEIVED_COMMAND           = 1100    # recv cmd
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
int32   NAV_IDLING_FAIL                = 6118    # wheel idling
int32   NAV_NO_TRANSFER_ELEVATOR       = 6119    # have no transfer elevator to target floor
int32   NAV_CLIFF                      = 6120    # cliff
int32   NAV_NO_POSE                    = 6121    # can not get robot pose
int32   NAV_DEPTH_LAUNCH_FAIL          = 6122    # depth camera launch failed
int32   NAV_LIDAR_SENSOR_FAULT         = 6123    # lidar sensor fault
int32   NAV_WIRED_CHARGING             = 6124    # wired charging
int32   NAV_EMBEDDED_DISCONNECT        = 6125    # embedded disconnect
int32   NAV_NO_ELEVATOR_INFO           = 6126    # have no elevator info
int32   NAV_MOTOR_SERVICE_CALL_FAILURE = 6127    # motor service call failure
int32   NAV_BUMP_FAIL                  = 6128    # robot bump failure
int32   NAV_ROS_SERVICE_CALL_FAILURE   = 6198    # ros service call failure
int32   NAV_MISSION_TIMEOUT            = 6199    # mission time out

int32   NAV_CONTROLLING_FAIL_STOP_CANCEL   = 6200      # stop cancel
int32   NAV_CONTROLLING_FAIL_STUCK         = 6201      # stuck
int32   NAV_CONTROLLING_FAIL_LOCAL_PLAN    = 6202      # local plan
int32   NAV_CONTROLLING_FAIL_TIMEOUT_PLAN  = 6203      # timeout plan
int32   NAV_CONTROLLING_FAIL_LOCAL_GOAL    = 6204      # local goal
int32   NAV_CONTROLLING_FAIL_ROBOT_POSE    = 6205      # robot pose
int32   NAV_CONTROLLING_FAIL_PATH_POSE     = 6206      # path pose
int32   NAV_CONTROLLING_FAIL_INIT          = 6207      # init
int32   NAV_CONTROLLING_FAIL_TF            = 6208      # tf
int32   NAV_CONTROLLING_FAIL_PLAN_GOAL     = 6209      # plan goal
int32   NAV_CONTROLLING_FAIL_COLLISION_BAR = 6210      # collision bar
int32   NAV_PLANNING_START_FAIL            = 6211      # planning start fail
int32   NAV_PLANNING_GOAL_FAIL             = 6212      # planning goal fail
int32   NAV_WAIT_TIMEOUT                   = 6213      # nav wait timeout
int32   NAV_BYPASS_TIMEOUT                 = 6214      # nav bypass timeout


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

int32   ELEVATOR_SERVICE_CALL_FAILURE   = 6310 # elevator service call failure
int32   ELEVATOR_GET_STATE_FAILURE      = 6311 # get elevator state failed
int32   ELEVATOR_GETTING_ON_FAILURE     = 6312 # getting on elevator failed
int32   ELEVATOR_GETTING_OFF_FAILURE    = 6313 # getting off elevator failed
int32   ELEVATOR_SWITCH_MAP_FAILURE     = 6314 # switch map failure
int32   ELEVATOR_FLOOR_MAP_NOT_EXIST    = 6315 # floor map does not exist
int32   ELEVATOR_WAITTING_TIMEOUT       = 6398 # waitting elevator timeout
int32   ELEVATOR_TAKING_TIMEOUT         = 6399 # taking elevator timeout

int32   nav_hsm_flex_state 	  # 状态枚举类型
string  nav_hsm_flex_state_str    # 状态字符串

