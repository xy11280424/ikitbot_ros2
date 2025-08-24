# msg definition

# elevator state enum
int32 ELEVATOR_STATUS_IDLE      = 0 # 停止空闲
int32 ELEVATOR_STATUS_UP        = 1 # 上行
int32 ELEVATOR_STATUS_DOWN      = 2 # 下行
int32 ELEVATOR_STATUS_ERROR     = 3 # grpc 通信异常
int32 ELEVATOR_STATUS_UNKNOWN   = 4 # 未知，梯控异常

# elevator door state enum
int32 DOOR_STATUS_IDLE          = 0 # 停止空闲
int32 DOOR_STATUS_OPENING       = 1
int32 DOOR_STATUS_CLOSING       = 2
int32 DOOR_STATUS_UNKNOWN       = 3 # 未知，通信异常

# afe state enum 闸机状态


std_msgs/Header header

int32 current_floor     # elevator current floor

int32 elevator_status   # elevator status

int32 door_status       # door status

int32 msg_code          # server code

string msg              # code string
