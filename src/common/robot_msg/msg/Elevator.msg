# 电梯信息

string                      device_name         # 设备名称
string                      device_id           # 梯控设备ID
int32[]                     reachable_floor     # 电梯可到达楼层
geometry_msgs/Vector3       wait_pose           # 候梯点
geometry_msgs/Vector3       take_pose           # 乘梯点
float32[]                   size                # 电梯尺寸，宽度，深度，单位m
geometry_msgs/Polygon       polygon             # 电梯尺寸坐标
