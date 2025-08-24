string                      device_name         # 设备名称
string                      device_id           # 设备ID
geometry_msgs/Vector3[]     gate_pose           # 标记闸机的线段(起点终点)
# 目前规按线段形式描述闸机，所以数组中只取前两个坐标信息，保存时少于两个点会报错
