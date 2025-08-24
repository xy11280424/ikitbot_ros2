std_msgs/Header header
string              status              # 状态
geometry_msgs/Pose  pose                # 全局位姿
float32             score               # 得分
float32             process             # 地图保存进图，100%为已保存状态
float32             expect_time         # 预测保存地图的时间，假设每秒优化2个
bool                relocated           # 重定位成功判断
float32             reloc_used_time     # 上次重定位耗费时间
int16               opt_works_remain    # 优化队列
float32             angle               # 地图旋转角度 radian
