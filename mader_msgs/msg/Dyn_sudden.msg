Header header
# 只给不可提前建图得到轨迹的动态障碍物使用，针对无人机预测模块
# 地图发布时仍需要动态障碍物的准确轨迹，即 DynTraj.msg
# 基本信息
int32 id                   # 障碍物唯一ID
bool is_agent              # true=智能体，false=障碍物

# 几何信息
float32[] bbox             # 边界框尺寸 [hx, hy, hz]
geometry_msgs/Vector3 pos  # 当前位置 (x,y,z)

# 运动状态（所有障碍物都需要）
geometry_msgs/Vector3 velocity  # 当前速度 (vx, vy, vz)

geometry_msgs/Vector3 acceleration  # 当前加速度 (ax, ay, az)   ？？？这个要吗  还不清楚

# 障碍物预测信息（可选，用于动态障碍物）
float32 prediction_horizon  # 预测时间范围（秒）
geometry_msgs/Vector3[] predicted_trajectory  # 简化的预测轨迹点序列