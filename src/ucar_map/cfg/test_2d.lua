-- 引入地图构建器配置
include "map_builder.lua"
-- 引入轨迹构建器配置
include "trajectory_builder.lua"

-- 定义Cartographer的全局配置选项
options = {
  -- 地图构建器配置，使用map_builder.lua中定义的MAP_BUILDER
  map_builder = MAP_BUILDER,
  -- 轨迹构建器配置，使用trajectory_builder.lua中定义的TRAJECTORY_BUILDER
  trajectory_builder = TRAJECTORY_BUILDER,
  -- 地图坐标系名称
  map_frame = "map",
  -- 跟踪坐标系名称，通常是机器人本体坐标系
  tracking_frame  = "base_link",
  -- 发布坐标系名称，通常是机器人本体坐标系
  published_frame = "base_link",
  -- 里程计坐标系名称
  odom_frame = "odom",
  -- 是否提供里程计坐标系到published_frame的变换
  provide_odom_frame = true,
  -- 是否使用里程计数据
  use_odometry = false,
  -- 是否使用激光雷达扫描数据
  use_laser_scan = true,
  -- 是否使用多回波激光雷达扫描数据
  use_multi_echo_laser_scan = false,
  -- 激光雷达扫描的数量
  num_laser_scans = 1,
  -- 点云的数量
  num_point_clouds = 0,
  -- 查找坐标变换的超时时间（秒）
  lookup_transform_timeout_sec = 0.2,
  -- 子图发布周期（秒）
  submap_publish_period_sec = 0.3,
  -- 位姿发布周期（秒）
  pose_publish_period_sec = 5e-3,
  -- 轨迹发布周期（秒）
  trajectory_publish_period_sec = 30e-3,
}

-- 配置MAP_BUILDER使用2D轨迹构建器
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D轨迹构建器配置
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- 每个子图使用的激光雷达数据帧数
TRAJECTORY_BUILDER_2D.min_range = 0.3  -- 激光雷达最小有效距离
TRAJECTORY_BUILDER_2D.max_range = 8.  -- 激光雷达最大有效距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.  -- 缺失数据射线的长度
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 是否使用IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 是否使用在线相关扫描匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- 实时相关扫描匹配的线性搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  -- 实时相关扫描匹配的平移增量成本权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1  -- 实时相关扫描匹配的旋转增量成本权重

-- 稀疏位姿图配置
SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- 优化问题中Huber损失函数的尺度参数
SPARSE_POSE_GRAPH.optimize_every_n_scans = 35  -- 每N次扫描进行一次稀疏位姿图优化
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.65  -- 约束构建器的最小分数，低于此分数不添加约束

-- 返回配置选项
return options
