-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- 这是一个Cartographer的2D定位配置文件示例。
-- 它通常用于在已知地图中进行机器人定位。

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
  tracking_frame = "base_link",
  -- 发布坐标系名称，通常是机器人本体坐标系
  published_frame = "base_link",
  -- 里程计坐标系名称
  odom_frame = "odom",
  -- 是否提供里程计坐标系到published_frame的变换
  provide_odom_frame = true,
  -- 是否发布投影到2D平面的坐标系
  publish_frame_projected_to_2d = false,
  -- 是否使用里程计数据
  use_odometry = false,
  -- 是否使用GNSS（全球导航卫星系统）数据
  use_nav_sat = false,
  -- 是否使用地标数据
  use_landmarks = false,
  -- 激光雷达扫描的数量
  num_laser_scans = 1,
  -- 多回波激光雷达扫描的数量
  num_multi_echo_laser_scans = 0,
  -- 每次激光雷达扫描的细分数量
  num_subdivisions_per_laser_scan = 1,
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
  -- 测距仪采样比例
  rangefinder_sampling_ratio = 1.,
  -- 里程计采样比例
  odometry_sampling_ratio = 1.,
  -- 固定坐标系位姿采样比例
  fixed_frame_pose_sampling_ratio = 1.,
  -- IMU采样比例
  imu_sampling_ratio = 1.,
  -- 地标采样比例
  landmarks_sampling_ratio = 1.,
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

-- 位姿图配置
POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- 优化问题中Huber损失函数的尺度参数
-- POSE_GRAPH.optimize_every_n_nodes = 35  -- 每N个节点进行一次位姿图优化 (此行被注释掉，使用下面的纯定位配置)
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 约束构建器的最小分数，低于此分数不添加约束

-- 纯定位模式：
-- 在纯定位模式下，Cartographer不会构建新地图，而是在现有地图中进行定位。
TRAJECTORY_BUILDER.pure_localization = true
-- 在纯定位模式下，位姿图优化的频率可以更高，以提高定位精度。
POSE_GRAPH.optimize_every_n_nodes = 10

-- 返回配置选项
return options
