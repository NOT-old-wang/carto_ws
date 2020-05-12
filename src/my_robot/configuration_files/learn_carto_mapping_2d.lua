include "map_builder.lua"
include "trajectory_builder.lua"

-- lua 配置中都是已 米 和 秒 为单位
-- options块中定义的值定义了Cartographer_ros 前端应如何与您的包进行交互
-- options段落后定义的值用于调整Cartographer的内部工作
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map", -- 坐标系全局坐标系
  tracking_frame = "base_link", -- 机器人中新坐标系 
  published_frame = "base_link", -- 用于发布（非循环关闭）本地SLAM结果
  odom_frame = "odom",
  provide_odom_frame = true,
  use_pose_extrapolator = true,  -- 使用位姿外推器
  publish_frame_projected_to_2d = false, -- 如果启用，则已发布的姿势将限制为纯2D姿势 无滚动，俯仰或z偏移
  use_odometry = false,
  use_nav_sat = false,  -- 启用GPS
  use_landmarks = false,  -- 启动地标
  num_laser_scans = 1,  -- sensor_msgs/LaserScan 雷达  /scan 的个数,多个如"scan_1" ,"scan_2"...
  num_multi_echo_laser_scans = 0,  -- sensor_msgs/MultiEchoLaserScan 回波雷达,多个激光，topics为“echoes_1”, “echoes_2”...
  num_subdivisions_per_laser_scan = 1,  -- 将每个接收到的（多回波）激光扫描分成的点云数
  num_point_clouds = 0,  -- sensor_msgs/PointCloud2 点云的个数,多个如“points2_1”, “points2_2”...    
  lookup_transform_timeout_sec = 0.2,  -- 使用tf2查找变换的超时秒数 
  submap_publish_period_sec = 0.3,  -- 发布子图的时间间隔（例如0.3秒）
  pose_publish_period_sec = 5e-3,  -- 发布姿势的秒数间隔，例如频率为200 Hz的5e-3
  trajectory_publish_period_sec = 30e-3,  -- 发布轨迹标记的间隔（以秒为单位），例如30e-3，持续30毫秒
  rangefinder_sampling_ratio = 1.,  -- 测距仪消息的固定比率采样
  odometry_sampling_ratio = 1.,  -- 里程消息的固定比率采样
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定帧消息的固定比率采样
  imu_sampling_ratio = 1.,  -- IMU消息的固定比率采样
  landmarks_sampling_ratio = 1.,  -- 地标消息的固定比率采样 
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- 开启2D slam
MAP_BUILDER.num_background_threads = 8  -- 核数

-- 开启 imu 用作扫描方向的初始猜测，大大降低了扫描匹配的复杂性 3D必须
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10

-- 带通滤波器 雷达测距数据的可用范围
TRAJECTORY_BUILDER_2D.min_range= 1. 
TRAJECTORY_BUILDER_2D.max_range = 25.
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 2
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 点云缓存到一定数目 scan 后插入 submap

-- 固定的体素滤波 原始点下采样为一个恒定大小的立方体，并只保留每个立方体的质心
-- 较小的立方体大小将导致更密集的数据,较大的立方体大小会导致数据丢失，但会更快
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.25

-- 自适应体素滤波 votex filter 此滤器尝试确定最佳体素大小（在最大长度下）以实现目标点数
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.
-- TODO:
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.

-- 两种扫描匹配策略 1.real time correlative scan matcher 2.ceres scan matcher

-- real time correlative
-- 没有其他传感器或者您不信任它们，则可以启用，这种扫描匹配器非常昂贵，并且基本上会覆盖来自其他传感器但除了测距仪的任何信号，但它在功能丰富的环境中非常强大。
-- 它使用的方法类似于在循环闭包中将扫描与子图匹配的方式，但它与当前子图匹配。然后将最佳匹配用作之前的CeresScanMatcher。
-- 工作原理是在搜索窗口中搜索类似的扫描，搜索窗口由最大距离半径和最大角度半径定义
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- ceres scan matcher
-- 取初始猜测事先并发现其中扫描匹配适合的子地图的最佳点,通过插入子图和子像素对齐扫描来实现
-- 无法修复明显大于子图分辨率的错误
-- 传感器设置和时间是合理的，那么仅使用CeresScanMatcher它通常是最佳选择
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40
-- Ceres使用下降算法针对给定的迭代次数优化运动。Ceres可以配置为根据您自己的需要调整收敛速度
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- 运动滤波器 motion filter [根据 距离,角度,时间]    
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 7.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 1.5
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(9.)

-- 子图配置  
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150  -- 子图尺寸: node 的个数
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_options_2d = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
-- 子图插入概率的更新 0,1是障碍物, 更新障碍物的概率 和 更新非障碍物的概率  
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true   
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.62
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.47
-- TODO:(other submap options)

-- 关闭全局SLAM：0
POSE_GRAPH.optimize_every_n_nodes = 0

-- 限制约束（和计算）的数量, 参数调整核心是闭环检测（约束检测）
-- 闭环检测是图优化过程中最为重要的部分，也是最为耗时的部分，
-- 因此减少约束总数和搜索范围可以有效提高实时性
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 采样少的节点不进入计算
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.55  -- 成为约束的最低分数,值越大，计算速度相对越快，约束数量相对越少
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
POSE_GRAPH.constraint_builder.log_matches = true  -- 获得格式化为直方图的约束构建器的常规报告
-- 实现实时循环闭合扫描匹配策略: fast correlative scan matcher
-- FastCorrelativeScanMatcher 依靠“ 分支定界 ”机制在不同的格点分辨率的工作，有效地消除不正确匹配数
-- TODO: 对应于闭环检测（约束检测）时的搜索范围，可控制深度的搜索树
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
-- TODO: fast correlative scan matcher 匹配完后(分数高于最低分数)，输入ceres扫描匹配器用于改进位姿
-- Ceres用于根据多个残差重新排列子图
-- 残差是使用加权损失函数计算的
-- 全局优化具有成本函数以考虑大量数据源：全局（循环闭包）约束，非全局（匹配器）约束，IMU加速和旋转测量，局部SLAM粗略姿态估计，测距源或固定框架（如GPS系统）
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3

-- 调整局部SLAM和里程计的各个权重
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight =1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 7

-- 迭代次数
POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.log_residual_histograms = true
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.

return options

